/*
 * VA-API -> apple-ave rate-control mapping and DMA-BUF import validation.
 *
 * Each VA rate field is either applied through an apple-ave control, or
 * explicitly documented here as having no effect. Anything else is refused.
 *
 * Ported from aurora-ave-vaapi e40b036 (src/encode_params.c,
 * src/surface_import.c).
 *
 * Copyright (c) 2026 snowf14k3
 * Copyright (c) 2026 libva-v4l2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <string.h>

#include <drm_fourcc.h>
#include <linux/v4l2-controls.h>

#include "ave_params.h"

int ave_parse_frame_rate(uint32_t value, uint32_t *result)
{
	uint32_t numerator = value & 0xffffu;
	uint32_t denominator = value >> 16;
	uint32_t rounded;

	if (denominator == 0)
		denominator = 1;
	if (numerator == 0)
		return -EINVAL;
	rounded = (numerator + denominator / 2) / denominator;
	/* V4L2 S_PARM on apple-ave accepts 1..240 frames per second. */
	if (rounded == 0 || rounded > 240)
		return -ERANGE;
	*result = rounded;
	return 0;
}

int ave_parse_misc(VAEncMiscParameterType type, const void *data,
		   size_t size, struct ave_rate_request *rate)
{
	if (!data || !rate)
		return -EINVAL;

	switch (type) {
	case VAEncMiscParameterTypeRateControl: {
		VAEncMiscParameterRateControl control;

		if (size < sizeof(control))
			return -EINVAL;
		memcpy(&control, data, sizeof(control));
		/*
		 * Not possible on apple-ave: resetting the rate controller
		 * mid-stream, per-temporal-layer rates, parallel/dynamic BRC,
		 * and turning off CBR bit stuffing (the kernel driver pads CBR
		 * output with filler data). disable_frame_skip is satisfied
		 * (AVE never skips). mb_rate_control and frame_tolerance_mode
		 * only apply with VA_RC_MB / FrameSizeToleranceSupport, neither
		 * advertised.
		 */
		if (control.rc_flags.bits.reset ||
		    control.rc_flags.bits.temporal_id ||
		    control.rc_flags.bits.disable_bit_stuffing ||
		    control.rc_flags.bits.enable_parallel_brc ||
		    control.rc_flags.bits.enable_dynamic_scaling)
			return -ENOTSUP;
		rate->rate_seen = true;
		rate->bits_per_second = control.bits_per_second;
		rate->target_percentage = control.target_percentage;
		rate->min_qp = control.min_qp;
		rate->max_qp = control.max_qp;
		/*
		 * window_size and initial_qp have no apple-ave control: the
		 * firmware uses its own one-second window and initial QP.
		 */
		return 0;
	}
	case VAEncMiscParameterTypeFrameRate: {
		VAEncMiscParameterFrameRate frame_rate;
		int status;

		if (size < sizeof(frame_rate))
			return -EINVAL;
		memcpy(&frame_rate, data, sizeof(frame_rate));
		if (frame_rate.framerate_flags.bits.temporal_id)
			return -ENOTSUP;
		status = ave_parse_frame_rate(frame_rate.framerate,
					      &rate->frames_per_second);
		if (status < 0)
			return status;
		rate->frame_rate_seen = true;
		return 0;
	}
	case VAEncMiscParameterTypeHRD:
		/*
		 * apple-ave uses a fixed 0.5 s VBV (ave-sess.c CBR filler
		 * model); buffer size/fullness are advisory and cannot be
		 * applied.
		 */
		if (size < sizeof(VAEncMiscParameterHRD))
			return -EINVAL;
		return 0;
	case VAEncMiscParameterTypeQualityLevel:
		/*
		 * VAConfigAttribEncQualityRange is not advertised, which per
		 * va.h means the driver has a single quality level.
		 */
		return 0;
	default:
		return -ENOTSUP;
	}
}

int ave_rate_settings_from_va(uint32_t va_rate_control,
			      const struct ave_rate_request *rate,
			      uint32_t sequence_bits_per_second,
			      struct ave_rate_settings *settings)
{
	uint32_t bits_per_second;

	if (!rate || !settings)
		return -EINVAL;
	memset(settings, 0, sizeof(*settings));

	bits_per_second = rate->rate_seen && rate->bits_per_second ?
			  rate->bits_per_second : sequence_bits_per_second;
	if (!bits_per_second)
		return -EINVAL;

	if (va_rate_control == VA_RC_CBR) {
		settings->v4l2_mode = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
		settings->bitrate = bits_per_second;
	} else if (va_rate_control == VA_RC_VBR) {
		uint32_t percentage = rate->rate_seen ? rate->target_percentage : 0;
		uint64_t target;

		/* VA VBR: bits_per_second is the peak, the target a percentage. */
		if (percentage == 0 || percentage > 100)
			percentage = 100;
		target = (uint64_t)bits_per_second * percentage / 100u;
		settings->v4l2_mode = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
		settings->bitrate = (uint32_t)target;
		/*
		 * A peak equal to the average starves apple-ave's VBR; 0
		 * selects the driver default (twice the average, as macOS
		 * configures it).
		 */
		settings->peak = percentage < 100 ? bits_per_second : 0;
	} else {
		return -ENOTSUP;
	}

	if (rate->rate_seen && (rate->min_qp || rate->max_qp)) {
		settings->qp_range = true;
		settings->min_qp = rate->min_qp;
		settings->max_qp = rate->max_qp ? rate->max_qp : 51u;
		if (settings->max_qp > 51u || settings->min_qp > settings->max_qp)
			return -ERANGE;
	}
	return 0;
}

uint32_t ave_gop_from_va(uint32_t intra_idr_period, uint32_t intra_period)
{
	uint32_t gop = intra_idr_period ? intra_idr_period : intra_period;

	/*
	 * apple-ave's GOP control is the IDR interval, 1..65535 frames (0
	 * would mean the firmware's own 120). "No periodic IDR" and longer
	 * periods become the longest interval it supports.
	 */
	if (gop == 0 || gop > 65535u)
		gop = 65535u;
	return gop;
}

/* --- DMA-BUF import layouts --- */

static unsigned int bytes_per_sample(uint32_t fourcc)
{
	return fourcc == VA_FOURCC_P010 ? 2u :
	       fourcc == VA_FOURCC_NV12 ? 1u : 0u;
}

static int finish_layout(uint32_t fourcc, uint32_t width, uint32_t height,
			 uint32_t pitch, uint64_t y_offset, uint64_t uv_offset,
			 uint64_t object_size, int fd,
			 struct ave_import_layout *layout)
{
	unsigned int bps = bytes_per_sample(fourcc);
	uint64_t row_bytes;
	uint64_t y_end;
	uint64_t uv_end;

	if (!bps || !width || !height || (width & 1u) || (height & 1u) || fd < 0)
		return -EINVAL;
	row_bytes = (uint64_t)width * bps;
	if (pitch < row_bytes)
		return -EINVAL;
	y_end = y_offset + (uint64_t)pitch * height;
	uv_end = uv_offset + (uint64_t)pitch * (height / 2u);
	/* Luma first, chroma after it, no overlap. */
	if (uv_offset < y_end)
		return -EINVAL;
	if (object_size && uv_end > object_size)
		return -EINVAL;
	if (uv_end > SIZE_MAX)
		return -EOVERFLOW;

	*layout = (struct ave_import_layout) {
		.fourcc = fourcc,
		.width = width,
		.height = height,
		.stride = pitch,
		.y_offset = (size_t)y_offset,
		.uv_offset = (size_t)uv_offset,
		.required_size = (size_t)uv_end,
		.object_size = (size_t)object_size,
		.fd = fd,
	};
	return 0;
}

int ave_import_layout_from_external(const VASurfaceAttribExternalBuffers *ext,
				    uint32_t width, uint32_t height,
				    uint32_t fourcc,
				    struct ave_import_layout *layout)
{
	if (!ext || !layout || !ext->buffers)
		return -EINVAL;
	/* Tiled or protected buffers cannot be read through a CPU mapping. */
	if (ext->flags & (VA_SURFACE_EXTBUF_DESC_ENABLE_TILING |
			  VA_SURFACE_EXTBUF_DESC_PROTECTED))
		return -ENOTSUP;
	if (ext->pixel_format != fourcc || ext->num_planes != 2 ||
	    ext->num_buffers != 1 || ext->width < width ||
	    ext->height < height || ext->pitches[0] != ext->pitches[1])
		return -EINVAL;
	if (ext->buffers[0] > (uintptr_t)INT32_MAX)
		return -EINVAL;
	return finish_layout(fourcc, width, height, ext->pitches[0],
			     ext->offsets[0], ext->offsets[1], ext->data_size,
			     (int)ext->buffers[0], layout);
}

int ave_import_layout_from_prime2(const VADRMPRIMESurfaceDescriptor *prime,
				  uint32_t width, uint32_t height,
				  uint32_t fourcc,
				  struct ave_import_layout *layout)
{
	uint32_t y_pitch;
	uint32_t uv_pitch;
	uint64_t y_offset;
	uint64_t uv_offset;
	bool p010 = fourcc == VA_FOURCC_P010;
	uint32_t luma_format = p010 ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
	uint32_t chroma_format = p010 ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
	uint32_t combined_format = p010 ? DRM_FORMAT_P010 : DRM_FORMAT_NV12;

	if (!prime || !layout)
		return -EINVAL;
	if (prime->fourcc != fourcc || prime->width < width ||
	    prime->height < height || prime->num_objects != 1)
		return -EINVAL;
	/* Only linear buffers can be read row by row through mmap. */
	if (prime->objects[0].drm_format_modifier != DRM_FORMAT_MOD_LINEAR)
		return -ENOTSUP;

	if (prime->num_layers == 1) {
		if (prime->layers[0].drm_format != combined_format ||
		    prime->layers[0].num_planes != 2 ||
		    prime->layers[0].object_index[0] != 0 ||
		    prime->layers[0].object_index[1] != 0)
			return -EINVAL;
		y_pitch = prime->layers[0].pitch[0];
		uv_pitch = prime->layers[0].pitch[1];
		y_offset = prime->layers[0].offset[0];
		uv_offset = prime->layers[0].offset[1];
	} else if (prime->num_layers == 2) {
		if (prime->layers[0].drm_format != luma_format ||
		    prime->layers[1].drm_format != chroma_format ||
		    prime->layers[0].num_planes != 1 ||
		    prime->layers[1].num_planes != 1 ||
		    prime->layers[0].object_index[0] != 0 ||
		    prime->layers[1].object_index[0] != 0)
			return -EINVAL;
		y_pitch = prime->layers[0].pitch[0];
		uv_pitch = prime->layers[1].pitch[0];
		y_offset = prime->layers[0].offset[0];
		uv_offset = prime->layers[1].offset[0];
	} else {
		return -EINVAL;
	}
	if (y_pitch != uv_pitch)
		return -EINVAL;
	return finish_layout(fourcc, width, height, y_pitch, y_offset, uv_offset,
			     prime->objects[0].size, prime->objects[0].fd, layout);
}
