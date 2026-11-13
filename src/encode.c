/*
 * Hardware encoding on the Apple AVE stateful V4L2 encoder (apple-ave).
 *
 * The rest of this driver drives stateless (Request API) decoders. Encoding
 * is a separate engine that shares only the VA object model with them:
 *
 *  - configs: VAEntrypointEncSlice configs have no decode codec; they carry
 *    the negotiated rate control and packed-header mode;
 *  - contexts: an encode context owns one apple-ave M2M session, opened at
 *    the first EndPicture (the stream parameters arrive in the sequence
 *    parameter buffer, not at vaCreateContext);
 *  - surfaces: any surface of this driver can be an encode input. Its pixels
 *    are read through the same memory view vaGetImage uses (the decoder
 *    CAPTURE buffer of a decoded surface, or the standalone backing of a
 *    client-filled one), or through a read-only mapping of an imported
 *    DMA-BUF. They are copied into AVE's OUTPUT layout at EndPicture, so the
 *    surface is free again once vaSyncSurface returns; nothing binds it to
 *    the encode context;
 *  - buffers: parameter buffers are snapshotted at RenderPicture; coded
 *    buffers collect the Annex B packets apple-ave returns.
 *
 * All encode state is serialized by drv->api_mutex: the picture-level and
 * surface-sync entrypoints already hold it (api.c), and the coded-buffer
 * paths of vaMapBuffer, vaDestroyBuffer and vaSyncBuffer take it here.
 * drv->mutex only guards handle lookups and is never held across a wait.
 *
 * The VA <-> apple-ave mapping is ported from aurora-ave-vaapi e40b036
 * (src/encode.c, src/objects.c, src/decode.c, src/backend.c), itself a port
 * of snowf14k3/venus-vaapi-driver 37e1030. See docs/AVE_ENCODE.md.
 *
 * Copyright (c) 2026 snowf14k3
 * Copyright (c) 2026 libva-v4l2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <va/va_drmcommon.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>

#include "v4l2_request.h"
#include "ave_encoder.h"
#include "ave_params.h"

/*
 * VA-facing limits of the apple-ave encoder (ave-v4l2.c: AVE_MIN_W/H
 * 320x240, AVE_MAX_W/H 4096x2304, even sizes; HEVC additionally caps the
 * picture area at AVE_HEVC_MAX_SAMPLES). The kernel silently clamps S_FMT
 * sizes, so the VA layer must reject anything outside these limits instead
 * of encoding a different size.
 */
#define AVE_MIN_WIDTH		320u
#define AVE_MIN_HEIGHT		240u
#define AVE_MAX_WIDTH		4096u
#define AVE_MAX_HEIGHT		2304u
#define AVE_HEVC_MAX_SAMPLES	8912896u
#define AVE_H264_MAX_MACROBLOCKS 36864u
/*
 * AVE writes its own SPS/PPS/VPS and slice headers and has no way to take
 * client-built headers or extra NAL units: no packed header is advertised.
 */
#define AVE_PACKED_HEADERS	VA_ENC_PACKED_HEADER_NONE
/* AVE rate control: CBR and VBR only (CQ/constant-QP is not exposed). */
#define AVE_RATE_CONTROLS	(VA_RC_CBR | VA_RC_VBR)

#define AVE_OUTPUT_BUFFERS	4u
#define AVE_CAPTURE_BUFFERS	8u
#define AVE_DEFAULT_FPS		30u
#define AVE_KEYFRAME_IDLE_TIMEOUT_MS	5000
#define AVE_SYNC_TIMEOUT_MS	30000
#define AVE_MAP_TIMEOUT_MS	5000
#define AVE_QUEUE_LENGTH	64

/* State of one VAEncCodedBufferType buffer. */
struct v4l2r_coded {
	VACodedBufferSegment segment;
	size_t size;			/* bytes collected so far */
	unsigned int packets;
	bool ready;			/* the complete frame arrived */
	bool delivered;			/* mapped in submission order */
	bool input_done;		/* apple-ave released the raw frame */
	bool keyframe_expected;
	uint64_t sequence;		/* submission number in its context */
	uint64_t tag;			/* V4L2 timestamp of its frame, 0 = idle */
	VASurfaceID source;
};

/* Read-only mapping of an imported DMA-BUF encode input. */
struct v4l2r_surface_import {
	int fd;
	void *map;
	size_t map_size;
	size_t y_offset;
	size_t uv_offset;
	uint32_t stride;
	uint32_t fourcc;
};

/* Parameters of the picture being assembled, copied at RenderPicture. */
struct encode_picture {
	int error;			/* first failure, reported at EndPicture */
	bool have_sequence;
	bool have_picture;
	union {
		VAEncSequenceParameterBufferH264 h264;
		VAEncSequenceParameterBufferHEVC hevc;
	} sequence;
	union {
		VAEncPictureParameterBufferH264 h264;
		VAEncPictureParameterBufferHEVC hevc;
	} picture;
	struct ave_rate_request rate;
	unsigned int slices;
	uint32_t slice_macroblocks;	/* H.264 */
	uint32_t num_ctu_in_slice;	/* HEVC */
	bool intra;
};

struct v4l2r_encode {
	struct v4l2r_context *ctx;
	VAProfile profile;
	uint32_t rate_control;
	bool hevc;
	bool main10;
	unsigned int width;		/* vaCreateContext size (coded) */
	unsigned int height;
	unsigned int encode_width;	/* size given to apple-ave */
	unsigned int encode_height;
	struct ave_encoder *encoder;

	VASurfaceID target;
	struct encode_picture pic;

	/* Coded buffers waiting for their packets, in submission order. */
	VABufferID queue[AVE_QUEUE_LENGTH];
	size_t queue_head;
	size_t queue_count;

	uint32_t frames_per_second;
	uint64_t sequence;		/* pictures submitted */
	uint64_t delivery_sequence;	/* next coded buffer to hand out */
	uint32_t bitrate;
	unsigned int keyframe_misses;
};

static const VAProfile encode_profiles[] = {
	VAProfileH264ConstrainedBaseline,
	VAProfileH264Main,
	VAProfileH264High,
	VAProfileHEVCMain,
	VAProfileHEVCMain10,
};

/* --- capabilities --- */

static bool is_h264_profile(VAProfile profile)
{
	return profile == VAProfileH264ConstrainedBaseline ||
	       profile == VAProfileH264Main || profile == VAProfileH264High;
}

bool v4l2r_encode_profile_supported(const struct v4l2r_driver *drv,
				    VAProfile profile)
{
	const struct ave_device *ave = &drv->ave;

	if (!ave->present)
		return false;
	if (is_h264_profile(profile))
		return (ave->coded & AVE_CODED_H264) && (ave->raw & AVE_RAW_NV12);
	if (profile == VAProfileHEVCMain)
		return (ave->coded & AVE_CODED_HEVC) && (ave->raw & AVE_RAW_NV12);
	if (profile == VAProfileHEVCMain10)
		return (ave->coded & AVE_CODED_HEVC) && (ave->raw & AVE_RAW_P010);
	return false;
}

unsigned int v4l2r_encode_profiles(const struct v4l2r_driver *drv,
				   VAProfile *profiles, unsigned int max)
{
	unsigned int count = 0;

	for (unsigned int i = 0;
	     i < sizeof(encode_profiles) / sizeof(encode_profiles[0]); i++) {
		if (count < max &&
		    v4l2r_encode_profile_supported(drv, encode_profiles[i]))
			profiles[count++] = encode_profiles[i];
	}
	return count;
}

static unsigned int encode_rt_format(VAProfile profile)
{
	return profile == VAProfileHEVCMain10 ? VA_RT_FORMAT_YUV420_10 :
						VA_RT_FORMAT_YUV420;
}

static uint32_t encode_fourcc(VAProfile profile)
{
	return profile == VAProfileHEVCMain10 ? VA_FOURCC_P010 : VA_FOURCC_NV12;
}

VAStatus v4l2r_encode_get_config_attributes(const struct v4l2r_driver *drv,
					    VAProfile profile,
					    VAConfigAttrib *attributes,
					    int count)
{
	if (!v4l2r_encode_profile_supported(drv, profile))
		return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

	for (int i = 0; i < count; i++) {
		switch (attributes[i].type) {
		case VAConfigAttribRTFormat:
			attributes[i].value = encode_rt_format(profile);
			break;
		case VAConfigAttribRateControl:
			/*
			 * V4L2_CID_MPEG_VIDEO_BITRATE_MODE on apple-ave offers
			 * CBR and VBR; CQ is masked out and there are no
			 * frame-QP controls, so constant-QP is not advertised.
			 */
			attributes[i].value = AVE_RATE_CONTROLS;
			break;
		case VAConfigAttribEncPackedHeaders:
			attributes[i].value = AVE_PACKED_HEADERS;
			break;
		case VAConfigAttribEncMaxRefFrames:
			/*
			 * Low-delay P only (B_FRAMES=0): one forward list, no
			 * backward list. The firmware picks its own references.
			 */
			attributes[i].value = 1;
			break;
		case VAConfigAttribEncMaxSlices:
			attributes[i].value = 1;
			break;
		case VAConfigAttribMaxPictureWidth:
			attributes[i].value = AVE_MAX_WIDTH;
			break;
		case VAConfigAttribMaxPictureHeight:
			attributes[i].value = AVE_MAX_HEIGHT;
			break;
		default:
			attributes[i].value = VA_ATTRIB_NOT_SUPPORTED;
			break;
		}
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_encode_create_config(const struct v4l2r_driver *drv,
				    VAProfile profile,
				    const VAConfigAttrib *attributes,
				    int count, struct v4l2r_config *config)
{
	uint32_t rate_control = VA_RC_VBR;
	uint32_t packed_headers = 0;

	if (!v4l2r_encode_profile_supported(drv, profile))
		return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

	for (int i = 0; i < count; i++) {
		switch (attributes[i].type) {
		case VAConfigAttribRTFormat:
			if (!(attributes[i].value & encode_rt_format(profile)))
				return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
			break;
		case VAConfigAttribRateControl:
			if (attributes[i].value != VA_RC_VBR &&
			    attributes[i].value != VA_RC_CBR)
				return VA_STATUS_ERROR_ATTR_NOT_SUPPORTED;
			rate_control = attributes[i].value;
			break;
		case VAConfigAttribEncPackedHeaders:
			if (attributes[i].value & ~(uint32_t)AVE_PACKED_HEADERS)
				return VA_STATUS_ERROR_ATTR_NOT_SUPPORTED;
			packed_headers = attributes[i].value;
			break;
		default:
			break;
		}
	}

	config->profile = profile;
	config->entrypoint = VAEntrypointEncSlice;
	config->codec = NULL;
	config->rt_format = encode_rt_format(profile);
	config->rate_control = rate_control;
	config->packed_headers = packed_headers;
	config->attributes[0] = (VAConfigAttrib) {
		.type = VAConfigAttribRTFormat, .value = config->rt_format,
	};
	config->attributes[1] = (VAConfigAttrib) {
		.type = VAConfigAttribRateControl, .value = rate_control,
	};
	config->attributes[2] = (VAConfigAttrib) {
		.type = VAConfigAttribEncPackedHeaders, .value = packed_headers,
	};
	config->nb_attributes = 3;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_encode_surface_attributes(const struct v4l2r_config *config,
					 VASurfaceAttrib *list,
					 unsigned int *count)
{
	const VASurfaceAttrib values[] = {
		{
			.type = VASurfaceAttribPixelFormat,
			.flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = (int)encode_fourcc(config->profile) },
		},
		{
			.type = VASurfaceAttribMemoryType,
			.flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
					      VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME |
					      VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2 },
		},
		{
			.type = VASurfaceAttribMinWidth,
			.flags = VA_SURFACE_ATTRIB_GETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = AVE_MIN_WIDTH },
		},
		{
			.type = VASurfaceAttribMinHeight,
			.flags = VA_SURFACE_ATTRIB_GETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = AVE_MIN_HEIGHT },
		},
		{
			.type = VASurfaceAttribMaxWidth,
			.flags = VA_SURFACE_ATTRIB_GETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = AVE_MAX_WIDTH },
		},
		{
			.type = VASurfaceAttribMaxHeight,
			.flags = VA_SURFACE_ATTRIB_GETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = AVE_MAX_HEIGHT },
		},
		{
			.type = VASurfaceAttribUsageHint,
			.flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE,
			.value = { .type = VAGenericValueTypeInteger,
				   .value.i = VA_SURFACE_ATTRIB_USAGE_HINT_GENERIC |
					      VA_SURFACE_ATTRIB_USAGE_HINT_ENCODER },
		},
		{
			.type = VASurfaceAttribExternalBufferDescriptor,
			.flags = VA_SURFACE_ATTRIB_SETTABLE,
			.value = { .type = VAGenericValueTypePointer, .value.p = NULL },
		},
	};
	unsigned int required = sizeof(values) / sizeof(values[0]);

	if (!list) {
		*count = required;
		return VA_STATUS_SUCCESS;
	}
	if (*count < required) {
		*count = required;
		return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
	}
	memcpy(list, values, sizeof(values));
	*count = required;
	return VA_STATUS_SUCCESS;
}

/* --- device discovery --- */

void v4l2r_encode_probe(struct v4l2r_driver *drv)
{
	const char *mode = getenv("LIBVA_V4L2_ENCODE");
	int status;

	memset(&drv->ave, 0, sizeof(drv->ave));
	if (mode && (!strcmp(mode, "0") || !strcmp(mode, "off")))
		return;

	status = ave_probe(&drv->ave, "/sys/class/video4linux", "/dev/video",
			   AVE_PROBE_LIMIT, getenv("LIBVA_V4L2_ENCODE_DEVICE"));
	if (status < 0)
		return;
	v4l2r_diag(NULL, V4L2R_DIAG_LEVEL_INFO, V4L2R_DIAG_INFO, "encoder-detect", 0,
		   "detected apple-ave encoder %s (%s%s%s, %s%s%s)",
		   drv->ave.video_path,
		   (drv->ave.coded & AVE_CODED_H264) ? "H.264" : "",
		   (drv->ave.coded & AVE_CODED_H264) &&
		   (drv->ave.coded & AVE_CODED_HEVC) ? " " : "",
		   (drv->ave.coded & AVE_CODED_HEVC) ? "HEVC" : "",
		   (drv->ave.raw & AVE_RAW_NV12) ? "NV12" : "",
		   (drv->ave.raw & AVE_RAW_NV12) &&
		   (drv->ave.raw & AVE_RAW_P010) ? " " : "",
		   (drv->ave.raw & AVE_RAW_P010) ? "P010" : "");
}

/* --- status mapping --- */

static VAStatus status_from_errno(int status)
{
	/* V4L2 helpers return negative errno values; VA clients expect VAStatus. */
	switch (-status) {
	case ENOMEM:
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	case ENOSPC:
		return VA_STATUS_ERROR_NOT_ENOUGH_BUFFER;
	case ETIMEDOUT:
		return VA_STATUS_ERROR_HW_BUSY;
	case ENOTSUP:
#if EOPNOTSUPP != ENOTSUP
	case EOPNOTSUPP:
#endif
		return VA_STATUS_ERROR_UNIMPLEMENTED;
	case EINVAL:
		return VA_STATUS_ERROR_INVALID_PARAMETER;
	default:
		return VA_STATUS_ERROR_ENCODING_ERROR;
	}
}

/* --- locked lookups (drv->mutex is never held across a wait) --- */

static struct v4l2r_buffer *find_buffer(struct v4l2r_driver *drv, VABufferID id)
{
	return V4L2R_BUFFER_GET(drv, id);
}

static struct v4l2r_surface *find_surface(struct v4l2r_driver *drv,
					  VASurfaceID id)
{
	return V4L2R_SURFACE_GET(drv, id);
}

static struct v4l2r_context *find_encode_context(struct v4l2r_driver *drv,
						 VAContextID id)
{
	struct v4l2r_context *ctx = V4L2R_CONTEXT_GET(drv, id);

	return ctx && ctx->enc ? ctx : NULL;
}

static struct v4l2r_buffer *find_coded(struct v4l2r_driver *drv, VABufferID id)
{
	struct v4l2r_buffer *buffer = find_buffer(drv, id);

	return buffer && buffer->coded ? buffer : NULL;
}

/* --- contexts --- */

VAStatus v4l2r_encode_context_create(struct v4l2r_context *ctx,
				     const struct v4l2r_config *config,
				     int width, int height,
				     const VASurfaceID *targets, int count)
{
	struct v4l2r_driver *drv = ctx->drv;
	struct v4l2r_encode *enc;

	if (width < (int)AVE_MIN_WIDTH || height < (int)AVE_MIN_HEIGHT ||
	    width > (int)AVE_MAX_WIDTH || height > (int)AVE_MAX_HEIGHT) {
		v4l2r_diag(ctx, V4L2R_DIAG_LEVEL_DEBUG, V4L2R_DIAG_UNSUPPORTED,
			   "encode-create", 0,
			   "%dx%d is outside the apple-ave range", width, height);
		return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
	}

	/* Reconstructed-picture surfaces must match the context, as with the
	 * standalone encoder driver; AVE keeps its references internally. */
	for (int i = 0; i < count; i++) {
		struct v4l2r_surface *surface = find_surface(drv, targets[i]);

		if (!surface || surface->width != (unsigned int)width ||
		    surface->height != (unsigned int)height ||
		    v4l2r_encode_surface_fourcc(surface) !=
		    encode_fourcc(config->profile))
			return VA_STATUS_ERROR_INVALID_SURFACE;
	}

	enc = calloc(1, sizeof(*enc));
	if (!enc)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	enc->ctx = ctx;
	enc->profile = config->profile;
	enc->rate_control = config->rate_control;
	enc->hevc = config->profile == VAProfileHEVCMain ||
		    config->profile == VAProfileHEVCMain10;
	enc->main10 = config->profile == VAProfileHEVCMain10;
	enc->width = (unsigned int)width;
	enc->height = (unsigned int)height;
	enc->target = VA_INVALID_ID;
	enc->delivery_sequence = 1;
	ctx->enc = enc;

	v4l2r_diag(ctx, V4L2R_DIAG_LEVEL_INFO, V4L2R_DIAG_INFO, "create-context", 0,
		   "encoding %s %dx%d via %s", enc->hevc ? "HEVC" : "H.264",
		   width, height, drv->ave.video_path);
	return VA_STATUS_SUCCESS;
}

void v4l2r_encode_context_destroy(struct v4l2r_context *ctx)
{
	struct v4l2r_driver *drv = ctx->drv;
	struct v4l2r_encode *enc = ctx->enc;
	struct v4l2r_surface *surface;
	struct v4l2r_buffer *buffer;
	unsigned int iter;

	if (!enc)
		return;

	ave_encoder_close(enc->encoder);
	enc->encoder = NULL;

	/* Nothing more will arrive: release inputs and idle coded buffers. */
	pthread_mutex_lock(&drv->mutex);
	iter = 0;
	while ((surface = v4l2r_handles_next(&drv->surfaces, &iter, NULL))) {
		if (surface->encode_context == ctx->id) {
			surface->encode_pending = false;
			surface->encode_context = VA_INVALID_ID;
			surface->encode_coded = VA_INVALID_ID;
		}
	}
	iter = 0;
	while ((buffer = v4l2r_handles_next(&drv->buffers, &iter, NULL))) {
		if (buffer->coded && buffer->context_id == ctx->id)
			buffer->coded->tag = 0;
	}
	pthread_mutex_unlock(&drv->mutex);

	free(enc);
	ctx->enc = NULL;
}

/* --- surfaces as encode inputs --- */

uint32_t v4l2r_encode_surface_fourcc(const struct v4l2r_surface *surface)
{
	if (surface->import)
		return surface->import->fourcc;
	if (surface->fourcc)
		return surface->fourcc;
	return (surface->rt_format & VA_RT_FORMAT_YUV420) ? VA_FOURCC_NV12 :
							    VA_FOURCC_P010;
}

static int wait_dmabuf(int fd, short events)
{
	struct pollfd fence = { .fd = fd, .events = events };

	/* POLLIN on a DMA-BUF waits for pending writers (e.g. GPU fills). */
	for (unsigned int attempts = 0; attempts < 50; attempts++) {
		int result;

		do {
			result = poll(&fence, 1, 100);
		} while (result < 0 && errno == EINTR);
		if (result < 0)
			return -errno;
		if (result == 0)
			continue;
		if (fence.revents & (POLLERR | POLLHUP | POLLNVAL))
			return -EIO;
		if (fence.revents & events)
			return 0;
	}
	return -ETIMEDOUT;
}

static int import_cpu_sync(struct v4l2r_surface_import *import, uint64_t flags)
{
	struct dma_buf_sync sync = { .flags = flags };
	int result;

	/*
	 * DMA_BUF_IOCTL_SYNC provides cache coherency, but does not wait for a
	 * GPU writer attached to the reservation; wait for that first.
	 */
	if (!(flags & DMA_BUF_SYNC_END)) {
		result = wait_dmabuf(import->fd, POLLIN);
		if (result < 0)
			return result;
	}
	do {
		result = ioctl(import->fd, DMA_BUF_IOCTL_SYNC, &sync);
	} while (result < 0 && errno == EINTR);
	return result < 0 ? -errno : 0;
}

void v4l2r_surface_free_import(struct v4l2r_surface *surface)
{
	struct v4l2r_surface_import *import = surface->import;

	if (!import)
		return;
	if (import->map)
		munmap(import->map, import->map_size);
	if (import->fd >= 0)
		close(import->fd);
	free(import);
	surface->import = NULL;
}

VAStatus v4l2r_encode_import_surface(struct v4l2r_driver *drv,
				     struct v4l2r_surface *surface,
				     uint32_t memory_type, const void *descriptor)
{
	struct v4l2r_surface_import *import;
	struct ave_import_layout layout;
	uint32_t fourcc = v4l2r_encode_surface_fourcc(surface);
	off_t end;
	size_t size;
	int status;

	(void)drv;
	status = memory_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2 ?
		 ave_import_layout_from_prime2(descriptor, surface->width,
					       surface->height, fourcc, &layout) :
		 ave_import_layout_from_external(descriptor, surface->width,
						 surface->height, fourcc, &layout);
	if (status == -ENOTSUP)
		return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
	if (status < 0)
		return VA_STATUS_ERROR_INVALID_PARAMETER;

	end = lseek(layout.fd, 0, SEEK_END);
	if (end > 0) {
		if ((uint64_t)end < layout.required_size)
			return VA_STATUS_ERROR_INVALID_PARAMETER;
		size = (size_t)end;
	} else {
		size = layout.object_size;
	}
	if (size < layout.required_size)
		return VA_STATUS_ERROR_INVALID_PARAMETER;

	import = calloc(1, sizeof(*import));
	if (!import)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	/* The caller may close its descriptor; keep our own reference. */
	import->fd = fcntl(layout.fd, F_DUPFD_CLOEXEC, 0);
	if (import->fd < 0) {
		free(import);
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	}
	import->map = mmap(NULL, size, PROT_READ, MAP_SHARED, import->fd, 0);
	if (import->map == MAP_FAILED) {
		close(import->fd);
		free(import);
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	}
	import->map_size = size;
	import->y_offset = layout.y_offset;
	import->uv_offset = layout.uv_offset;
	import->stride = layout.stride;
	import->fourcc = fourcc;
	surface->import = import;
	return VA_STATUS_SUCCESS;
}

/* Pixels of an encode input, valid until input_release(). */
struct encode_input {
	struct ave_frame frame;
	struct v4l2r_surface_import *import;	/* needs a CPU sync end */
};

static int input_acquire(struct v4l2r_driver *drv, struct v4l2r_encode *enc,
			 struct v4l2r_surface *surface, struct encode_input *in)
{
	uint32_t fourcc = encode_fourcc(enc->profile);
	struct v4l2r_frame_view view;
	VAStatus status;

	memset(in, 0, sizeof(*in));

	if (surface->import) {
		struct v4l2r_surface_import *import = surface->import;
		int result;

		if (import->fourcc != fourcc)
			return -EINVAL;
		result = import_cpu_sync(import, DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ);
		if (result < 0)
			return result;
		in->import = import;
		in->frame = (struct ave_frame) {
			.y = (const uint8_t *)import->map + import->y_offset,
			.y_size = import->map_size - import->y_offset,
			.uv = (const uint8_t *)import->map + import->uv_offset,
			.uv_size = import->map_size - import->uv_offset,
			.stride = import->stride,
		};
		return 0;
	}

	/* A decoded surface must have finished decoding (and conversion). */
	status = v4l2r_surface_ready(surface);
	if (status != VA_STATUS_SUCCESS)
		return -EIO;
	status = v4l2r_surface_view(drv, surface, true, &view);
	if (status != VA_STATUS_SUCCESS)
		return status == VA_STATUS_ERROR_ALLOCATION_FAILED ? -ENOMEM : -EIO;
	if (!view.info || !view.info->linear || view.info->va_fourcc != fourcc ||
	    view.width < enc->encode_width || view.height < enc->encode_height ||
	    !view.map[0])
		return -EINVAL;

	in->frame.y = view.map[0];
	in->frame.y_size = view.plane_size[0];
	in->frame.stride = view.pitch;
	if (view.nb_planes == 1) {
		size_t offset = (size_t)view.pitch * view.height;

		if (offset >= view.plane_size[0])
			return -EINVAL;
		in->frame.uv = (const uint8_t *)view.map[0] + offset;
		in->frame.uv_size = view.plane_size[0] - offset;
	} else if (view.nb_planes == 2 && view.map[1]) {
		in->frame.uv = view.map[1];
		in->frame.uv_size = view.plane_size[1];
	} else {
		return -EINVAL;
	}
	return 0;
}

static int input_release(struct encode_input *in)
{
	if (!in->import)
		return 0;
	return import_cpu_sync(in->import, DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ);
}

/* --- coded buffers --- */

VAStatus v4l2r_encode_buffer_init(struct v4l2r_buffer *buffer)
{
	if (buffer->type != VAEncCodedBufferType)
		return VA_STATUS_SUCCESS;
	buffer->coded = calloc(1, sizeof(*buffer->coded));
	if (!buffer->coded)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	buffer->coded->segment.buf = buffer->data;
	buffer->coded->source = VA_INVALID_ID;
	memset(buffer->data, 0, v4l2r_buffer_bytes(buffer));
	return VA_STATUS_SUCCESS;
}

static int queue_coded(struct v4l2r_encode *enc, struct v4l2r_buffer *buffer,
		       VABufferID id, uint64_t tag)
{
	size_t tail;

	if (!tag)
		return -EINVAL;
	if (enc->queue_count >= AVE_QUEUE_LENGTH)
		return -ENOSPC;
	for (size_t offset = 0; offset < enc->queue_count; offset++) {
		if (enc->queue[(enc->queue_head + offset) % AVE_QUEUE_LENGTH] == id)
			return -EALREADY;
	}

	tail = (enc->queue_head + enc->queue_count) % AVE_QUEUE_LENGTH;
	enc->queue[tail] = id;
	enc->queue_count++;
	buffer->coded->sequence = enc->sequence;
	buffer->coded->tag = tag;
	return 0;
}

static void remove_queue_entry(struct v4l2r_encode *enc, size_t offset)
{
	for (size_t index = offset; index + 1 < enc->queue_count; index++) {
		size_t destination = (enc->queue_head + index) % AVE_QUEUE_LENGTH;
		size_t source = (enc->queue_head + index + 1) % AVE_QUEUE_LENGTH;

		enc->queue[destination] = enc->queue[source];
	}
	enc->queue_count--;
}

static void rollback_coded(struct v4l2r_encode *enc, struct v4l2r_buffer *buffer,
			   VABufferID id)
{
	size_t tail;

	if (!enc->queue_count)
		return;
	tail = (enc->queue_head + enc->queue_count - 1) % AVE_QUEUE_LENGTH;
	if (enc->queue[tail] == id) {
		enc->queue_count--;
		buffer->coded->sequence = 0;
		buffer->coded->tag = 0;
	}
}

static void finish_input(struct v4l2r_driver *drv, struct v4l2r_buffer *buffer,
			 VABufferID id)
{
	struct v4l2r_surface *surface = find_surface(drv, buffer->coded->source);

	if (surface && surface->encode_coded == id &&
	    buffer->coded->ready && buffer->coded->input_done)
		surface->encode_pending = false;
}

static int store_packet(const struct ave_packet *packet, void *opaque)
{
	struct v4l2r_encode *enc = opaque;
	struct v4l2r_driver *drv = enc->ctx->drv;
	struct v4l2r_buffer *buffer = NULL;
	struct v4l2r_coded *coded;
	VABufferID id = VA_INVALID_ID;
	size_t capacity;
	size_t offset;

	if (!packet || !packet->tag || !enc->queue_count)
		return -EINVAL;

	for (offset = 0; offset < enc->queue_count; offset++) {
		struct v4l2r_buffer *candidate;

		id = enc->queue[(enc->queue_head + offset) % AVE_QUEUE_LENGTH];
		candidate = find_coded(drv, id);
		if (candidate && candidate->coded->tag == packet->tag) {
			buffer = candidate;
			break;
		}
	}
	if (!buffer)
		return -ENOENT;
	coded = buffer->coded;

	capacity = v4l2r_buffer_bytes(buffer);
	if (packet->size > capacity - coded->size) {
		coded->segment.status |= VA_CODED_BUF_STATUS_FRAME_SIZE_OVERFLOW;
		return -ENOSPC;
	}

	memcpy((uint8_t *)buffer->data + coded->size, packet->data, packet->size);
	coded->size += packet->size;
	coded->packets++;
	coded->segment.size = (uint32_t)coded->size;
	coded->segment.buf = buffer->data;
	coded->segment.bit_offset = 0;
	coded->segment.next = NULL;

	if (packet->flags & V4L2_BUF_FLAG_ERROR)
		coded->segment.status |= VA_CODED_BUF_STATUS_BAD_BITSTREAM;

	if (!(packet->flags & (V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME |
			       V4L2_BUF_FLAG_BFRAME | V4L2_BUF_FLAG_ERROR)))
		return 0;	/* more packets of this frame follow */

	/* A requested key frame must come back flagged as one. */
	if (coded->keyframe_expected &&
	    !(packet->flags & (V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_ERROR)) &&
	    enc->keyframe_misses++ == 0)
		v4l2r_diag(enc->ctx, V4L2R_DIAG_LEVEL_WARNING, V4L2R_DIAG_DEVICE,
			   "encode-keyframe", 0,
			   "requested key frame was not coded as a key frame");

	coded->ready = true;
	remove_queue_entry(enc, offset);
	finish_input(drv, buffer, id);
	return 0;
}

static int store_output_done(uint64_t tag, void *opaque)
{
	struct v4l2r_encode *enc = opaque;
	struct v4l2r_driver *drv = enc->ctx->drv;
	struct v4l2r_buffer *buffer;
	struct v4l2r_buffer *found = NULL;
	VABufferID found_id = VA_INVALID_ID;
	unsigned int iter = 0;
	uint32_t id;

	pthread_mutex_lock(&drv->mutex);
	while ((buffer = v4l2r_handles_next(&drv->buffers, &iter, &id))) {
		if (buffer->coded && buffer->context_id == enc->ctx->id &&
		    buffer->coded->tag == tag) {
			found = buffer;
			found_id = id;
			break;
		}
	}
	pthread_mutex_unlock(&drv->mutex);
	if (!found)
		return -ENOENT;

	found->coded->input_done = true;
	finish_input(drv, found, found_id);
	return 0;
}

/* --- picture assembly --- */

VAStatus v4l2r_encode_begin_picture(struct v4l2r_context *ctx,
				    struct v4l2r_surface *surface)
{
	struct v4l2r_encode *enc = ctx->enc;

	if (v4l2r_encode_surface_fourcc(surface) != encode_fourcc(enc->profile) ||
	    surface->width > enc->width || surface->height > enc->height)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	/* Reusing an input whose previous encode is still in flight. */
	if (surface->encode_pending) {
		VAStatus status = v4l2r_encode_sync_surface(ctx->drv, surface);

		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	memset(&enc->pic, 0, sizeof(enc->pic));
	enc->target = surface->id;
	ctx->in_picture = true;
	return VA_STATUS_SUCCESS;
}

static void picture_fail(struct encode_picture *pic, int error)
{
	if (!pic->error)
		pic->error = error;
}

static void render_h264(struct encode_picture *pic,
			const struct v4l2r_buffer *buffer)
{
	switch (buffer->type) {
	case VAEncSequenceParameterBufferType:
		if (pic->have_sequence || buffer->nb_elements != 1 ||
		    buffer->element_size < sizeof(pic->sequence.h264))
			return picture_fail(pic, -EINVAL);
		memcpy(&pic->sequence.h264, buffer->data, sizeof(pic->sequence.h264));
		pic->have_sequence = true;
		return;
	case VAEncPictureParameterBufferType:
		if (pic->have_picture || buffer->nb_elements != 1 ||
		    buffer->element_size < sizeof(pic->picture.h264))
			return picture_fail(pic, -EINVAL);
		memcpy(&pic->picture.h264, buffer->data, sizeof(pic->picture.h264));
		pic->have_picture = true;
		return;
	case VAEncSliceParameterBufferType:
		if (buffer->element_size < sizeof(VAEncSliceParameterBufferH264))
			return picture_fail(pic, -EINVAL);
		for (unsigned int i = 0; i < buffer->nb_elements; i++) {
			VAEncSliceParameterBufferH264 slice;
			unsigned int slice_type;

			memcpy(&slice, (const uint8_t *)buffer->data +
			       (size_t)i * buffer->element_size, sizeof(slice));
			slice_type = slice.slice_type % 5u;
			/* AVE runs B_FRAMES=0; SP/SI do not exist in AVE output. */
			if (slice_type != 0 && slice_type != 2)
				return picture_fail(pic, -ENOTSUP);
			if (slice.num_macroblocks == 0 || slice.macroblock_address != 0)
				return picture_fail(pic, -ENOTSUP);
			pic->intra = slice_type == 2;
			pic->slice_macroblocks = slice.num_macroblocks;
			pic->slices++;
		}
		return;
	default:
		return picture_fail(pic, -ENOTSUP);
	}
}

static void render_hevc(struct encode_picture *pic,
			const struct v4l2r_buffer *buffer)
{
	VAEncSliceParameterBufferHEVC slice;

	switch (buffer->type) {
	case VAEncSequenceParameterBufferType:
		if (pic->have_sequence || buffer->nb_elements != 1 ||
		    buffer->element_size < sizeof(pic->sequence.hevc))
			return picture_fail(pic, -EINVAL);
		memcpy(&pic->sequence.hevc, buffer->data, sizeof(pic->sequence.hevc));
		pic->have_sequence = true;
		return;
	case VAEncPictureParameterBufferType:
		if (pic->have_picture || buffer->nb_elements != 1 ||
		    buffer->element_size < sizeof(pic->picture.hevc))
			return picture_fail(pic, -EINVAL);
		memcpy(&pic->picture.hevc, buffer->data, sizeof(pic->picture.hevc));
		pic->have_picture = true;
		return;
	case VAEncSliceParameterBufferType:
		if (pic->slices || buffer->nb_elements != 1 ||
		    buffer->element_size < sizeof(slice))
			return picture_fail(pic, pic->slices ? -ENOTSUP : -EINVAL);
		memcpy(&slice, buffer->data, sizeof(slice));
		/* HEVC slice_type: 0 B, 1 P, 2 I. */
		if (slice.slice_segment_address != 0 || slice.num_ctu_in_slice == 0 ||
		    (slice.slice_type != 1 && slice.slice_type != 2))
			return picture_fail(pic, -ENOTSUP);
		pic->num_ctu_in_slice = slice.num_ctu_in_slice;
		pic->intra = slice.slice_type == 2;
		pic->slices = 1;
		return;
	default:
		return picture_fail(pic, -ENOTSUP);
	}
}

VAStatus v4l2r_encode_render_buffer(struct v4l2r_context *ctx,
				    struct v4l2r_buffer *buffer)
{
	struct encode_picture *pic = &ctx->enc->pic;

	if (pic->error)
		return VA_STATUS_SUCCESS;
	if (!buffer->data || !v4l2r_buffer_bytes(buffer)) {
		picture_fail(pic, -EINVAL);
		return VA_STATUS_SUCCESS;
	}

	switch (buffer->type) {
	case VAEncPackedHeaderParameterBufferType:
	case VAEncPackedHeaderDataBufferType:
		/* Not advertised: AVE writes its own headers (see above). */
		picture_fail(pic, -ENOTSUP);
		break;
	case VAEncMiscParameterBufferType: {
		const VAEncMiscParameterBuffer *header = buffer->data;
		size_t size = v4l2r_buffer_bytes(buffer);
		int status;

		if (size < sizeof(*header)) {
			picture_fail(pic, -EINVAL);
			break;
		}
		status = ave_parse_misc(header->type, header->data,
					size - sizeof(*header), &pic->rate);
		if (status < 0)
			picture_fail(pic, status);
		break;
	}
	default:
		if (ctx->enc->hevc)
			render_hevc(pic, buffer);
		else
			render_h264(pic, buffer);
		break;
	}

	/* Like the standalone encoder driver, report problems at EndPicture. */
	return VA_STATUS_SUCCESS;
}

/* --- stream setup --- */

static int h264_dimensions(const VAEncSequenceParameterBufferH264 *sequence,
			   uint32_t *width, uint32_t *height)
{
	uint32_t crop_width = 0;
	uint32_t crop_height = 0;

	if (sequence->picture_width_in_mbs == 0 ||
	    sequence->picture_height_in_mbs == 0)
		return -EINVAL;
	if (sequence->seq_fields.bits.chroma_format_idc != 1 ||
	    !sequence->seq_fields.bits.frame_mbs_only_flag ||
	    sequence->bit_depth_luma_minus8 != 0 ||
	    sequence->bit_depth_chroma_minus8 != 0)
		return -ENOTSUP;
	if ((uint32_t)sequence->picture_width_in_mbs *
	    sequence->picture_height_in_mbs > AVE_H264_MAX_MACROBLOCKS)
		return -EINVAL;

	if (sequence->frame_cropping_flag) {
		uint64_t horizontal = sequence->frame_crop_right_offset;
		uint64_t vertical = sequence->frame_crop_bottom_offset;

		/* AVE encodes from the top-left corner; only right/bottom crop. */
		if (sequence->frame_crop_left_offset ||
		    sequence->frame_crop_top_offset)
			return -ENOTSUP;
		horizontal *= 2u;
		vertical *= 2u;
		if (horizontal >= 16u || vertical >= 16u)
			return -ENOTSUP;
		crop_width = (uint32_t)horizontal;
		crop_height = (uint32_t)vertical;
	}

	*width = (uint32_t)sequence->picture_width_in_mbs * 16u - crop_width;
	*height = (uint32_t)sequence->picture_height_in_mbs * 16u - crop_height;
	if (*width < AVE_MIN_WIDTH || *height < AVE_MIN_HEIGHT ||
	    *width > AVE_MAX_WIDTH || *height > AVE_MAX_HEIGHT ||
	    (*width & 1u) || (*height & 1u))
		return -ERANGE;
	return 0;
}

static uint32_t h264_profile_to_v4l2(VAProfile profile)
{
	switch (profile) {
	case VAProfileH264ConstrainedBaseline:
		return V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE;
	case VAProfileH264Main:
		return V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
	case VAProfileH264High:
		return V4L2_MPEG_VIDEO_H264_PROFILE_HIGH;
	default:
		return UINT32_MAX;
	}
}

static uint32_t h264_level_to_v4l2(uint8_t level_idc)
{
	static const struct { uint8_t idc; uint32_t control; } levels[] = {
		{ 9, V4L2_MPEG_VIDEO_H264_LEVEL_1B },
		{ 10, V4L2_MPEG_VIDEO_H264_LEVEL_1_0 },
		{ 11, V4L2_MPEG_VIDEO_H264_LEVEL_1_1 },
		{ 12, V4L2_MPEG_VIDEO_H264_LEVEL_1_2 },
		{ 13, V4L2_MPEG_VIDEO_H264_LEVEL_1_3 },
		{ 20, V4L2_MPEG_VIDEO_H264_LEVEL_2_0 },
		{ 21, V4L2_MPEG_VIDEO_H264_LEVEL_2_1 },
		{ 22, V4L2_MPEG_VIDEO_H264_LEVEL_2_2 },
		{ 30, V4L2_MPEG_VIDEO_H264_LEVEL_3_0 },
		{ 31, V4L2_MPEG_VIDEO_H264_LEVEL_3_1 },
		{ 32, V4L2_MPEG_VIDEO_H264_LEVEL_3_2 },
		{ 40, V4L2_MPEG_VIDEO_H264_LEVEL_4_0 },
		{ 41, V4L2_MPEG_VIDEO_H264_LEVEL_4_1 },
		{ 42, V4L2_MPEG_VIDEO_H264_LEVEL_4_2 },
		{ 50, V4L2_MPEG_VIDEO_H264_LEVEL_5_0 },
		{ 51, V4L2_MPEG_VIDEO_H264_LEVEL_5_1 },
		{ 52, V4L2_MPEG_VIDEO_H264_LEVEL_5_2 },
	};

	for (size_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
		if (levels[i].idc == level_idc)
			return levels[i].control;
	}
	return UINT32_MAX;
}

static uint32_t hevc_level_to_v4l2(uint8_t level_idc)
{
	static const struct { uint8_t idc; uint32_t control; } levels[] = {
		{ 30, V4L2_MPEG_VIDEO_HEVC_LEVEL_1 },
		{ 60, V4L2_MPEG_VIDEO_HEVC_LEVEL_2 },
		{ 63, V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1 },
		{ 90, V4L2_MPEG_VIDEO_HEVC_LEVEL_3 },
		{ 93, V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1 },
		{ 120, V4L2_MPEG_VIDEO_HEVC_LEVEL_4 },
		{ 123, V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1 },
		{ 150, V4L2_MPEG_VIDEO_HEVC_LEVEL_5 },
		{ 153, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1 },
		{ 156, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2 },
		{ 180, V4L2_MPEG_VIDEO_HEVC_LEVEL_6 },
		{ 183, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1 },
		{ 186, V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2 },
	};

	for (size_t i = 0; i < sizeof(levels) / sizeof(levels[0]); i++) {
		if (levels[i].idc == level_idc)
			return levels[i].control;
	}
	return UINT32_MAX;
}

static uint32_t frame_rate_or_default(const struct ave_rate_request *rate,
				      uint32_t fallback)
{
	if (rate->frame_rate_seen)
		return rate->frames_per_second;
	if (fallback > 0 && fallback <= 240)
		return fallback;
	return AVE_DEFAULT_FPS;
}

static int open_encoder(struct v4l2r_encode *enc,
			struct ave_encoder_config *config,
			const struct ave_rate_settings *rate,
			uint32_t encode_width, uint32_t encode_height)
{
	char operation[64];
	int status;

	config->device = enc->ctx->drv->ave.video_path;
	config->width = encode_width;
	config->height = encode_height;
	config->bitrate_mode = rate->v4l2_mode;
	config->bitrate = rate->bitrate;
	config->bitrate_peak = rate->peak;
	config->qp_range = rate->qp_range;
	config->min_qp = rate->min_qp;
	config->max_qp = rate->max_qp;
	config->output_buffers = AVE_OUTPUT_BUFFERS;
	config->capture_buffers = AVE_CAPTURE_BUFFERS;
	config->output_done = store_output_done;
	config->output_done_opaque = enc;

	status = ave_encoder_compressed_size(encode_width, encode_height,
					     &config->capture_buffer_size);
	if (status < 0)
		return status;

	status = ave_encoder_open(config, &enc->encoder, operation,
				  sizeof(operation));
	if (status < 0) {
		v4l2r_diag(enc->ctx, V4L2R_DIAG_LEVEL_ERROR,
			   v4l2r_diag_errno_category(status), "encode-open",
			   status, "%s encoder setup failed at %s",
			   enc->hevc ? "HEVC" : "H.264",
			   operation[0] ? operation : "parameters");
		return status;
	}
	enc->encode_width = encode_width;
	enc->encode_height = encode_height;
	enc->frames_per_second = config->frames_per_second;
	enc->bitrate = rate->bitrate;
	return 0;
}

static int open_h264(struct v4l2r_encode *enc)
{
	const struct encode_picture *pic = &enc->pic;
	const VAEncSequenceParameterBufferH264 *sequence = &pic->sequence.h264;
	struct ave_encoder_config config = {0};
	struct ave_rate_settings rate;
	uint32_t profile = h264_profile_to_v4l2(enc->profile);
	uint32_t level;
	uint32_t vui_fps = 0;
	uint32_t width;
	uint32_t height;
	int status;

	/* The first picture of a stream must carry the sequence parameters. */
	if (!pic->have_sequence || profile == UINT32_MAX)
		return -EINVAL;
	if (sequence->ip_period > 1)
		return -ENOTSUP;
	/*
	 * apple-ave treats H264_LEVEL as a minimum and raises it to fit the
	 * size, rate and bit rate (ave_pick_level), up to 5.2.
	 */
	level = h264_level_to_v4l2(sequence->level_idc);
	if (level == UINT32_MAX)
		return -ENOTSUP;

	status = h264_dimensions(sequence, &width, &height);
	if (status < 0)
		return status;
	if ((width + 15u) / 16u * 16u != enc->width ||
	    (height + 15u) / 16u * 16u != enc->height)
		return -EINVAL;
	if (pic->slice_macroblocks != (uint32_t)sequence->picture_width_in_mbs *
	    sequence->picture_height_in_mbs)
		return -ENOTSUP;

	status = ave_rate_settings_from_va(enc->rate_control, &pic->rate,
					   sequence->bits_per_second, &rate);
	if (status < 0)
		return status;
	if (sequence->vui_fields.bits.timing_info_present_flag &&
	    sequence->num_units_in_tick > 0) {
		uint64_t derived = sequence->time_scale /
				   ((uint64_t)sequence->num_units_in_tick * 2u);

		if (derived > 0 && derived <= 240)
			vui_fps = (uint32_t)derived;
	}

	config.coded_format = V4L2_PIX_FMT_H264;
	config.raw_format = V4L2_PIX_FMT_NV12;
	config.frames_per_second = frame_rate_or_default(&pic->rate, vui_fps);
	config.gop_size = ave_gop_from_va(sequence->intra_idr_period,
					  sequence->intra_period);
	config.h264_profile = profile;
	config.h264_level = level;
	return open_encoder(enc, &config, &rate, width, height);
}

static int open_hevc(struct v4l2r_encode *enc, const struct v4l2r_surface *input)
{
	const struct encode_picture *pic = &enc->pic;
	const VAEncSequenceParameterBufferHEVC *sequence = &pic->sequence.hevc;
	const VAEncPictureParameterBufferHEVC *picture = &pic->picture.hevc;
	bool main10 = enc->main10;
	struct ave_encoder_config config = {0};
	struct ave_rate_settings rate;
	uint32_t level;
	uint32_t vui_fps = 0;
	uint32_t ctu;
	uint32_t ctu_count;
	uint32_t width;
	uint32_t height;
	int status;

	if (!pic->have_sequence || !pic->intra ||
	    !picture->pic_fields.bits.idr_pic_flag ||
	    sequence->general_profile_idc != (main10 ? 2u : 1u) ||
	    sequence->seq_fields.bits.chroma_format_idc != 1 ||
	    sequence->seq_fields.bits.bit_depth_luma_minus8 != (main10 ? 2u : 0u) ||
	    sequence->seq_fields.bits.bit_depth_chroma_minus8 != (main10 ? 2u : 0u) ||
	    sequence->seq_fields.bits.separate_colour_plane_flag ||
	    sequence->seq_fields.bits.pcm_enabled_flag ||
	    sequence->ip_period > 1 ||
	    /* apple-ave has no tier control: Main tier only. */
	    sequence->general_tier_flag ||
	    picture->num_tile_columns_minus1 ||
	    picture->num_tile_rows_minus1 ||
	    picture->pic_fields.bits.tiles_enabled_flag)
		return -ENOTSUP;
	level = hevc_level_to_v4l2(sequence->general_level_idc);
	/* The apple-ave HEVC_LEVEL menu stops at 5.1 (a minimum level). */
	if (level == UINT32_MAX || level > V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1)
		return -ENOTSUP;
	if (sequence->log2_min_luma_coding_block_size_minus3 > 3 ||
	    sequence->log2_diff_max_min_luma_coding_block_size > 3 ||
	    sequence->log2_min_luma_coding_block_size_minus3 +
	    sequence->log2_diff_max_min_luma_coding_block_size > 3)
		return -EINVAL;
	ctu = (1u << (sequence->log2_min_luma_coding_block_size_minus3 + 3)) <<
	      sequence->log2_diff_max_min_luma_coding_block_size;
	if (sequence->pic_width_in_luma_samples != enc->width ||
	    sequence->pic_height_in_luma_samples != enc->height)
		return -EINVAL;
	/*
	 * VA HEVC has no conformance window outside packed headers. Clients
	 * such as FFmpeg pad the coded size to a coding-tree-block boundary and
	 * upload the visible picture into a surface of the visible size.
	 * FFmpeg can advertise an 8-pixel minimum CB in the sequence while
	 * using its 16-pixel fallback surface alignment, so validate the
	 * padding against the CTU rather than the advertised minimum CB.
	 * Encode the input size and let apple-ave write the conformance window
	 * (ave_build_process crop flag). Otherwise the coded size is encoded.
	 */
	width = enc->width;
	height = enc->height;
	if (input->width <= enc->width && input->height <= enc->height &&
	    enc->width - input->width < ctu && enc->height - input->height < ctu) {
		width = input->width;
		height = input->height;
	}
	if ((width & 1u) || (height & 1u) || width < AVE_MIN_WIDTH ||
	    height < AVE_MIN_HEIGHT ||
	    (uint64_t)width * height > AVE_HEVC_MAX_SAMPLES)
		return -ERANGE;
	ctu_count = ((sequence->pic_width_in_luma_samples + ctu - 1) / ctu) *
		    ((sequence->pic_height_in_luma_samples + ctu - 1) / ctu);
	if (pic->num_ctu_in_slice != ctu_count)
		return -ENOTSUP;

	status = ave_rate_settings_from_va(enc->rate_control, &pic->rate,
					   sequence->bits_per_second, &rate);
	if (status < 0)
		return status;
	if (sequence->vui_num_units_in_tick && sequence->vui_time_scale)
		vui_fps = sequence->vui_time_scale / sequence->vui_num_units_in_tick;

	config.coded_format = V4L2_PIX_FMT_HEVC;
#if HAVE_V4L2_PIX_FMT_P010
	config.raw_format = main10 ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12;
#else
	if (main10)
		return -ENOTSUP;
	config.raw_format = V4L2_PIX_FMT_NV12;
#endif
	config.frames_per_second = frame_rate_or_default(&pic->rate, vui_fps);
	config.gop_size = ave_gop_from_va(sequence->intra_idr_period,
					  sequence->intra_period);
	config.hevc_profile = main10 ? V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10 :
				       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	config.hevc_level = level;
	return open_encoder(enc, &config, &rate, width, height);
}

/*
 * Rate changes after the session started: apple-ave applies a new average
 * bit rate at the next frame (ave_build_process). Mode, peak, frame rate
 * and QP range are read once at STREAMON and cannot change mid-stream.
 */
static int update_rate(struct v4l2r_encode *enc,
		       uint32_t sequence_bits_per_second)
{
	struct ave_rate_settings rate;
	int status;

	if (!enc->pic.rate.rate_seen && !sequence_bits_per_second)
		return 0;
	status = ave_rate_settings_from_va(enc->rate_control, &enc->pic.rate,
					   sequence_bits_per_second, &rate);
	if (status < 0)
		return status;
	if (rate.bitrate == enc->bitrate)
		return 0;
	status = ave_encoder_set_bitrate(enc->encoder, rate.bitrate);
	if (status < 0)
		return status;
	enc->bitrate = rate.bitrate;
	return 0;
}

static int check_picture(const struct v4l2r_encode *enc)
{
	const struct encode_picture *pic = &enc->pic;

	if (pic->error)
		return pic->error;
	if (enc->hevc) {
		if (!pic->have_picture || pic->slices != 1)
			return -EINVAL;
		if (pic->picture.hevc.pic_fields.bits.idr_pic_flag && !pic->intra)
			return -ENOTSUP;
		return 0;
	}
	/* VAConfigAttribEncMaxSlices is 1: one slice covering the picture. */
	if (!pic->have_picture || pic->slices != 1)
		return pic->slices > 1 ? -ENOTSUP : -EINVAL;
	return 0;
}

static VAStatus encode_picture(struct v4l2r_context *ctx)
{
	struct v4l2r_driver *drv = ctx->drv;
	struct v4l2r_encode *enc = ctx->enc;
	struct encode_picture *pic = &enc->pic;
	struct v4l2r_buffer *coded;
	struct v4l2r_surface *surface;
	struct encode_input input;
	VABufferID coded_id;
	bool keyframe_requested;
	size_t capacity;
	uint64_t tag;
	int status;

	status = check_picture(enc);
	if (status < 0)
		return status_from_errno(status);

	coded_id = enc->hevc ? pic->picture.hevc.coded_buf :
			       pic->picture.h264.coded_buf;
	coded = find_coded(drv, coded_id);
	surface = find_surface(drv, enc->target);
	if (!coded || coded->context_id != ctx->id)
		return VA_STATUS_ERROR_INVALID_BUFFER;
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	capacity = v4l2r_buffer_bytes(coded);
	if (capacity == 0 || capacity > UINT32_MAX)
		return VA_STATUS_ERROR_INVALID_BUFFER;

	if (!enc->encoder) {
		status = enc->hevc ? open_hevc(enc, surface) : open_h264(enc);
		if (status < 0)
			return status_from_errno(status);
	} else if (enc->hevc) {
		const VAEncSequenceParameterBufferHEVC *sequence =
			pic->have_sequence ? &pic->sequence.hevc : NULL;

		if (sequence &&
		    (sequence->pic_width_in_luma_samples != enc->width ||
		     sequence->pic_height_in_luma_samples != enc->height))
			return VA_STATUS_ERROR_INVALID_PARAMETER;
		status = update_rate(enc, sequence ? sequence->bits_per_second : 0);
		if (status < 0)
			return status_from_errno(status);
	} else {
		const VAEncSequenceParameterBufferH264 *sequence =
			pic->have_sequence ? &pic->sequence.h264 : NULL;

		if (sequence) {
			uint32_t width;
			uint32_t height;

			status = h264_dimensions(sequence, &width, &height);
			if (status < 0 || width != enc->encode_width ||
			    height != enc->encode_height)
				return VA_STATUS_ERROR_INVALID_PARAMETER;
		}
		status = update_rate(enc, sequence ? sequence->bits_per_second : 0);
		if (status < 0)
			return status_from_errno(status);
	}

	/* An IDR or any intra picture is produced as an AVE key frame. */
	keyframe_requested = enc->hevc ? pic->intra :
			     pic->picture.h264.pic_fields.bits.idr_pic_flag ||
			     pic->intra;
	if (enc->sequence > 0 && keyframe_requested) {
		/*
		 * FORCE_KEY_FRAME applies to the next frame apple-ave hands to
		 * the firmware. Make sure that is this frame, not an earlier
		 * one still waiting in the OUTPUT queue.
		 */
		status = ave_encoder_wait_output_idle(enc->encoder,
						      AVE_KEYFRAME_IDLE_TIMEOUT_MS,
						      store_packet, enc);
		if (status == 0)
			status = ave_encoder_force_keyframe(enc->encoder);
		if (status < 0)
			return status_from_errno(status);
	}

	if (!enc->frames_per_second)
		return VA_STATUS_ERROR_OPERATION_FAILED;
	if (surface->width < enc->encode_width ||
	    surface->height < enc->encode_height)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	status = input_acquire(drv, enc, surface, &input);
	if (status < 0) {
		v4l2r_diag(ctx, V4L2R_DIAG_LEVEL_ERROR, V4L2R_DIAG_CLIENT,
			   "encode-input", status,
			   "cannot read encode input surface 0x%08x",
			   surface->id);
		return status == -EINVAL ? VA_STATUS_ERROR_INVALID_SURFACE :
					   status_from_errno(status);
	}

	enc->sequence++;
	if (enc->sequence == 0)
		enc->sequence++;
	tag = 1u + (enc->sequence - 1u) * 1000000u / enc->frames_per_second;

	coded->coded->size = 0;
	coded->coded->ready = false;
	coded->coded->delivered = false;
	coded->coded->input_done = false;
	coded->coded->packets = 0;
	coded->coded->sequence = 0;
	coded->coded->tag = 0;
	coded->coded->keyframe_expected = keyframe_requested || enc->sequence == 1;
	coded->coded->source = surface->id;
	memset(&coded->coded->segment, 0, sizeof(coded->coded->segment));
	coded->coded->segment.buf = coded->data;

	status = queue_coded(enc, coded, coded_id, tag);
	if (status < 0) {
		input_release(&input);
		return status_from_errno(status);
	}

	surface->encode_pending = true;
	surface->encode_context = ctx->id;
	surface->encode_coded = coded_id;

	status = ave_encoder_submit(enc->encoder, &input.frame, tag,
				    store_packet, enc);
	{
		int sync_status = input_release(&input);

		if (sync_status < 0 && status == 0)
			status = sync_status;
	}
	if (status < 0) {
		rollback_coded(enc, coded, coded_id);
		surface->encode_pending = false;
		v4l2r_diag(ctx, V4L2R_DIAG_LEVEL_ERROR,
			   v4l2r_diag_errno_category(status), "encode-submit",
			   status, "frame submission failed at %s",
			   ave_encoder_last_operation(enc->encoder));
		return status_from_errno(status);
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_encode_end_picture(struct v4l2r_context *ctx)
{
	/* A rejected RenderPicture fails the whole picture, as for decoding. */
	VAStatus status = ctx->picture_status != VA_STATUS_SUCCESS ?
			  ctx->picture_status : encode_picture(ctx);

	memset(&ctx->enc->pic, 0, sizeof(ctx->enc->pic));
	ctx->enc->target = VA_INVALID_ID;
	ctx->in_picture = false;
	return status;
}

/* --- completion --- */

/* Pump the context's encoder until @done is true or the deadline passes. */
static VAStatus pump_until(struct v4l2r_encode *enc, const bool *done,
			   int timeout_ms)
{
	uint64_t deadline = v4l2r_now_ns() + (uint64_t)timeout_ms * 1000000u;

	while (!*done) {
		uint64_t now = v4l2r_now_ns();
		int64_t remaining_ms;
		int status;

		if (now >= deadline)
			break;
		remaining_ms = (int64_t)((deadline - now + 999999u) / 1000000u);
		status = ave_encoder_pump(enc->encoder,
					  remaining_ms > 1000 ? 1000 : (int)remaining_ms,
					  store_packet, enc, NULL);
		if (status == -ETIMEDOUT || status == -EAGAIN)
			continue;
		if (status < 0)
			return status_from_errno(status);
	}
	return *done ? VA_STATUS_SUCCESS : VA_STATUS_ERROR_HW_BUSY;
}

static VAStatus sync_surface_timeout(struct v4l2r_driver *drv,
				     struct v4l2r_surface *surface,
				     int timeout_ms)
{
	struct v4l2r_context *ctx;
	uint64_t deadline;

	if (!surface->encode_pending)
		return VA_STATUS_SUCCESS;
	if (!find_coded(drv, surface->encode_coded))
		return VA_STATUS_ERROR_INVALID_BUFFER;
	ctx = find_encode_context(drv, surface->encode_context);
	if (!ctx || !ctx->enc->encoder)
		return VA_STATUS_ERROR_INVALID_CONTEXT;

	/* encode_pending is cleared from the packet callbacks. */
	deadline = v4l2r_now_ns() + (uint64_t)timeout_ms * 1000000u;
	while (surface->encode_pending) {
		uint64_t now = v4l2r_now_ns();
		int64_t remaining_ms;
		int result;

		if (now >= deadline)
			return VA_STATUS_ERROR_HW_BUSY;
		remaining_ms = (int64_t)((deadline - now + 999999u) / 1000000u);
		result = ave_encoder_pump(ctx->enc->encoder,
					  remaining_ms > 1000 ? 1000 : (int)remaining_ms,
					  store_packet, ctx->enc, NULL);
		if (result == -ETIMEDOUT || result == -EAGAIN)
			continue;
		if (result < 0)
			return status_from_errno(result);
	}
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_encode_sync_surface(struct v4l2r_driver *drv,
				   struct v4l2r_surface *surface)
{
	return sync_surface_timeout(drv, surface, AVE_SYNC_TIMEOUT_MS);
}

void v4l2r_encode_query_surface(struct v4l2r_driver *drv,
				struct v4l2r_surface *surface,
				VASurfaceStatus *status)
{
	struct v4l2r_context *ctx = find_encode_context(drv, surface->encode_context);

	/* One non-blocking sweep so polling clients make progress. */
	if (ctx && ctx->enc->encoder)
		ave_encoder_pump(ctx->enc->encoder, 0, store_packet, ctx->enc, NULL);
	*status = surface->encode_pending ? VASurfaceRendering : VASurfaceReady;
}

/* Called with drv->api_mutex held. */
static VAStatus sync_coded(struct v4l2r_driver *drv, struct v4l2r_buffer *buffer,
			   int timeout_ms)
{
	struct v4l2r_context *ctx;

	if (buffer->coded->ready)
		return VA_STATUS_SUCCESS;
	ctx = find_encode_context(drv, buffer->context_id);
	if (!ctx || !ctx->enc->encoder)
		return VA_STATUS_ERROR_INVALID_CONTEXT;
	return pump_until(ctx->enc, &buffer->coded->ready, timeout_ms);
}

/*
 * Hand out coded buffers in submission order. A client mapping the buffers
 * of one context from several threads gets them in order; the api mutex is
 * released while waiting so the earlier buffer can be mapped meanwhile.
 */
static VAStatus wait_delivery(struct v4l2r_driver *drv, VABufferID id,
			      struct v4l2r_buffer **result)
{
	struct v4l2r_buffer *buffer = find_coded(drv, id);
	struct v4l2r_context *ctx;
	unsigned int attempts = 0;

	ctx = buffer ? find_encode_context(drv, buffer->context_id) : NULL;
	if (!buffer)
		return VA_STATUS_ERROR_INVALID_BUFFER;
	if (!ctx || buffer->coded->tag == 0)
		return VA_STATUS_ERROR_INVALID_CONTEXT;
	if (buffer->coded->delivered) {
		*result = buffer;
		return VA_STATUS_SUCCESS;
	}

	while (buffer->coded->sequence != ctx->enc->delivery_sequence) {
		if (buffer->coded->sequence < ctx->enc->delivery_sequence)
			return VA_STATUS_ERROR_OPERATION_FAILED;
		if (attempts++ >= 5000)
			return VA_STATUS_ERROR_HW_BUSY;

		pthread_mutex_unlock(&drv->api_mutex);
		usleep(1000);
		pthread_mutex_lock(&drv->api_mutex);

		buffer = find_coded(drv, id);
		ctx = buffer ? find_encode_context(drv, buffer->context_id) : NULL;
		if (!buffer || !ctx)
			return VA_STATUS_ERROR_INVALID_BUFFER;
		if (buffer->coded->delivered)
			break;
	}

	if (!buffer->coded->delivered) {
		ctx->enc->delivery_sequence++;
		buffer->coded->delivered = true;
	}
	*result = buffer;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_encode_map_coded(struct v4l2r_driver *drv, VABufferID id,
				void **mapped)
{
	struct v4l2r_buffer *buffer;
	VAStatus status;

	pthread_mutex_lock(&drv->api_mutex);
	buffer = find_coded(drv, id);
	if (!buffer) {
		status = VA_STATUS_ERROR_INVALID_BUFFER;
		goto out;
	}
	if (!buffer->coded->ready) {
		status = sync_coded(drv, buffer, AVE_MAP_TIMEOUT_MS);
		if (status != VA_STATUS_SUCCESS)
			goto out;
	}
	status = wait_delivery(drv, id, &buffer);
	if (status == VA_STATUS_SUCCESS)
		*mapped = &buffer->coded->segment;
out:
	pthread_mutex_unlock(&drv->api_mutex);
	return status;
}

/* Wait for a coded buffer still in flight, then release its state. */
VAStatus v4l2r_encode_destroy_coded(struct v4l2r_driver *drv,
				    struct v4l2r_buffer *buffer)
{
	VAStatus status = VA_STATUS_SUCCESS;

	if (buffer->coded->tag && !buffer->coded->ready &&
	    find_encode_context(drv, buffer->context_id))
		status = sync_coded(drv, buffer, AVE_MAP_TIMEOUT_MS);
	if (status != VA_STATUS_SUCCESS)
		return status;

	struct v4l2r_surface *surface = find_surface(drv, buffer->coded->source);

	if (surface && surface->encode_coded != VA_INVALID_ID &&
	    find_coded(drv, surface->encode_coded) == buffer) {
		surface->encode_pending = false;
		surface->encode_coded = VA_INVALID_ID;
	}
	free(buffer->coded);
	buffer->coded = NULL;
	return VA_STATUS_SUCCESS;
}

#if VA_CHECK_VERSION(1, 9, 0)
VAStatus v4l2r_SyncBuffer(VADriverContextP va_ctx, VABufferID buf_id,
			  uint64_t timeout_ns)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_buffer *buffer;
	uint64_t timeout_ms;
	VAStatus status;

	timeout_ms = timeout_ns == VA_TIMEOUT_INFINITE ? AVE_SYNC_TIMEOUT_MS :
		     (timeout_ns + 999999u) / 1000000u;
	if (timeout_ms > INT_MAX)
		timeout_ms = INT_MAX;

	pthread_mutex_lock(&drv->api_mutex);
	buffer = find_coded(drv, buf_id);
	status = buffer ? sync_coded(drv, buffer, (int)timeout_ms) :
			  VA_STATUS_ERROR_INVALID_BUFFER;
	pthread_mutex_unlock(&drv->api_mutex);
	return status;
}
#endif
