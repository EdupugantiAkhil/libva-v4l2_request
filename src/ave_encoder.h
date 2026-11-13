/*
 * Apple AVE stateful V4L2 encoder (apple-ave kernel driver): device probe,
 * control mapping, OUTPUT frame packing and queue handling.
 *
 * Ported from aurora-ave-vaapi e40b036 (src/v4l2_encoder.[ch],
 * src/v4l2_probe.[ch]), itself a port of snowf14k3/venus-vaapi-driver
 * 37e1030aba11f66e303e4f5fb0c3a13e2a78265e.
 *
 * Copyright (c) 2026 snowf14k3
 * Copyright (c) 2026 libva-v4l2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef AVE_ENCODER_H
#define AVE_ENCODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct v4l2_capability;

/* video_device name (sysfs) and QUERYCAP driver of the stateful encoder. */
#define AVE_V4L2_DRIVER_NAME	"apple-ave"

/* Highest /dev/videoN index looked at by the sysfs probe. */
#define AVE_PROBE_LIMIT		64

/* Coded/raw formats the probe found on the encoder node. */
#define AVE_CODED_H264		(1u << 0)
#define AVE_CODED_HEVC		(1u << 1)
#define AVE_RAW_NV12		(1u << 0)
#define AVE_RAW_P010		(1u << 1)

struct ave_device {
	bool present;
	char video_path[64];
	uint32_t coded;		/* AVE_CODED_* */
	uint32_t raw;		/* AVE_RAW_* */
};

/*
 * Find the apple-ave node. Candidates are selected from sysfs
 * (<sysfs_root>/videoN/name) without opening anything, so unrelated V4L2
 * nodes (camera, decoder) are never touched; only a matching node is opened
 * and confirmed with VIDIOC_QUERYCAP. @requested (may be NULL) names one
 * /dev/videoN node explicitly; it is still checked against sysfs and
 * QUERYCAP. Returns 0 or a negative errno.
 */
int ave_probe(struct ave_device *device, const char *sysfs_root,
	      const char *device_prefix, unsigned int limit,
	      const char *requested);

/* Exposed for host tests: no device is opened by these helpers. */
bool ave_probe_name_matches(const char *contents);
size_t ave_probe_select_nodes(const char *sysfs_root, unsigned int limit,
			      unsigned int *indices, size_t max_indices);
bool ave_capability_is_encoder(const struct v4l2_capability *capability);

struct ave_encoder;

/* Called when an OUTPUT (raw) buffer is released by the encoder. */
typedef int (*ave_output_done_callback)(uint64_t tag, void *opaque);

/*
 * Everything the apple-ave stateful encoder can be told. Each field maps to
 * one V4L2 control/format the kernel driver implements (ave-v4l2.c); there
 * are deliberately no fields for controls apple-ave lacks (frame QP,
 * entropy mode, 8x8 transform, AU delimiter, tier, frame RC enable).
 */
struct ave_encoder_config {
	const char *device;
	uint32_t coded_format;		/* V4L2_PIX_FMT_H264 / _HEVC */
	uint32_t raw_format;		/* V4L2_PIX_FMT_NV12 / _P010 */
	uint32_t width;			/* visible size, even */
	uint32_t height;
	uint32_t frames_per_second;	/* 1..240 */
	uint32_t bitrate_mode;		/* V4L2_MPEG_VIDEO_BITRATE_MODE_CBR / _VBR */
	uint32_t bitrate;		/* average (VBR) or constant (CBR) bit/s */
	uint32_t bitrate_peak;		/* VBR peak bit/s, 0 = driver default (2x) */
	bool qp_range;			/* apply min_qp/max_qp */
	uint32_t min_qp;
	uint32_t max_qp;
	uint32_t gop_size;		/* IDR interval in frames, 1..65535 */
	uint32_t h264_profile;		/* V4L2_MPEG_VIDEO_H264_PROFILE_* */
	uint32_t h264_level;		/* minimum level; the driver may raise it */
	uint32_t hevc_profile;		/* V4L2_MPEG_VIDEO_HEVC_PROFILE_* */
	uint32_t hevc_level;		/* minimum level; the driver may raise it */
	size_t capture_buffer_size;
	unsigned int output_buffers;
	unsigned int capture_buffers;
	ave_output_done_callback output_done;
	void *output_done_opaque;
};

struct ave_control {
	uint32_t id;
	int32_t value;
	const char *name;
};

#define AVE_MAX_CONTROLS	16

/*
 * Pure translation of a config to the ordered S_CTRL list (host-testable).
 * Returns 0 or a negative errno for values apple-ave cannot honour.
 */
int ave_encoder_controls(const struct ave_encoder_config *config,
			 struct ave_control *controls, size_t max_controls,
			 size_t *count);

/*
 * Copy a visible 4:2:0 two-plane frame into AVE's OUTPUT layout: luma rows
 * at @destination_stride, chroma rows immediately after @height luma rows
 * (no padding between planes; any padding is at the end, ave-sess.c).
 */
int ave_pack_frame(uint8_t *destination, size_t destination_size,
		   uint32_t destination_stride,
		   const uint8_t *source_y, const uint8_t *source_uv,
		   uint32_t source_stride, uint32_t width, uint32_t height,
		   unsigned int bytes_per_sample, size_t *packed_size);

/* apple-ave's own minimum CAPTURE size (ave_fill_coded). */
int ave_encoder_compressed_size(uint32_t width, uint32_t height, size_t *size);

struct ave_packet {
	const uint8_t *data;
	size_t size;
	/* Tags are V4L2 timestamps in microseconds and identify the input. */
	uint64_t tag;
	uint32_t flags;
};

typedef int (*ave_packet_callback)(const struct ave_packet *packet,
				   void *opaque);

/* One raw input frame: two planes sharing one stride. */
struct ave_frame {
	const uint8_t *y;
	size_t y_size;			/* readable bytes from y */
	const uint8_t *uv;
	size_t uv_size;			/* readable bytes from uv */
	uint32_t stride;
};

/* Open the stateful encoder and configure its raw and coded queues.
 * @operation (64 bytes, may be NULL) receives the failed step. */
int ave_encoder_open(const struct ave_encoder_config *config,
		     struct ave_encoder **encoder, char *operation,
		     size_t operation_size);
/* Queue one frame (copied into an OUTPUT buffer before returning). */
int ave_encoder_submit(struct ave_encoder *encoder,
		       const struct ave_frame *frame, uint64_t tag,
		       ave_packet_callback callback, void *opaque);
/* Wait until the encoder holds no queued raw frame (keyframe sequencing). */
int ave_encoder_wait_output_idle(struct ave_encoder *encoder, int timeout_ms,
				 ave_packet_callback callback, void *opaque);
int ave_encoder_force_keyframe(struct ave_encoder *encoder);
/* Bit rate changes take effect at the next frame sent to the firmware. */
int ave_encoder_set_bitrate(struct ave_encoder *encoder, uint32_t bitrate);
/* Poll once and dequeue whatever completed: 0 on progress, -EAGAIN without
 * progress, -ETIMEDOUT when nothing became ready, other negative errno. */
int ave_encoder_pump(struct ave_encoder *encoder, int timeout_ms,
		     ave_packet_callback callback, void *opaque,
		     bool *end_of_stream);
void ave_encoder_close(struct ave_encoder *encoder);

const char *ave_encoder_last_operation(const struct ave_encoder *encoder);

#endif /* AVE_ENCODER_H */
