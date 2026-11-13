/*
 * Apple AVE stateful V4L2 encoder (apple-ave kernel driver).
 *
 * apple-ave is a stateful V4L2 M2M encoder: raw NV12/P010 frames are queued
 * on OUTPUT and Annex B access units are dequeued from CAPTURE. Both queues
 * are MMAP; raw frames are copied into AVE's layout (stride = S_FMT
 * bytesperline, chroma straight after the visible luma rows).
 *
 * Ported from aurora-ave-vaapi e40b036 (src/v4l2_encoder.c,
 * src/v4l2_probe.c), itself a port of snowf14k3/venus-vaapi-driver
 * 37e1030aba11f66e303e4f5fb0c3a13e2a78265e. Changes for this driver: the
 * frame is passed as separate luma/chroma planes (decoder CAPTURE buffers
 * and imported dma-bufs need not be contiguous), and the Venus VP8/DMABUF
 * OUTPUT leftovers are dropped.
 *
 * Copyright (c) 2026 snowf14k3
 * Copyright (c) 2026 libva-v4l2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "config.h"
#include "ave_encoder.h"

struct mapped_buffer {
	void *data;
	size_t length;
	uint64_t tag;
	bool queued;
};

struct ave_encoder {
	int fd;
	struct mapped_buffer *output;
	struct mapped_buffer *capture;
	unsigned int output_count;
	unsigned int capture_count;
	struct v4l2_pix_format_mplane output_format;
	struct v4l2_pix_format_mplane capture_format;
	uint32_t visible_width;
	uint32_t visible_height;
	bool output_streaming;
	bool capture_streaming;
	bool stop_sent;
	ave_output_done_callback output_done;
	void *output_done_opaque;
	char last_operation[64];
};

static int xioctl(int fd, unsigned long request, void *argument)
{
	int result;

	do {
		result = ioctl(fd, request, argument);
	} while (result < 0 && errno == EINTR);

	return result;
}

static int64_t monotonic_milliseconds(void)
{
	struct timespec now;

	if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
		return -1;
	return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/* --- probe --- */

bool ave_probe_name_matches(const char *contents)
{
	size_t length;

	if (!contents)
		return false;
	length = strcspn(contents, "\r\n");
	return length == strlen(AVE_V4L2_DRIVER_NAME) &&
	       strncmp(contents, AVE_V4L2_DRIVER_NAME, length) == 0;
}

static bool sysfs_node_is_ave(const char *sysfs_root, unsigned int index)
{
	char path[256];
	char contents[64] = {0};
	ssize_t length;
	int fd;

	if (snprintf(path, sizeof(path), "%s/video%u/name", sysfs_root, index) >=
	    (int)sizeof(path))
		return false;
	fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return false;
	length = read(fd, contents, sizeof(contents) - 1);
	close(fd);
	if (length <= 0)
		return false;
	contents[length] = '\0';
	return ave_probe_name_matches(contents);
}

size_t ave_probe_select_nodes(const char *sysfs_root, unsigned int limit,
			      unsigned int *indices, size_t max_indices)
{
	size_t count = 0;

	if (!sysfs_root || !indices)
		return 0;
	for (unsigned int index = 0; index < limit && count < max_indices; index++) {
		if (sysfs_node_is_ave(sysfs_root, index))
			indices[count++] = index;
	}
	return count;
}

bool ave_capability_is_encoder(const struct v4l2_capability *capability)
{
	uint32_t caps;

	if (!capability)
		return false;
	caps = capability->capabilities & V4L2_CAP_DEVICE_CAPS ?
	       capability->device_caps : capability->capabilities;
	return strcmp((const char *)capability->driver, AVE_V4L2_DRIVER_NAME) == 0 &&
	       strstr((const char *)capability->card, "encoder") &&
	       (caps & V4L2_CAP_VIDEO_M2M_MPLANE) &&
	       (caps & V4L2_CAP_STREAMING);
}

static void enumerate_formats(int fd, struct ave_device *device)
{
	struct v4l2_fmtdesc format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	};

	for (format.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &format) == 0;
	     format.index++) {
		if (format.pixelformat == V4L2_PIX_FMT_H264)
			device->coded |= AVE_CODED_H264;
		else if (format.pixelformat == V4L2_PIX_FMT_HEVC)
			device->coded |= AVE_CODED_HEVC;
	}

	format = (struct v4l2_fmtdesc){ .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	for (format.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &format) == 0;
	     format.index++) {
		if (format.pixelformat == V4L2_PIX_FMT_NV12)
			device->raw |= AVE_RAW_NV12;
#if HAVE_V4L2_PIX_FMT_P010
		else if (format.pixelformat == V4L2_PIX_FMT_P010)
			device->raw |= AVE_RAW_P010;
#endif
	}
}

static int probe_node(struct ave_device *device, const char *path)
{
	struct v4l2_capability capability = {0};
	int status = -ENODEV;
	int fd;

	fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0)
		return -errno;
	if (xioctl(fd, VIDIOC_QUERYCAP, &capability) == 0 &&
	    ave_capability_is_encoder(&capability)) {
		snprintf(device->video_path, sizeof(device->video_path), "%s", path);
		enumerate_formats(fd, device);
		device->present = device->coded && device->raw;
		status = device->present ? 0 : -ENODEV;
	}
	close(fd);
	return status;
}

static int parse_device_index(const char *path, const char *device_prefix,
			      unsigned int *index)
{
	size_t prefix_length = strlen(device_prefix);
	char *end = NULL;
	unsigned long value;

	if (strncmp(path, device_prefix, prefix_length) != 0 ||
	    !path[prefix_length])
		return -EINVAL;
	errno = 0;
	value = strtoul(path + prefix_length, &end, 10);
	if (errno || !end || *end || value > 4096)
		return -EINVAL;
	*index = (unsigned int)value;
	return 0;
}

int ave_probe(struct ave_device *device, const char *sysfs_root,
	      const char *device_prefix, unsigned int limit,
	      const char *requested)
{
	unsigned int indices[8];
	size_t count;

	if (!device || !sysfs_root || !device_prefix || limit == 0)
		return -EINVAL;

	memset(device, 0, sizeof(*device));

	if (requested && requested[0]) {
		unsigned int index;

		if (parse_device_index(requested, device_prefix, &index) < 0 ||
		    !sysfs_node_is_ave(sysfs_root, index))
			return -ENODEV;
		return probe_node(device, requested);
	}

	count = ave_probe_select_nodes(sysfs_root, limit, indices,
				       sizeof(indices) / sizeof(indices[0]));
	for (size_t i = 0; i < count; i++) {
		char path[64];

		if (snprintf(path, sizeof(path), "%s%u", device_prefix,
			     indices[i]) >= (int)sizeof(path))
			return -ENAMETOOLONG;
		if (probe_node(device, path) == 0)
			return 0;
		memset(device, 0, sizeof(*device));
	}

	return -ENODEV;
}

/* --- encoder --- */

static int encoder_error(struct ave_encoder *encoder, const char *operation)
{
	int saved = errno;

	snprintf(encoder->last_operation, sizeof(encoder->last_operation),
		 "%s", operation);
	return -saved;
}

static int verify_device(struct ave_encoder *encoder)
{
	struct v4l2_capability capability = {0};

	if (xioctl(encoder->fd, VIDIOC_QUERYCAP, &capability) < 0)
		return encoder_error(encoder, "VIDIOC_QUERYCAP");

	if (!ave_capability_is_encoder(&capability)) {
		errno = ENODEV;
		return encoder_error(encoder, "encoder capability contract");
	}

	return 0;
}

static unsigned int raw_bytes_per_sample(uint32_t raw_format)
{
#if HAVE_V4L2_PIX_FMT_P010
	if (raw_format == V4L2_PIX_FMT_P010)
		return 2;
#else
	(void)raw_format;
#endif
	return 1;
}

static bool raw_format_is_p010(uint32_t raw_format)
{
#if HAVE_V4L2_PIX_FMT_P010
	return raw_format == V4L2_PIX_FMT_P010;
#else
	(void)raw_format;
	return false;
#endif
}

/* The coded format first: apple-ave sizes and validates the raw side from it. */
static int set_capture_format(struct ave_encoder *encoder,
			      const struct ave_encoder_config *config)
{
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	};

	format.fmt.pix_mp.width = config->width;
	format.fmt.pix_mp.height = config->height;
	format.fmt.pix_mp.pixelformat = config->coded_format;
	format.fmt.pix_mp.field = V4L2_FIELD_NONE;
	format.fmt.pix_mp.num_planes = 1;
	format.fmt.pix_mp.plane_fmt[0].sizeimage =
		(uint32_t)config->capture_buffer_size;

	if (xioctl(encoder->fd, VIDIOC_S_FMT, &format) < 0)
		return encoder_error(encoder, "VIDIOC_S_FMT(CAPTURE)");

	if (format.fmt.pix_mp.pixelformat != config->coded_format ||
	    format.fmt.pix_mp.num_planes != 1 ||
	    format.fmt.pix_mp.plane_fmt[0].sizeimage == 0) {
		errno = EINVAL;
		return encoder_error(encoder, "coded capture format contract");
	}

	encoder->capture_format = format.fmt.pix_mp;
	return 0;
}

static int set_output_format(struct ave_encoder *encoder,
			     const struct ave_encoder_config *config)
{
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	};
	const struct v4l2_pix_format_mplane *result = &format.fmt.pix_mp;
	uint64_t row_bytes =
		(uint64_t)config->width * raw_bytes_per_sample(config->raw_format);

	format.fmt.pix_mp.width = config->width;
	format.fmt.pix_mp.height = config->height;
	format.fmt.pix_mp.pixelformat = config->raw_format;
	format.fmt.pix_mp.field = V4L2_FIELD_NONE;
	format.fmt.pix_mp.num_planes = 1;

	if (xioctl(encoder->fd, VIDIOC_S_FMT, &format) < 0)
		return encoder_error(encoder, "VIDIOC_S_FMT(OUTPUT)");

	/*
	 * apple-ave clamps sizes and silently swaps P010 for NV12 when the
	 * pairing is not HEVC; any adjustment means this request cannot be
	 * encoded as asked.
	 */
	if (result->pixelformat != config->raw_format ||
	    result->num_planes != 1 ||
	    result->width != config->width ||
	    result->height != config->height ||
	    result->plane_fmt[0].bytesperline < row_bytes ||
	    (uint64_t)result->plane_fmt[0].sizeimage <
	    (uint64_t)result->plane_fmt[0].bytesperline * config->height * 3u / 2u) {
		errno = ERANGE;
		return encoder_error(encoder, "raw output format contract");
	}

	encoder->output_format = format.fmt.pix_mp;
	return 0;
}

/*
 * Set one control and check that the driver kept the value: integer
 * controls are clamped by the V4L2 core rather than rejected, which would
 * otherwise hide an unsupported request.
 */
static int set_control(struct ave_encoder *encoder, uint32_t id, int32_t value,
		       const char *operation)
{
	struct v4l2_control control = {
		.id = id,
		.value = value,
	};

	if (xioctl(encoder->fd, VIDIOC_S_CTRL, &control) < 0)
		return encoder_error(encoder, operation);
	if (id != V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME && control.value != value) {
		errno = ERANGE;
		return encoder_error(encoder, operation);
	}
	return 0;
}

int ave_encoder_compressed_size(uint32_t width, uint32_t height, size_t *size)
{
	size_t aligned_width;
	size_t aligned_height;
	size_t compressed;

	if (!width || !height || !size)
		return -EINVAL;
	/* apple-ave's own minimum (ave_fill_coded): 1.5 B/pixel + 64 KiB. */
	aligned_width = ((size_t)width + 31u) & ~(size_t)31u;
	aligned_height = ((size_t)height + 15u) & ~(size_t)15u;
	if (aligned_width > SIZE_MAX / aligned_height / 3u)
		return -EOVERFLOW;
	compressed = aligned_width * aligned_height * 3u / 2u;
	compressed = (compressed + 16383u) & ~(size_t)16383u;
	*size = compressed + 65536u;
	return 0;
}

int ave_encoder_controls(const struct ave_encoder_config *config,
			 struct ave_control *controls, size_t max_controls,
			 size_t *count)
{
	size_t n = 0;
	bool hevc;

#define AVE_PUSH(control_id, control_value, control_name)			\
	do {									\
		if (n >= max_controls)						\
			return -ENOSPC;						\
		controls[n++] = (struct ave_control) {				\
			(control_id), (int32_t)(control_value), (control_name)	\
		};								\
	} while (0)

	if (!config || !controls || !count)
		return -EINVAL;
	*count = 0;
	hevc = config->coded_format == V4L2_PIX_FMT_HEVC;
	if (!hevc && config->coded_format != V4L2_PIX_FMT_H264)
		return -ENOTSUP;

	/* apple-ave ranges (ave_init_ctrls): bit rate 100 kbit/s..100 Mbit/s. */
	if (config->bitrate < 100000u || config->bitrate > 100000000u)
		return -ERANGE;
	if (config->bitrate_mode != V4L2_MPEG_VIDEO_BITRATE_MODE_CBR &&
	    config->bitrate_mode != V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
		return -ENOTSUP;
	if (config->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
	    config->bitrate_peak &&
	    (config->bitrate_peak < config->bitrate ||
	     config->bitrate_peak > 200000000u))
		return -ERANGE;
	if (config->gop_size < 1 || config->gop_size > 65535u)
		return -ERANGE;
	if (config->qp_range &&
	    (config->min_qp > config->max_qp || config->max_qp > 51u))
		return -ERANGE;

	/* Low-delay P frames: VA encode returns one coded frame per input. */
	AVE_PUSH(V4L2_CID_MPEG_VIDEO_B_FRAMES, 0, "S_CTRL(B_FRAMES)");
	AVE_PUSH(V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		 V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		 "S_CTRL(HEADER_MODE)");
	AVE_PUSH(V4L2_CID_MPEG_VIDEO_BITRATE_MODE, config->bitrate_mode,
		 "S_CTRL(BITRATE_MODE)");
	AVE_PUSH(V4L2_CID_MPEG_VIDEO_BITRATE, config->bitrate, "S_CTRL(BITRATE)");
	if (config->bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
		AVE_PUSH(V4L2_CID_MPEG_VIDEO_BITRATE_PEAK, config->bitrate_peak,
			 "S_CTRL(BITRATE_PEAK)");
	AVE_PUSH(V4L2_CID_MPEG_VIDEO_GOP_SIZE, config->gop_size,
		 "S_CTRL(GOP_SIZE)");

	if (hevc) {
		if (config->hevc_profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN &&
		    config->hevc_profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10)
			return -ENOTSUP;
		if ((config->hevc_profile == V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10) !=
		    raw_format_is_p010(config->raw_format))
			return -EINVAL;
		/* The apple-ave HEVC_LEVEL menu stops at 5.1. */
		if (config->hevc_level > V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1)
			return -ENOTSUP;
		AVE_PUSH(V4L2_CID_MPEG_VIDEO_HEVC_PROFILE, config->hevc_profile,
			 "S_CTRL(HEVC_PROFILE)");
		AVE_PUSH(V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, config->hevc_level,
			 "S_CTRL(HEVC_LEVEL)");
		if (config->qp_range) {
			AVE_PUSH(V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, config->min_qp,
				 "S_CTRL(HEVC_MIN_QP)");
			AVE_PUSH(V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, config->max_qp,
				 "S_CTRL(HEVC_MAX_QP)");
		}
	} else {
		if (config->raw_format != V4L2_PIX_FMT_NV12)
			return -EINVAL;
		if (config->h264_profile !=
		    V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE &&
		    config->h264_profile != V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE &&
		    config->h264_profile != V4L2_MPEG_VIDEO_H264_PROFILE_MAIN &&
		    config->h264_profile != V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)
			return -ENOTSUP;
		if (config->h264_level > V4L2_MPEG_VIDEO_H264_LEVEL_5_2)
			return -ENOTSUP;
		AVE_PUSH(V4L2_CID_MPEG_VIDEO_H264_PROFILE, config->h264_profile,
			 "S_CTRL(H264_PROFILE)");
		AVE_PUSH(V4L2_CID_MPEG_VIDEO_H264_LEVEL, config->h264_level,
			 "S_CTRL(H264_LEVEL)");
		if (config->qp_range) {
			AVE_PUSH(V4L2_CID_MPEG_VIDEO_H264_MIN_QP, config->min_qp,
				 "S_CTRL(H264_MIN_QP)");
			AVE_PUSH(V4L2_CID_MPEG_VIDEO_H264_MAX_QP, config->max_qp,
				 "S_CTRL(H264_MAX_QP)");
		}
	}
#undef AVE_PUSH

	*count = n;
	return 0;
}

static int set_parameters(struct ave_encoder *encoder,
			  const struct ave_encoder_config *config)
{
	struct v4l2_streamparm parameters = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	};
	struct ave_control controls[AVE_MAX_CONTROLS];
	size_t count;
	int status;

	status = ave_encoder_controls(config, controls, AVE_MAX_CONTROLS, &count);
	if (status < 0) {
		snprintf(encoder->last_operation, sizeof(encoder->last_operation),
			 "unsupported encoder parameters");
		return status;
	}

	parameters.parm.output.timeperframe.numerator = 1;
	parameters.parm.output.timeperframe.denominator = config->frames_per_second;
	if (xioctl(encoder->fd, VIDIOC_S_PARM, &parameters) < 0)
		return encoder_error(encoder, "VIDIOC_S_PARM(OUTPUT)");
	if (parameters.parm.output.timeperframe.numerator != 1 ||
	    parameters.parm.output.timeperframe.denominator !=
	    config->frames_per_second) {
		errno = ERANGE;
		return encoder_error(encoder, "VIDIOC_S_PARM(OUTPUT) adjusted");
	}

	for (size_t index = 0; index < count; index++) {
		status = set_control(encoder, controls[index].id,
				     controls[index].value, controls[index].name);
		if (status < 0)
			return status;
	}
	return 0;
}

static unsigned int device_min_output_buffers(struct ave_encoder *encoder)
{
	struct v4l2_control control = {
		.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
	};

	if (xioctl(encoder->fd, VIDIOC_G_CTRL, &control) < 0 || control.value < 1)
		return 0;
	return (unsigned int)control.value;
}

static int request_and_map(struct ave_encoder *encoder, enum v4l2_buf_type type,
			   unsigned int requested, struct mapped_buffer **mapped,
			   unsigned int *mapped_count)
{
	bool output = type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	struct v4l2_requestbuffers request = {
		.count = requested,
		.type = type,
		.memory = V4L2_MEMORY_MMAP,
	};
	struct mapped_buffer *buffers;
	unsigned int index;
	int status;

	if (xioctl(encoder->fd, VIDIOC_REQBUFS, &request) < 0)
		return encoder_error(encoder, output ? "VIDIOC_REQBUFS(OUTPUT)" :
						       "VIDIOC_REQBUFS(CAPTURE)");
	if (request.count == 0)
		return -ENOMEM;

	buffers = calloc(request.count, sizeof(*buffers));
	if (!buffers)
		return -ENOMEM;

	for (index = 0; index < request.count; index++) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer buffer = {
			.index = index,
			.type = type,
			.memory = V4L2_MEMORY_MMAP,
			.length = 1,
			.m.planes = planes,
		};

		if (xioctl(encoder->fd, VIDIOC_QUERYBUF, &buffer) < 0) {
			status = encoder_error(encoder, output ?
					       "VIDIOC_QUERYBUF(OUTPUT)" :
					       "VIDIOC_QUERYBUF(CAPTURE)");
			goto fail;
		}
		if (buffer.length != 1 || planes[0].length == 0) {
			errno = EINVAL;
			status = encoder_error(encoder, "VIDIOC_QUERYBUF(plane contract)");
			goto fail;
		}

		buffers[index].length = planes[0].length;
		buffers[index].data = mmap(NULL, planes[0].length,
					   PROT_READ | PROT_WRITE, MAP_SHARED,
					   encoder->fd, planes[0].m.mem_offset);
		if (buffers[index].data == MAP_FAILED) {
			buffers[index].data = NULL;
			status = encoder_error(encoder, output ? "mmap(OUTPUT)" :
							       "mmap(CAPTURE)");
			goto fail;
		}
	}

	*mapped = buffers;
	*mapped_count = request.count;
	return 0;

fail:
	while (index > 0) {
		index--;
		if (buffers[index].data)
			munmap(buffers[index].data, buffers[index].length);
	}
	free(buffers);
	return status;
}

static void release_buffers(struct ave_encoder *encoder, enum v4l2_buf_type type,
			    struct mapped_buffer **mapped,
			    unsigned int *mapped_count)
{
	struct v4l2_requestbuffers request = {
		.count = 0,
		.type = type,
		.memory = V4L2_MEMORY_MMAP,
	};

	if (*mapped) {
		for (unsigned int index = 0; index < *mapped_count; index++) {
			if ((*mapped)[index].data)
				munmap((*mapped)[index].data,
				       (*mapped)[index].length);
		}
		free(*mapped);
		*mapped = NULL;
		*mapped_count = 0;
	}

	if (encoder->fd >= 0)
		xioctl(encoder->fd, VIDIOC_REQBUFS, &request);
}

static int queue_capture(struct ave_encoder *encoder, unsigned int index)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {
		.index = index,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_MMAP,
		.length = 1,
		.m.planes = planes,
	};

	planes[0].length = (uint32_t)encoder->capture[index].length;
	if (xioctl(encoder->fd, VIDIOC_QBUF, &buffer) < 0)
		return encoder_error(encoder, "VIDIOC_QBUF(CAPTURE)");

	encoder->capture[index].queued = true;
	return 0;
}

static int stream_on(struct ave_encoder *encoder, enum v4l2_buf_type type)
{
	if (xioctl(encoder->fd, VIDIOC_STREAMON, &type) < 0)
		return encoder_error(encoder,
				     type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ?
				     "VIDIOC_STREAMON(OUTPUT)" :
				     "VIDIOC_STREAMON(CAPTURE)");

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		encoder->output_streaming = true;
	else
		encoder->capture_streaming = true;
	return 0;
}

static void stream_off(struct ave_encoder *encoder, enum v4l2_buf_type type)
{
	if (encoder->fd >= 0)
		xioctl(encoder->fd, VIDIOC_STREAMOFF, &type);
}

static void subscribe_eos(struct ave_encoder *encoder)
{
	struct v4l2_event_subscription subscription = {
		.type = V4L2_EVENT_EOS,
	};

	xioctl(encoder->fd, VIDIOC_SUBSCRIBE_EVENT, &subscription);
}

int ave_encoder_open(const struct ave_encoder_config *config,
		     struct ave_encoder **result, char *operation,
		     size_t operation_size)
{
	struct ave_encoder *encoder;
	unsigned int output_buffers;
	unsigned int minimum;
	int status;

	if (operation && operation_size)
		operation[0] = '\0';
	if (result)
		*result = NULL;

	if (!config || !result || !config->device ||
	    (config->coded_format != V4L2_PIX_FMT_H264 &&
	     config->coded_format != V4L2_PIX_FMT_HEVC) ||
	    (config->raw_format != V4L2_PIX_FMT_NV12 &&
	     !(config->coded_format == V4L2_PIX_FMT_HEVC &&
	       raw_format_is_p010(config->raw_format))) ||
	    config->width == 0 || config->height == 0 ||
	    (config->width & 1u) || (config->height & 1u) ||
	    config->frames_per_second == 0 ||
	    config->frames_per_second > 240 ||
	    config->capture_buffer_size == 0 ||
	    config->capture_buffer_size > UINT32_MAX ||
	    config->output_buffers == 0 ||
	    config->capture_buffers == 0)
		return -EINVAL;

	encoder = calloc(1, sizeof(*encoder));
	if (!encoder)
		return -ENOMEM;
	encoder->fd = -1;
	encoder->visible_width = config->width;
	encoder->visible_height = config->height;
	encoder->output_done = config->output_done;
	encoder->output_done_opaque = config->output_done_opaque;

	encoder->fd = open(config->device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (encoder->fd < 0) {
		status = encoder_error(encoder, "open(encoder device)");
		goto fail;
	}

	status = verify_device(encoder);
	if (status < 0)
		goto fail;
	status = set_capture_format(encoder, config);
	if (status < 0)
		goto fail;
	status = set_output_format(encoder, config);
	if (status < 0)
		goto fail;

	/* The firmware holds up to AVE_MAX_SLOTS frames; the driver says so. */
	output_buffers = config->output_buffers;
	minimum = device_min_output_buffers(encoder);
	if (minimum > output_buffers)
		output_buffers = minimum > 16u ? 16u : minimum;

	status = request_and_map(encoder, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
				 output_buffers, &encoder->output,
				 &encoder->output_count);
	if (status < 0)
		goto fail;
	status = request_and_map(encoder, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
				 config->capture_buffers, &encoder->capture,
				 &encoder->capture_count);
	if (status < 0)
		goto fail;

	for (unsigned int index = 0; index < encoder->capture_count; index++) {
		status = queue_capture(encoder, index);
		if (status < 0)
			goto fail;
	}

	subscribe_eos(encoder);
	/* apple-ave reads its controls when both queues start streaming. */
	status = set_parameters(encoder, config);
	if (status < 0)
		goto fail;

	*result = encoder;
	return 0;

fail:
	if (operation && operation_size)
		snprintf(operation, operation_size, "%s",
			 ave_encoder_last_operation(encoder));
	ave_encoder_close(encoder);
	return status;
}

static int dequeue_output(struct ave_encoder *encoder, bool *made_progress)
{
	for (;;) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer buffer = {
			.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.length = 1,
			.m.planes = planes,
		};
		uint64_t tag;

		if (xioctl(encoder->fd, VIDIOC_DQBUF, &buffer) < 0) {
			if (errno == EAGAIN)
				return 0;
			return encoder_error(encoder, "VIDIOC_DQBUF(OUTPUT)");
		}
		if (buffer.index >= encoder->output_count)
			return -EIO;

		tag = encoder->output[buffer.index].tag;
		encoder->output[buffer.index].queued = false;
		encoder->output[buffer.index].tag = 0;
		if (tag && encoder->output_done) {
			int status = encoder->output_done(tag,
							  encoder->output_done_opaque);

			if (status < 0)
				return status;
		}
		*made_progress = true;
	}
}

static int dequeue_capture(struct ave_encoder *encoder,
			   ave_packet_callback callback, void *opaque,
			   bool *end_of_stream, bool *made_progress)
{
	for (;;) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer buffer = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_MMAP,
			.length = 1,
			.m.planes = planes,
		};
		struct ave_packet packet;
		int status;

		if (xioctl(encoder->fd, VIDIOC_DQBUF, &buffer) < 0) {
			if (errno == EAGAIN)
				return 0;
			return encoder_error(encoder, "VIDIOC_DQBUF(CAPTURE)");
		}
		if (buffer.index >= encoder->capture_count ||
		    planes[0].bytesused > encoder->capture[buffer.index].length)
			return -EIO;

		encoder->capture[buffer.index].queued = false;
		*made_progress = true;
		packet = (struct ave_packet) {
			.data = encoder->capture[buffer.index].data,
			.size = planes[0].bytesused,
			.tag = (uint64_t)buffer.timestamp.tv_sec * 1000000u +
			       (uint64_t)buffer.timestamp.tv_usec,
			.flags = buffer.flags,
		};

		/* Errored frames come back empty but still finish a VA picture. */
		if ((packet.size > 0 || (buffer.flags & V4L2_BUF_FLAG_ERROR)) &&
		    callback) {
			status = callback(&packet, opaque);
			if (status < 0)
				return status;
		}

		if (buffer.flags & V4L2_BUF_FLAG_LAST) {
			if (end_of_stream)
				*end_of_stream = true;
			return 0;
		}

		status = queue_capture(encoder, buffer.index);
		if (status < 0)
			return status;
	}
}

int ave_encoder_pump(struct ave_encoder *encoder, int timeout_ms,
		     ave_packet_callback callback, void *opaque,
		     bool *end_of_stream)
{
	struct pollfd poll_fd;
	bool made_progress = false;
	int status;

	if (!encoder || encoder->fd < 0 || timeout_ms < 0)
		return -EINVAL;
	if (end_of_stream)
		*end_of_stream = false;

	poll_fd = (struct pollfd) {
		.fd = encoder->fd,
		.events = POLLIN | POLLOUT | POLLPRI,
	};
	do {
		status = poll(&poll_fd, 1, timeout_ms);
	} while (status < 0 && errno == EINTR);
	if (status < 0)
		return encoder_error(encoder, "poll");
	if (status == 0)
		return -ETIMEDOUT;

	status = dequeue_output(encoder, &made_progress);
	if (status < 0)
		return status;
	status = dequeue_capture(encoder, callback, opaque, end_of_stream,
				 &made_progress);
	if (status < 0)
		return status;

	if ((poll_fd.revents & (POLLERR | POLLHUP | POLLNVAL)) && !made_progress)
		return -EIO;
	return made_progress ? 0 : -EAGAIN;
}

static int find_free_output(struct ave_encoder *encoder,
			    ave_packet_callback callback, void *opaque)
{
	for (;;) {
		for (unsigned int index = 0; index < encoder->output_count; index++) {
			if (!encoder->output[index].queued)
				return (int)index;
		}

		int status = ave_encoder_pump(encoder, 5000, callback, opaque, NULL);

		if (status < 0 && status != -EAGAIN)
			return status;
	}
}

int ave_pack_frame(uint8_t *destination, size_t destination_size,
		   uint32_t destination_stride,
		   const uint8_t *source_y, const uint8_t *source_uv,
		   uint32_t source_stride, uint32_t width, uint32_t height,
		   unsigned int bytes_per_sample, size_t *packed_size)
{
	size_t row_bytes;
	size_t required;
	size_t chroma;

	if (!destination || !source_y || !source_uv || !packed_size ||
	    !width || !height || (width & 1u) || (height & 1u) ||
	    (bytes_per_sample != 1 && bytes_per_sample != 2))
		return -EINVAL;
	row_bytes = (size_t)width * bytes_per_sample;
	if (destination_stride < row_bytes || source_stride < row_bytes)
		return -EINVAL;
	if ((size_t)destination_stride > SIZE_MAX / height / 2u)
		return -EOVERFLOW;
	chroma = (size_t)destination_stride * height;
	required = chroma + (size_t)destination_stride * (height / 2u);
	if (required > destination_size)
		return -ENOSPC;

	for (unsigned int row = 0; row < height; row++)
		memcpy(destination + (size_t)row * destination_stride,
		       source_y + (size_t)row * source_stride, row_bytes);
	for (unsigned int row = 0; row < height / 2u; row++)
		memcpy(destination + chroma + (size_t)row * destination_stride,
		       source_uv + (size_t)row * source_stride, row_bytes);

	*packed_size = required;
	return 0;
}

int ave_encoder_submit(struct ave_encoder *encoder,
		       const struct ave_frame *frame, uint64_t tag,
		       ave_packet_callback callback, void *opaque)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_MMAP,
		.length = 1,
		.m.planes = planes,
	};
	unsigned int bytes_per_sample;
	uint32_t output_size;
	size_t row_bytes;
	size_t packed_size;
	size_t luma_rows, chroma_rows;
	int index;
	int status;

	if (!encoder || !frame || !frame->y || !frame->uv || !tag)
		return -EINVAL;

	bytes_per_sample = raw_bytes_per_sample(encoder->output_format.pixelformat);
	row_bytes = (size_t)encoder->visible_width * bytes_per_sample;
	luma_rows = encoder->visible_height;
	chroma_rows = encoder->visible_height / 2u;
	/* Every row read must lie inside the readable part of its plane. */
	if (frame->stride < row_bytes ||
	    (size_t)frame->stride > SIZE_MAX / luma_rows ||
	    frame->y_size < (size_t)frame->stride * (luma_rows - 1u) + row_bytes ||
	    frame->uv_size < (size_t)frame->stride * (chroma_rows - 1u) + row_bytes)
		return -EINVAL;

	output_size = encoder->output_format.plane_fmt[0].sizeimage;
	if (!output_size)
		return -EINVAL;

	index = find_free_output(encoder, callback, opaque);
	if (index < 0)
		return index;
	if (output_size > encoder->output[index].length)
		return -ENOSPC;

	status = ave_pack_frame(encoder->output[index].data,
				encoder->output[index].length,
				encoder->output_format.plane_fmt[0].bytesperline,
				frame->y, frame->uv, frame->stride,
				encoder->visible_width, encoder->visible_height,
				bytes_per_sample, &packed_size);
	if (status < 0)
		return status;
	if (packed_size > output_size)
		return -ENOSPC;

	buffer.index = (unsigned int)index;
	buffer.field = V4L2_FIELD_NONE;
	buffer.timestamp.tv_sec = (long)(tag / 1000000u);
	buffer.timestamp.tv_usec = (long)(tag % 1000000u);
	/*
	 * A payload of the full sizeimage tells apple-ave the frame already
	 * uses its stride (ave_sess_input only re-strides smaller payloads).
	 */
	planes[0].bytesused = output_size;
	planes[0].data_offset = 0;
	planes[0].length = (uint32_t)encoder->output[index].length;

	if (xioctl(encoder->fd, VIDIOC_QBUF, &buffer) < 0) {
		int saved_errno = errno;

		snprintf(encoder->last_operation, sizeof(encoder->last_operation),
			 "QBUF_OUT %ux%u stride=%u used=%u",
			 encoder->visible_width, encoder->visible_height,
			 encoder->output_format.plane_fmt[0].bytesperline,
			 output_size);
		return -saved_errno;
	}

	encoder->output[index].tag = tag;
	encoder->output[index].queued = true;

	if (!encoder->output_streaming) {
		status = stream_on(encoder, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
		if (status < 0)
			return status;
	}
	if (!encoder->capture_streaming) {
		status = stream_on(encoder, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		if (status < 0)
			return status;
	}

	return 0;
}

static bool output_idle(const struct ave_encoder *encoder)
{
	for (unsigned int index = 0; index < encoder->output_count; index++) {
		if (encoder->output[index].queued)
			return false;
	}
	return true;
}

int ave_encoder_wait_output_idle(struct ave_encoder *encoder, int timeout_ms,
				 ave_packet_callback callback, void *opaque)
{
	int64_t deadline;

	if (!encoder || encoder->fd < 0 || timeout_ms < 0)
		return -EINVAL;
	deadline = monotonic_milliseconds() + timeout_ms;
	while (!output_idle(encoder)) {
		int64_t remaining = deadline - monotonic_milliseconds();
		int status;

		if (remaining <= 0)
			return -ETIMEDOUT;
		status = ave_encoder_pump(encoder,
					  remaining > 1000 ? 1000 : (int)remaining,
					  callback, opaque, NULL);
		if (status < 0 && status != -EAGAIN && status != -ETIMEDOUT)
			return status;
	}
	return 0;
}

int ave_encoder_force_keyframe(struct ave_encoder *encoder)
{
	if (!encoder || encoder->fd < 0)
		return -EINVAL;
	return set_control(encoder, V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0,
			   "S_CTRL(FORCE_KEY_FRAME)");
}

int ave_encoder_set_bitrate(struct ave_encoder *encoder, uint32_t bitrate)
{
	if (!encoder || encoder->fd < 0)
		return -EINVAL;
	if (bitrate < 100000u || bitrate > 100000000u)
		return -ERANGE;
	return set_control(encoder, V4L2_CID_MPEG_VIDEO_BITRATE, (int32_t)bitrate,
			   "S_CTRL(BITRATE)");
}

static int encoder_stop(struct ave_encoder *encoder)
{
	struct v4l2_encoder_cmd command = {
		.cmd = V4L2_ENC_CMD_STOP,
	};

	if (encoder->stop_sent)
		return 0;
	if (xioctl(encoder->fd, VIDIOC_ENCODER_CMD, &command) < 0)
		return encoder_error(encoder, "VIDIOC_ENCODER_CMD(STOP)");

	encoder->stop_sent = true;
	return 0;
}

void ave_encoder_close(struct ave_encoder *encoder)
{
	if (!encoder)
		return;

	/* Drain politely (bounded) so the firmware session ends cleanly. */
	if (encoder->fd >= 0 && encoder->output_streaming &&
	    encoder->capture_streaming && encoder_stop(encoder) == 0) {
		int64_t deadline = monotonic_milliseconds() + 2000;

		while (monotonic_milliseconds() < deadline) {
			bool eos = false;
			int status = ave_encoder_pump(encoder, 200, NULL, NULL, &eos);

			if (eos || (status < 0 && status != -EAGAIN &&
				    status != -ETIMEDOUT))
				break;
		}
	}

	if (encoder->output_streaming)
		stream_off(encoder, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
	if (encoder->capture_streaming)
		stream_off(encoder, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	release_buffers(encoder, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			&encoder->capture, &encoder->capture_count);
	release_buffers(encoder, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			&encoder->output, &encoder->output_count);

	if (encoder->fd >= 0)
		close(encoder->fd);
	free(encoder);
}

const char *ave_encoder_last_operation(const struct ave_encoder *encoder)
{
	if (!encoder || encoder->last_operation[0] == '\0')
		return "none";
	return encoder->last_operation;
}
