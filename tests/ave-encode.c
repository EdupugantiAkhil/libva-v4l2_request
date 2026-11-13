/* SPDX-License-Identifier: MIT */
/*
 * AVE encoding through this driver, host only. open/ioctl/mmap/munmap/
 * close/poll are wrapped at link time: "model-avd" is a decoder that only
 * hands out CAPTURE storage (standalone surface backing, exported as real
 * memfds), FAKE_AVE_PATH is an apple-ave encoder model following ave-v4l2.c
 * (ave_fill_raw / ave_fill_coded / ave_s_fmt_out) that records the OUTPUT
 * payload. Everything else goes to libc.
 *
 * Cases ported from aurora-ave-vaapi e40b036 (ave-vaapi/tests/
 * test_ave_units.c, test_chrome_gating.c, test_fake_ave.c) plus checks that
 * the decode answers do not change without an encoder, that one surface
 * serves both paths, and encode teardown orders.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <linux/dma-buf.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <va/va_drmcommon.h>
#include <va/va_enc_h264.h>
#include <va/va_enc_hevc.h>

#include "v4l2_request.h"
#include "ave_encoder.h"
#include "ave_params.h"

static int failures;

#define CHECK(condition)							\
	do {									\
		if (!(condition)) {						\
			fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__,	\
				__LINE__, #condition);				\
			failures++;						\
		}								\
	} while (0)

#define ALIGN_UP(x, a) (((x) + (a) - 1) / (a) * (a))

/* The models compare against P010 even when the UAPI headers predate it
 * (Linux 6.0); the driver itself only requests it under
 * HAVE_V4L2_PIX_FMT_P010. */
#ifndef V4L2_PIX_FMT_P010
#define V4L2_PIX_FMT_P010 v4l2_fourcc('P', '0', '1', '0')
#endif

#define FAKE_AVE_PATH	"/nonexistent/fake-apple-ave"
#define MISSING_AVE	"/nonexistent/ave-host-test-video"
#define FAKE_AVE_FD	9001
#define AVD_FD_BASE	8000
#define FAKE_BUFS	16

/* ---- model AVD: CAPTURE storage only ---------------------------------- */

static struct {
	unsigned int opens;
	unsigned int live;
	struct v4l2_format capture;
	unsigned int exports;
} avd;

/* DMA_BUF_IOCTL_SYNC calls on real descriptors (memfds stand in). */
static unsigned int dmabuf_syncs;

static int avd_ioctl(unsigned long request, void *arg)
{
	switch (request) {
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *cap = arg;

		memset(cap, 0, sizeof(*cap));
		strcpy((char *)cap->driver, "avd");
		cap->capabilities = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
		return 0;
	}
	case VIDIOC_S_FMT: {
		struct v4l2_format *f = arg;
		struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;

		if (V4L2_TYPE_IS_OUTPUT(f->type))
			return 0;
		/* AVD-like padding: 16-line height, 256-byte pitch. */
		p->height = ALIGN_UP(p->height, 16);
		p->num_planes = 1;
		p->plane_fmt[0].bytesperline =
			ALIGN_UP(p->width * (p->pixelformat == V4L2_PIX_FMT_P010 ? 2 : 1), 256);
		p->plane_fmt[0].sizeimage =
			p->plane_fmt[0].bytesperline * p->height * 3 / 2 + 4096;
		avd.capture = *f;
		return 0;
	}
	case VIDIOC_S_EXT_CTRLS:
		return 0;
	case VIDIOC_CREATE_BUFS:
		((struct v4l2_create_buffers *)arg)->index = 0;
		return 0;
	case VIDIOC_QUERYBUF: {
		struct v4l2_buffer *b = arg;

		b->length = 1;
		b->m.planes[0].length = avd.capture.fmt.pix_mp.plane_fmt[0].sizeimage;
		return 0;
	}
	case VIDIOC_EXPBUF: {
		struct v4l2_exportbuffer *e = arg;
		int fd = memfd_create("model-avd-capture", MFD_CLOEXEC);

		if (fd < 0 || ftruncate(fd, avd.capture.fmt.pix_mp.plane_fmt[0].sizeimage))
			return -1;
		e->fd = fd;
		avd.exports++;
		return 0;
	}
	default:
		errno = ENOTTY;
		return -1;
	}
}

/* ---- model apple-ave ---------------------------------------------------- */

struct fake_buf {
	uint8_t *mem;
	size_t size;
	bool queued;
	bool done;
	struct timeval timestamp;
	uint32_t bytesused;
	uint32_t flags;
};

static struct {
	bool open;
	unsigned int opens;
	struct v4l2_pix_format_mplane src, dst;
	struct fake_buf out[FAKE_BUFS], cap[FAKE_BUFS];
	unsigned int nout, ncap;
	bool out_on, cap_on;
	unsigned int frames;
	int32_t b_frames;
	int32_t bitrate_mode;
	int32_t bitrate;
	unsigned int force_keyframes;
	uint8_t last_luma0, last_chroma0, last_luma_end, last_chroma_end;
	uint32_t last_stride, last_height, last_bytesused, last_sizeimage;
} ave;

/* Lazy mode: frames finish only when the client polls (like hardware). */
static bool lazy;

static void fake_fill_raw(struct v4l2_pix_format_mplane *f, uint32_t coded)
{
	uint32_t bytes, stride;

	if (f->pixelformat != V4L2_PIX_FMT_P010 || coded != V4L2_PIX_FMT_HEVC)
		f->pixelformat = V4L2_PIX_FMT_NV12;
	bytes = f->pixelformat == V4L2_PIX_FMT_P010 ? 2 : 1;
	f->width = ALIGN_UP(f->width, 2);
	f->height = ALIGN_UP(f->height, 2);
	if (f->width < 320) f->width = 320;
	if (f->height < 240) f->height = 240;
	if (f->width > 4096) f->width = 4096;
	if (f->height > 2304) f->height = 2304;
	stride = f->plane_fmt[0].bytesperline;
	if (stride < f->width * bytes) stride = f->width * bytes;
	stride = ALIGN_UP(stride, 64);
	f->num_planes = 1;
	f->plane_fmt[0].bytesperline = stride;
	f->plane_fmt[0].sizeimage = stride * ALIGN_UP(f->height, 16) * 3 / 2;
}

static void fake_fill_coded(struct v4l2_pix_format_mplane *f)
{
	uint32_t min;

	if (f->pixelformat != V4L2_PIX_FMT_H264 && f->pixelformat != V4L2_PIX_FMT_HEVC)
		f->pixelformat = V4L2_PIX_FMT_H264;
	min = ALIGN_UP(3 * ALIGN_UP(f->width, 32) * ALIGN_UP(f->height, 16) / 2,
		       16384) + 65536;
	f->num_planes = 1;
	f->plane_fmt[0].bytesperline = 0;
	if (f->plane_fmt[0].sizeimage < min)
		f->plane_fmt[0].sizeimage = min;
}

/* "Encode" queued OUTPUT buffers into free CAPTURE buffers, in order. */
static void fake_process(void)
{
	if (!ave.out_on || !ave.cap_on)
		return;
	for (unsigned int o = 0; o < ave.nout; o++) {
		struct fake_buf *in = &ave.out[o];
		unsigned int c;

		if (!in->queued || in->done)
			continue;
		for (c = 0; c < ave.ncap; c++) {
			if (ave.cap[c].queued && !ave.cap[c].done)
				break;
		}
		if (c == ave.ncap)
			return;
		ave.last_stride = ave.src.plane_fmt[0].bytesperline;
		ave.last_height = ave.src.height;
		ave.last_bytesused = in->bytesused;
		ave.last_sizeimage = ave.src.plane_fmt[0].sizeimage;
		ave.last_luma0 = in->mem[0];
		/* Last visible luma byte and last visible chroma byte. */
		ave.last_luma_end = in->mem[(size_t)ave.last_stride * (ave.last_height - 1) +
					    ave.src.width - 1];
		/* apple-ave: chroma right after `height` luma rows. */
		ave.last_chroma0 = in->mem[(size_t)ave.last_stride * ave.last_height];
		ave.last_chroma_end = in->mem[(size_t)ave.last_stride * ave.last_height +
					      (size_t)ave.last_stride *
					      (ave.last_height / 2 - 1) +
					      ave.src.width - 1];
		{
			static const uint8_t au[] = { 0, 0, 0, 1, 0x26, 0x01, 0xaf };
			struct fake_buf *out = &ave.cap[c];

			memcpy(out->mem, au, sizeof(au));
			out->bytesused = sizeof(au);
			out->timestamp = in->timestamp;
			out->flags = ave.frames == 0 || ave.force_keyframes ?
				     V4L2_BUF_FLAG_KEYFRAME : V4L2_BUF_FLAG_PFRAME;
			out->done = true;
		}
		in->done = true;
		ave.frames++;
	}
}

static int ave_ioctl(unsigned long request, void *arg)
{
	switch (request) {
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *cap = arg;

		memset(cap, 0, sizeof(*cap));
		strcpy((char *)cap->driver, "apple-ave");
		strcpy((char *)cap->card, "Apple AVE video encoder");
		cap->capabilities = V4L2_CAP_DEVICE_CAPS | V4L2_CAP_VIDEO_M2M_MPLANE |
				    V4L2_CAP_STREAMING;
		cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
		return 0;
	}
	case VIDIOC_S_FMT: {
		struct v4l2_format *f = arg;

		if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			fake_fill_coded(&f->fmt.pix_mp);
			ave.dst = f->fmt.pix_mp;
		} else {
			fake_fill_raw(&f->fmt.pix_mp, ave.dst.pixelformat);
			ave.src = f->fmt.pix_mp;
		}
		return 0;
	}
	case VIDIOC_S_PARM:
		return 0;
	case VIDIOC_S_CTRL: {
		struct v4l2_control *ctrl = arg;

		if (ctrl->id == V4L2_CID_MPEG_VIDEO_B_FRAMES)
			ave.b_frames = ctrl->value;
		if (ctrl->id == V4L2_CID_MPEG_VIDEO_BITRATE_MODE)
			ave.bitrate_mode = ctrl->value;
		if (ctrl->id == V4L2_CID_MPEG_VIDEO_BITRATE)
			ave.bitrate = ctrl->value;
		if (ctrl->id == V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME)
			ave.force_keyframes++;
		return 0;
	}
	case VIDIOC_G_CTRL: {
		struct v4l2_control *ctrl = arg;

		if (ctrl->id != V4L2_CID_MIN_BUFFERS_FOR_OUTPUT) {
			errno = EINVAL;
			return -1;
		}
		ctrl->value = 8;
		return 0;
	}
	case VIDIOC_REQBUFS: {
		struct v4l2_requestbuffers *r = arg;
		bool output = r->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		struct fake_buf *bufs = output ? ave.out : ave.cap;
		unsigned int *n = output ? &ave.nout : &ave.ncap;
		size_t size = output ? ave.src.plane_fmt[0].sizeimage :
				       ave.dst.plane_fmt[0].sizeimage;

		for (unsigned int i = 0; i < *n; i++)
			free(bufs[i].mem);
		memset(bufs, 0, sizeof(struct fake_buf) * FAKE_BUFS);
		if (r->count > FAKE_BUFS)
			r->count = FAKE_BUFS;
		for (unsigned int i = 0; i < r->count; i++) {
			bufs[i].size = size;
			bufs[i].mem = calloc(1, size);
		}
		*n = r->count;
		return 0;
	}
	case VIDIOC_QUERYBUF: {
		struct v4l2_buffer *b = arg;
		bool output = b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		struct fake_buf *buf = output ? &ave.out[b->index] : &ave.cap[b->index];

		b->length = 1;
		b->m.planes[0].length = (uint32_t)buf->size;
		/* Offset encodes queue and index for the fake mmap. */
		b->m.planes[0].m.mem_offset =
			((output ? 0x10000u : 0x20000u) + b->index) * 4096u;
		return 0;
	}
	case VIDIOC_QBUF: {
		struct v4l2_buffer *b = arg;
		bool output = b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		struct fake_buf *buf = output ? &ave.out[b->index] : &ave.cap[b->index];

		buf->queued = true;
		buf->done = false;
		if (output) {
			buf->timestamp = b->timestamp;
			buf->bytesused = b->m.planes[0].bytesused;
		}
		if (!lazy)
			fake_process();
		return 0;
	}
	case VIDIOC_DQBUF: {
		struct v4l2_buffer *b = arg;
		bool output = b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		struct fake_buf *bufs = output ? ave.out : ave.cap;
		unsigned int n = output ? ave.nout : ave.ncap;

		fake_process();
		for (unsigned int i = 0; i < n; i++) {
			if (bufs[i].queued && bufs[i].done) {
				bufs[i].queued = false;
				b->index = i;
				b->timestamp = bufs[i].timestamp;
				b->flags = output ? 0 : bufs[i].flags;
				b->m.planes[0].bytesused = output ? 0 : bufs[i].bytesused;
				return 0;
			}
		}
		errno = EAGAIN;
		return -1;
	}
	case VIDIOC_STREAMON: {
		enum v4l2_buf_type *type = arg;

		if (*type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
			ave.out_on = true;
		else
			ave.cap_on = true;
		if (!lazy)
			fake_process();
		return 0;
	}
	case VIDIOC_STREAMOFF:
	case VIDIOC_SUBSCRIBE_EVENT:
		return 0;
	case VIDIOC_ENCODER_CMD:
		errno = EINVAL;	/* no drain emulation; close copes */
		return -1;
	default:
		errno = ENOTTY;
		return -1;
	}
}

static void ave_free(void)
{
	for (unsigned int i = 0; i < FAKE_BUFS; i++) {
		free(ave.out[i].mem);
		free(ave.cap[i].mem);
		ave.out[i].mem = ave.cap[i].mem = NULL;
	}
	ave.nout = ave.ncap = 0;
}

/* ---- link-time wrappers ---------------------------------------------- */

int __real_open(const char *path, int flags, ...);
int __real_open64(const char *path, int flags, ...);
int __real_ioctl(int fd, unsigned long request, ...);
void *__real_mmap(void *addr, size_t length, int prot, int flags, int fd,
		  off_t offset);
void *__real_mmap64(void *addr, size_t length, int prot, int flags, int fd,
		    off_t offset);
int __real_munmap(void *addr, size_t length);
int __real_close(int fd);
int __real_poll(struct pollfd *fds, nfds_t nfds, int timeout);

static bool is_avd_fd(int fd)
{
	return fd >= AVD_FD_BASE && fd < AVD_FD_BASE + 1000;
}

static int wrap_open(const char *path, int flags, mode_t mode, bool large)
{
	if (path && !strcmp(path, FAKE_AVE_PATH)) {
		if (ave.open) {
			errno = EBUSY;
			return -1;
		}
		ave_free();
		memset(&ave, 0, sizeof(ave));
		ave.open = true;
		ave.opens++;
		return FAKE_AVE_FD;
	}
	if (path && !strcmp(path, "model-avd")) {
		avd.live++;
		return AVD_FD_BASE + (int)(avd.opens++ % 1000);
	}
	return large ? __real_open64(path, flags, mode) :
		       __real_open(path, flags, mode);
}

int __wrap_open(const char *path, int flags, ...)
{
	mode_t mode = 0;

	if (flags & O_CREAT) {
		va_list args;

		va_start(args, flags);
		mode = va_arg(args, mode_t);
		va_end(args);
	}
	return wrap_open(path, flags, mode, false);
}

int __wrap_open64(const char *path, int flags, ...)
{
	mode_t mode = 0;

	if (flags & O_CREAT) {
		va_list args;

		va_start(args, flags);
		mode = va_arg(args, mode_t);
		va_end(args);
	}
	return wrap_open(path, flags, mode, true);
}

int __wrap___open_2(const char *path, int flags)
{
	return wrap_open(path, flags, 0, false);
}

int __wrap___open64_2(const char *path, int flags)
{
	return wrap_open(path, flags, 0, true);
}

int __wrap_ioctl(int fd, unsigned long request, ...)
{
	va_list args;
	void *arg;

	va_start(args, request);
	arg = va_arg(args, void *);
	va_end(args);
	if (fd == FAKE_AVE_FD)
		return ave_ioctl(request, arg);
	if (is_avd_fd(fd))
		return avd_ioctl(request, arg);
	/* memfds stand in for DMA-BUFs: accept the CPU access brackets. */
	if (request == DMA_BUF_IOCTL_SYNC) {
		dmabuf_syncs++;
		return 0;
	}
	return __real_ioctl(fd, request, arg);
}

void *__wrap_mmap(void *addr, size_t length, int prot, int flags, int fd,
		  off_t offset)
{
	if (fd == FAKE_AVE_FD) {
		unsigned int key = (unsigned int)(offset / 4096);
		bool output = key < 0x20000u;
		unsigned int index = key - (output ? 0x10000u : 0x20000u);

		return output ? ave.out[index].mem : ave.cap[index].mem;
	}
	return __real_mmap(addr, length, prot, flags, fd, offset);
}

void *__wrap_mmap64(void *addr, size_t length, int prot, int flags, int fd,
		    off_t offset)
{
	return __wrap_mmap(addr, length, prot, flags, fd, offset);
}

int __wrap_munmap(void *addr, size_t length)
{
	for (unsigned int i = 0; i < FAKE_BUFS; i++) {
		if ((ave.out[i].mem && addr == ave.out[i].mem) ||
		    (ave.cap[i].mem && addr == ave.cap[i].mem))
			return 0;	/* freed at the next REQBUFS / ave_free */
	}
	return __real_munmap(addr, length);
}

int __wrap_close(int fd)
{
	if (fd == FAKE_AVE_FD) {
		ave.open = false;
		return 0;
	}
	if (is_avd_fd(fd)) {
		avd.live--;
		return 0;
	}
	return __real_close(fd);
}

int __wrap_poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
	if (nfds == 1 && fds[0].fd == FAKE_AVE_FD) {
		fake_process();
		fds[0].revents = POLLIN | POLLOUT;
		return 1;
	}
	return __real_poll(fds, nfds, timeout);
}

int __wrap___poll_chk(struct pollfd *fds, nfds_t nfds, int timeout, size_t len)
{
	(void)len;
	return __wrap_poll(fds, nfds, timeout);
}

/* ---- driver under test ------------------------------------------------ */

static struct VADriverContext driver;
static struct VADriverVTable vtable;
static struct v4l2r_driver *drv;

static void fill_vtable(struct VADriverVTable *t)
{
	t->vaQueryConfigProfiles = v4l2r_QueryConfigProfiles;
	t->vaQueryConfigEntrypoints = v4l2r_QueryConfigEntrypoints;
	t->vaQueryConfigAttributes = v4l2r_QueryConfigAttributes;
	t->vaCreateConfig = v4l2r_CreateConfig;
	t->vaDestroyConfig = v4l2r_DestroyConfig;
	t->vaGetConfigAttributes = v4l2r_GetConfigAttributes;
	t->vaCreateSurfaces2 = v4l2r_CreateSurfaces2;
	t->vaDestroySurfaces = v4l2r_DestroySurfaces;
	t->vaExportSurfaceHandle = v4l2r_ExportSurfaceHandle;
	t->vaCreateContext = v4l2r_CreateContext;
	t->vaDestroyContext = v4l2r_DestroyContext;
	t->vaCreateBuffer = v4l2r_CreateBuffer;
	t->vaBufferSetNumElements = v4l2r_BufferSetNumElements;
	t->vaMapBuffer = v4l2r_MapBuffer;
	t->vaUnmapBuffer = v4l2r_UnmapBuffer;
	t->vaDestroyBuffer = v4l2r_DestroyBuffer;
	t->vaBeginPicture = v4l2r_BeginPicture;
	t->vaRenderPicture = v4l2r_RenderPicture;
	t->vaEndPicture = v4l2r_EndPicture;
	t->vaSyncSurface = v4l2r_SyncSurface;
	t->vaQuerySurfaceAttributes = v4l2r_QuerySurfaceAttributes;
	t->vaQuerySurfaceStatus = v4l2r_QuerySurfaceStatus;
	t->vaCreateImage = v4l2r_CreateImage;
	t->vaDeriveImage = v4l2r_DeriveImage;
	t->vaDestroyImage = v4l2r_DestroyImage;
	t->vaGetImage = v4l2r_GetImage;
	t->vaPutImage = v4l2r_PutImage;
#if VA_CHECK_VERSION(1, 9, 0)
	t->vaSyncBuffer = v4l2r_SyncBuffer;
#endif
	t->vaTerminate = v4l2r_Terminate;
	/* Same api_mutex wrappers as the real driver. */
	v4l2r_lock_surface_api(t);
}

static void setup(const char *ave_path)
{
	drv = calloc(1, sizeof(*drv));
	CHECK(drv != NULL);
	pthread_mutex_init(&drv->mutex, NULL);
	pthread_mutex_init(&drv->api_mutex, NULL);
	CHECK(!v4l2r_handles_init(&drv->configs, V4L2R_ID_OFFSET_CONFIG));
	CHECK(!v4l2r_handles_init(&drv->contexts, V4L2R_ID_OFFSET_CONTEXT));
	CHECK(!v4l2r_handles_init(&drv->surfaces, V4L2R_ID_OFFSET_SURFACE));
	CHECK(!v4l2r_handles_init(&drv->buffers, V4L2R_ID_OFFSET_BUFFER));
	CHECK(!v4l2r_handles_init(&drv->images, V4L2R_ID_OFFSET_IMAGE));
	/* AVD as on the M4: H.264 and HEVC (with 10-bit) stateless decoding. */
	drv->nb_decoders = 1;
	strcpy(drv->decoders[0].video_path, "model-avd");
	strcpy(drv->decoders[0].media_path, "model-media");
#if HAVE_V4L2_CTRL_H264
	drv->decoders[0].pixelformats[drv->decoders[0].nb_pixelformats++] =
		V4L2_PIX_FMT_H264_SLICE;
#endif
#if HAVE_V4L2_CTRL_HEVC
	drv->decoders[0].pixelformats[drv->decoders[0].nb_pixelformats++] =
		V4L2_PIX_FMT_HEVC_SLICE;
	drv->decoders[0].hevc_10bit = true;
#endif
	if (!drv->decoders[0].nb_pixelformats)
		drv->decoders[0].pixelformats[drv->decoders[0].nb_pixelformats++] =
			V4L2_PIX_FMT_NV12;
	drv->converter_probed = true;	/* no converter on Apple silicon */
	if (ave_path) {
		drv->ave.present = true;
		snprintf(drv->ave.video_path, sizeof(drv->ave.video_path), "%s",
			 ave_path);
		drv->ave.coded = AVE_CODED_H264 | AVE_CODED_HEVC;
		drv->ave.raw = AVE_RAW_NV12 | AVE_RAW_P010;
	}

	memset(&driver, 0, sizeof(driver));
	memset(&vtable, 0, sizeof(vtable));
	driver.vtable = &vtable;
	driver.pDriverData = drv;
	driver.max_profiles = V4L2R_MAX_PROFILES;
	driver.max_entrypoints = V4L2R_MAX_ENTRYPOINTS;
	driver.max_attributes = V4L2R_MAX_CONFIG_ATTRIBUTES;
	fill_vtable(&vtable);
}

static void teardown(void)
{
	CHECK(vtable.vaTerminate(&driver) == VA_STATUS_SUCCESS);
	ave_free();
	CHECK(avd.live == 0);
}

static VABufferID make_buffer(VAContextID context, VABufferType type,
			      unsigned int size, const void *data)
{
	VABufferID id = VA_INVALID_ID;

	CHECK(vtable.vaCreateBuffer(&driver, context, type, size, 1, (void *)data,
				    &id) == VA_STATUS_SUCCESS);
	return id;
}

/* Chrome's AllocateMiscParameterBuffer layout: header type + payload. */
static VABufferID make_misc(VAContextID context, VAEncMiscParameterType type,
			    const void *payload, size_t size)
{
	uint8_t bytes[sizeof(VAEncMiscParameterBuffer) + 128] = {0};
	VAEncMiscParameterBuffer header = { .type = type };

	memcpy(bytes, &header, sizeof(header));
	memcpy(bytes + sizeof(header), payload, size);
	return make_buffer(context, VAEncMiscParameterBufferType,
			   (unsigned int)(sizeof(header) + size), bytes);
}

/* ---- units (test_ave_units.c) ------------------------------------------ */

/* The controls apple-ave registers (ave-v4l2.c ave_init_ctrls/ave_s_ctrl);
 * menu_mask lists allowed menu entries (bit = value). */
struct ave_ctrl_spec {
	uint32_t id;
	int32_t min, max, step;
	uint64_t menu_mask;
};

static const struct ave_ctrl_spec ave_ctrls[] = {
	{ V4L2_CID_MPEG_VIDEO_BITRATE_MODE, 0, 2, 1,
	  (1u << V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) |
	  (1u << V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) },
	{ V4L2_CID_MPEG_VIDEO_BITRATE, 100000, 100000000, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_BITRATE_PEAK, 0, 200000000, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_GOP_SIZE, 0, 65535, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 0, 0, 0, 0 },
	{ V4L2_CID_MPEG_VIDEO_B_FRAMES, 0, 3, 3, 0 },
	{ V4L2_CID_MPEG_VIDEO_HEADER_MODE, 0, 1, 1,
	  1u << V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME },
	{ V4L2_CID_MPEG_VIDEO_H264_PROFILE, 0, 4, 1,
	  (1u << V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
	  (1u << V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE) |
	  (1u << V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
	  (1u << V4L2_MPEG_VIDEO_H264_PROFILE_HIGH) },
	{ V4L2_CID_MPEG_VIDEO_H264_LEVEL, 0, V4L2_MPEG_VIDEO_H264_LEVEL_5_2, 1,
	  (1u << (V4L2_MPEG_VIDEO_H264_LEVEL_5_2 + 1)) - 1 },
	{ V4L2_CID_MPEG_VIDEO_H264_MIN_QP, 0, 51, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_H264_MAX_QP, 0, 51, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_HEVC_PROFILE, 0, 2, 1,
	  (1u << V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN) |
	  (1u << V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10) },
	{ V4L2_CID_MPEG_VIDEO_HEVC_LEVEL, 0, V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1, 1,
	  (1u << (V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1 + 1)) - 1 },
	{ V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP, 0, 51, 1, 0 },
	{ V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP, 0, 51, 1, 0 },
};

static const struct ave_ctrl_spec *find_spec(uint32_t id)
{
	for (size_t i = 0; i < sizeof(ave_ctrls) / sizeof(ave_ctrls[0]); i++) {
		if (ave_ctrls[i].id == id)
			return &ave_ctrls[i];
	}
	return NULL;
}

static bool value_accepted(const struct ave_ctrl_spec *spec, int32_t value)
{
	if (value < spec->min || value > spec->max)
		return false;
	if (spec->menu_mask)
		return (spec->menu_mask >> value) & 1u;
	if (spec->step > 1)
		return (value - spec->min) % spec->step == 0;
	return true;
}

static struct ave_encoder_config base_config(bool hevc)
{
	return (struct ave_encoder_config) {
		.device = "/nonexistent",
		.coded_format = hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264,
		.raw_format = V4L2_PIX_FMT_NV12,
		.width = 1280,
		.height = 720,
		.frames_per_second = 30,
		.bitrate_mode = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
		.bitrate = 2000000,
		.qp_range = true,
		.min_qp = 1,
		.max_qp = 42,
		.gop_size = 2048,
		.h264_profile = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
		.h264_level = V4L2_MPEG_VIDEO_H264_LEVEL_3_1,
		.hevc_profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.hevc_level = V4L2_MPEG_VIDEO_HEVC_LEVEL_4,
		.capture_buffer_size = 1 << 20,
		.output_buffers = 4,
		.capture_buffers = 8,
	};
}

static void check_controls_supported(const struct ave_encoder_config *c)
{
	struct ave_control controls[AVE_MAX_CONTROLS];
	size_t count = 0;
	bool b_frames_zero = false;

	CHECK(ave_encoder_controls(c, controls, AVE_MAX_CONTROLS, &count) == 0);
	for (size_t i = 0; i < count; i++) {
		const struct ave_ctrl_spec *spec = find_spec(controls[i].id);

		if (!spec) {
			fprintf(stderr, "control %s (0x%x) is not an apple-ave control\n",
				controls[i].name, controls[i].id);
			failures++;
			continue;
		}
		if (!value_accepted(spec, controls[i].value)) {
			fprintf(stderr, "%s value %d outside apple-ave range\n",
				controls[i].name, controls[i].value);
			failures++;
		}
		if (controls[i].id == V4L2_CID_MPEG_VIDEO_B_FRAMES)
			b_frames_zero = controls[i].value == 0;
		/* The exact Venus controls that do not exist on apple-ave. */
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_AU_DELIMITER);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_H264_8X8_TRANSFORM);
		CHECK(controls[i].id != V4L2_CID_MPEG_VIDEO_HEVC_TIER);
	}
	/* apple-ave defaults to 3 B-frames; VA low-delay needs 0. */
	CHECK(b_frames_zero);
}

static void units_controls(void)
{
	struct ave_control controls[AVE_MAX_CONTROLS];
	struct ave_encoder_config c;
	size_t count;

	c = base_config(false);
	check_controls_supported(&c);
	c.bitrate_mode = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
	c.bitrate_peak = 4000000;
	check_controls_supported(&c);
	c.h264_profile = V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE;
	c.qp_range = false;
	check_controls_supported(&c);

	c = base_config(true);
	check_controls_supported(&c);
#if HAVE_V4L2_PIX_FMT_P010
	c.raw_format = V4L2_PIX_FMT_P010;
	c.hevc_profile = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10;
	c.hevc_level = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1;
	check_controls_supported(&c);
#endif

	/* Values apple-ave would silently clamp are refused instead. */
	c = base_config(false);
	c.bitrate = 50000;
	CHECK(ave_encoder_controls(&c, controls, AVE_MAX_CONTROLS, &count) == -ERANGE);
	c = base_config(false);
	/* V4L2_MPEG_VIDEO_BITRATE_MODE_CQ, an enum value from Linux 5.10. */
	c.bitrate_mode = 2;
	CHECK(ave_encoder_controls(&c, controls, AVE_MAX_CONTROLS, &count) == -ENOTSUP);
	c = base_config(false);
	c.gop_size = 0;
	CHECK(ave_encoder_controls(&c, controls, AVE_MAX_CONTROLS, &count) == -ERANGE);
	c = base_config(true);
	c.hevc_level = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2;
	CHECK(ave_encoder_controls(&c, controls, AVE_MAX_CONTROLS, &count) == -ENOTSUP);
#if HAVE_V4L2_PIX_FMT_P010
	c = base_config(true);
	c.raw_format = V4L2_PIX_FMT_P010;	/* Main profile with P010 input */
	CHECK(ave_encoder_controls(&c, controls, AVE_MAX_CONTROLS, &count) == -EINVAL);
#endif
}

static void units_rate(void)
{
	struct ave_rate_request request = {0};
	struct ave_rate_settings settings;
	VAEncMiscParameterRateControl rc = {0};
	VAEncMiscParameterFrameRate fr = {0};
	VAEncMiscParameterHRD hrd = {0};

	rc.bits_per_second = 2000000;
	rc.target_percentage = 100;
	rc.min_qp = 1;
	rc.max_qp = 42;
	rc.rc_flags.bits.disable_frame_skip = 1;
	CHECK(ave_parse_misc(VAEncMiscParameterTypeRateControl, &rc, sizeof(rc),
			     &request) == 0);
	fr.framerate = 30;
	CHECK(ave_parse_misc(VAEncMiscParameterTypeFrameRate, &fr, sizeof(fr),
			     &request) == 0);
	CHECK(request.frames_per_second == 30);
	fr.framerate = 30000 | (1001u << 16);
	CHECK(ave_parse_misc(VAEncMiscParameterTypeFrameRate, &fr, sizeof(fr),
			     &request) == 0);
	CHECK(request.frames_per_second == 30);
	CHECK(ave_parse_misc(VAEncMiscParameterTypeHRD, &hrd, sizeof(hrd),
			     &request) == 0);
	CHECK(ave_parse_misc(VAEncMiscParameterTypeROI, &hrd, sizeof(hrd),
			     &request) == -ENOTSUP);
	CHECK(ave_parse_misc(VAEncMiscParameterTypeMaxFrameSize, &hrd, sizeof(hrd),
			     &request) == -ENOTSUP);

	/* CBR: constant bit rate, no peak, QP clamp from VA. */
	CHECK(ave_rate_settings_from_va(VA_RC_CBR, &request, 0, &settings) == 0);
	CHECK(settings.v4l2_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	CHECK(settings.bitrate == 2000000 && settings.peak == 0);
	CHECK(settings.qp_range && settings.min_qp == 1 && settings.max_qp == 42);

	/* VBR: VA bits_per_second is the peak, target a percentage of it. */
	request.target_percentage = 50;
	request.bits_per_second = 8000000;
	CHECK(ave_rate_settings_from_va(VA_RC_VBR, &request, 0, &settings) == 0);
	CHECK(settings.v4l2_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
	CHECK(settings.bitrate == 4000000 && settings.peak == 8000000);
	request.target_percentage = 100;
	CHECK(ave_rate_settings_from_va(VA_RC_VBR, &request, 0, &settings) == 0);
	CHECK(settings.bitrate == 8000000 && settings.peak == 0);

	CHECK(ave_rate_settings_from_va(VA_RC_CQP, &request, 0, &settings) == -ENOTSUP);
	{
		struct ave_rate_request none = {0};

		CHECK(ave_rate_settings_from_va(VA_RC_CBR, &none, 0, &settings) == -EINVAL);
		CHECK(ave_rate_settings_from_va(VA_RC_CBR, &none, 3000000,
						&settings) == 0);
		CHECK(settings.bitrate == 3000000 && !settings.qp_range);
	}

	rc.rc_flags.bits.disable_bit_stuffing = 1;
	CHECK(ave_parse_misc(VAEncMiscParameterTypeRateControl, &rc, sizeof(rc),
			     &request) == -ENOTSUP);
	rc.rc_flags.value = 0;
	rc.rc_flags.bits.temporal_id = 1;
	CHECK(ave_parse_misc(VAEncMiscParameterTypeRateControl, &rc, sizeof(rc),
			     &request) == -ENOTSUP);

	CHECK(ave_gop_from_va(2048, 0) == 2048);
	CHECK(ave_gop_from_va(0, 120) == 120);
	CHECK(ave_gop_from_va(0, 0) == 65535);
	CHECK(ave_gop_from_va(100000, 0) == 65535);
	CHECK(ave_gop_from_va(1, 0) == 1);
}

static void units_pack(void)
{
	/* 1920x1080 NV12: AVE stride 1920, chroma at stride*1080 (not 1088). */
	const uint32_t width = 1920, height = 1080, src_stride = 2048;
	const uint32_t dst_stride = 1920;
	size_t src_size = (size_t)src_stride * 1088 * 3 / 2;
	size_t dst_size = (size_t)dst_stride * 1088 * 3 / 2;
	uint8_t *src = malloc(src_size);
	uint8_t *dst = calloc(1, dst_size);
	size_t uv_offset = (size_t)src_stride * 1088;	/* padded source */
	size_t packed = 0;

	CHECK(src && dst);
	if (!src || !dst)
		goto out;
	for (uint32_t y = 0; y < 1088; y++)
		for (uint32_t x = 0; x < src_stride; x++)
			src[(size_t)y * src_stride + x] = (uint8_t)(y * 7 + x);
	for (uint32_t y = 0; y < 544; y++)
		for (uint32_t x = 0; x < src_stride; x++)
			src[uv_offset + (size_t)y * src_stride + x] =
				(uint8_t)(0x80 ^ (y * 3 + x));

	CHECK(ave_pack_frame(dst, dst_size, dst_stride, src, src + uv_offset,
			     src_stride, width, height, 1, &packed) == 0);
	CHECK(packed == (size_t)dst_stride * height * 3 / 2);
	for (uint32_t y = 0; y < height; y += 97)
		CHECK(!memcmp(dst + (size_t)y * dst_stride,
			      src + (size_t)y * src_stride, width));
	for (uint32_t y = 0; y < height / 2; y += 53)
		CHECK(!memcmp(dst + (size_t)dst_stride * height + (size_t)y * dst_stride,
			      src + uv_offset + (size_t)y * src_stride, width));
	/* Too small a destination is refused, not overrun. */
	CHECK(ave_pack_frame(dst, packed - 1, dst_stride, src, src + uv_offset,
			     src_stride, width, height, 1, &packed) == -ENOSPC);
	/* P010: two bytes per sample. */
	CHECK(ave_pack_frame(dst, dst_size, 1920 * 2, src, src + uv_offset,
			     src_stride, 640, 360, 2, &packed) == 0);
	CHECK(packed == (size_t)1920 * 2 * 360 * 3 / 2);
out:
	free(src);
	free(dst);
}

static void units_import_layouts(void)
{
	VASurfaceAttribExternalBuffers ext = {0};
	VADRMPRIMESurfaceDescriptor prime = {0};
	struct ave_import_layout layout;
	uintptr_t fds[1] = { 5 };

	/* Chrome legacy DRM_PRIME: one fd, two planes (vaapi_wrapper.cc). */
	ext.pixel_format = VA_FOURCC_NV12;
	ext.width = 1280;
	ext.height = 720;
	ext.num_planes = 2;
	ext.pitches[0] = ext.pitches[1] = 1280;
	ext.offsets[0] = 0;
	ext.offsets[1] = 1280 * 736;
	ext.data_size = 1280 * 736 * 3 / 2;
	ext.buffers = fds;
	ext.num_buffers = 1;
	CHECK(ave_import_layout_from_external(&ext, 1280, 720, VA_FOURCC_NV12,
					      &layout) == 0);
	CHECK(layout.stride == 1280 && layout.uv_offset == 1280 * 736 && layout.fd == 5);
	ext.flags = VA_SURFACE_EXTBUF_DESC_ENABLE_TILING;
	CHECK(ave_import_layout_from_external(&ext, 1280, 720, VA_FOURCC_NV12,
					      &layout) == -ENOTSUP);
	ext.flags = 0;
	ext.offsets[1] = 1280 * 700;	/* chroma overlapping luma */
	CHECK(ave_import_layout_from_external(&ext, 1280, 720, VA_FOURCC_NV12,
					      &layout) == -EINVAL);
	ext.offsets[1] = 1280 * 736;
	ext.pitches[1] = 1536;
	CHECK(ave_import_layout_from_external(&ext, 1280, 720, VA_FOURCC_NV12,
					      &layout) == -EINVAL);

	prime.fourcc = VA_FOURCC_NV12;
	prime.width = 1280;
	prime.height = 720;
	prime.num_objects = 1;
	prime.objects[0].fd = 7;
	prime.objects[0].size = 1280 * 720 * 3 / 2;
	prime.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
	prime.num_layers = 2;
	prime.layers[0].drm_format = DRM_FORMAT_R8;
	prime.layers[0].num_planes = 1;
	prime.layers[0].pitch[0] = 1280;
	prime.layers[1].drm_format = DRM_FORMAT_GR88;
	prime.layers[1].num_planes = 1;
	prime.layers[1].offset[0] = 1280 * 720;
	prime.layers[1].pitch[0] = 1280;
	CHECK(ave_import_layout_from_prime2(&prime, 1280, 720, VA_FOURCC_NV12,
					    &layout) == 0);
	prime.objects[0].drm_format_modifier = DRM_FORMAT_MOD_INVALID;
	CHECK(ave_import_layout_from_prime2(&prime, 1280, 720, VA_FOURCC_NV12,
					    &layout) == -ENOTSUP);
	prime.objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
	prime.objects[0].size = 1280 * 720;	/* chroma beyond the object */
	CHECK(ave_import_layout_from_prime2(&prime, 1280, 720, VA_FOURCC_NV12,
					    &layout) == -EINVAL);
}

static void write_name(const char *root, unsigned int index, const char *name)
{
	char path[512];
	FILE *file;

	snprintf(path, sizeof(path), "%s/video%u", root, index);
	CHECK(mkdir(path, 0700) == 0);
	snprintf(path, sizeof(path), "%s/video%u/name", root, index);
	file = fopen(path, "w");
	CHECK(file != NULL);
	if (file) {
		fputs(name, file);
		fclose(file);
	}
}

static void units_probe_filter(void)
{
	char root[] = "/tmp/ave-probe-test-XXXXXX";
	unsigned int indices[8];
	struct ave_device device;
	size_t count;

	CHECK(ave_probe_name_matches("apple-ave\n"));
	CHECK(ave_probe_name_matches("apple-ave"));
	CHECK(!ave_probe_name_matches("apple-avex\n"));
	CHECK(!ave_probe_name_matches("avd\n"));
	CHECK(!ave_probe_name_matches("apple-isp\n"));

	if (!mkdtemp(root)) {
		CHECK(!"mkdtemp");
		return;
	}
	/* The M4 Air layout: ISP, AVD decoder, AVE encoder at video10. */
	write_name(root, 0, "apple-isp\n");
	write_name(root, 1, "avd\n");
	write_name(root, 10, "apple-ave\n");
	count = ave_probe_select_nodes(root, 64, indices, 8);
	CHECK(count == 1 && indices[0] == 10);
	/* An explicit node is still checked against sysfs before opening. */
	CHECK(ave_probe(&device, root, "/nonexistent/video", 64,
			"/nonexistent/video1") == -ENODEV);
	CHECK(!device.present);
	for (unsigned int i = 0; i <= 10; i++) {
		char path[512];

		snprintf(path, sizeof(path), "%s/video%u/name", root, i);
		unlink(path);
		snprintf(path, sizeof(path), "%s/video%u", root, i);
		rmdir(path);
	}
	CHECK(rmdir(root) == 0);
}

static void case_units(void)
{
	units_controls();
	units_rate();
	units_pack();
	units_import_layouts();
	units_probe_filter();
}

/* ---- capability answers with and without an encoder ------------------- */

static bool lists_profile(VAProfile profile)
{
	VAProfile profiles[V4L2R_MAX_PROFILES];
	int count = 0;

	CHECK(vtable.vaQueryConfigProfiles(&driver, profiles, &count) ==
	      VA_STATUS_SUCCESS);
	for (int i = 0; i < count; i++) {
		if (profiles[i] == profile)
			return true;
	}
	return false;
}

static int entrypoints(VAProfile profile, VAEntrypoint *list)
{
	int count = -1;

	CHECK(vtable.vaQueryConfigEntrypoints(&driver, profile, list, &count) ==
	      VA_STATUS_SUCCESS);
	return count;
}

static void case_capabilities(void)
{
	VAEntrypoint list[V4L2R_MAX_ENTRYPOINTS];
	VAConfigAttrib attributes[3] = {
		{ .type = VAConfigAttribRTFormat },
		{ .type = VAConfigAttribRateControl },
		{ .type = VAConfigAttribEncPackedHeaders },
	};
	VASurfaceAttrib surface_attributes[16];
	unsigned int count = 16;
	VAConfigID config;
	VAProfile with[V4L2R_MAX_PROFILES], without[V4L2R_MAX_PROFILES];
	int with_count = 0, without_count = 0;

	/* Without an encoder: the decode-only answers. */
	setup(NULL);
	CHECK(vtable.vaQueryConfigProfiles(&driver, without, &without_count) ==
	      VA_STATUS_SUCCESS);
#if HAVE_V4L2_CTRL_H264
	CHECK(entrypoints(VAProfileH264High, list) == 1 && list[0] == VAEntrypointVLD);
	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264High, VAEntrypointEncSlice,
				    NULL, 0, &config) ==
	      VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT);
	/* GetConfigAttributes ignores the entrypoint as before. */
	CHECK(vtable.vaGetConfigAttributes(&driver, VAProfileH264High,
					   VAEntrypointEncSlice, attributes, 3) ==
	      VA_STATUS_SUCCESS);
	CHECK(attributes[0].value == VA_RT_FORMAT_YUV420);
	CHECK(attributes[1].value == VA_ATTRIB_NOT_SUPPORTED);
	CHECK(attributes[2].value == VA_ATTRIB_NOT_SUPPORTED);
#endif
	CHECK(!drv->ave.present);
	teardown();

	/* With the encoder: the same decode list, plus EncSlice entrypoints. */
	setup(FAKE_AVE_PATH);
	CHECK(vtable.vaQueryConfigProfiles(&driver, with, &with_count) ==
	      VA_STATUS_SUCCESS);
	CHECK(lists_profile(VAProfileH264ConstrainedBaseline));
	CHECK(lists_profile(VAProfileH264Main));
	CHECK(lists_profile(VAProfileH264High));
	CHECK(lists_profile(VAProfileHEVCMain));
	CHECK(lists_profile(VAProfileHEVCMain10));
	/* Decode profiles keep their order ahead of the encode-only ones. */
	CHECK(with_count >= without_count);
	for (int i = 0; i + 1 < without_count; i++)
		CHECK(with[i] == without[i]);
	CHECK(with[with_count - 1] == VAProfileNone);
#if HAVE_V4L2_CTRL_H264
	CHECK(entrypoints(VAProfileH264High, list) == 2 &&
	      list[0] == VAEntrypointVLD && list[1] == VAEntrypointEncSlice);
#else
	CHECK(entrypoints(VAProfileH264High, list) == 1 &&
	      list[0] == VAEntrypointEncSlice);
#endif
	CHECK(entrypoints(VAProfileVP9Profile0, list) == 0 ||
	      list[0] == VAEntrypointVLD);

	CHECK(vtable.vaGetConfigAttributes(&driver, VAProfileHEVCMain10,
					   VAEntrypointEncSlice, attributes, 3) ==
	      VA_STATUS_SUCCESS);
	CHECK(attributes[0].value == VA_RT_FORMAT_YUV420_10);
	CHECK(attributes[1].value == (VA_RC_CBR | VA_RC_VBR));
	CHECK(attributes[2].value == VA_ENC_PACKED_HEADER_NONE);

	CHECK(vtable.vaCreateConfig(&driver, VAProfileHEVCMain10, VAEntrypointEncSlice,
				    NULL, 0, &config) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaQuerySurfaceAttributes(&driver, config, surface_attributes,
					      &count) == VA_STATUS_SUCCESS);
	CHECK(count == 8);
	CHECK(surface_attributes[0].type == VASurfaceAttribPixelFormat &&
	      surface_attributes[0].value.value.i == VA_FOURCC_P010);
	CHECK(surface_attributes[1].type == VASurfaceAttribMemoryType &&
	      (surface_attributes[1].value.value.i &
	       VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME));
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
	teardown();
}

/* Encode buffer types and imports keep their old answers without AVE. */
static void case_decode_isolation(void)
{
	VASurfaceAttribExternalBuffers ext = {0};
	VASurfaceAttrib attributes[2];
	uintptr_t fds[1];
	VASurfaceID surface;
	VABufferID buffer;
	struct v4l2r_surface *object;
	int fd = memfd_create("ave-isolation", MFD_CLOEXEC);

	setup(NULL);
	CHECK(fd >= 0 && ftruncate(fd, 1280 * 736 * 3 / 2) == 0);
	fds[0] = (uintptr_t)fd;
	ext.pixel_format = VA_FOURCC_NV12;
	ext.width = 1280;
	ext.height = 720;
	ext.num_planes = 2;
	ext.pitches[0] = ext.pitches[1] = 1280;
	ext.offsets[1] = 1280 * 736;
	ext.buffers = fds;
	ext.num_buffers = 1;
	memset(attributes, 0, sizeof(attributes));
	attributes[0].type = VASurfaceAttribMemoryType;
	attributes[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
	attributes[0].value.type = VAGenericValueTypeInteger;
	attributes[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
	attributes[1].type = VASurfaceAttribExternalBufferDescriptor;
	attributes[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
	attributes[1].value.type = VAGenericValueTypePointer;
	attributes[1].value.value.p = &ext;
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420, 1280, 720,
				       &surface, 1, attributes, 2) ==
	      VA_STATUS_SUCCESS);
	object = V4L2R_SURFACE(drv, surface);
	CHECK(object && !object->import);
	CHECK(vtable.vaCreateBuffer(&driver, VA_INVALID_ID, VAEncCodedBufferType,
				    4096, 1, NULL, &buffer) ==
	      VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE);
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
	close(fd);
	teardown();
}

/* ---- Chrome gating (test_chrome_gating.c) ------------------------------ */

struct chrome_profile_info {
	bool supported;
	int min_w, min_h, max_w, max_h;
	bool yuv420;
};

/*
 * One iteration of Chrome 156 VASupportedProfiles::FillSupportedProfileInfos
 * for (mode, profile): GetRequiredAttribs, AreAttribsSupported,
 * FillProfileInfo_Locked (media/gpu/vaapi/vaapi_wrapper.cc).
 */
static struct chrome_profile_info chrome_fill(VAProfile profile,
					      uint32_t rate_control)
{
	struct chrome_profile_info info = {0};
	VAEntrypoint list[V4L2R_MAX_ENTRYPOINTS];
	VAConfigAttrib required[3];
	VAConfigAttrib query[3];
	VAConfigAttrib packed = { .type = VAConfigAttribEncPackedHeaders };
	VASurfaceAttrib surface_attributes[16];
	VAConfigAttrib config_attributes[V4L2R_MAX_CONFIG_ATTRIBUTES];
	unsigned int num_surface_attributes = 0;
	int num_config_attributes = 0;
	VAProfile queried_profile;
	VAEntrypoint queried_entrypoint;
	VAConfigID config;
	int num = 0;
	bool has_encslice = false;
	uint32_t wanted = VA_ENC_PACKED_HEADER_SEQUENCE |
			  VA_ENC_PACKED_HEADER_PICTURE | VA_ENC_PACKED_HEADER_SLICE;

	if (vtable.vaQueryConfigEntrypoints(&driver, profile, list, &num) !=
	    VA_STATUS_SUCCESS)
		return info;
	for (int i = 0; i < num; i++)
		has_encslice |= list[i] == VAEntrypointEncSlice;
	if (!has_encslice)
		return info;

	required[0] = (VAConfigAttrib) { VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420 };
	required[1] = (VAConfigAttrib) { VAConfigAttribRateControl, rate_control };
	if (vtable.vaGetConfigAttributes(&driver, profile, VAEntrypointEncSlice,
					 &packed, 1) != VA_STATUS_SUCCESS)
		return info;
	required[2] = (VAConfigAttrib) {
		VAConfigAttribEncPackedHeaders,
		(packed.value & wanted) == wanted ? wanted : VA_ENC_PACKED_HEADER_NONE,
	};

	memcpy(query, required, sizeof(query));
	for (int i = 0; i < 3; i++)
		query[i].value = 0;
	if (vtable.vaGetConfigAttributes(&driver, profile, VAEntrypointEncSlice,
					 query, 3) != VA_STATUS_SUCCESS)
		return info;
	for (int i = 0; i < 3; i++) {
		if (query[i].type != required[i].type ||
		    (query[i].value & required[i].value) != required[i].value)
			return info;
	}

	if (vtable.vaCreateConfig(&driver, profile, VAEntrypointEncSlice, required,
				  3, &config) != VA_STATUS_SUCCESS)
		return info;
	if (vtable.vaQuerySurfaceAttributes(&driver, config, NULL,
					    &num_surface_attributes) !=
	    VA_STATUS_SUCCESS ||
	    num_surface_attributes == 0 || num_surface_attributes > 16 ||
	    vtable.vaQuerySurfaceAttributes(&driver, config, surface_attributes,
					    &num_surface_attributes) !=
	    VA_STATUS_SUCCESS) {
		vtable.vaDestroyConfig(&driver, config);
		return info;
	}
	for (unsigned int i = 0; i < num_surface_attributes; i++) {
		int value = surface_attributes[i].value.value.i;

		switch (surface_attributes[i].type) {
		case VASurfaceAttribMaxWidth: info.max_w = value; break;
		case VASurfaceAttribMaxHeight: info.max_h = value; break;
		case VASurfaceAttribMinWidth: info.min_w = value; break;
		case VASurfaceAttribMinHeight: info.min_h = value; break;
		default: break;
		}
	}
	vtable.vaDestroyConfig(&driver, config);
	if (!info.max_w || !info.max_h)
		return info;
	/* min_resolution.SetToMax(16x16) then QVGA+1 for encoders. */
	if (info.min_w < 321) info.min_w = 321;
	if (info.min_h < 241) info.min_h = 241;

	if (vtable.vaCreateConfig(&driver, profile, VAEntrypointEncSlice, NULL, 0,
				  &config) != VA_STATUS_SUCCESS)
		return info;
	if (vtable.vaQueryConfigAttributes(&driver, config, &queried_profile,
					   &queried_entrypoint, config_attributes,
					   &num_config_attributes) == VA_STATUS_SUCCESS) {
		for (int i = 0; i < num_config_attributes; i++) {
			if (config_attributes[i].type == VAConfigAttribRTFormat)
				info.yuv420 = (config_attributes[i].value &
					       VA_RT_FORMAT_YUV420) != 0;
		}
	}
	vtable.vaDestroyConfig(&driver, config);
	info.supported = info.yuv420;
	return info;
}

static void chrome_profile_gating(void)
{
	/* Chrome kSupportableEncoderProfiles (vaapi_wrapper.cc). */
	static const VAProfile profiles[] = {
		VAProfileH264ConstrainedBaseline, VAProfileH264Main, VAProfileH264High,
		VAProfileVP8Version0_3, VAProfileVP9Profile0,
#if VA_CHECK_VERSION(1, 8, 0)
		VAProfileAV1Profile0,
#endif
	};

	for (size_t i = 0; i < sizeof(profiles) / sizeof(profiles[0]); i++) {
		VAProfile profile = profiles[i];
		bool h264 = profile == VAProfileH264ConstrainedBaseline ||
			    profile == VAProfileH264Main || profile == VAProfileH264High;
		struct chrome_profile_info cbr, vbr, cqp;

		if (!lists_profile(profile)) {
			CHECK(!h264);
			continue;
		}
		cbr = chrome_fill(profile, VA_RC_CBR);
		vbr = chrome_fill(profile, VA_RC_VBR);
		cqp = chrome_fill(profile, VA_RC_CQP);
		/* kEncodeConstantBitrate drives GetSupportedEncodeProfiles. */
		CHECK(cbr.supported == h264);
		/* Driver-side VBR works; Chrome blocks it without ChromeOSHWVBREncoding. */
		CHECK(vbr.supported == h264);
		/* AVE has no constant-QP mode: must not be advertised. */
		CHECK(!cqp.supported);
		if (h264) {
			CHECK(cbr.max_w == 4096 && cbr.max_h == 2304);
			CHECK(cbr.min_w == 321 && cbr.min_h == 241);
		}
	}

	/* HEVC is listed for FFmpeg-style clients, but Chrome never asks. */
	CHECK(lists_profile(VAProfileHEVCMain));
	CHECK(lists_profile(VAProfileHEVCMain10));
}

/* A Chrome-shaped 1280x720 picture (h264_vaapi_video_encoder_delegate.cc). */
static VAStatus submit_h264(VAContextID context, VASurfaceID surface,
			    VABufferID coded, bool packed, int slice_type,
			    int slices, bool idr)
{
	VAEncSequenceParameterBufferH264 sequence = {0};
	VAEncPictureParameterBufferH264 picture = {0};
	VAEncSliceParameterBufferH264 slice = {0};
	VAEncMiscParameterRateControl rate = {0};
	VAEncMiscParameterFrameRate frame_rate = {0};
	VAEncMiscParameterHRD hrd = {0};
	VABufferID buffers[10];
	int count = 0;

	sequence.level_idc = 31;
	sequence.intra_period = 0;
	sequence.intra_idr_period = 2048;
	sequence.ip_period = 1;
	sequence.bits_per_second = 2000000;
	sequence.picture_width_in_mbs = 80;
	sequence.picture_height_in_mbs = 45;
	sequence.seq_fields.bits.chroma_format_idc = 1;
	sequence.seq_fields.bits.frame_mbs_only_flag = 1;
	picture.coded_buf = coded;
	picture.pic_fields.bits.idr_pic_flag = idr;
	picture.pic_fields.bits.entropy_coding_mode_flag = 1;
	picture.pic_init_qp = 26;
	slice.num_macroblocks = 80 * 45;
	slice.macroblock_info = VA_INVALID_ID;
	slice.slice_type = slice_type;
	rate.bits_per_second = 2000000;
	rate.target_percentage = 100;
	rate.min_qp = 1;
	rate.max_qp = 42;
	rate.rc_flags.bits.disable_frame_skip = 1;
	frame_rate.framerate = 30;
	hrd.buffer_size = 3000000;

	buffers[count++] = make_buffer(context, VAEncSequenceParameterBufferType,
				       sizeof(sequence), &sequence);
	buffers[count++] = make_buffer(context, VAEncPictureParameterBufferType,
				       sizeof(picture), &picture);
	for (int i = 0; i < slices; i++)
		buffers[count++] = make_buffer(context, VAEncSliceParameterBufferType,
					       sizeof(slice), &slice);
	buffers[count++] = make_misc(context, VAEncMiscParameterTypeRateControl,
				     &rate, sizeof(rate));
	buffers[count++] = make_misc(context, VAEncMiscParameterTypeFrameRate,
				     &frame_rate, sizeof(frame_rate));
	buffers[count++] = make_misc(context, VAEncMiscParameterTypeHRD, &hrd,
				     sizeof(hrd));
	if (packed) {
		VAEncPackedHeaderParameterBuffer header = {
			.type = VAEncPackedHeaderSequence,
			.bit_length = 32,
		};
		uint8_t bytes[4] = { 0, 0, 0, 1 };

		buffers[count++] = make_buffer(context,
					       VAEncPackedHeaderParameterBufferType,
					       sizeof(header), &header);
		buffers[count++] = make_buffer(context, VAEncPackedHeaderDataBufferType,
					       sizeof(bytes), bytes);
	}

	CHECK(vtable.vaBeginPicture(&driver, context, surface) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaRenderPicture(&driver, context, buffers, count) ==
	      VA_STATUS_SUCCESS);
	VAStatus status = vtable.vaEndPicture(&driver, context);

	for (int i = 0; i < count; i++)
		vtable.vaDestroyBuffer(&driver, buffers[i]);
	return status;
}

static void chrome_encode_validation(void)
{
	VAConfigAttrib attributes[3] = {
		{ VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420 },
		{ VAConfigAttribRateControl, VA_RC_CBR },
		{ VAConfigAttribEncPackedHeaders, VA_ENC_PACKED_HEADER_NONE },
	};
	VAConfigAttrib bad_packed[2] = {
		{ VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420 },
		{ VAConfigAttribEncPackedHeaders, VA_ENC_PACKED_HEADER_SEQUENCE },
	};
	VAConfigAttrib cqp[1] = { { VAConfigAttribRateControl, VA_RC_CQP } };
	VAConfigID config, rejected;
	VAContextID context;
	VASurfaceID surface;
	VABufferID coded;

	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264Main, VAEntrypointEncSlice,
				    bad_packed, 2, &rejected) ==
	      VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264Main, VAEntrypointEncSlice,
				    cqp, 1, &rejected) ==
	      VA_STATUS_ERROR_ATTR_NOT_SUPPORTED);
	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264Main, VAEntrypointEncSlice,
				    attributes, 3, &config) == VA_STATUS_SUCCESS);
	/* AVE minimum is 320x240: smaller contexts are refused up front. */
	CHECK(vtable.vaCreateContext(&driver, config, 160, 128, VA_PROGRESSIVE,
				     NULL, 0, &context) ==
	      VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED);
	CHECK(vtable.vaCreateContext(&driver, config, 1280, 720, VA_PROGRESSIVE,
				     NULL, 0, &context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420, 1280, 720,
				       &surface, 1, NULL, 0) == VA_STATUS_SUCCESS);
	coded = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);

	/* Packed headers outside the negotiated config: refused before open. */
	CHECK(submit_h264(context, surface, coded, true, 2, 1, true) ==
	      VA_STATUS_ERROR_UNIMPLEMENTED);
	/* B slices and multi-slice pictures are not supported. */
	CHECK(submit_h264(context, surface, coded, false, 1, 1, true) ==
	      VA_STATUS_ERROR_UNIMPLEMENTED);
	CHECK(submit_h264(context, surface, coded, false, 2, 2, true) ==
	      VA_STATUS_ERROR_UNIMPLEMENTED);
	/* A valid Chrome-shaped picture passes every check and fails only at
	 * opening the (absent) device: ENOENT -> ENCODING_ERROR. */
	CHECK(submit_h264(context, surface, coded, false, 2, 1, true) ==
	      VA_STATUS_ERROR_ENCODING_ERROR);

	CHECK(vtable.vaDestroyBuffer(&driver, coded) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
}

/* Chrome imports WebCodecs frames as legacy DRM_PRIME NV12 buffers
 * (VaapiWrapper::CreateVASurfaceForPixmap for VAImplementation::kOther). */
static VASurfaceID import_nv12(int fd, unsigned int width, unsigned int height,
			       unsigned int pitch, unsigned int chroma_rows,
			       VAStatus *status)
{
	VASurfaceAttribExternalBuffers ext = {0};
	VASurfaceAttrib attributes[2];
	uintptr_t fds[1] = { (uintptr_t)fd };
	VASurfaceID surface = VA_INVALID_ID;

	ext.pixel_format = VA_FOURCC_NV12;
	ext.width = width;
	ext.height = height;
	ext.data_size = pitch * chroma_rows * 3 / 2;
	ext.num_planes = 2;
	ext.pitches[0] = ext.pitches[1] = pitch;
	ext.offsets[1] = pitch * chroma_rows;
	ext.buffers = fds;
	ext.num_buffers = 1;
	memset(attributes, 0, sizeof(attributes));
	attributes[0].type = VASurfaceAttribMemoryType;
	attributes[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
	attributes[0].value.type = VAGenericValueTypeInteger;
	attributes[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
	attributes[1].type = VASurfaceAttribExternalBufferDescriptor;
	attributes[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
	attributes[1].value.type = VAGenericValueTypePointer;
	attributes[1].value.value.p = &ext;
	*status = vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420, width, height,
					   &surface, 1, attributes, 2);
	return surface;
}

static void chrome_import(void)
{
	const unsigned int width = 1280, height = 720, pitch = 1280;
	VADRMPRIMESurfaceDescriptor prime;
	VASurfaceID surface;
	VAStatus status;
	VAImage image;
	int fd = memfd_create("ave-import-test", MFD_CLOEXEC);

	CHECK(fd >= 0 && ftruncate(fd, (off_t)pitch * 736 * 3 / 2) == 0);
	surface = import_nv12(fd, width, height, pitch, 736, &status);
	CHECK(status == VA_STATUS_SUCCESS);
	/* The caller may close its fd; the driver holds its own reference. */
	close(fd);
	/* Read-only import: no derived (writable) image, no export. */
	CHECK(vtable.vaDeriveImage(&driver, surface, &image) ==
	      VA_STATUS_ERROR_OPERATION_FAILED);
	CHECK(vtable.vaExportSurfaceHandle(&driver, surface,
					   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
					   VA_EXPORT_SURFACE_SEPARATE_LAYERS,
					   &prime) != VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
}

/* FFmpeg-shaped HEVC IDR: 1920x1088 coded, 1920x1080 input. */
static VAStatus submit_hevc(VAContextID context, VASurfaceID surface,
			    VABufferID coded, bool high_tier, bool main10,
			    unsigned int frame)
{
	VAEncSequenceParameterBufferHEVC sequence = {0};
	VAEncPictureParameterBufferHEVC picture = {0};
	VAEncSliceParameterBufferHEVC slice = {0};
	VAEncMiscParameterRateControl rate = {0};
	VABufferID buffers[4];
	int count = 0;
	bool idr = frame == 0;

	sequence.general_profile_idc = main10 ? 2 : 1;
	sequence.seq_fields.bits.bit_depth_luma_minus8 = main10 ? 2 : 0;
	sequence.seq_fields.bits.bit_depth_chroma_minus8 = main10 ? 2 : 0;
	sequence.general_level_idc = 123;
	sequence.general_tier_flag = high_tier;
	sequence.intra_idr_period = 120;
	sequence.ip_period = 1;
	sequence.bits_per_second = 3000000;
	sequence.pic_width_in_luma_samples = 1920;
	sequence.pic_height_in_luma_samples = 1088;
	sequence.seq_fields.bits.chroma_format_idc = 1;
	/* FFmpeg's fallback reports min CB 8 here but aligns surfaces to 16. */
	sequence.log2_min_luma_coding_block_size_minus3 = 0;	/* 8 */
	sequence.log2_diff_max_min_luma_coding_block_size = 2;	/* CTU 32 */
	picture.coded_buf = coded;
	picture.pic_fields.bits.idr_pic_flag = idr;
	picture.pic_init_qp = 30;
	slice.num_ctu_in_slice = 60 * 34;
	slice.slice_type = idr ? 2 : 1;
	rate.bits_per_second = 3000000;
	rate.target_percentage = 100;

	if (idr)
		buffers[count++] = make_buffer(context, VAEncSequenceParameterBufferType,
					       sizeof(sequence), &sequence);
	buffers[count++] = make_buffer(context, VAEncPictureParameterBufferType,
				       sizeof(picture), &picture);
	buffers[count++] = make_buffer(context, VAEncSliceParameterBufferType,
				       sizeof(slice), &slice);
	buffers[count++] = make_misc(context, VAEncMiscParameterTypeRateControl,
				     &rate, sizeof(rate));
	CHECK(vtable.vaBeginPicture(&driver, context, surface) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaRenderPicture(&driver, context, buffers, count) ==
	      VA_STATUS_SUCCESS);
	VAStatus status = vtable.vaEndPicture(&driver, context);

	for (int i = 0; i < count; i++)
		vtable.vaDestroyBuffer(&driver, buffers[i]);
	return status;
}

static void chrome_hevc_validation(void)
{
	VAConfigAttrib attributes[2] = {
		{ VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420 },
		{ VAConfigAttribRateControl, VA_RC_CBR },
	};
	VAConfigID config;
	VAContextID context;
	VASurfaceID surface;
	VABufferID coded;

	CHECK(vtable.vaCreateConfig(&driver, VAProfileHEVCMain, VAEntrypointEncSlice,
				    attributes, 2, &config) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateContext(&driver, config, 1920, 1088, VA_PROGRESSIVE,
				     NULL, 0, &context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420, 1920, 1080,
				       &surface, 1, NULL, 0) == VA_STATUS_SUCCESS);
	coded = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	/* No tier control on apple-ave: High tier is refused. */
	CHECK(submit_hevc(context, surface, coded, true, false, 0) ==
	      VA_STATUS_ERROR_UNIMPLEMENTED);
	/* Valid: reaches the (absent) device. */
	CHECK(submit_hevc(context, surface, coded, false, false, 0) ==
	      VA_STATUS_ERROR_ENCODING_ERROR);
	CHECK(vtable.vaDestroyBuffer(&driver, coded) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
}

/*
 * FFmpeg hevc_vaapi -profile:v main10 with "format=p010,hwupload": recon
 * pool 1920x1088 YUV420_10 passed to vaCreateContext, hwupload pool
 * 1920x1080 YUV420_10 + PixelFormat P010, upload through vaDeriveImage +
 * vaMapBuffer (standalone AVD backing here), then Begin/Render/EndPicture.
 */
static void chrome_main10_ffmpeg_sequence(void)
{
	VAConfigAttrib attributes[2] = {
		{ VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420_10 },
		{ VAConfigAttribRateControl, VA_RC_VBR },
	};
	VASurfaceAttrib p010 = {
		.type = VASurfaceAttribPixelFormat,
		.flags = VA_SURFACE_ATTRIB_SETTABLE,
		.value = { .type = VAGenericValueTypeInteger, .value.i = VA_FOURCC_P010 },
	};
	VASurfaceID recon[2];
	VASurfaceID input;
	VAConfigID config;
	VAContextID context;
	VABufferID coded;
	VAImage image;
	void *mapped = NULL;

	CHECK(vtable.vaCreateConfig(&driver, VAProfileHEVCMain10, VAEntrypointEncSlice,
				    attributes, 2, &config) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420_10, 1920, 1088,
				       recon, 2, &p010, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateContext(&driver, config, 1920, 1088, VA_PROGRESSIVE,
				     recon, 2, &context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420_10, 1920, 1080,
				       &input, 1, &p010, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDeriveImage(&driver, input, &image) == VA_STATUS_SUCCESS);
	CHECK(image.format.fourcc == VA_FOURCC_P010);
	CHECK(vtable.vaMapBuffer(&driver, image.buf, &mapped) == VA_STATUS_SUCCESS);
	if (mapped)
		memset(mapped, 0x40, (size_t)image.pitches[0] * 1080);
	CHECK(vtable.vaUnmapBuffer(&driver, image.buf) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyImage(&driver, image.image_id) == VA_STATUS_SUCCESS);
	coded = make_buffer(context, VAEncCodedBufferType, 1 << 21, NULL);
	/* Must reach the (absent) device, like Main does. */
	CHECK(submit_hevc(context, input, coded, false, true, 0) ==
	      VA_STATUS_ERROR_ENCODING_ERROR);
	CHECK(vtable.vaDestroyBuffer(&driver, coded) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, &input, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, recon, 2) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
}

static void case_chrome(void)
{
	setup(MISSING_AVE);
	chrome_profile_gating();
	chrome_encode_validation();
	chrome_import();
	chrome_hevc_validation();
	chrome_main10_ffmpeg_sequence();
	teardown();
}

/* ---- FFmpeg-shaped end to end (test_fake_ave.c) ------------------------ */

enum codec { H264, HEVC, HEVC10 };

/* hwupload: vaDeriveImage + vaMapBuffer, luma 0x11.., chroma 0x22.. */
static void upload(VASurfaceID surface, unsigned int height, bool p010,
		   uint8_t luma, uint8_t chroma)
{
	VAImage image;
	void *mapped = NULL;

	CHECK(vtable.vaDeriveImage(&driver, surface, &image) == VA_STATUS_SUCCESS);
	CHECK(image.format.fourcc == (p010 ? VA_FOURCC_P010 : VA_FOURCC_NV12));
	CHECK(vtable.vaMapBuffer(&driver, image.buf, &mapped) == VA_STATUS_SUCCESS);
	if (mapped) {
		memset(mapped, luma, (size_t)image.pitches[0] * height);
		memset((uint8_t *)mapped + image.offsets[1], chroma,
		       (size_t)image.pitches[1] * (height / 2));
	}
	CHECK(vtable.vaUnmapBuffer(&driver, image.buf) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyImage(&driver, image.image_id) == VA_STATUS_SUCCESS);
}

static VAStatus picture(enum codec codec, VAContextID context, VASurfaceID input,
			VASurfaceID recon, VABufferID coded, unsigned int frame,
			unsigned int width, unsigned int coded_height,
			unsigned int height)
{
	VAEncMiscParameterRateControl rate = {
		.bits_per_second = 16000000, .target_percentage = 50,
	};
	VAEncMiscParameterFrameRate frame_rate = { .framerate = 30 };
	VABufferID buffers[6];
	int n = 0;
	bool idr = frame == 0;

	if (codec == H264) {
		VAEncSequenceParameterBufferH264 seq = {0};
		VAEncPictureParameterBufferH264 pic = {0};
		VAEncSliceParameterBufferH264 slice = {0};

		seq.level_idc = 40;
		seq.intra_idr_period = 60;
		seq.ip_period = 1;
		seq.bits_per_second = 16000000;
		seq.picture_width_in_mbs = width / 16;
		seq.picture_height_in_mbs = coded_height / 16;
		seq.seq_fields.bits.chroma_format_idc = 1;
		seq.seq_fields.bits.frame_mbs_only_flag = 1;
		if (coded_height != height) {
			seq.frame_cropping_flag = 1;
			seq.frame_crop_bottom_offset = (coded_height - height) / 2;
		}
		pic.coded_buf = coded;
		pic.CurrPic.picture_id = recon;
		pic.pic_fields.bits.idr_pic_flag = idr;
		pic.pic_init_qp = 26;
		slice.num_macroblocks = (width / 16) * (coded_height / 16);
		slice.slice_type = idr ? 2 : 0;
		if (idr)
			buffers[n++] = make_buffer(context, VAEncSequenceParameterBufferType,
						   sizeof(seq), &seq);
		buffers[n++] = make_buffer(context, VAEncPictureParameterBufferType,
					   sizeof(pic), &pic);
		buffers[n++] = make_buffer(context, VAEncSliceParameterBufferType,
					   sizeof(slice), &slice);
	} else {
		VAEncSequenceParameterBufferHEVC seq = {0};
		VAEncPictureParameterBufferHEVC pic = {0};
		VAEncSliceParameterBufferHEVC slice = {0};
		bool main10 = codec == HEVC10;

		seq.general_profile_idc = main10 ? 2 : 1;
		seq.general_level_idc = 120;
		seq.intra_idr_period = 60;
		seq.ip_period = 1;
		seq.bits_per_second = 16000000;
		seq.pic_width_in_luma_samples = width;
		seq.pic_height_in_luma_samples = coded_height;
		seq.seq_fields.bits.chroma_format_idc = 1;
		seq.seq_fields.bits.bit_depth_luma_minus8 = main10 ? 2 : 0;
		seq.seq_fields.bits.bit_depth_chroma_minus8 = main10 ? 2 : 0;
		/* FFmpeg fallback: min CB 8 in VA sequence, surface alignment 16. */
		seq.log2_min_luma_coding_block_size_minus3 = 0;
		seq.log2_diff_max_min_luma_coding_block_size = 2;
		pic.coded_buf = coded;
		pic.decoded_curr_pic.picture_id = recon;
		pic.pic_fields.bits.idr_pic_flag = idr;
		pic.pic_init_qp = 30;
		slice.num_ctu_in_slice = (width / 32) * ((coded_height + 31) / 32);
		slice.slice_type = idr ? 2 : 1;
		if (idr)
			buffers[n++] = make_buffer(context, VAEncSequenceParameterBufferType,
						   sizeof(seq), &seq);
		buffers[n++] = make_buffer(context, VAEncPictureParameterBufferType,
					   sizeof(pic), &pic);
		buffers[n++] = make_buffer(context, VAEncSliceParameterBufferType,
					   sizeof(slice), &slice);
	}
	if (idr) {
		buffers[n++] = make_misc(context, VAEncMiscParameterTypeRateControl,
					 &rate, sizeof(rate));
		buffers[n++] = make_misc(context, VAEncMiscParameterTypeFrameRate,
					 &frame_rate, sizeof(frame_rate));
	}
	VAStatus status = VA_STATUS_ERROR_OPERATION_FAILED;

	if (vtable.vaBeginPicture(&driver, context, input) == VA_STATUS_SUCCESS &&
	    vtable.vaRenderPicture(&driver, context, buffers, n) == VA_STATUS_SUCCESS)
		status = vtable.vaEndPicture(&driver, context);
	/* FFmpeg destroys parameter buffers right after EndPicture. */
	for (int i = 0; i < n; i++)
		CHECK(vtable.vaDestroyBuffer(&driver, buffers[i]) == VA_STATUS_SUCCESS);
	return status;
}

static void run(const char *name, enum codec codec, unsigned int width,
		unsigned int coded_height, unsigned int height)
{
	bool p010 = codec == HEVC10;
	unsigned int rt = p010 ? VA_RT_FORMAT_YUV420_10 : VA_RT_FORMAT_YUV420;
	VAProfile profile = codec == H264 ? VAProfileH264High :
			    p010 ? VAProfileHEVCMain10 : VAProfileHEVCMain;
	VAConfigAttrib attributes[2] = {
		{ VAConfigAttribRTFormat, rt },
		{ VAConfigAttribRateControl, VA_RC_VBR },
	};
	VASurfaceAttrib format = {
		.type = VASurfaceAttribPixelFormat,
		.flags = VA_SURFACE_ATTRIB_SETTABLE,
		.value = { .type = VAGenericValueTypeInteger,
			   .value.i = p010 ? VA_FOURCC_P010 : VA_FOURCC_NV12 },
	};
	VASurfaceID recon[3], input[3];
	VABufferID coded[3];
	VAConfigID config;
	VAContextID context;
	unsigned int bps = p010 ? 2 : 1;
	bool lazy_issued = false;

	fprintf(stderr, "-- %s\n", name);
	CHECK(vtable.vaCreateConfig(&driver, profile, VAEntrypointEncSlice,
				    attributes, 2, &config) == VA_STATUS_SUCCESS);
	/* FFmpeg: recon pool at the coded size, upload pool at the visible size. */
	CHECK(vtable.vaCreateSurfaces2(&driver, rt, width, coded_height, recon, 3,
				       &format, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateContext(&driver, config, (int)width, (int)coded_height,
				     VA_PROGRESSIVE, recon, 3, &context) ==
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, rt, width, height, input, 3,
				       &format, 1) == VA_STATUS_SUCCESS);
	for (unsigned int frame = 0; frame < 3; frame++)
		coded[frame] = make_buffer(context, VAEncCodedBufferType, 1 << 22, NULL);

	for (unsigned int frame = 0; frame < 3; frame++) {
		VACodedBufferSegment *segment = NULL;
		VASurfaceStatus surface_status;
		VAStatus status;

		if (frame == 0) {
			/* Lazy: issue every frame before syncing any (async depth). */
			for (unsigned int issue = 0; issue < (lazy ? 3u : 1u); issue++) {
				upload(input[issue], height, p010, 0x11, 0x22);
				status = picture(codec, context, input[issue], recon[issue],
						 coded[issue], issue, width, coded_height,
						 height);
				if (status != VA_STATUS_SUCCESS)
					fprintf(stderr, "%s frame %u: EndPicture -> %d (%s)\n",
						name, issue, status, vaErrorStr(status));
				CHECK(status == VA_STATUS_SUCCESS);
			}
			if (lazy) {
				lazy_issued = true;
				CHECK(vtable.vaQuerySurfaceStatus(&driver, input[2],
								  &surface_status) ==
				      VA_STATUS_SUCCESS);
			}
		}
		if (!lazy && frame > 0) {
			upload(input[frame], height, p010, 0x11, 0x22);
			status = picture(codec, context, input[frame], recon[frame],
					 coded[frame], frame, width, coded_height, height);
			if (status != VA_STATUS_SUCCESS)
				fprintf(stderr, "%s frame %u: EndPicture -> %d (%s)\n",
					name, frame, status, vaErrorStr(status));
			CHECK(status == VA_STATUS_SUCCESS);
		}
		/* The input is not bound to the encode context. */
		CHECK(!V4L2R_SURFACE(drv, input[frame])->ctx);
		if (frame == 1) {
#if VA_CHECK_VERSION(1, 9, 0)
			CHECK(vtable.vaSyncBuffer(&driver, coded[frame],
						  VA_TIMEOUT_INFINITE) ==
			      VA_STATUS_SUCCESS);
#endif
		} else {
			CHECK(vtable.vaSyncSurface(&driver, input[frame]) ==
			      VA_STATUS_SUCCESS);
		}
		CHECK(vtable.vaMapBuffer(&driver, coded[frame], (void **)&segment) ==
		      VA_STATUS_SUCCESS);
		CHECK(segment && segment->size == 7 && segment->next == NULL &&
		      segment->status == 0);
		CHECK(vtable.vaUnmapBuffer(&driver, coded[frame]) == VA_STATUS_SUCCESS);
		CHECK(vtable.vaQuerySurfaceStatus(&driver, input[frame],
						  &surface_status) == VA_STATUS_SUCCESS &&
		      surface_status == VASurfaceReady);
		if (lazy_issued)
			continue;	/* layout already checked in eager mode */
		/* AVE layout: visible size, its stride, chroma after `height` rows. */
		CHECK(ave.last_height == height);
		CHECK(ave.last_stride == ALIGN_UP(width * bps, 64));
		CHECK(ave.last_bytesused == ave.last_sizeimage);
		CHECK(ave.last_luma0 == 0x11 && ave.last_chroma0 == 0x22);
		CHECK(ave.last_luma_end == 0x11 && ave.last_chroma_end == 0x22);
	}
	CHECK(ave.b_frames == 0);
	CHECK(ave.bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
	CHECK(ave.bitrate == 8000000);
	CHECK(ave.src.pixelformat == (p010 ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12));
	/* Re-mapping an already delivered coded buffer is fine (FFmpeg
	 * maps once per packet, Chrome may map again). */
	{
		VACodedBufferSegment *segment = NULL;

		CHECK(vtable.vaMapBuffer(&driver, coded[0], (void **)&segment) ==
		      VA_STATUS_SUCCESS);
		CHECK(vtable.vaUnmapBuffer(&driver, coded[0]) == VA_STATUS_SUCCESS);
	}

	for (unsigned int frame = 0; frame < 3; frame++)
		CHECK(vtable.vaDestroyBuffer(&driver, coded[frame]) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(!ave.open);
	CHECK(vtable.vaDestroySurfaces(&driver, input, 3) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, recon, 3) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
}

static void case_ffmpeg(void)
{
	setup(FAKE_AVE_PATH);
	run("h264 1920x1080 (coded 1088)", H264, 1920, 1088, 1080);
	run("hevc main 1280x720", HEVC, 1280, 720, 720);
	run("hevc main 1920x1080 (coded 1088)", HEVC, 1920, 1088, 1080);
#if HAVE_V4L2_PIX_FMT_P010
	run("hevc main10 p010 1920x1080 (coded 1088)", HEVC10, 1920, 1088, 1080);
	run("hevc main10 p010 1280x720", HEVC10, 1280, 720, 720);
#endif
	lazy = true;
	run("async h264 1920x1080", H264, 1920, 1088, 1080);
#if HAVE_V4L2_PIX_FMT_P010
	run("async hevc main10 p010 1920x1080", HEVC10, 1920, 1088, 1080);
#endif
	lazy = false;
	teardown();
}

/* ---- Chrome-shaped import encode --------------------------------------- */

static void case_import(void)
{
	VAConfigAttrib attributes[3] = {
		{ VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420 },
		{ VAConfigAttribRateControl, VA_RC_CBR },
		{ VAConfigAttribEncPackedHeaders, VA_ENC_PACKED_HEADER_NONE },
	};
	const unsigned int pitch = 1536, rows = 736;
	size_t size = (size_t)pitch * rows * 3 / 2;
	VACodedBufferSegment *segment = NULL;
	VAConfigID config;
	VAContextID context;
	VASurfaceID surface;
	VABufferID coded;
	VAStatus status;
	uint8_t *pixels;
	int fd = memfd_create("ave-import-encode", MFD_CLOEXEC);

	setup(FAKE_AVE_PATH);
	CHECK(fd >= 0 && ftruncate(fd, (off_t)size) == 0);
	pixels = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	CHECK(pixels != MAP_FAILED);
	memset(pixels, 0x33, (size_t)pitch * rows);
	memset(pixels + (size_t)pitch * rows, 0x44, (size_t)pitch * rows / 2);

	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264High, VAEntrypointEncSlice,
				    attributes, 3, &config) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateContext(&driver, config, 1280, 720, VA_PROGRESSIVE,
				     NULL, 0, &context) == VA_STATUS_SUCCESS);
	surface = import_nv12(fd, 1280, 720, pitch, rows, &status);
	CHECK(status == VA_STATUS_SUCCESS);
	close(fd);
	coded = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	dmabuf_syncs = 0;
	CHECK(submit_h264(context, surface, coded, false, 2, 1, true) ==
	      VA_STATUS_SUCCESS);
	/* Read bracketed by DMA_BUF_SYNC start and end. */
	CHECK(dmabuf_syncs == 2);
	CHECK(vtable.vaSyncSurface(&driver, surface) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaMapBuffer(&driver, coded, (void **)&segment) == VA_STATUS_SUCCESS);
	CHECK(segment && segment->size == 7);
	CHECK(vtable.vaUnmapBuffer(&driver, coded) == VA_STATUS_SUCCESS);
	CHECK(ave.last_luma0 == 0x33 && ave.last_chroma0 == 0x44);
	CHECK(ave.last_luma_end == 0x33 && ave.last_chroma_end == 0x44);
	CHECK(ave.bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
	CHECK(ave.bitrate == 2000000);

	/* Chrome destroys the per-frame import once its encode finished. */
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
	/* A second import is tracked as an import (decode BeginPicture and
	 * vaDeriveImage refuse those; see case_chrome for the latter). */
	{
		int fd2 = memfd_create("ave-import-2", MFD_CLOEXEC);
		VASurfaceID second;

		CHECK(fd2 >= 0 && ftruncate(fd2, (off_t)size) == 0);
		second = import_nv12(fd2, 1280, 720, pitch, rows, &status);
		CHECK(status == VA_STATUS_SUCCESS);
		close(fd2);
		CHECK(V4L2R_SURFACE(drv, second)->import != NULL);
		CHECK(vtable.vaDestroySurfaces(&driver, &second, 1) == VA_STATUS_SUCCESS);
	}
	munmap(pixels, size);
	CHECK(vtable.vaDestroyBuffer(&driver, coded) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyConfig(&driver, config) == VA_STATUS_SUCCESS);
	teardown();
}

/* ---- one surface, both paths; teardown orders -------------------------- */

static void case_lifetime(void)
{
	VAConfigAttrib attributes[1] = { { VAConfigAttribRateControl, VA_RC_CBR } };
	VASurfaceID surface;
	VAConfigID config;
	VAContextID context;
	VABufferID coded[2];
	VACodedBufferSegment *segment;
	VAImageFormat nv12 = { .fourcc = VA_FOURCC_NV12 };
	VAImage image;
	uint8_t *data = NULL;

	setup(FAKE_AVE_PATH);
	CHECK(vtable.vaCreateConfig(&driver, VAProfileH264High, VAEntrypointEncSlice,
				    attributes, 1, &config) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateContext(&driver, config, 1280, 720, VA_PROGRESSIVE,
				     NULL, 0, &context) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaCreateSurfaces2(&driver, VA_RT_FORMAT_YUV420, 1280, 720,
				       &surface, 1, NULL, 0) == VA_STATUS_SUCCESS);
	coded[0] = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	coded[1] = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	/* Coded buffers cannot be resized under their segment. */
	CHECK(vtable.vaBufferSetNumElements(&driver, coded[0], 2) ==
	      VA_STATUS_ERROR_INVALID_BUFFER);
#if VA_CHECK_VERSION(1, 9, 0)
	/* FFmpeg probes vaSyncBuffer with an invalid ID. */
	CHECK(vtable.vaSyncBuffer(&driver, VA_INVALID_ID, 0) ==
	      VA_STATUS_ERROR_INVALID_BUFFER);
#endif

	/* Upload with PutImage, encode, then read the same surface back. */
	CHECK(vtable.vaCreateImage(&driver, &nv12, 1280, 720, &image) ==
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaMapBuffer(&driver, image.buf, (void **)&data) ==
	      VA_STATUS_SUCCESS);
	memset(data, 0x55, image.offsets[1]);
	memset(data + image.offsets[1], 0x66, image.data_size - image.offsets[1]);
	CHECK(vtable.vaUnmapBuffer(&driver, image.buf) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaPutImage(&driver, surface, image.image_id, 0, 0, 1280, 720,
				0, 0, 1280, 720) == VA_STATUS_SUCCESS);

	lazy = true;
	CHECK(submit_h264(context, surface, coded[0], false, 2, 1, true) ==
	      VA_STATUS_SUCCESS);
	CHECK(V4L2R_SURFACE(drv, surface)->encode_pending);
	/* Reusing a pending input syncs it first. */
	CHECK(submit_h264(context, surface, coded[1], false, 0, 1, false) ==
	      VA_STATUS_SUCCESS);
	CHECK(ave.last_luma0 == 0x55 && ave.last_chroma0 == 0x66);
	/* The pixels stay readable through the decode-side image path. */
	memset(data, 0, image.data_size);
	CHECK(vtable.vaGetImage(&driver, surface, 0, 0, 1280, 720, image.image_id) ==
	      VA_STATUS_SUCCESS);
	CHECK(data[0] == 0x55 && data[image.offsets[1]] == 0x66);
	/* Destroying a coded buffer still in flight waits for it. */
	CHECK(vtable.vaDestroyBuffer(&driver, coded[1]) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaMapBuffer(&driver, coded[0], (void **)&segment) ==
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaUnmapBuffer(&driver, coded[0]) == VA_STATUS_SUCCESS);

	/* A pending encode at context teardown: the session closes, the input
	 * is released and the coded buffer stays destroyable. */
	coded[1] = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	CHECK(submit_h264(context, surface, coded[1], false, 0, 1, false) ==
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyContext(&driver, context) == VA_STATUS_SUCCESS);
	CHECK(!ave.open);
	CHECK(!V4L2R_SURFACE(drv, surface)->encode_pending);
	CHECK(vtable.vaMapBuffer(&driver, coded[1], (void **)&segment) !=
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyBuffer(&driver, coded[1]) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroyBuffer(&driver, coded[0]) == VA_STATUS_SUCCESS);
	lazy = false;

	/* A new context: an input pending at vaDestroySurfaces is synced. */
	CHECK(vtable.vaCreateContext(&driver, config, 1280, 720, VA_PROGRESSIVE,
				     NULL, 0, &context) == VA_STATUS_SUCCESS);
	coded[0] = make_buffer(context, VAEncCodedBufferType, 1 << 20, NULL);
	lazy = true;
	CHECK(submit_h264(context, surface, coded[0], false, 2, 1, true) ==
	      VA_STATUS_SUCCESS);
	CHECK(vtable.vaDestroySurfaces(&driver, &surface, 1) == VA_STATUS_SUCCESS);
	CHECK(vtable.vaMapBuffer(&driver, coded[0], (void **)&segment) ==
	      VA_STATUS_SUCCESS);
	CHECK(segment && segment->size == 7);
	CHECK(vtable.vaUnmapBuffer(&driver, coded[0]) == VA_STATUS_SUCCESS);
	lazy = false;
	CHECK(vtable.vaDestroyImage(&driver, image.image_id) == VA_STATUS_SUCCESS);
	/* Terminate cleans up the remaining context and coded buffer. */
	teardown();
}

int main(int argc, char **argv)
{
	const char *name = argc > 1 ? argv[1] : "";

	if (!strcmp(name, "units"))
		case_units();
	else if (!strcmp(name, "capabilities"))
		case_capabilities();
	else if (!strcmp(name, "decode-isolation"))
		case_decode_isolation();
	else if (!strcmp(name, "chrome"))
		case_chrome();
	else if (!strcmp(name, "ffmpeg"))
		case_ffmpeg();
	else if (!strcmp(name, "import"))
		case_import();
	else if (!strcmp(name, "lifetime"))
		case_lifetime();
	else {
		fprintf(stderr, "unknown case '%s'\n", name);
		return 2;
	}
	if (failures) {
		fprintf(stderr, "%s: %d failure(s)\n", name, failures);
		return 1;
	}
	printf("ave-encode %s: ok\n", name);
	return 0;
}
