/*
 * VA-API -> apple-ave parameter mapping (rate control, GOP) and validation
 * of imported DMA-BUF encode inputs. Pure functions, host-testable.
 *
 * Ported from aurora-ave-vaapi e40b036 (src/encode_params.[ch],
 * src/surface_import.[ch]).
 *
 * Copyright (c) 2026 snowf14k3
 * Copyright (c) 2026 libva-v4l2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef AVE_PARAMS_H
#define AVE_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <va/va.h>
#include <va/va_drmcommon.h>

/* Rate-related values collected from VAEncMiscParameterBuffer entries. */
struct ave_rate_request {
	bool rate_seen;
	uint32_t bits_per_second;
	uint32_t target_percentage;
	uint32_t min_qp;
	uint32_t max_qp;
	bool frame_rate_seen;
	uint32_t frames_per_second;
};

/* What apple-ave is told (V4L2 bit-rate mode, bit rates, QP range). */
struct ave_rate_settings {
	uint32_t v4l2_mode;
	uint32_t bitrate;
	uint32_t peak;
	bool qp_range;
	uint32_t min_qp;
	uint32_t max_qp;
};

int ave_parse_misc(VAEncMiscParameterType type, const void *data,
		   size_t size, struct ave_rate_request *rate);
int ave_rate_settings_from_va(uint32_t va_rate_control,
			      const struct ave_rate_request *rate,
			      uint32_t sequence_bits_per_second,
			      struct ave_rate_settings *settings);
uint32_t ave_gop_from_va(uint32_t intra_idr_period, uint32_t intra_period);
int ave_parse_frame_rate(uint32_t value, uint32_t *result);

/*
 * Validated description of an imported two-plane 4:2:0 DMA-BUF. Offsets
 * are from the start of the single DMA-BUF object; both planes share one
 * pitch, the only layout the copy into AVE's OUTPUT buffer handles.
 */
struct ave_import_layout {
	uint32_t fourcc;
	uint32_t width;
	uint32_t height;
	uint32_t stride;
	size_t y_offset;
	size_t uv_offset;
	size_t required_size;
	size_t object_size;
	int fd;
};

/* Legacy VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME + VASurfaceAttribExternalBuffers. */
int ave_import_layout_from_external(const VASurfaceAttribExternalBuffers *ext,
				    uint32_t width, uint32_t height,
				    uint32_t fourcc,
				    struct ave_import_layout *layout);
/* VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2 + VADRMPRIMESurfaceDescriptor. */
int ave_import_layout_from_prime2(const VADRMPRIMESurfaceDescriptor *prime,
				  uint32_t width, uint32_t height,
				  uint32_t fourcc,
				  struct ave_import_layout *layout);

#endif /* AVE_PARAMS_H */
