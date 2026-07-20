// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

/*
 * v4l2wc.h — public C entry point for v4l2-webrtc-codec.
 *
 * Self-contained C header. This is the entire surface the factory injection
 * and the flat C API consume: build a webrtc::VideoDecoderFactory* that
 * produces hardware-decoded frames as LwDmabufDescriptor-carrying native
 * buffers.
 *
 * The returned pointer is a webrtc::VideoDecoderFactory* (opaque here to keep
 * this header C-clean and free of webrtc includes). The caller owns it and
 * transfers ownership into the peerconnection factory pre-Initialize().
 */
#ifndef V4L2WC_H_
#define V4L2WC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bumped when this C entry surface changes incompatibly. */
#define V4L2WC_ABI_VERSION 1

/* Backend selection. AUTO probes /dev/video* + /dev/media* and picks the
 * stateless (request-API) or stateful (Venus) path per device caps. */
typedef enum V4l2WcBackend {
  V4L2WC_BACKEND_AUTO = 0,
  V4L2WC_BACKEND_STATELESS = 1, /* rkvdec2 / rkvdec, request API */
  V4L2WC_BACKEND_STATEFUL = 2,  /* Venus (qcom) */
} V4l2WcBackend;

/* Configuration. `size` first for forward evolution (sized-struct pattern,
 * matching LW_ABI convention). Zero-init then set fields. */
typedef struct V4l2WcConfig {
  uint32_t size;         /* sizeof(V4l2WcConfig) */
  V4l2WcBackend backend; /* default AUTO */

  /* Explicit device paths; NULL/empty => probe. The request-API media-device
   * pairing is board-specific. */
  const char* video_device; /* e.g. "/dev/video0"  (NULL => probe) */
  const char* media_device; /* e.g. "/dev/media0"  (NULL => probe) */

  /* Pool sizing headroom added on top of (reorder + pipeline + 1). 0 =>
   * default. */
  uint32_t extra_capture_buffers;
} V4l2WcConfig;

/* Build the decoder factory. Returns webrtc::VideoDecoderFactory* as void*
 * (opaque), or NULL on failure (no usable device / caps mismatch).
 * `cfg` may be NULL for all-defaults. */
void* v4l2wc_create_factory(const V4l2WcConfig* cfg);

/* Human-readable build/version string for logs and the tier-probe dump. */
const char* v4l2wc_version_string(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* V4L2WC_H_ */
