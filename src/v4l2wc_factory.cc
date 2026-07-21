// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Public C entry point: builds the webrtc::VideoDecoderFactory the fork injects
// and returns it as an opaque void*. See include/v4l2wc/v4l2wc.h.
//
// NOTE: only compiled when absorbed in-tree (lw_enable_v4l2_codec); not yet
// built. See src/v4l2_decoder.h.

#include "api/video_codecs/video_decoder_factory.h"
#include "src/v4l2_decoder.h"
#include "v4l2wc/v4l2wc.h"

extern "C" {

void* v4l2wc_create_factory(const V4l2WcConfig* cfg) {
  V4l2WcConfig config{};
  config.size = sizeof(V4l2WcConfig);
  config.backend = V4L2WC_BACKEND_AUTO;
  if (cfg) {
    config = *cfg;
  }
  // Return the webrtc base pointer so the caller's reinterpret_cast back to
  // webrtc::VideoDecoderFactory* is value-correct (single inheritance).
  auto* factory = static_cast<webrtc::VideoDecoderFactory*>(
      new v4l2wc::V4l2DecoderFactory(config));
  return factory;
}

const char* v4l2wc_version_string(void) {
  return "v4l2-webrtc-codec (stateful H.264 via drm-cxx V4l2DecoderSource)";
}

}  // extern "C"
