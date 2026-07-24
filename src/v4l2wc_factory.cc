// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Public C entry point: builds the webrtc::VideoDecoderFactory the fork injects
// and returns it as an opaque void*. See include/v4l2wc/v4l2wc.h.
//
// NOTE: only compiled when absorbed in-tree (lw_enable_v4l2_codec); not yet
// built. See src/v4l2_decoder.h.

#include "api/video_codecs/video_decoder_factory.h"
#include "rtc_base/logging.h"
#include "src/log.h"
#include "src/v4l2_decoder.h"
#include "v4l2wc/v4l2wc.h"

namespace {

// The engines log through the webrtc-free seam in src/log.h. In the in-tree
// build we route those messages to RTC_LOG so nothing is lost, which the
// standalone build does not do (it keeps the default stderr sink).
void ForwardToRtcLog(v4l2wc::LogSeverity severity, const char* message) {
  switch (severity) {
    case v4l2wc::LogSeverity::kInfo:
      RTC_LOG(LS_INFO) << message;
      break;
    case v4l2wc::LogSeverity::kWarning:
      RTC_LOG(LS_WARNING) << message;
      break;
    case v4l2wc::LogSeverity::kError:
      RTC_LOG(LS_ERROR) << message;
      break;
  }
}

}  // namespace

extern "C" {

void* v4l2wc_create_factory(const V4l2WcConfig* cfg) {
  v4l2wc::SetLogSink(&ForwardToRtcLog);

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
