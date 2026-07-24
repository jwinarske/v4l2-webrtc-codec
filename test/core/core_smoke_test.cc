// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Smoke test for the webrtc-free decode core. It proves two things the split
// exists to provide: the logging seam carries a streamed message to an
// installed sink, and the decode engines link and run without webrtc, failing
// cleanly on a device that is not present. No framework, no hardware.

#include <linux/videodev2.h>

#include <cstdio>
#include <memory>
#include <string>

#include "src/log.h"
#include "src/v4l2_m2m_decoder.h"
#if V4L2WC_HAVE_VAAPI
#include "src/vaapi_h264_decoder.h"
#endif

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

namespace {

std::string g_last_message;
v4l2wc::LogSeverity g_last_severity = v4l2wc::LogSeverity::kInfo;
int g_sink_calls = 0;

void CaptureSink(v4l2wc::LogSeverity severity, const char* message) {
  g_last_severity = severity;
  g_last_message = message;
  ++g_sink_calls;
}

}  // namespace

int main() {
  v4l2wc::SetLogSink(&CaptureSink);

  // A streamed message reaches the sink as a single call, with its severity.
  V4L2WC_LOG(V4L2WC_ERROR) << "answer " << 42;
  CHECK(g_sink_calls == 1);
  CHECK(g_last_severity == v4l2wc::LogSeverity::kError);
  CHECK(g_last_message == "answer 42");

  // The V4L2 engine links and runs; on a device that is not there it fails
  // cleanly and logs through the sink, which also shows a real engine call
  // path reaching the seam.
  g_sink_calls = 0;
  auto v4l2 = v4l2wc::V4l2M2mDecoder::Create(
      "/dev/v4l2wc-does-not-exist", V4L2_PIX_FMT_H264, V4L2_PIX_FMT_NV12,
      /*coded_width=*/1280, /*coded_height=*/720, /*output_buffer_count=*/4,
      /*capture_buffer_count=*/16, /*output_buffer_size=*/1U << 20);
  CHECK(v4l2 == nullptr);
  CHECK(g_sink_calls >= 1);

#if V4L2WC_HAVE_VAAPI
  // The VAAPI engine likewise links and refuses a render node that is not
  // there. dlopen'ing libva is fine; the missing node is what fails.
  auto vaapi =
      v4l2wc::VaapiH264Decoder::Create("/dev/dri/v4l2wc-nonexistent", 8);
  CHECK(vaapi == nullptr);
#endif

  v4l2wc::SetLogSink(nullptr);  // restore the default sink

  if (g_failures == 0) {
    std::printf("core smoke test: ok\n");
  }
  return g_failures == 0 ? 0 : 1;
}
