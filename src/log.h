// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// A tiny logging seam so the decode engines carry no webrtc dependency. The
// engines log through V4L2WC_LOG; a consumer installs a sink to route the
// messages wherever it wants (the webrtc wrapper forwards them to RTC_LOG),
// and the default sink writes to stderr so the standalone build still logs.
#ifndef V4L2WC_SRC_LOG_H_
#define V4L2WC_SRC_LOG_H_

#include <ostream>
#include <sstream>

namespace v4l2wc {

enum class LogSeverity { kInfo, kWarning, kError };

using LogSink = void (*)(LogSeverity severity, const char* message);

// Installs the sink the engines log through. Passing nullptr restores the
// default, which writes to stderr. Install it once at startup: it is not
// synchronized against concurrent logging.
void SetLogSink(LogSink sink);

// Accumulates one streamed log line and hands it to the sink when it goes out
// of scope, so a whole message reaches the sink in a single call.
class LogMessage {
 public:
  explicit LogMessage(LogSeverity severity) : severity_(severity) {}
  ~LogMessage();

  LogMessage(const LogMessage&) = delete;
  LogMessage& operator=(const LogMessage&) = delete;

  std::ostream& stream() { return stream_; }

 private:
  LogSeverity severity_;
  std::ostringstream stream_;
};

}  // namespace v4l2wc

#define V4L2WC_INFO ::v4l2wc::LogSeverity::kInfo
#define V4L2WC_WARNING ::v4l2wc::LogSeverity::kWarning
#define V4L2WC_ERROR ::v4l2wc::LogSeverity::kError

// The streamed form the engines use:
//   V4L2WC_LOG(V4L2WC_ERROR) << "message " << value;
#define V4L2WC_LOG(severity) ::v4l2wc::LogMessage(severity).stream()

#endif  // V4L2WC_SRC_LOG_H_
