// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/log.h"

#include <cstdio>
#include <string>

namespace v4l2wc {
namespace {

void DefaultSink(LogSeverity severity, const char* message) {
  const char* label = "INFO";
  switch (severity) {
    case LogSeverity::kInfo:
      label = "INFO";
      break;
    case LogSeverity::kWarning:
      label = "WARN";
      break;
    case LogSeverity::kError:
      label = "ERROR";
      break;
  }
  std::fprintf(stderr, "[v4l2wc %s] %s\n", label, message);
}

LogSink g_sink = &DefaultSink;

}  // namespace

void SetLogSink(LogSink sink) {
  g_sink = sink != nullptr ? sink : &DefaultSink;
}

LogMessage::~LogMessage() {
  const std::string text = stream_.str();
  g_sink(severity_, text.c_str());
}

}  // namespace v4l2wc
