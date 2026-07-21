# SPDX-FileCopyrightText: 2026 Joel Winarske
# SPDX-License-Identifier: MIT

# Standalone dev/CI build of the pure-logic layers (bitstream parse/) and their
# unit tests. No webrtc, no V4L2 — this builds and runs on the host so the
# memory-safety-critical parsers are unit-tested and fuzzed off-target. The
# decoder itself is absorbed into libwebrtc.so via BUILD.gn (lw_enable_v4l2_codec).
cmake_minimum_required(VERSION 3.16)
project(v4l2_webrtc_codec_parse LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

add_library(v4l2wc_parse STATIC
  parse/h264/nal.cc
  parse/h264/sps.cc
  parse/h264/pps.cc
)
target_include_directories(v4l2wc_parse PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(v4l2wc_parse PRIVATE -Wall -Wextra)

add_executable(h264_nal_test test/parse/h264_nal_test.cc)
target_link_libraries(h264_nal_test PRIVATE v4l2wc_parse)
add_test(NAME h264_nal_test COMMAND h264_nal_test)

add_executable(h264_sps_test test/parse/h264_sps_test.cc)
target_link_libraries(h264_sps_test PRIVATE v4l2wc_parse)
add_test(NAME h264_sps_test COMMAND h264_sps_test)

add_executable(h264_pps_test test/parse/h264_pps_test.cc)
target_link_libraries(h264_pps_test PRIVATE v4l2wc_parse)
add_test(NAME h264_pps_test COMMAND h264_pps_test)
