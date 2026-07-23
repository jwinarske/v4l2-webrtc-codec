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
  parse/h264/slice_header.cc
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

add_executable(h264_slice_header_test test/parse/h264_slice_header_test.cc)
target_link_libraries(h264_slice_header_test PRIVATE v4l2wc_parse)
add_test(NAME h264_slice_header_test COMMAND h264_slice_header_test)

# ---- fuzz gate -------------------------------------------------------------
# The bitstream is attacker-controlled and reachable once DTLS is up, so the
# parsers are fuzzed rather than only unit-tested. Needs a clang that can build
# libFuzzer; the targets are simply absent otherwise, so the unit tests still
# build with any compiler.
#
# The gate runs each target for V4L2WC_FUZZ_SECONDS (10 by default), which is
# enough for a regression to show up in a run without making ctest slow. Give
# it a much larger value to actually hunt:
#
#   ctest -R fuzz --timeout 4000        # after -DV4L2WC_FUZZ_SECONDS=900
set(V4L2WC_FUZZ_SECONDS 10 CACHE STRING
    "Seconds each fuzz target runs under ctest")

include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-fsanitize=fuzzer")
check_cxx_source_compiles(
  "#include <cstddef>\n#include <cstdint>\nextern \"C\" int LLVMFuzzerTestOneInput(const uint8_t*, size_t) { return 0; }"
  V4L2WC_HAVE_LIBFUZZER)
unset(CMAKE_REQUIRED_FLAGS)

if(V4L2WC_HAVE_LIBFUZZER)
  foreach(target nal sps pps slice_header)
    add_executable(${target}_fuzzer test/fuzz/${target}_fuzzer.cc
      parse/h264/nal.cc parse/h264/sps.cc parse/h264/pps.cc
      parse/h264/slice_header.cc)
    target_include_directories(${target}_fuzzer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    # Sanitizers as well as the fuzzer: a bounds violation the fuzzer reaches
    # is only useful if something reports it rather than reading on.
    target_compile_options(${target}_fuzzer PRIVATE
      -g -O1 -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer)
    target_link_options(${target}_fuzzer PRIVATE
      -fsanitize=fuzzer,address,undefined)
    add_test(NAME ${target}_fuzz
      COMMAND ${target}_fuzzer -max_total_time=${V4L2WC_FUZZ_SECONDS}
              -print_final_stats=1)
    # A crash is a failure: libFuzzer exits nonzero, which ctest reads. Allow
    # generous headroom over the run time so a slow machine is not a failure.
    math(EXPR _fuzz_timeout "${V4L2WC_FUZZ_SECONDS} * 4 + 60")
    set_tests_properties(${target}_fuzz PROPERTIES TIMEOUT ${_fuzz_timeout})
  endforeach()
else()
  message(STATUS
    "libFuzzer unavailable; the fuzz gate is not built (unit tests still are)")
endif()
