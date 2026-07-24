# SPDX-FileCopyrightText: 2026 Joel Winarske
# SPDX-License-Identifier: MIT

# Standalone dev/CI build of the host-buildable layers: the bitstream parse/
# parsers and the webrtc-free decode core (v4l2wc_core). Builds on the host so
# the memory-safety-critical parsers are unit-tested and fuzzed off-target and
# the engines are proven to link without webrtc. The full decoder is still
# absorbed into libwebrtc.so via BUILD.gn (lw_enable_v4l2_codec).
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
  parse/h265/nal.cc
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

add_executable(h265_nal_test test/parse/h265_nal_test.cc)
target_link_libraries(h265_nal_test PRIVATE v4l2wc_parse)
add_test(NAME h265_nal_test COMMAND h265_nal_test)

# ---- webrtc-free decode core -----------------------------------------------
# The decode engines, hoisted out of the libwebrtc build so a consumer that is
# not webrtc (the ivi-homescreen HLS player plugin) can link them. The engines
# log through the seam in src/log.h instead of RTC_LOG, so nothing here pulls
# in webrtc.
#
# VAAPI needs the libva headers at build time; it dlopens the library itself at
# runtime. Without the headers only the V4L2 M2M engine is built, which is the
# right shape for a device (e.g. a Pi) that has no VAAPI anyway.
find_path(V4L2WC_VA_INCLUDE va/va.h)

add_library(v4l2wc_core STATIC
  src/log.cc
  src/dma_decoder.cc
  src/dma_decoder_factory.cc
  src/v4l2_m2m_decoder.cc
)
target_link_libraries(v4l2wc_core PUBLIC v4l2wc_parse)
target_include_directories(v4l2wc_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(v4l2wc_core PRIVATE -Wall -Wextra)

if(V4L2WC_VA_INCLUDE)
  target_sources(v4l2wc_core PRIVATE
    src/va_loader.cc
    src/vaapi_h264_decoder.cc)
  target_include_directories(v4l2wc_core PRIVATE ${V4L2WC_VA_INCLUDE})
  target_link_libraries(v4l2wc_core PUBLIC ${CMAKE_DL_LIBS})
  target_compile_definitions(v4l2wc_core PUBLIC V4L2WC_HAVE_VAAPI=1)
  message(STATUS "v4l2wc_core: VAAPI engine enabled (${V4L2WC_VA_INCLUDE})")
else()
  target_compile_definitions(v4l2wc_core PUBLIC V4L2WC_HAVE_VAAPI=0)
  message(STATUS "v4l2wc_core: libva headers not found, VAAPI engine disabled")
endif()

add_executable(v4l2wc_core_test test/core/core_smoke_test.cc)
target_link_libraries(v4l2wc_core_test PRIVATE v4l2wc_core)
add_test(NAME v4l2wc_core_test COMMAND v4l2wc_core_test)

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
  # Each entry pairs a fuzz target name with the parser sources its harness
  # links, so a crash points at a single parser. The H.264 targets share the
  # whole H.264 parser; the H.265 one links only its own translation unit.
  set(_fuzz_nal_sources
    parse/h264/nal.cc parse/h264/sps.cc parse/h264/pps.cc
    parse/h264/slice_header.cc)
  set(_fuzz_sps_sources ${_fuzz_nal_sources})
  set(_fuzz_pps_sources ${_fuzz_nal_sources})
  set(_fuzz_slice_header_sources ${_fuzz_nal_sources})
  set(_fuzz_h265_nal_sources parse/h265/nal.cc)
  foreach(target nal sps pps slice_header h265_nal)
    add_executable(${target}_fuzzer test/fuzz/${target}_fuzzer.cc
      ${_fuzz_${target}_sources})
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
