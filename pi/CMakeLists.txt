cmake_minimum_required(VERSION 3.16)
project(pi_hevc_stateless CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Repo root holds the parse/ tree; includes are root-relative ("parse/...").
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_executable(pi_hevc_stateless
  pi_hevc_stateless.cc
  ${REPO_ROOT}/parse/h265/nal.cc
  ${REPO_ROOT}/parse/h265/sps.cc
  ${REPO_ROOT}/parse/h265/pps.cc
  ${REPO_ROOT}/parse/h265/slice_header.cc
  ${REPO_ROOT}/parse/h265/scaling_list.cc)
target_include_directories(pi_hevc_stateless PRIVATE ${REPO_ROOT})
target_compile_options(pi_hevc_stateless PRIVATE -Wall -Wextra)

# The full IDmaDecoder backend (src/) plus an on-device test that drives it.
add_executable(pi_hevc_backend_test
  pi_hevc_backend_test.cc
  ${REPO_ROOT}/src/v4l2_stateless_h265_decoder.cc
  ${REPO_ROOT}/src/dma_decoder.cc
  ${REPO_ROOT}/src/log.cc
  ${REPO_ROOT}/parse/h265/nal.cc
  ${REPO_ROOT}/parse/h265/sps.cc
  ${REPO_ROOT}/parse/h265/pps.cc
  ${REPO_ROOT}/parse/h265/slice_header.cc
  ${REPO_ROOT}/parse/h265/scaling_list.cc)
target_include_directories(pi_hevc_backend_test PRIVATE ${REPO_ROOT})
target_compile_options(pi_hevc_backend_test PRIVATE -Wall -Wextra)

# Broadcom zero-copy display: decode + KMS overlay-plane scanout of the SAND
# dma-buf. Needs libdrm from the sysroot.
find_path(DRM_INCLUDE_DIR NAMES xf86drm.h)
find_path(DRM_LIBDRM_DIR NAMES drm.h PATH_SUFFIXES libdrm)
find_library(DRM_LIB NAMES drm)
if(DRM_INCLUDE_DIR AND DRM_LIBDRM_DIR AND DRM_LIB)
  add_executable(pi_hevc_kms
    pi_hevc_kms.cc
    ${REPO_ROOT}/src/v4l2_stateless_h265_decoder.cc
    ${REPO_ROOT}/src/dma_decoder.cc
    ${REPO_ROOT}/src/log.cc
    ${REPO_ROOT}/parse/h265/nal.cc
    ${REPO_ROOT}/parse/h265/sps.cc
    ${REPO_ROOT}/parse/h265/pps.cc
    ${REPO_ROOT}/parse/h265/slice_header.cc
    ${REPO_ROOT}/parse/h265/scaling_list.cc)
  target_include_directories(pi_hevc_kms PRIVATE ${REPO_ROOT} ${DRM_INCLUDE_DIR} ${DRM_LIBDRM_DIR})
  target_link_libraries(pi_hevc_kms PRIVATE ${DRM_LIB})
  target_compile_options(pi_hevc_kms PRIVATE -Wall -Wextra)
endif()
