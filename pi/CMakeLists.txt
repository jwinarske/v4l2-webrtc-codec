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
