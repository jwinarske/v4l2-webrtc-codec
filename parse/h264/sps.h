// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.264 Sequence Parameter Set (SPS) parsing — enough to recover the coded
// picture geometry (profile/level and cropped luma dimensions), which the
// decoder needs to size its CAPTURE-queue buffer pool. Attacker-controlled
// input: every field is read through the bounds-checked BitReader and the
// derived dimensions are range-limited.
#ifndef V4L2WC_PARSE_H264_SPS_H_
#define V4L2WC_PARSE_H264_SPS_H_

#include <cstddef>
#include <cstdint>

namespace v4l2wc::h264 {

struct Sps {
  uint8_t profile_idc = 0;
  uint8_t level_idc = 0;
  uint32_t seq_parameter_set_id = 0;
  uint32_t chroma_format_idc = 1;  // 1 = 4:2:0 (default for non-high profiles)
  bool frame_mbs_only_flag = true;
  uint32_t width = 0;   // cropped luma width in pixels
  uint32_t height = 0;  // cropped luma height in pixels
};

// Largest coded dimension accepted; anything larger is treated as malformed.
inline constexpr uint32_t kMaxDimension = 16384;

// Parses an SPS from its RBSP (NAL header stripped, emulation-prevention bytes
// already removed — e.g. Nal::rbsp). Returns true and fills *out on success,
// false on malformed or out-of-range input.
bool ParseSps(const uint8_t* rbsp, size_t size, Sps* out);

}  // namespace v4l2wc::h264

#endif  // V4L2WC_PARSE_H264_SPS_H_
