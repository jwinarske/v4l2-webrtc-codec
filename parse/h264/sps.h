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

  // Coded (uncropped) macroblock geometry, and the picture-order and reference
  // fields a hardware decoder needs to build its picture parameters and to
  // populate a SliceContext.
  uint32_t pic_width_in_mbs = 0;         // pic_width_in_mbs_minus1 + 1
  uint32_t pic_height_in_map_units = 0;  // pic_height_in_map_units_minus1 + 1
  uint32_t log2_max_frame_num = 4;       // log2_max_frame_num_minus4 + 4
  uint32_t pic_order_cnt_type = 0;
  uint32_t log2_max_pic_order_cnt_lsb = 4;        // poc type 0; _minus4 + 4
  bool delta_pic_order_always_zero_flag = false;  // poc type 1
  uint32_t max_num_ref_frames = 0;
  bool direct_8x8_inference_flag = false;
  bool mb_adaptive_frame_field_flag = false;
  bool separate_colour_plane_flag = false;
};

// Largest coded dimension accepted; anything larger is treated as malformed.
inline constexpr uint32_t kMaxDimension = 16384;

// Spec bound on log2_max_frame_num_minus4 and
// log2_max_pic_order_cnt_lsb_minus4. Both size u(v) reads in the slice header.
inline constexpr uint32_t kMaxLog2Minus4 = 12;

// Parses an SPS from its RBSP (NAL header stripped, emulation-prevention bytes
// already removed — e.g. Nal::rbsp). Returns true and fills *out on success,
// false on malformed or out-of-range input.
bool ParseSps(const uint8_t* rbsp, size_t size, Sps* out);

}  // namespace v4l2wc::h264

#endif  // V4L2WC_PARSE_H264_SPS_H_
