// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// H.265 / HEVC Sequence Parameter Set parsing. Recovers the coded picture
// geometry (profile/tier/level and conformance-cropped luma dimensions, which
// size the decoder's CAPTURE-queue pool) and the fields a stateless decoder and
// the slice-segment header parser depend on: the CTB grid, POC LSB width, and
// the short-term reference-picture sets.
//
// Attacker-controlled input: every field is read through the bounds-checked
// BitReader and each count that bounds a loop or sizes a later u(v) read is
// range-limited, so malformed input fails the parse rather than reading past
// the buffer or looping unboundedly.
#ifndef V4L2WC_PARSE_H265_SPS_H_
#define V4L2WC_PARSE_H265_SPS_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace v4l2wc::h265 {

// Largest coded luma dimension accepted; anything larger is treated as
// malformed. HEVC level 6.2 tops out well under this.
inline constexpr uint32_t kMaxDimension = 16384;

// Spec maxima that bound the parse loops (Table A-1 / clause 7.4.3.2.1).
inline constexpr uint32_t kMaxShortTermRps = 64;
inline constexpr uint32_t kMaxLongTermRps = 32;
inline constexpr uint32_t kMaxLog2Minus4 = 12;  // log2_max_pic_order_cnt_lsb
// NumDeltaPocs per short-term RPS is bounded by the decoded-picture buffer
// size, itself at most 16 (MaxDpbSize). Caps the delta-POC arrays.
inline constexpr uint32_t kMaxRefPics = 16;

// A short-term reference-picture set, after derivation (clause 7.4.8). Both the
// explicitly coded form and the inter-predicted form resolve to the same
// negative/positive delta-POC lists; the slice-segment header needs
// num_delta_pocs of each SPS-defined set to parse a set it defines inline.
struct ShortTermRps {
  uint32_t num_negative_pics = 0;
  uint32_t num_positive_pics = 0;
  uint32_t num_delta_pocs = 0;  // num_negative_pics + num_positive_pics
  int32_t delta_poc_s0[kMaxRefPics] = {};
  int32_t delta_poc_s1[kMaxRefPics] = {};
  bool used_s0[kMaxRefPics] = {};
  bool used_s1[kMaxRefPics] = {};
};

struct Sps {
  Sps();

  uint32_t sps_video_parameter_set_id = 0;
  uint8_t sps_max_sub_layers_minus1 = 0;
  bool sps_temporal_id_nesting_flag = false;

  uint8_t general_profile_idc = 0;
  bool general_tier_flag = false;
  uint8_t general_level_idc = 0;

  uint32_t sps_seq_parameter_set_id = 0;
  uint32_t chroma_format_idc = 1;  // 1 = 4:2:0
  bool separate_colour_plane_flag = false;

  uint32_t pic_width_in_luma_samples = 0;   // coded (uncropped)
  uint32_t pic_height_in_luma_samples = 0;  // coded (uncropped)
  uint32_t width = 0;                       // conformance-cropped width
  uint32_t height = 0;                      // conformance-cropped height

  uint32_t bit_depth_luma = 8;              // bit_depth_luma_minus8 + 8
  uint32_t bit_depth_chroma = 8;            // bit_depth_chroma_minus8 + 8
  uint32_t log2_max_pic_order_cnt_lsb = 4;  // _minus4 + 4; sizes a slice u(v)
  uint32_t sps_max_dec_pic_buffering_minus1 = 0;  // highest sub-layer

  // Coding-tree geometry, derived. The slice-segment header's
  // slice_segment_address is u(Ceil(Log2(pic_size_in_ctbs))).
  uint32_t log2_min_cb_size = 3;
  uint32_t log2_ctb_size = 4;
  uint32_t pic_width_in_ctbs = 0;
  uint32_t pic_height_in_ctbs = 0;
  uint32_t pic_size_in_ctbs = 0;

  bool scaling_list_enabled_flag = false;
  bool amp_enabled_flag = false;
  bool sample_adaptive_offset_enabled_flag = false;
  bool pcm_enabled_flag = false;

  bool long_term_ref_pics_present_flag = false;
  uint32_t num_long_term_ref_pics_sps = 0;
  bool sps_temporal_mvp_enabled_flag = false;
  bool strong_intra_smoothing_enabled_flag = false;

  // The SPS-defined short-term RPSs, indexed 0..size()-1. Size equals
  // num_short_term_ref_pic_sets.
  std::vector<ShortTermRps> short_term_rps;
};

// Parses an SPS from its RBSP (NAL header stripped, emulation-prevention bytes
// already removed — e.g. Nal::rbsp). Returns true and fills *out on success,
// false on malformed or out-of-range input.
bool ParseSps(const uint8_t* rbsp, size_t size, Sps* out);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_SPS_H_
