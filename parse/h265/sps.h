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

#include "parse/bit_reader.h"
#include "parse/h265/scaling_list.h"

namespace v4l2wc::h265 {

using v4l2wc::BitReader;

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
  uint32_t log2_min_tb_size = 2;  // transform block, minus2 + 2
  uint32_t log2_diff_max_min_tb_size = 0;
  uint32_t max_transform_hierarchy_depth_inter = 0;
  uint32_t max_transform_hierarchy_depth_intra = 0;
  uint32_t pic_width_in_ctbs = 0;
  uint32_t pic_height_in_ctbs = 0;
  uint32_t pic_size_in_ctbs = 0;

  bool scaling_list_enabled_flag = false;
  bool sps_scaling_list_data_present_flag = false;
  // The scaling lists in effect from the SPS: parsed when signalled, otherwise
  // the HEVC defaults. Meaningful only when scaling_list_enabled_flag is set.
  ScalingListData scaling_list;
  bool amp_enabled_flag = false;
  bool sample_adaptive_offset_enabled_flag = false;
  bool pcm_enabled_flag = false;

  bool long_term_ref_pics_present_flag = false;
  uint32_t num_long_term_ref_pics_sps = 0;
  // Per SPS long-term entry (size() is num_long_term_ref_pics_sps): the POC LSB
  // and the used-by-current flag. The slice header reads these for a long-term
  // reference chosen by lt_idx_sps.
  std::vector<bool> used_by_curr_pic_lt_sps;
  std::vector<uint32_t> lt_ref_pic_poc_lsb_sps;
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

// Parses one st_ref_pic_set (clause 7.3.7) at index `st_rps_idx` and derives
// its negative/positive delta-POC lists into *out. `sets` holds the already
// parsed sets (0..st_rps_idx-1); `num_short_term_rps` is the SPS count, used to
// detect the slice-header case (st_rps_idx == num_short_term_rps) where the
// reference set is chosen by delta_idx_minus1 rather than being the immediately
// preceding set. Shared by the SPS parser and the slice-segment header parser.
// Returns false on malformed input or an out-of-range reference index.
bool ParseShortTermRps(BitReader* br, uint32_t st_rps_idx,
                       uint32_t num_short_term_rps,
                       const std::vector<ShortTermRps>& sets,
                       ShortTermRps* out);

}  // namespace v4l2wc::h265

#endif  // V4L2WC_PARSE_H265_SPS_H_
