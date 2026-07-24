// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/sps.h"

#include "parse/bit_reader.h"
#include "parse/h265/scaling_list.h"

namespace v4l2wc::h265 {

Sps::Sps() = default;

namespace {

// Bound on a delta value read as ue(v) before it feeds signed POC arithmetic.
// Well above any real delta_poc / abs_delta_rps, and keeps the running sums
// far inside int32.
constexpr uint32_t kMaxDeltaPoc = 0x7fff;

// profile_tier_level (clause 7.3.3). profilePresentFlag is always 1 here (SPS
// always signals the general profile). Captures general_profile_idc /
// general_tier_flag / general_level_idc and skips the rest; each fixed-size
// block is consumed so the bit position stays aligned for the fields after.
bool ParseProfileTierLevel(BitReader* br, uint32_t max_sub_layers_minus1,
                           Sps* sps) {
  uint32_t profile_space = 0;
  bool tier_flag = false;
  uint32_t profile_idc = 0;
  if (!br->ReadBits(2, &profile_space) || !br->ReadFlag(&tier_flag) ||
      !br->ReadBits(5, &profile_idc)) {
    return false;
  }
  sps->general_tier_flag = tier_flag;
  sps->general_profile_idc = static_cast<uint8_t>(profile_idc);
  // 32 compatibility flags + 4 source flags + 43 constraint flags + 1
  // reserved/inbld = 80 bits, completing the 88-bit general profile block.
  if (!br->SkipBits(80)) {
    return false;
  }
  uint32_t level_idc = 0;
  if (!br->ReadBits(8, &level_idc)) {  // general_level_idc
    return false;
  }
  sps->general_level_idc = static_cast<uint8_t>(level_idc);

  bool sub_layer_profile_present[8] = {};
  bool sub_layer_level_present[8] = {};
  for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
    if (!br->ReadFlag(&sub_layer_profile_present[i]) ||
        !br->ReadFlag(&sub_layer_level_present[i])) {
      return false;
    }
  }
  if (max_sub_layers_minus1 > 0) {
    // reserved_zero_2bits for i in max_sub_layers_minus1..7.
    if (!br->SkipBits((8 - max_sub_layers_minus1) * 2)) {
      return false;
    }
  }
  for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
    if (sub_layer_profile_present[i] && !br->SkipBits(88)) {
      return false;  // sub-layer profile block, same 88-bit shape
    }
    if (sub_layer_level_present[i] && !br->SkipBits(8)) {
      return false;  // sub_layer_level_idc
    }
  }
  return true;
}

// hrd_parameters (clause E.2.2). Consumed and discarded; the VUI only needs to
// be advanced past to reach the SPS extension. commonInfPresentFlag is 1 here.
bool SkipHrdParameters(BitReader* br, uint32_t max_sub_layers_minus1) {
  bool nal_hrd = false;
  bool vcl_hrd = false;
  bool sub_pic_hrd = false;
  if (!br->ReadFlag(&nal_hrd) || !br->ReadFlag(&vcl_hrd)) {
    return false;
  }
  if (nal_hrd || vcl_hrd) {
    if (!br->ReadFlag(&sub_pic_hrd)) {
      return false;
    }
    if (sub_pic_hrd && !br->SkipBits(19)) {  // tick_divisor + 3 length fields
      return false;
    }
    if (!br->SkipBits(8)) {  // bit_rate_scale + cpb_size_scale
      return false;
    }
    if (sub_pic_hrd && !br->SkipBits(4)) {  // cpb_size_du_scale
      return false;
    }
    if (!br->SkipBits(15)) {  // three *_length_minus1 fields
      return false;
    }
  }
  for (uint32_t i = 0; i <= max_sub_layers_minus1; ++i) {
    bool fixed_pic_rate_general = false;
    bool fixed_pic_rate_within_cvs = true;
    bool low_delay_hrd = false;
    uint32_t cpb_cnt_minus1 = 0;
    if (!br->ReadFlag(&fixed_pic_rate_general)) {
      return false;
    }
    if (!fixed_pic_rate_general && !br->ReadFlag(&fixed_pic_rate_within_cvs)) {
      return false;
    }
    if (fixed_pic_rate_within_cvs) {
      uint32_t elemental_duration = 0;
      if (!br->ReadUe(&elemental_duration)) {
        return false;
      }
    } else if (!br->ReadFlag(&low_delay_hrd)) {
      return false;
    }
    if (!low_delay_hrd) {
      if (!br->ReadUe(&cpb_cnt_minus1) || cpb_cnt_minus1 > 31) {
        return false;  // spec bound; also caps the loop below
      }
    }
    for (int nal_or_vcl = 0; nal_or_vcl < 2; ++nal_or_vcl) {
      const bool present = (nal_or_vcl == 0) ? nal_hrd : vcl_hrd;
      if (!present) {
        continue;
      }
      for (uint32_t j = 0; j <= cpb_cnt_minus1; ++j) {
        uint32_t ignore = 0;
        if (!br->ReadUe(&ignore) ||
            !br->ReadUe(&ignore)) {  // bit_rate/cpb_size
          return false;
        }
        if (sub_pic_hrd &&
            (!br->ReadUe(&ignore) || !br->ReadUe(&ignore))) {  // du values
          return false;
        }
        if (!br->SkipBits(1)) {  // cbr_flag
          return false;
        }
      }
    }
  }
  return true;
}

// vui_parameters (clause E.2.1). Consumed and discarded to reach the SPS
// extension.
bool SkipVui(BitReader* br, uint32_t max_sub_layers_minus1) {
  bool flag = false;
  if (!br->ReadFlag(&flag)) {  // aspect_ratio_info_present_flag
    return false;
  }
  if (flag) {
    uint32_t aspect_ratio_idc = 0;
    if (!br->ReadBits(8, &aspect_ratio_idc)) {
      return false;
    }
    if (aspect_ratio_idc == 255 && !br->SkipBits(32)) {  // EXTENDED_SAR
      return false;
    }
  }
  if (!br->ReadFlag(&flag)) {  // overscan_info_present_flag
    return false;
  }
  if (flag && !br->SkipBits(1)) {  // overscan_appropriate_flag
    return false;
  }
  if (!br->ReadFlag(&flag)) {  // video_signal_type_present_flag
    return false;
  }
  if (flag) {
    if (!br->SkipBits(4)) {  // video_format + video_full_range_flag
      return false;
    }
    bool colour_desc = false;
    if (!br->ReadFlag(&colour_desc)) {
      return false;
    }
    if (colour_desc && !br->SkipBits(24)) {  // primaries/transfer/matrix
      return false;
    }
  }
  if (!br->ReadFlag(&flag)) {  // chroma_loc_info_present_flag
    return false;
  }
  if (flag) {
    uint32_t ignore = 0;
    if (!br->ReadUe(&ignore) || !br->ReadUe(&ignore)) {
      return false;
    }
  }
  if (!br->SkipBits(3)) {  // neutral_chroma / field_seq / frame_field_info
    return false;
  }
  if (!br->ReadFlag(&flag)) {  // default_display_window_flag
    return false;
  }
  if (flag) {
    uint32_t ignore = 0;
    for (int i = 0; i < 4; ++i) {
      if (!br->ReadUe(&ignore)) {
        return false;
      }
    }
  }
  bool vui_timing = false;
  if (!br->ReadFlag(&vui_timing)) {  // vui_timing_info_present_flag
    return false;
  }
  if (vui_timing) {
    if (!br->SkipBits(32) || !br->SkipBits(32)) {  // units_in_tick + time_scale
      return false;
    }
    bool poc_proportional = false;
    if (!br->ReadFlag(&poc_proportional)) {
      return false;
    }
    if (poc_proportional) {
      uint32_t ignore = 0;
      if (!br->ReadUe(&ignore)) {
        return false;
      }
    }
    bool hrd_present = false;
    if (!br->ReadFlag(&hrd_present)) {
      return false;
    }
    if (hrd_present && !SkipHrdParameters(br, max_sub_layers_minus1)) {
      return false;
    }
  }
  bool bitstream_restriction = false;
  if (!br->ReadFlag(&bitstream_restriction)) {
    return false;
  }
  if (bitstream_restriction) {
    if (!br->SkipBits(3)) {  // three restriction flags
      return false;
    }
    uint32_t ignore = 0;
    for (int i = 0; i < 5; ++i) {  // five ue fields
      if (!br->ReadUe(&ignore)) {
        return false;
      }
    }
  }
  return true;
}

// sps_range_extension (clause 7.3.2.2.2): nine coding-tool flags.
bool ParseSpsRangeExtension(BitReader* br, SpsRangeExtension* out) {
  return br->ReadFlag(&out->transform_skip_rotation_enabled_flag) &&
         br->ReadFlag(&out->transform_skip_context_enabled_flag) &&
         br->ReadFlag(&out->implicit_rdpcm_enabled_flag) &&
         br->ReadFlag(&out->explicit_rdpcm_enabled_flag) &&
         br->ReadFlag(&out->extended_precision_processing_flag) &&
         br->ReadFlag(&out->intra_smoothing_disabled_flag) &&
         br->ReadFlag(&out->high_precision_offsets_enabled_flag) &&
         br->ReadFlag(&out->persistent_rice_adaptation_enabled_flag) &&
         br->ReadFlag(&out->cabac_bypass_alignment_enabled_flag);
}

}  // namespace

// st_ref_pic_set (clause 7.3.7). For an SPS-defined set (st_rps_idx <
// num_short_term_rps) delta_idx_minus1 is absent and the reference set is the
// immediately preceding one; for the slice-header set (st_rps_idx ==
// num_short_term_rps) delta_idx_minus1 selects the reference set. `sets` holds
// the already parsed sets 0..st_rps_idx-1. Derives the negative/positive
// delta-POC lists.
bool ParseShortTermRps(BitReader* br, uint32_t st_rps_idx,
                       uint32_t num_short_term_rps,
                       const std::vector<ShortTermRps>& sets,
                       ShortTermRps* out) {
  bool inter_pred = false;
  if (st_rps_idx != 0 && !br->ReadFlag(&inter_pred)) {
    return false;
  }

  if (inter_pred) {
    uint32_t delta_idx_minus1 = 0;
    if (st_rps_idx == num_short_term_rps && !br->ReadUe(&delta_idx_minus1)) {
      return false;
    }
    // RefRpsIdx = st_rps_idx - (delta_idx_minus1 + 1); it must index an already
    // parsed set.
    if (delta_idx_minus1 + 1 > st_rps_idx) {
      return false;
    }
    const uint32_t ref_rps_idx = st_rps_idx - (delta_idx_minus1 + 1);
    if (ref_rps_idx >= sets.size()) {
      return false;
    }
    const ShortTermRps& ref = sets[ref_rps_idx];
    bool delta_rps_sign = false;
    uint32_t abs_delta_rps_minus1 = 0;
    if (!br->ReadFlag(&delta_rps_sign) || !br->ReadUe(&abs_delta_rps_minus1)) {
      return false;
    }
    if (abs_delta_rps_minus1 > kMaxDeltaPoc) {
      return false;
    }
    const int32_t delta_rps = (delta_rps_sign ? -1 : 1) *
                              static_cast<int32_t>(abs_delta_rps_minus1 + 1);

    // used_by_curr_pic_flag[j] / use_delta_flag[j] for j in 0..NumDeltaPocs.
    const uint32_t num_delta = ref.num_delta_pocs;  // <= kMaxRefPics
    bool used[kMaxRefPics + 1] = {};
    bool use_delta[kMaxRefPics + 1] = {};
    for (uint32_t j = 0; j <= num_delta; ++j) {
      bool used_flag = false;
      if (!br->ReadFlag(&used_flag)) {
        return false;
      }
      used[j] = used_flag;
      use_delta[j] = true;
      if (!used_flag && !br->ReadFlag(&use_delta[j])) {
        return false;
      }
    }

    // Derive DeltaPocS0 / UsedByCurrPicS0 (negative side).
    uint32_t i = 0;
    for (int32_t j = static_cast<int32_t>(ref.num_positive_pics) - 1; j >= 0;
         --j) {
      const int32_t d_poc = ref.delta_poc_s1[j] + delta_rps;
      if (d_poc < 0 && use_delta[ref.num_negative_pics + j]) {
        if (i >= kMaxRefPics) {
          return false;
        }
        out->delta_poc_s0[i] = d_poc;
        out->used_s0[i] = used[ref.num_negative_pics + j];
        ++i;
      }
    }
    if (delta_rps < 0 && use_delta[num_delta]) {
      if (i >= kMaxRefPics) {
        return false;
      }
      out->delta_poc_s0[i] = delta_rps;
      out->used_s0[i] = used[num_delta];
      ++i;
    }
    for (uint32_t j = 0; j < ref.num_negative_pics; ++j) {
      const int32_t d_poc = ref.delta_poc_s0[j] + delta_rps;
      if (d_poc < 0 && use_delta[j]) {
        if (i >= kMaxRefPics) {
          return false;
        }
        out->delta_poc_s0[i] = d_poc;
        out->used_s0[i] = used[j];
        ++i;
      }
    }
    out->num_negative_pics = i;

    // Derive DeltaPocS1 / UsedByCurrPicS1 (positive side).
    i = 0;
    for (int32_t j = static_cast<int32_t>(ref.num_negative_pics) - 1; j >= 0;
         --j) {
      const int32_t d_poc = ref.delta_poc_s0[j] + delta_rps;
      if (d_poc > 0 && use_delta[j]) {
        if (i >= kMaxRefPics) {
          return false;
        }
        out->delta_poc_s1[i] = d_poc;
        out->used_s1[i] = used[j];
        ++i;
      }
    }
    if (delta_rps > 0 && use_delta[num_delta]) {
      if (i >= kMaxRefPics) {
        return false;
      }
      out->delta_poc_s1[i] = delta_rps;
      out->used_s1[i] = used[num_delta];
      ++i;
    }
    for (uint32_t j = 0; j < ref.num_positive_pics; ++j) {
      const int32_t d_poc = ref.delta_poc_s1[j] + delta_rps;
      if (d_poc > 0 && use_delta[ref.num_negative_pics + j]) {
        if (i >= kMaxRefPics) {
          return false;
        }
        out->delta_poc_s1[i] = d_poc;
        out->used_s1[i] = used[ref.num_negative_pics + j];
        ++i;
      }
    }
    out->num_positive_pics = i;
  } else {
    uint32_t num_neg = 0;
    uint32_t num_pos = 0;
    if (!br->ReadUe(&num_neg) || !br->ReadUe(&num_pos)) {
      return false;
    }
    if (num_neg > kMaxRefPics || num_pos > kMaxRefPics ||
        num_neg + num_pos > kMaxRefPics) {
      return false;
    }
    out->num_negative_pics = num_neg;
    out->num_positive_pics = num_pos;
    int32_t poc = 0;
    for (uint32_t i = 0; i < num_neg; ++i) {
      uint32_t delta_minus1 = 0;
      bool used = false;
      if (!br->ReadUe(&delta_minus1) || !br->ReadFlag(&used)) {
        return false;
      }
      if (delta_minus1 > kMaxDeltaPoc) {
        return false;
      }
      poc -= static_cast<int32_t>(delta_minus1 + 1);  // monotonically negative
      out->delta_poc_s0[i] = poc;
      out->used_s0[i] = used;
    }
    poc = 0;
    for (uint32_t i = 0; i < num_pos; ++i) {
      uint32_t delta_minus1 = 0;
      bool used = false;
      if (!br->ReadUe(&delta_minus1) || !br->ReadFlag(&used)) {
        return false;
      }
      if (delta_minus1 > kMaxDeltaPoc) {
        return false;
      }
      poc += static_cast<int32_t>(delta_minus1 + 1);  // monotonically positive
      out->delta_poc_s1[i] = poc;
      out->used_s1[i] = used;
    }
  }
  out->num_delta_pocs = out->num_negative_pics + out->num_positive_pics;
  // NumDeltaPocs is bounded by the DPB size (clause 7.4.8). The inter-predicted
  // derivation can otherwise reach kMaxRefPics on each side independently, and
  // a later set referencing this one would size its flag arrays by this count.
  if (out->num_delta_pocs > kMaxRefPics) {
    return false;
  }
  return true;
}

bool ParseSps(const uint8_t* rbsp, size_t size, Sps* out) {
  if (rbsp == nullptr || out == nullptr) {
    return false;
  }
  BitReader br(rbsp, size);
  Sps sps;

  uint32_t max_sub_layers_minus1 = 0;
  if (!br.ReadBits(4, &sps.sps_video_parameter_set_id) ||
      !br.ReadBits(3, &max_sub_layers_minus1) ||
      !br.ReadFlag(&sps.sps_temporal_id_nesting_flag)) {
    return false;
  }
  sps.sps_max_sub_layers_minus1 = static_cast<uint8_t>(max_sub_layers_minus1);

  if (!ParseProfileTierLevel(&br, max_sub_layers_minus1, &sps)) {
    return false;
  }

  if (!br.ReadUe(&sps.sps_seq_parameter_set_id) ||
      !br.ReadUe(&sps.chroma_format_idc)) {
    return false;
  }
  if (sps.chroma_format_idc > 3) {
    return false;
  }
  if (sps.chroma_format_idc == 3 &&
      !br.ReadFlag(&sps.separate_colour_plane_flag)) {
    return false;
  }

  if (!br.ReadUe(&sps.pic_width_in_luma_samples) ||
      !br.ReadUe(&sps.pic_height_in_luma_samples)) {
    return false;
  }
  if (sps.pic_width_in_luma_samples == 0 ||
      sps.pic_height_in_luma_samples == 0 ||
      sps.pic_width_in_luma_samples > kMaxDimension ||
      sps.pic_height_in_luma_samples > kMaxDimension) {
    return false;
  }

  uint32_t conf_win_left = 0;
  uint32_t conf_win_right = 0;
  uint32_t conf_win_top = 0;
  uint32_t conf_win_bottom = 0;
  bool conformance_window = false;
  if (!br.ReadFlag(&conformance_window)) {
    return false;
  }
  if (conformance_window) {
    if (!br.ReadUe(&conf_win_left) || !br.ReadUe(&conf_win_right) ||
        !br.ReadUe(&conf_win_top) || !br.ReadUe(&conf_win_bottom)) {
      return false;
    }
  }

  uint32_t bit_depth_luma_minus8 = 0;
  uint32_t bit_depth_chroma_minus8 = 0;
  uint32_t log2_max_poc_lsb_minus4 = 0;
  if (!br.ReadUe(&bit_depth_luma_minus8) ||
      !br.ReadUe(&bit_depth_chroma_minus8) ||
      !br.ReadUe(&log2_max_poc_lsb_minus4)) {
    return false;
  }
  if (bit_depth_luma_minus8 > 8 || bit_depth_chroma_minus8 > 8 ||
      log2_max_poc_lsb_minus4 > kMaxLog2Minus4) {
    return false;
  }
  sps.bit_depth_luma = bit_depth_luma_minus8 + 8;
  sps.bit_depth_chroma = bit_depth_chroma_minus8 + 8;
  sps.log2_max_pic_order_cnt_lsb = log2_max_poc_lsb_minus4 + 4;

  bool sub_layer_ordering_info_present = false;
  if (!br.ReadFlag(&sub_layer_ordering_info_present)) {
    return false;
  }
  for (uint32_t i = sub_layer_ordering_info_present ? 0 : max_sub_layers_minus1;
       i <= max_sub_layers_minus1; ++i) {
    uint32_t max_dec_pic_buffering_minus1 = 0;
    uint32_t max_num_reorder_pics = 0;
    uint32_t max_latency_increase_plus1 = 0;
    if (!br.ReadUe(&max_dec_pic_buffering_minus1) ||
        !br.ReadUe(&max_num_reorder_pics) ||
        !br.ReadUe(&max_latency_increase_plus1)) {
      return false;
    }
    if (max_dec_pic_buffering_minus1 >= kMaxRefPics) {
      return false;  // MaxDpbSize is at most 16
    }
    sps.sps_max_dec_pic_buffering_minus1 = max_dec_pic_buffering_minus1;
  }

  uint32_t log2_min_cb_minus3 = 0;
  uint32_t log2_diff_max_min_cb = 0;
  uint32_t log2_min_tb_minus2 = 0;
  uint32_t log2_diff_max_min_tb = 0;
  uint32_t max_transform_hierarchy_inter = 0;
  uint32_t max_transform_hierarchy_intra = 0;
  if (!br.ReadUe(&log2_min_cb_minus3) || !br.ReadUe(&log2_diff_max_min_cb) ||
      !br.ReadUe(&log2_min_tb_minus2) || !br.ReadUe(&log2_diff_max_min_tb) ||
      !br.ReadUe(&max_transform_hierarchy_inter) ||
      !br.ReadUe(&max_transform_hierarchy_intra)) {
    return false;
  }
  // MinCbLog2SizeY in [3,6] and CtbLog2SizeY in [4,6]; anything outside is
  // malformed and would also throw off the CTB-count derivation below.
  if (log2_min_cb_minus3 > 3 || log2_diff_max_min_cb > 3) {
    return false;
  }
  sps.log2_min_cb_size = log2_min_cb_minus3 + 3;
  sps.log2_ctb_size = sps.log2_min_cb_size + log2_diff_max_min_cb;
  if (sps.log2_ctb_size < 4 || sps.log2_ctb_size > 6) {
    return false;
  }
  // Transform-block geometry and hierarchy depths, kept for the decoder's
  // picture parameters. log2_min_tb_size is 2..5 and never exceeds the CTB.
  if (log2_min_tb_minus2 > 3 || log2_diff_max_min_tb > 3) {
    return false;
  }
  sps.log2_min_tb_size = log2_min_tb_minus2 + 2;
  sps.log2_diff_max_min_tb_size = log2_diff_max_min_tb;
  sps.max_transform_hierarchy_depth_inter = max_transform_hierarchy_inter;
  sps.max_transform_hierarchy_depth_intra = max_transform_hierarchy_intra;
  const uint32_t ctb_size = 1u << sps.log2_ctb_size;
  sps.pic_width_in_ctbs =
      (sps.pic_width_in_luma_samples + ctb_size - 1) / ctb_size;
  sps.pic_height_in_ctbs =
      (sps.pic_height_in_luma_samples + ctb_size - 1) / ctb_size;
  sps.pic_size_in_ctbs = sps.pic_width_in_ctbs * sps.pic_height_in_ctbs;

  if (!br.ReadFlag(&sps.scaling_list_enabled_flag)) {
    return false;
  }
  if (sps.scaling_list_enabled_flag) {
    if (!br.ReadFlag(&sps.sps_scaling_list_data_present_flag)) {
      return false;
    }
    if (sps.sps_scaling_list_data_present_flag) {
      if (!ParseScalingListData(&br, &sps.scaling_list)) {
        return false;
      }
    } else {
      // Enabled but not signalled: the default scaling lists apply.
      SetDefaultScalingList(&sps.scaling_list);
    }
  }

  if (!br.ReadFlag(&sps.amp_enabled_flag) ||
      !br.ReadFlag(&sps.sample_adaptive_offset_enabled_flag) ||
      !br.ReadFlag(&sps.pcm_enabled_flag)) {
    return false;
  }
  if (sps.pcm_enabled_flag) {
    uint32_t pcm_bit_depth_luma_minus1 = 0;
    uint32_t pcm_bit_depth_chroma_minus1 = 0;
    uint32_t log2_min_pcm_cb_minus3 = 0;
    uint32_t log2_diff_max_min_pcm_cb = 0;
    bool pcm_loop_filter_disabled = false;
    if (!br.ReadBits(4, &pcm_bit_depth_luma_minus1) ||
        !br.ReadBits(4, &pcm_bit_depth_chroma_minus1) ||
        !br.ReadUe(&log2_min_pcm_cb_minus3) ||
        !br.ReadUe(&log2_diff_max_min_pcm_cb) ||
        !br.ReadFlag(&pcm_loop_filter_disabled)) {
      return false;
    }
    if (log2_min_pcm_cb_minus3 > 3 || log2_diff_max_min_pcm_cb > 3) {
      return false;
    }
  }

  uint32_t num_short_term_rps = 0;
  if (!br.ReadUe(&num_short_term_rps)) {
    return false;
  }
  if (num_short_term_rps > kMaxShortTermRps) {
    return false;
  }
  sps.short_term_rps.reserve(num_short_term_rps);
  for (uint32_t i = 0; i < num_short_term_rps; ++i) {
    ShortTermRps rps;
    if (!ParseShortTermRps(&br, i, num_short_term_rps, sps.short_term_rps,
                           &rps)) {
      return false;
    }
    sps.short_term_rps.push_back(rps);
  }

  if (!br.ReadFlag(&sps.long_term_ref_pics_present_flag)) {
    return false;
  }
  if (sps.long_term_ref_pics_present_flag) {
    if (!br.ReadUe(&sps.num_long_term_ref_pics_sps)) {
      return false;
    }
    if (sps.num_long_term_ref_pics_sps > kMaxLongTermRps) {
      return false;
    }
    sps.used_by_curr_pic_lt_sps.reserve(sps.num_long_term_ref_pics_sps);
    sps.lt_ref_pic_poc_lsb_sps.reserve(sps.num_long_term_ref_pics_sps);
    for (uint32_t i = 0; i < sps.num_long_term_ref_pics_sps; ++i) {
      uint32_t lt_ref_pic_poc_lsb = 0;
      bool used_by_curr_pic_lt = false;
      if (!br.ReadBits(sps.log2_max_pic_order_cnt_lsb, &lt_ref_pic_poc_lsb) ||
          !br.ReadFlag(&used_by_curr_pic_lt)) {
        return false;
      }
      sps.lt_ref_pic_poc_lsb_sps.push_back(lt_ref_pic_poc_lsb);
      // Needed by the slice-segment header to derive NumPicTotalCurr for a
      // long-term reference that selects this SPS entry by lt_idx_sps.
      sps.used_by_curr_pic_lt_sps.push_back(used_by_curr_pic_lt);
    }
  }

  if (!br.ReadFlag(&sps.sps_temporal_mvp_enabled_flag) ||
      !br.ReadFlag(&sps.strong_intra_smoothing_enabled_flag)) {
    return false;
  }

  // VUI and the SPS extensions. The VUI is skipped; the range extension carries
  // coding-tool flags a decoder needs for the 4:2:2 / 4:4:4 profiles. Skipping
  // the VUI must be exact to land on the extension flags.
  bool vui_present = false;
  if (!br.ReadFlag(&vui_present)) {
    return false;
  }
  if (vui_present && !SkipVui(&br, max_sub_layers_minus1)) {
    return false;
  }
  bool sps_extension_present = false;
  if (!br.ReadFlag(&sps_extension_present)) {
    return false;
  }
  if (sps_extension_present) {
    bool sps_multilayer_extension = false;
    bool sps_3d_extension = false;
    bool sps_scc_extension = false;
    uint32_t sps_extension_4bits = 0;
    if (!br.ReadFlag(&sps.sps_range_extension_flag) ||
        !br.ReadFlag(&sps_multilayer_extension) ||
        !br.ReadFlag(&sps_3d_extension) || !br.ReadFlag(&sps_scc_extension) ||
        !br.ReadBits(4, &sps_extension_4bits)) {
      return false;
    }
    if (sps.sps_range_extension_flag &&
        !ParseSpsRangeExtension(&br, &sps.range_extension)) {
      return false;
    }
    // The multilayer / 3D / SCC / reserved extensions are not parsed; nothing
    // after them is consumed.
  }

  // Conformance-window cropping. SubWidthC/SubHeightC follow chroma_format_idc;
  // separate colour planes are coded as monochrome (1,1).
  uint32_t sub_width_c = 1;
  uint32_t sub_height_c = 1;
  if (!sps.separate_colour_plane_flag) {
    switch (sps.chroma_format_idc) {
      case 1:  // 4:2:0
        sub_width_c = 2;
        sub_height_c = 2;
        break;
      case 2:  // 4:2:2
        sub_width_c = 2;
        sub_height_c = 1;
        break;
      default:  // 0 monochrome, 3 4:4:4
        break;
    }
  }
  const uint64_t crop_x =
      static_cast<uint64_t>(conf_win_left + conf_win_right) * sub_width_c;
  const uint64_t crop_y =
      static_cast<uint64_t>(conf_win_top + conf_win_bottom) * sub_height_c;
  if (crop_x >= sps.pic_width_in_luma_samples ||
      crop_y >= sps.pic_height_in_luma_samples) {
    return false;  // crop would meet or exceed the coded size
  }
  sps.width = sps.pic_width_in_luma_samples - static_cast<uint32_t>(crop_x);
  sps.height = sps.pic_height_in_luma_samples - static_cast<uint32_t>(crop_y);

  *out = sps;
  return true;
}

}  // namespace v4l2wc::h265
