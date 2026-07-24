// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h265/slice_header.h"

namespace v4l2wc::h265 {

SliceContext::SliceContext() = default;
SliceHeader::SliceHeader() = default;
SliceHeader::~SliceHeader() = default;
SliceHeader::SliceHeader(const SliceHeader&) = default;
SliceHeader& SliceHeader::operator=(const SliceHeader&) = default;
SliceHeader::SliceHeader(SliceHeader&&) noexcept = default;
SliceHeader& SliceHeader::operator=(SliceHeader&&) noexcept = default;

namespace {

// Bound on num_long_term_sps + num_long_term_pics, which sizes the long-term
// reference loop. Well above any real count (both are DPB-bounded).
constexpr uint32_t kMaxLongTermTotal = kMaxLongTermRps + kMaxRefPics;

// Bound on num_entry_point_offsets. One entry point starts each tile or CTB
// row beyond the first, so PicSizeInCtbsY is a safe over-bound; kMaxDimension /
// 16 squared is the ceiling when the context does not supply the picture size.
constexpr uint32_t kMaxEntryPointOffsets =
    (kMaxDimension / 16) * (kMaxDimension / 16);

// Ceil(Log2(n)): the number of bits needed to hold values 0..n-1. Zero for
// n <= 1.
uint32_t CeilLog2(uint32_t n) {
  uint32_t bits = 0;
  while ((1u << bits) < n) {
    ++bits;
  }
  return bits;
}

// ref_pic_lists_modification (clause 7.3.6.2). Consumed and discarded; only
// correct advancement matters for the slice_data offset. list_entry_lX are
// u(Ceil(Log2(NumPicTotalCurr))).
bool SkipRefPicListsModification(BitReader* br, const SliceContext& ctx,
                                 const SliceHeader& sh) {
  const uint32_t entry_bits = CeilLog2(sh.num_pic_total_curr);
  bool ref_pic_list_modification_flag_l0 = false;
  if (!br->ReadFlag(&ref_pic_list_modification_flag_l0)) {
    return false;
  }
  if (ref_pic_list_modification_flag_l0) {
    for (uint32_t i = 0; i <= sh.num_ref_idx_l0_active_minus1; ++i) {
      if (!br->SkipBits(entry_bits)) {  // list_entry_l0[i]
        return false;
      }
    }
  }
  if (sh.slice_type == SliceType::kB) {
    bool ref_pic_list_modification_flag_l1 = false;
    if (!br->ReadFlag(&ref_pic_list_modification_flag_l1)) {
      return false;
    }
    if (ref_pic_list_modification_flag_l1) {
      for (uint32_t i = 0; i <= sh.num_ref_idx_l1_active_minus1; ++i) {
        if (!br->SkipBits(entry_bits)) {  // list_entry_l1[i]
          return false;
        }
      }
    }
  }
  return true;
}

// One reference list's weight entries within pred_weight_table (clause
// 7.3.6.3). num_active is num_ref_idx_lX_active_minus1.
bool SkipPredWeightList(BitReader* br, uint32_t chroma_array_type,
                        uint32_t num_active) {
  bool luma_weight_flag[kMaxRefIdxDefault + 1] = {};
  bool chroma_weight_flag[kMaxRefIdxDefault + 1] = {};
  for (uint32_t i = 0; i <= num_active; ++i) {
    if (!br->ReadFlag(&luma_weight_flag[i])) {
      return false;
    }
  }
  if (chroma_array_type != 0) {
    for (uint32_t i = 0; i <= num_active; ++i) {
      if (!br->ReadFlag(&chroma_weight_flag[i])) {
        return false;
      }
    }
  }
  int32_t ignore = 0;
  for (uint32_t i = 0; i <= num_active; ++i) {
    if (luma_weight_flag[i]) {
      if (!br->ReadSe(&ignore) || !br->ReadSe(&ignore)) {  // delta_luma_*
        return false;
      }
    }
    if (chroma_weight_flag[i]) {
      for (uint32_t j = 0; j < 2; ++j) {
        if (!br->ReadSe(&ignore) || !br->ReadSe(&ignore)) {  // delta_chroma_*
          return false;
        }
      }
    }
  }
  return true;
}

// pred_weight_table (clause 7.3.6.3). Consumed and discarded.
bool SkipPredWeightTable(BitReader* br, const SliceContext& ctx,
                         const SliceHeader& sh) {
  uint32_t luma_log2_weight_denom = 0;
  if (!br->ReadUe(&luma_log2_weight_denom)) {
    return false;
  }
  if (ctx.chroma_array_type != 0) {
    int32_t delta_chroma_log2_weight_denom = 0;
    if (!br->ReadSe(&delta_chroma_log2_weight_denom)) {
      return false;
    }
  }
  // Active counts are bounded to kMaxRefIdxDefault before this is called, so
  // the per-list flag arrays cannot overflow.
  if (!SkipPredWeightList(br, ctx.chroma_array_type,
                          sh.num_ref_idx_l0_active_minus1)) {
    return false;
  }
  if (sh.slice_type == SliceType::kB &&
      !SkipPredWeightList(br, ctx.chroma_array_type,
                          sh.num_ref_idx_l1_active_minus1)) {
    return false;
  }
  return true;
}

}  // namespace

bool ParseSliceHeader(const uint8_t* rbsp, size_t size, NalUnitType nal_type,
                      const SliceContext& ctx, SliceHeader* out) {
  if (rbsp == nullptr || out == nullptr) {
    return false;
  }
  BitReader br(rbsp, size);
  SliceHeader sh;

  if (!br.ReadFlag(&sh.first_slice_segment_in_pic_flag)) {
    return false;
  }
  if (IsIrap(nal_type) && !br.ReadFlag(&sh.no_output_of_prior_pics_flag)) {
    return false;
  }
  if (!br.ReadUe(&sh.slice_pic_parameter_set_id)) {
    return false;
  }

  if (!sh.first_slice_segment_in_pic_flag) {
    if (ctx.dependent_slice_segments_enabled_flag &&
        !br.ReadFlag(&sh.dependent_slice_segment_flag)) {
      return false;
    }
    if (!br.ReadBits(CeilLog2(ctx.pic_size_in_ctbs),
                     &sh.slice_segment_address)) {
      return false;
    }
    if (sh.slice_segment_address >= ctx.pic_size_in_ctbs) {
      return false;  // must address a CTB within the picture
    }
  }

  if (sh.dependent_slice_segment_flag) {
    // A dependent slice segment inherits everything below from its parent
    // independent segment; the header ends after the address.
    *out = std::move(sh);
    return true;
  }

  for (uint32_t i = 0; i < ctx.num_extra_slice_header_bits; ++i) {
    if (!br.SkipBits(1)) {  // slice_reserved_flag[i]
      return false;
    }
  }

  uint32_t slice_type = 0;
  if (!br.ReadUe(&slice_type) || slice_type > 2) {
    return false;
  }
  sh.slice_type = static_cast<SliceType>(slice_type);

  if (ctx.output_flag_present_flag && !br.ReadFlag(&sh.pic_output_flag)) {
    return false;
  }
  if (ctx.separate_colour_plane_flag && !br.ReadBits(2, &sh.colour_plane_id)) {
    return false;
  }

  uint32_t num_long_term_used = 0;
  if (!IsIdr(nal_type)) {
    if (!br.ReadBits(ctx.log2_max_pic_order_cnt_lsb,
                     &sh.slice_pic_order_cnt_lsb)) {
      return false;
    }
    if (!br.ReadFlag(&sh.short_term_ref_pic_set_sps_flag)) {
      return false;
    }
    const uint32_t num_st_rps =
        static_cast<uint32_t>(ctx.short_term_rps.size());
    if (!sh.short_term_ref_pic_set_sps_flag) {
      // Inline set, index == num_short_term_ref_pic_sets; may inter-predict
      // from an SPS-defined set.
      if (!ParseShortTermRps(&br, num_st_rps, num_st_rps, ctx.short_term_rps,
                             &sh.current_rps)) {
        return false;
      }
    } else if (num_st_rps > 1) {
      if (!br.ReadBits(CeilLog2(num_st_rps), &sh.short_term_ref_pic_set_idx)) {
        return false;
      }
      if (sh.short_term_ref_pic_set_idx >= num_st_rps) {
        return false;
      }
      sh.current_rps = ctx.short_term_rps[sh.short_term_ref_pic_set_idx];
    } else if (num_st_rps == 1) {
      sh.current_rps = ctx.short_term_rps[0];
    }

    if (ctx.long_term_ref_pics_present_flag) {
      if (ctx.num_long_term_ref_pics_sps > 0 &&
          !br.ReadUe(&sh.num_long_term_sps)) {
        return false;
      }
      if (!br.ReadUe(&sh.num_long_term_pics)) {
        return false;
      }
      if (sh.num_long_term_sps > ctx.num_long_term_ref_pics_sps ||
          sh.num_long_term_pics > kMaxRefPics ||
          sh.num_long_term_sps + sh.num_long_term_pics > kMaxLongTermTotal) {
        return false;
      }
      const uint32_t total = sh.num_long_term_sps + sh.num_long_term_pics;
      for (uint32_t i = 0; i < total; ++i) {
        bool used_by_curr_pic_lt = false;
        if (i < sh.num_long_term_sps) {
          uint32_t lt_idx_sps = 0;
          if (ctx.num_long_term_ref_pics_sps > 1 &&
              !br.ReadBits(CeilLog2(ctx.num_long_term_ref_pics_sps),
                           &lt_idx_sps)) {
            return false;
          }
          if (lt_idx_sps >= ctx.used_by_curr_pic_lt_sps.size()) {
            return false;
          }
          used_by_curr_pic_lt = ctx.used_by_curr_pic_lt_sps[lt_idx_sps];
        } else {
          uint32_t poc_lsb_lt = 0;
          if (!br.ReadBits(ctx.log2_max_pic_order_cnt_lsb, &poc_lsb_lt) ||
              !br.ReadFlag(&used_by_curr_pic_lt)) {
            return false;
          }
        }
        if (used_by_curr_pic_lt) {
          ++num_long_term_used;
        }
        bool delta_poc_msb_present = false;
        if (!br.ReadFlag(&delta_poc_msb_present)) {
          return false;
        }
        if (delta_poc_msb_present) {
          uint32_t delta_poc_msb_cycle_lt = 0;
          if (!br.ReadUe(&delta_poc_msb_cycle_lt)) {
            return false;
          }
        }
      }
    }

    if (ctx.sps_temporal_mvp_enabled_flag &&
        !br.ReadFlag(&sh.slice_temporal_mvp_enabled_flag)) {
      return false;
    }
  }

  if (ctx.sample_adaptive_offset_enabled_flag) {
    if (!br.ReadFlag(&sh.slice_sao_luma_flag)) {
      return false;
    }
    if (ctx.chroma_array_type != 0 && !br.ReadFlag(&sh.slice_sao_chroma_flag)) {
      return false;
    }
  }

  // Resolve the active reference counts, defaulting to the PPS values.
  sh.num_ref_idx_l0_active_minus1 = ctx.num_ref_idx_l0_default_active_minus1;
  sh.num_ref_idx_l1_active_minus1 = ctx.num_ref_idx_l1_default_active_minus1;
  // NumPicTotalCurr (clause 7.4.7.2): reference pictures used by the current
  // picture. pps_curr_pic_ref_enabled_flag is a range-extension feature not
  // parsed here, so it contributes nothing.
  uint32_t num_pic_total_curr = num_long_term_used;
  for (uint32_t i = 0; i < sh.current_rps.num_negative_pics; ++i) {
    if (sh.current_rps.used_s0[i]) {
      ++num_pic_total_curr;
    }
  }
  for (uint32_t i = 0; i < sh.current_rps.num_positive_pics; ++i) {
    if (sh.current_rps.used_s1[i]) {
      ++num_pic_total_curr;
    }
  }
  sh.num_pic_total_curr = num_pic_total_curr;

  if (sh.slice_type == SliceType::kP || sh.slice_type == SliceType::kB) {
    if (!br.ReadFlag(&sh.num_ref_idx_active_override_flag)) {
      return false;
    }
    if (sh.num_ref_idx_active_override_flag) {
      if (!br.ReadUe(&sh.num_ref_idx_l0_active_minus1)) {
        return false;
      }
      if (sh.slice_type == SliceType::kB &&
          !br.ReadUe(&sh.num_ref_idx_l1_active_minus1)) {
        return false;
      }
      if (sh.num_ref_idx_l0_active_minus1 > kMaxRefIdxDefault ||
          sh.num_ref_idx_l1_active_minus1 > kMaxRefIdxDefault) {
        return false;
      }
    }
    if (ctx.lists_modification_present_flag && sh.num_pic_total_curr > 1 &&
        !SkipRefPicListsModification(&br, ctx, sh)) {
      return false;
    }
    if (sh.slice_type == SliceType::kB && !br.ReadFlag(&sh.mvd_l1_zero_flag)) {
      return false;
    }
    if (ctx.cabac_init_present_flag && !br.ReadFlag(&sh.cabac_init_flag)) {
      return false;
    }
    if (sh.slice_temporal_mvp_enabled_flag) {
      if (sh.slice_type == SliceType::kB &&
          !br.ReadFlag(&sh.collocated_from_l0_flag)) {
        return false;
      }
      const bool from_l0 = sh.collocated_from_l0_flag;
      if (((from_l0 && sh.num_ref_idx_l0_active_minus1 > 0) ||
           (!from_l0 && sh.num_ref_idx_l1_active_minus1 > 0)) &&
          !br.ReadUe(&sh.collocated_ref_idx)) {
        return false;
      }
    }
    const bool weighted =
        (ctx.weighted_pred_flag && sh.slice_type == SliceType::kP) ||
        (ctx.weighted_bipred_flag && sh.slice_type == SliceType::kB);
    if (weighted && !SkipPredWeightTable(&br, ctx, sh)) {
      return false;
    }
    // MaxNumMergeCand = 5 - five_minus_max_num_merge_cand, which must be 1..5.
    if (!br.ReadUe(&sh.five_minus_max_num_merge_cand) ||
        sh.five_minus_max_num_merge_cand > 4) {
      return false;
    }
  }

  if (!br.ReadSe(&sh.slice_qp_delta)) {
    return false;
  }
  if (ctx.pps_slice_chroma_qp_offsets_present_flag &&
      (!br.ReadSe(&sh.slice_cb_qp_offset) ||
       !br.ReadSe(&sh.slice_cr_qp_offset))) {
    return false;
  }
  // The PPS range extension (cu_chroma_qp_offset_enabled_flag and
  // deblocking_filter_override at the range-extension level) is out of scope,
  // in line with the Main / Main-10 profiles this targets.
  bool deblocking_filter_override = false;
  if (ctx.deblocking_filter_override_enabled_flag &&
      !br.ReadFlag(&deblocking_filter_override)) {
    return false;
  }
  if (deblocking_filter_override) {
    if (!br.ReadFlag(&sh.slice_deblocking_filter_disabled_flag)) {
      return false;
    }
    if (!sh.slice_deblocking_filter_disabled_flag &&
        (!br.ReadSe(&sh.slice_beta_offset_div2) ||
         !br.ReadSe(&sh.slice_tc_offset_div2))) {
      return false;
    }
  } else {
    sh.slice_deblocking_filter_disabled_flag =
        ctx.pps_deblocking_filter_disabled_flag;
  }
  if (ctx.pps_loop_filter_across_slices_enabled_flag &&
      (sh.slice_sao_luma_flag || sh.slice_sao_chroma_flag ||
       !sh.slice_deblocking_filter_disabled_flag) &&
      !br.ReadFlag(&sh.slice_loop_filter_across_slices_enabled_flag)) {
    return false;
  }

  if (ctx.tiles_enabled_flag || ctx.entropy_coding_sync_enabled_flag) {
    if (!br.ReadUe(&sh.num_entry_point_offsets)) {
      return false;
    }
    if (sh.num_entry_point_offsets > kMaxEntryPointOffsets) {
      return false;
    }
    if (sh.num_entry_point_offsets > 0) {
      uint32_t offset_len_minus1 = 0;
      if (!br.ReadUe(&offset_len_minus1) || offset_len_minus1 > 31) {
        return false;
      }
      for (uint32_t i = 0; i < sh.num_entry_point_offsets; ++i) {
        if (!br.SkipBits(offset_len_minus1 + 1)) {  // entry_point_offset_minus1
          return false;
        }
      }
    }
  }

  if (ctx.slice_segment_header_extension_present_flag) {
    uint32_t ext_len = 0;
    if (!br.ReadUe(&ext_len)) {
      return false;
    }
    for (uint32_t i = 0; i < ext_len; ++i) {
      if (!br.SkipBits(8)) {  // slice_segment_header_extension_data_byte
        return false;
      }
    }
  }

  // byte_alignment(): a 1 bit followed by 0 bits to the next byte boundary;
  // slice_data() begins there. The 1 bit is a hard invariant -- a header that
  // mis-sized any earlier field lands here on a 0 bit or off a byte boundary,
  // so this doubles as an end-to-end check on the whole parse.
  bool alignment_bit_equal_to_one = false;
  if (!br.ReadFlag(&alignment_bit_equal_to_one) ||
      !alignment_bit_equal_to_one) {
    return false;
  }
  while ((br.bit_pos() & 7) != 0) {
    bool zero_bit = true;
    if (!br.ReadFlag(&zero_bit) || zero_bit) {
      return false;
    }
  }
  sh.slice_data_bit_offset_rbsp = static_cast<uint32_t>(br.bit_pos());

  *out = std::move(sh);
  return true;
}

}  // namespace v4l2wc::h265
