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

// Ceil(Log2(n)): the number of bits needed to hold values 0..n-1. Zero for
// n <= 1.
uint32_t CeilLog2(uint32_t n) {
  uint32_t bits = 0;
  while ((1u << bits) < n) {
    ++bits;
  }
  return bits;
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
        if (i < sh.num_long_term_sps) {
          if (ctx.num_long_term_ref_pics_sps > 1) {
            uint32_t lt_idx_sps = 0;
            if (!br.ReadBits(CeilLog2(ctx.num_long_term_ref_pics_sps),
                             &lt_idx_sps)) {
              return false;
            }
          }
        } else {
          uint32_t poc_lsb_lt = 0;
          bool used_by_curr_pic_lt = false;
          if (!br.ReadBits(ctx.log2_max_pic_order_cnt_lsb, &poc_lsb_lt) ||
              !br.ReadFlag(&used_by_curr_pic_lt)) {
            return false;
          }
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
  }

  // The parse stops here: ref_pic_lists_modification, pred_weight_table, the QP
  // and deblocking overrides, the entry-point offsets, and slice_data are out
  // of scope.

  *out = std::move(sh);
  return true;
}

}  // namespace v4l2wc::h265
