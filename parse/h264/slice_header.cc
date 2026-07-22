// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/h264/slice_header.h"

#include "parse/bit_reader.h"

namespace v4l2wc::h264 {

SliceContext::SliceContext() = default;

SliceHeader::SliceHeader() = default;
SliceHeader::~SliceHeader() = default;
SliceHeader::SliceHeader(const SliceHeader&) = default;
SliceHeader& SliceHeader::operator=(const SliceHeader&) = default;
SliceHeader::SliceHeader(SliceHeader&&) noexcept = default;
SliceHeader& SliceHeader::operator=(SliceHeader&&) noexcept = default;
namespace {

// Bounds on the attacker-controlled variable-length structures. Real slices
// stay well below these; the caps turn a hostile stream into a parse failure
// instead of unbounded work.
constexpr uint32_t kMaxRefListMods = 128;
constexpr uint32_t kMaxMmcoOps = 128;
constexpr uint32_t kMaxNumRefIdxMinus1 = 31;  // spec maximum

uint32_t SliceTypeMod5(uint32_t slice_type) { return slice_type % 5; }
bool IsP(uint32_t t) { return t == 0; }
bool IsB(uint32_t t) { return t == 1; }
bool IsSp(uint32_t t) { return t == 3; }
bool IsI(uint32_t t) { return t == 2; }
bool IsSi(uint32_t t) { return t == 4; }

// ref_pic_list_modification (spec 7.3.3.1). Reads entries until idc == 3.
bool ParseRefPicListMod(BitReader* br, std::vector<RefPicListMod>* out) {
  bool present = false;
  if (!br->ReadFlag(&present)) {
    return false;
  }
  if (!present) {
    return true;
  }
  for (;;) {
    if (out->size() > kMaxRefListMods) {
      return false;
    }
    RefPicListMod mod;
    if (!br->ReadUe(&mod.idc)) {
      return false;
    }
    if (mod.idc == 3) {
      break;
    }
    if (mod.idc > 3) {
      return false;
    }
    if (!br->ReadUe(
            &mod.value)) {  // abs_diff_pic_num_minus1 / long_term_pic_num
      return false;
    }
    out->push_back(mod);
  }
  return true;
}

// pred_weight_table (spec 7.3.3.2), read and discarded. num_ref_l0/l1 are the
// effective active counts (already capped by the caller).
bool SkipPredWeightTable(BitReader* br, uint32_t chroma_array_type,
                         uint32_t num_ref_l0, uint32_t num_ref_l1, bool is_b) {
  uint32_t ue = 0;
  int32_t se = 0;
  if (!br->ReadUe(&ue)) {  // luma_log2_weight_denom
    return false;
  }
  if (chroma_array_type != 0 && !br->ReadUe(&ue)) {  // chroma_log2_weight_denom
    return false;
  }
  for (int list = 0; list < (is_b ? 2 : 1); ++list) {
    const uint32_t count = (list == 0) ? num_ref_l0 : num_ref_l1;
    for (uint32_t i = 0; i < count; ++i) {
      bool luma_flag = false;
      if (!br->ReadFlag(&luma_flag)) {
        return false;
      }
      if (luma_flag && (!br->ReadSe(&se) || !br->ReadSe(&se))) {
        return false;  // luma_weight / luma_offset
      }
      if (chroma_array_type != 0) {
        bool chroma_flag = false;
        if (!br->ReadFlag(&chroma_flag)) {
          return false;
        }
        if (chroma_flag) {
          for (int j = 0; j < 2; ++j) {
            if (!br->ReadSe(&se) || !br->ReadSe(&se)) {
              return false;  // chroma_weight / chroma_offset
            }
          }
        }
      }
    }
  }
  return true;
}

// dec_ref_pic_marking (spec 7.3.3.3).
bool ParseDecRefPicMarking(BitReader* br, bool idr, SliceHeader* out) {
  if (idr) {
    return br->ReadFlag(&out->no_output_of_prior_pics_flag) &&
           br->ReadFlag(&out->long_term_reference_flag);
  }
  if (!br->ReadFlag(&out->adaptive_ref_pic_marking_mode_flag)) {
    return false;
  }
  if (!out->adaptive_ref_pic_marking_mode_flag) {
    return true;
  }
  for (;;) {
    if (out->mmco.size() > kMaxMmcoOps) {
      return false;
    }
    Mmco op;
    if (!br->ReadUe(&op.op)) {
      return false;
    }
    if (op.op == 0) {
      break;
    }
    if (op.op > 6) {
      return false;
    }
    // arg1: difference_of_pic_nums_minus1 (ops 1,3) or long_term_pic_num (op
    // 2).
    if (op.op == 1 || op.op == 2 || op.op == 3) {
      if (!br->ReadUe(&op.arg1)) {
        return false;
      }
    }
    // arg2: long_term_frame_idx (ops 3,6) or max_long_term_frame_idx_plus1 (4).
    if (op.op == 3 || op.op == 4 || op.op == 6) {
      if (!br->ReadUe(&op.arg2)) {
        return false;
      }
    }
    out->mmco.push_back(op);
  }
  return true;
}

}  // namespace

bool ParseSliceHeader(const uint8_t* rbsp, size_t size, uint32_t nal_ref_idc,
                      bool idr, const SliceContext& ctx, SliceHeader* out) {
  if (rbsp == nullptr || out == nullptr) {
    return false;
  }
  BitReader br(rbsp, size);
  SliceHeader sh;
  sh.idr = idr;

  if (!br.ReadUe(&sh.first_mb_in_slice) || !br.ReadUe(&sh.slice_type) ||
      !br.ReadUe(&sh.pic_parameter_set_id)) {
    return false;
  }
  if (ctx.separate_colour_plane_flag && !br.SkipBits(2)) {  // colour_plane_id
    return false;
  }
  if (ctx.log2_max_frame_num < 1 || ctx.log2_max_frame_num > 32) {
    return false;
  }
  if (!br.ReadBits(ctx.log2_max_frame_num, &sh.frame_num)) {
    return false;
  }
  if (!ctx.frame_mbs_only_flag) {
    if (!br.ReadFlag(&sh.field_pic_flag)) {
      return false;
    }
    if (sh.field_pic_flag && !br.ReadFlag(&sh.bottom_field_flag)) {
      return false;
    }
  }
  if (idr && !br.ReadUe(&sh.idr_pic_id)) {
    return false;
  }

  if (ctx.pic_order_cnt_type == 0) {
    if (ctx.log2_max_pic_order_cnt_lsb < 1 ||
        ctx.log2_max_pic_order_cnt_lsb > 32) {
      return false;
    }
    if (!br.ReadBits(ctx.log2_max_pic_order_cnt_lsb, &sh.pic_order_cnt_lsb)) {
      return false;
    }
    int32_t se = 0;
    if (ctx.bottom_field_pic_order_in_frame_present_flag &&
        !sh.field_pic_flag && !br.ReadSe(&se)) {  // delta_pic_order_cnt_bottom
      return false;
    }
  } else if (ctx.pic_order_cnt_type == 1 &&
             !ctx.delta_pic_order_always_zero_flag) {
    int32_t se = 0;
    if (!br.ReadSe(&se)) {  // delta_pic_order_cnt[0]
      return false;
    }
    if (ctx.bottom_field_pic_order_in_frame_present_flag &&
        !sh.field_pic_flag && !br.ReadSe(&se)) {  // delta_pic_order_cnt[1]
      return false;
    }
  }

  if (ctx.redundant_pic_cnt_present_flag) {
    uint32_t redundant = 0;
    if (!br.ReadUe(&redundant)) {
      return false;
    }
  }

  const uint32_t type = SliceTypeMod5(sh.slice_type);
  if (IsB(type) && !br.SkipBits(1)) {  // direct_spatial_mv_pred_flag
    return false;
  }

  sh.num_ref_idx_l0_active_minus1 = ctx.num_ref_idx_l0_default_active_minus1;
  sh.num_ref_idx_l1_active_minus1 = ctx.num_ref_idx_l1_default_active_minus1;
  if (IsP(type) || IsSp(type) || IsB(type)) {
    bool override_flag = false;
    if (!br.ReadFlag(&override_flag)) {
      return false;
    }
    if (override_flag) {
      if (!br.ReadUe(&sh.num_ref_idx_l0_active_minus1)) {
        return false;
      }
      if (IsB(type) && !br.ReadUe(&sh.num_ref_idx_l1_active_minus1)) {
        return false;
      }
    }
  }
  if (sh.num_ref_idx_l0_active_minus1 > kMaxNumRefIdxMinus1 ||
      sh.num_ref_idx_l1_active_minus1 > kMaxNumRefIdxMinus1) {
    return false;
  }

  // ref_pic_list_modification (non-MVC): L0 for P/SP/B, L1 for B.
  if (type != 2 && type != 4) {  // not I / SI
    if (!ParseRefPicListMod(&br, &sh.ref_pic_list_mod_l0)) {
      return false;
    }
  }
  if (IsB(type)) {
    if (!ParseRefPicListMod(&br, &sh.ref_pic_list_mod_l1)) {
      return false;
    }
  }

  const bool weighted = (ctx.weighted_pred_flag && (IsP(type) || IsSp(type))) ||
                        (ctx.weighted_bipred_idc == 1 && IsB(type));
  if (weighted &&
      !SkipPredWeightTable(&br, ctx.chroma_array_type,
                           sh.num_ref_idx_l0_active_minus1 + 1,
                           sh.num_ref_idx_l1_active_minus1 + 1, IsB(type))) {
    return false;
  }

  if (nal_ref_idc != 0 && !ParseDecRefPicMarking(&br, idr, &sh)) {
    return false;
  }

  // Remaining header syntax, parsed so that the recorded offset really is the
  // start of slice data (spec 7.3.3).
  uint32_t ue = 0;
  if (ctx.entropy_coding_mode_flag && !IsI(type) && !IsSi(type) &&
      !br.ReadUe(&ue)) {  // cabac_init_idc
    return false;
  }
  if (!br.ReadSe(&sh.slice_qp_delta)) {
    return false;
  }
  if (IsSp(type) || IsSi(type)) {
    int32_t se = 0;
    if ((IsSp(type) && !br.SkipBits(1)) ||  // sp_for_switch_flag
        !br.ReadSe(&se)) {                  // slice_qs_delta
      return false;
    }
  }
  if (ctx.deblocking_filter_control_present_flag) {
    uint32_t idc = 0;
    if (!br.ReadUe(&idc)) {  // disable_deblocking_filter_idc
      return false;
    }
    if (idc > 2) {
      return false;
    }
    if (idc != 1) {
      int32_t se = 0;
      if (!br.ReadSe(&se) ||  // slice_alpha_c0_offset_div2
          !br.ReadSe(&se)) {  // slice_beta_offset_div2
        return false;
      }
    }
  }
  // slice_group_change_cycle follows for map types 3..5, and its width depends
  // on PPS state this parser does not carry. Rather than report an offset that
  // silently excludes it, refuse: no supported profile uses slice groups
  // (constrained baseline prohibits them).
  if (ctx.num_slice_groups_minus1 > 0) {
    return false;
  }

  // Slice data begins here. CAVLC needs no alignment; for CABAC the consumer
  // aligns to the next byte itself. Recorded in RBSP bit space -- convert with
  // RbspToRawBitOffset() for a hardware slice_data_bit_offset.
  sh.slice_data_bit_offset_rbsp = static_cast<uint32_t>(br.bit_pos());

  *out = sh;
  return true;
}

}  // namespace v4l2wc::h264
