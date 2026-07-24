// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/vaapi_h265_decoder.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>

#include "src/log.h"
#include "va/va.h"
#include "va/va_dec_hevc.h"
#include "va/va_drm.h"
#include "va/va_drmcommon.h"

namespace v4l2wc {

std::unique_ptr<VaapiH265Decoder> VaapiH265Decoder::Create(
    const char* render_node, std::uint32_t pool_size) {
  auto dec = std::unique_ptr<VaapiH265Decoder>(new VaapiH265Decoder());
  if (!va::VaLoad(&dec->va_)) {
    V4L2WC_LOG(V4L2WC_WARNING) << "vaapi-h265: libva unavailable (dlopen)";
    return nullptr;
  }
  dec->drm_fd_ = ::open(render_node, O_RDWR | O_CLOEXEC);
  if (dec->drm_fd_ < 0) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "vaapi-h265: open(" << render_node << ") failed";
    return nullptr;
  }
  dec->dpy_ = dec->va_.GetDisplayDRM(dec->drm_fd_);
  if (!dec->dpy_) {
    V4L2WC_LOG(V4L2WC_ERROR) << "vaapi-h265: vaGetDisplayDRM failed";
    return nullptr;
  }
  int major = 0, minor = 0;
  VAStatus s = dec->va_.Initialize(dec->dpy_, &major, &minor);
  if (s != VA_STATUS_SUCCESS) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "vaapi-h265: vaInitialize: " << dec->va_.ErrorStr(s);
    return nullptr;
  }
  dec->pool_size_ = pool_size;
  V4L2WC_LOG(V4L2WC_INFO) << "vaapi-h265: VA-API " << major << "." << minor
                          << " on " << render_node;
  return dec;
}

VaapiH265Decoder::VaapiH265Decoder() = default;

VaapiH265Decoder::Slot::Slot() = default;
VaapiH265Decoder::Slot::~Slot() = default;
VaapiH265Decoder::Slot::Slot(const Slot&) = default;
VaapiH265Decoder::Slot& VaapiH265Decoder::Slot::operator=(const Slot&) =
    default;

// libva decodes synchronously in SubmitBitstream, so there is nothing to pump.
DriveResult VaapiH265Decoder::Drive() { return DriveResult::kOk; }

VaapiH265Decoder::~VaapiH265Decoder() {
  for (auto& sl : slots_)
    if (sl.fd >= 0) ::close(sl.fd);
  if (dpy_) {
    if (context_) va_.DestroyContext(dpy_, context_);
    if (!slots_.empty()) {
      std::vector<VASurfaceID> surfs;
      for (auto& sl : slots_) surfs.push_back(sl.surface);
      va_.DestroySurfaces(dpy_, surfs.data(), static_cast<int>(surfs.size()));
    }
    if (config_) va_.DestroyConfig(dpy_, config_);
    va_.Terminate(dpy_);
  }
  if (drm_fd_ >= 0) ::close(drm_fd_);
}

bool VaapiH265Decoder::EnsureConfigured(const h265::Sps& sps) {
  if (configured_) return true;
  coded_w_ = sps.pic_width_in_luma_samples;
  coded_h_ = sps.pic_height_in_luma_samples;
  VAConfigAttrib attr = {VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420};
  VAConfigID cfg = 0;
  VAStatus s = va_.CreateConfig(dpy_, VAProfileHEVCMain, VAEntrypointVLD, &attr,
                                1, &cfg);
  if (s != VA_STATUS_SUCCESS) {
    V4L2WC_LOG(V4L2WC_ERROR) << "vaapi-h265: CreateConfig: " << va_.ErrorStr(s);
    return false;
  }
  config_ = cfg;
  std::vector<VASurfaceID> surfs(pool_size_);
  VASurfaceAttrib sa = {VASurfaceAttribPixelFormat,
                        VA_SURFACE_ATTRIB_SETTABLE,
                        {VAGenericValueTypeInteger, {.i = VA_FOURCC_NV12}}};
  s = va_.CreateSurfaces(dpy_, VA_RT_FORMAT_YUV420, coded_w_, coded_h_,
                         surfs.data(), pool_size_, &sa, 1);
  if (s != VA_STATUS_SUCCESS) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "vaapi-h265: CreateSurfaces: " << va_.ErrorStr(s);
    return false;
  }
  slots_.resize(pool_size_);
  for (std::uint32_t i = 0; i < pool_size_; ++i) slots_[i].surface = surfs[i];
  VAContextID ctx = 0;
  s = va_.CreateContext(dpy_, config_, coded_w_, coded_h_, VA_PROGRESSIVE,
                        surfs.data(), pool_size_, &ctx);
  if (s != VA_STATUS_SUCCESS) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "vaapi-h265: CreateContext: " << va_.ErrorStr(s);
    return false;
  }
  context_ = ctx;
  configured_ = true;
  V4L2WC_LOG(V4L2WC_INFO) << "vaapi-h265: configured " << coded_w_ << "x"
                          << coded_h_ << " pool=" << pool_size_;
  return true;
}

int VaapiH265Decoder::PickFreeSlot() {
  for (std::uint32_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i].is_reference || slots_[i].checked_out) continue;
    if (have_ready_ && i == ready_slot_) continue;
    return static_cast<int>(i);
  }
  return -1;
}

int VaapiH265Decoder::ComputePoc(const h265::SliceHeader& sh,
                                 const h265::Nal& nal) {
  const int max_poc_lsb = 1 << sps_.log2_max_pic_order_cnt_lsb;
  const int poc_lsb = static_cast<int>(sh.slice_pic_order_cnt_lsb);
  const bool irap = h265::IsIrap(nal.type);
  // NoRaslOutputFlag is 1 for an IDR and for the first IRAP in the stream (a
  // clean random-access point), where the POC MSB resets to 0. A later CRA
  // keeps its leading pictures and uses the normal derivation. BLA, which would
  // also reset, is treated as a plain IRAP here.
  const bool no_rasl_output =
      h265::IsIdr(nal.type) || (irap && !seen_first_picture_);
  int poc_msb = 0;
  if (!no_rasl_output) {
    if (poc_lsb < prev_poc_lsb_ && (prev_poc_lsb_ - poc_lsb) >= max_poc_lsb / 2)
      poc_msb = prev_poc_msb_ + max_poc_lsb;
    else if (poc_lsb > prev_poc_lsb_ &&
             (poc_lsb - prev_poc_lsb_) > max_poc_lsb / 2)
      poc_msb = prev_poc_msb_ - max_poc_lsb;
    else
      poc_msb = prev_poc_msb_;
  }
  const int poc = poc_msb + poc_lsb;

  // Update the prevTid0 anchor from a TemporalId-0 picture that is not a RASL
  // or RADL picture (clause 8.3.1).
  const int temporal_id = nal.nuh_temporal_id_plus1 - 1;
  const bool rasl_or_radl = nal.type == h265::NalUnitType::kRadlN ||
                            nal.type == h265::NalUnitType::kRadlR ||
                            nal.type == h265::NalUnitType::kRaslN ||
                            nal.type == h265::NalUnitType::kRaslR;
  if (temporal_id == 0 && !rasl_or_radl) {
    prev_poc_lsb_ = poc_lsb;
    prev_poc_msb_ = poc_msb;
  }
  seen_first_picture_ = true;
  return poc;
}

bool VaapiH265Decoder::DecodeSlice(const h265::Nal& nal) {
  // Gather the active SPS/PPS state the slice parser needs.
  h265::SliceContext ctx;
  ctx.pic_size_in_ctbs = sps_.pic_size_in_ctbs;
  ctx.log2_max_pic_order_cnt_lsb = sps_.log2_max_pic_order_cnt_lsb;
  ctx.separate_colour_plane_flag = sps_.separate_colour_plane_flag;
  ctx.chroma_array_type =
      sps_.separate_colour_plane_flag ? 0 : sps_.chroma_format_idc;
  ctx.sample_adaptive_offset_enabled_flag =
      sps_.sample_adaptive_offset_enabled_flag;
  ctx.sps_temporal_mvp_enabled_flag = sps_.sps_temporal_mvp_enabled_flag;
  ctx.long_term_ref_pics_present_flag = sps_.long_term_ref_pics_present_flag;
  ctx.num_long_term_ref_pics_sps = sps_.num_long_term_ref_pics_sps;
  ctx.used_by_curr_pic_lt_sps = sps_.used_by_curr_pic_lt_sps;
  ctx.short_term_rps = sps_.short_term_rps;
  ctx.dependent_slice_segments_enabled_flag =
      pps_.dependent_slice_segments_enabled_flag;
  ctx.num_extra_slice_header_bits = pps_.num_extra_slice_header_bits;
  ctx.output_flag_present_flag = pps_.output_flag_present_flag;
  ctx.num_ref_idx_l0_default_active_minus1 =
      pps_.num_ref_idx_l0_default_active_minus1;
  ctx.num_ref_idx_l1_default_active_minus1 =
      pps_.num_ref_idx_l1_default_active_minus1;
  ctx.cabac_init_present_flag = pps_.cabac_init_present_flag;
  ctx.weighted_pred_flag = pps_.weighted_pred_flag;
  ctx.weighted_bipred_flag = pps_.weighted_bipred_flag;
  ctx.pps_slice_chroma_qp_offsets_present_flag =
      pps_.pps_slice_chroma_qp_offsets_present_flag;
  ctx.deblocking_filter_override_enabled_flag =
      pps_.deblocking_filter_override_enabled_flag;
  ctx.pps_deblocking_filter_disabled_flag =
      pps_.pps_deblocking_filter_disabled_flag;
  ctx.pps_loop_filter_across_slices_enabled_flag =
      pps_.pps_loop_filter_across_slices_enabled_flag;
  ctx.tiles_enabled_flag = pps_.tiles_enabled_flag;
  ctx.entropy_coding_sync_enabled_flag = pps_.entropy_coding_sync_enabled_flag;
  ctx.lists_modification_present_flag = pps_.lists_modification_present_flag;
  ctx.slice_segment_header_extension_present_flag =
      pps_.slice_segment_header_extension_present_flag;

  h265::SliceHeader sh{};
  if (!h265::ParseSliceHeader(nal.rbsp.data(), nal.rbsp.size(), nal.type, ctx,
                              &sh)) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "vaapi-h265: malformed slice header; dropping";
    return false;
  }

  // Unsupported tools are dropped rather than mis-decoded. A non-flat scaling
  // list needs scaling_list_data (the parser skips it) and an IQ-matrix buffer;
  // long-term references need their POC bookkeeping. Common HLS HEVC uses
  // neither. Reference-list modification is now applied (see below).
  if (sps_.scaling_list_enabled_flag) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "vaapi-h265: scaling lists unsupported; dropping";
    return false;
  }
  if (pps_.tiles_enabled_flag) {
    V4L2WC_LOG(V4L2WC_WARNING) << "vaapi-h265: tiles unsupported; dropping";
    return false;
  }
  if (sps_.long_term_ref_pics_present_flag) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "vaapi-h265: long-term references unsupported; dropping";
    return false;
  }

  // The hardware addresses slice data in the raw NAL. slice_data_byte_offset is
  // measured in the emulation-removed domain and includes the two-byte NAL
  // header; the driver re-adds the emulation bytes it is told about.
  std::uint32_t raw_bit_offset = 0;
  if (!h265::RbspToRawBitOffset(nal, sh.slice_data_bit_offset_rbsp,
                                &raw_bit_offset)) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "vaapi-h265: slice data offset outside NAL; dropping";
    return false;
  }
  const std::uint32_t rbsp_byte_offset = sh.slice_data_bit_offset_rbsp / 8;
  const std::uint32_t slice_data_byte_offset = 2 + rbsp_byte_offset;
  // Emulation bytes removed between the NAL header and slice_data: the raw
  // position minus the emulation-removed position (2-byte header + rbsp bytes).
  const std::uint32_t num_emu_bytes =
      raw_bit_offset / 8 - slice_data_byte_offset;

  const int poc = ComputePoc(sh, nal);

  // An IDR clears the DPB before its own reference-picture set is applied.
  if (h265::IsIdr(nal.type)) {
    for (auto& r : dpb_) slots_[r.slot].is_reference = false;
    dpb_.clear();
  }

  // Reference-picture-set derivation (clause 8.3.2). Each used short-term entry
  // resolves to a DPB picture by POC; the not-used ("follow") entries are kept
  // in the DPB so a later picture can reference them. A reference the DPB no
  // longer holds (e.g. a dropped frame) resolves to an invalid surface.
  struct RpsEntry {
    int poc;
    std::uint32_t slot;
    bool found;
  };
  std::vector<RpsEntry> st_curr_before, st_curr_after;
  std::vector<int> kept_pocs;
  auto find_ref = [&](int target) -> RpsEntry {
    for (const auto& r : dpb_)
      if (r.poc == target) return {target, r.slot, true};
    return {target, 0, false};
  };
  for (std::uint32_t i = 0; i < sh.current_rps.num_negative_pics; ++i) {
    const int tpoc = poc + sh.current_rps.delta_poc_s0[i];
    kept_pocs.push_back(tpoc);
    if (sh.current_rps.used_s0[i]) st_curr_before.push_back(find_ref(tpoc));
  }
  for (std::uint32_t i = 0; i < sh.current_rps.num_positive_pics; ++i) {
    const int tpoc = poc + sh.current_rps.delta_poc_s1[i];
    kept_pocs.push_back(tpoc);
    if (sh.current_rps.used_s1[i]) st_curr_after.push_back(find_ref(tpoc));
  }

  // Evict every DPB picture the current set does not keep (marks it unused for
  // reference; its surface frees for reuse).
  {
    std::vector<RefPic> kept;
    for (const auto& r : dpb_) {
      bool keep = false;
      for (int p : kept_pocs)
        if (p == r.poc) {
          keep = true;
          break;
        }
      if (keep) {
        kept.push_back(r);
      } else {
        slots_[r.slot].is_reference = false;
      }
    }
    dpb_.swap(kept);
  }

  int slot = PickFreeSlot();
  if (slot < 0) {
    V4L2WC_LOG(V4L2WC_ERROR) << "vaapi-h265: no free surface";
    return false;
  }
  const VASurfaceID surf = slots_[slot].surface;

  VAPictureParameterBufferHEVC pp{};
  pp.CurrPic.picture_id = surf;
  pp.CurrPic.pic_order_cnt = poc;
  pp.CurrPic.flags = 0;
  for (auto& ref : pp.ReferenceFrames) {
    ref.picture_id = VA_INVALID_SURFACE;
    ref.flags = VA_PICTURE_HEVC_INVALID;
  }
  // ReferenceFrames[] holds the DPB pictures; a slot -> ReferenceFrames index
  // map lets the slice reference lists point into it. Pictures used by the
  // current picture carry the ST_CURR_BEFORE / ST_CURR_AFTER flags.
  int ref_index_by_slot[64];
  for (int& e : ref_index_by_slot) e = -1;
  int num_ref = 0;
  for (const auto& r : dpb_) {
    if (num_ref >= 15) break;
    VAPictureHEVC& p = pp.ReferenceFrames[num_ref];
    p.picture_id = slots_[r.slot].surface;
    p.pic_order_cnt = r.poc;
    p.flags = 0;
    for (const auto& e : st_curr_before)
      if (e.found && e.slot == r.slot)
        p.flags |= VA_PICTURE_HEVC_RPS_ST_CURR_BEFORE;
    for (const auto& e : st_curr_after)
      if (e.found && e.slot == r.slot)
        p.flags |= VA_PICTURE_HEVC_RPS_ST_CURR_AFTER;
    if (r.slot < 64) ref_index_by_slot[r.slot] = num_ref;
    ++num_ref;
  }

  pp.pic_width_in_luma_samples =
      static_cast<std::uint16_t>(sps_.pic_width_in_luma_samples);
  pp.pic_height_in_luma_samples =
      static_cast<std::uint16_t>(sps_.pic_height_in_luma_samples);

  pp.pic_fields.bits.chroma_format_idc = sps_.chroma_format_idc;
  pp.pic_fields.bits.separate_colour_plane_flag =
      sps_.separate_colour_plane_flag;
  pp.pic_fields.bits.pcm_enabled_flag = sps_.pcm_enabled_flag;
  pp.pic_fields.bits.scaling_list_enabled_flag = sps_.scaling_list_enabled_flag;
  pp.pic_fields.bits.transform_skip_enabled_flag =
      pps_.transform_skip_enabled_flag;
  pp.pic_fields.bits.amp_enabled_flag = sps_.amp_enabled_flag;
  pp.pic_fields.bits.strong_intra_smoothing_enabled_flag =
      sps_.strong_intra_smoothing_enabled_flag;
  pp.pic_fields.bits.sign_data_hiding_enabled_flag =
      pps_.sign_data_hiding_enabled_flag;
  pp.pic_fields.bits.constrained_intra_pred_flag =
      pps_.constrained_intra_pred_flag;
  pp.pic_fields.bits.cu_qp_delta_enabled_flag = pps_.cu_qp_delta_enabled_flag;
  pp.pic_fields.bits.weighted_pred_flag = pps_.weighted_pred_flag;
  pp.pic_fields.bits.weighted_bipred_flag = pps_.weighted_bipred_flag;
  pp.pic_fields.bits.transquant_bypass_enabled_flag =
      pps_.transquant_bypass_enabled_flag;
  pp.pic_fields.bits.tiles_enabled_flag = pps_.tiles_enabled_flag;
  pp.pic_fields.bits.entropy_coding_sync_enabled_flag =
      pps_.entropy_coding_sync_enabled_flag;
  pp.pic_fields.bits.pps_loop_filter_across_slices_enabled_flag =
      pps_.pps_loop_filter_across_slices_enabled_flag;
  pp.pic_fields.bits.loop_filter_across_tiles_enabled_flag =
      pps_.loop_filter_across_tiles_enabled_flag;

  pp.sps_max_dec_pic_buffering_minus1 =
      static_cast<std::uint8_t>(sps_.sps_max_dec_pic_buffering_minus1);
  pp.bit_depth_luma_minus8 = static_cast<std::uint8_t>(sps_.bit_depth_luma - 8);
  pp.bit_depth_chroma_minus8 =
      static_cast<std::uint8_t>(sps_.bit_depth_chroma - 8);
  pp.log2_min_luma_coding_block_size_minus3 =
      static_cast<std::uint8_t>(sps_.log2_min_cb_size - 3);
  pp.log2_diff_max_min_luma_coding_block_size =
      static_cast<std::uint8_t>(sps_.log2_ctb_size - sps_.log2_min_cb_size);
  pp.log2_min_transform_block_size_minus2 =
      static_cast<std::uint8_t>(sps_.log2_min_tb_size - 2);
  pp.log2_diff_max_min_transform_block_size =
      static_cast<std::uint8_t>(sps_.log2_diff_max_min_tb_size);
  pp.max_transform_hierarchy_depth_intra =
      static_cast<std::uint8_t>(sps_.max_transform_hierarchy_depth_intra);
  pp.max_transform_hierarchy_depth_inter =
      static_cast<std::uint8_t>(sps_.max_transform_hierarchy_depth_inter);
  pp.init_qp_minus26 = static_cast<std::int8_t>(pps_.init_qp_minus26);
  pp.diff_cu_qp_delta_depth =
      static_cast<std::uint8_t>(pps_.diff_cu_qp_delta_depth);
  pp.pps_cb_qp_offset = static_cast<std::int8_t>(pps_.pps_cb_qp_offset);
  pp.pps_cr_qp_offset = static_cast<std::int8_t>(pps_.pps_cr_qp_offset);
  pp.log2_parallel_merge_level_minus2 =
      static_cast<std::uint8_t>(pps_.log2_parallel_merge_level_minus2);
  pp.num_tile_columns_minus1 =
      static_cast<std::uint8_t>(pps_.num_tile_columns_minus1);
  pp.num_tile_rows_minus1 =
      static_cast<std::uint8_t>(pps_.num_tile_rows_minus1);

  pp.slice_parsing_fields.bits.lists_modification_present_flag =
      pps_.lists_modification_present_flag;
  pp.slice_parsing_fields.bits.long_term_ref_pics_present_flag =
      sps_.long_term_ref_pics_present_flag;
  pp.slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag =
      sps_.sps_temporal_mvp_enabled_flag;
  pp.slice_parsing_fields.bits.cabac_init_present_flag =
      pps_.cabac_init_present_flag;
  pp.slice_parsing_fields.bits.output_flag_present_flag =
      pps_.output_flag_present_flag;
  pp.slice_parsing_fields.bits.dependent_slice_segments_enabled_flag =
      pps_.dependent_slice_segments_enabled_flag;
  pp.slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag =
      pps_.pps_slice_chroma_qp_offsets_present_flag;
  pp.slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag =
      sps_.sample_adaptive_offset_enabled_flag;
  pp.slice_parsing_fields.bits.deblocking_filter_override_enabled_flag =
      pps_.deblocking_filter_override_enabled_flag;
  pp.slice_parsing_fields.bits.pps_disable_deblocking_filter_flag =
      pps_.pps_deblocking_filter_disabled_flag;
  pp.slice_parsing_fields.bits.slice_segment_header_extension_present_flag =
      pps_.slice_segment_header_extension_present_flag;
  pp.slice_parsing_fields.bits.RapPicFlag = h265::IsIrap(nal.type) ? 1 : 0;
  pp.slice_parsing_fields.bits.IdrPicFlag = h265::IsIdr(nal.type) ? 1 : 0;
  // Single-slice pictures (WPP, the packager default): the picture is intra iff
  // this slice is.
  pp.slice_parsing_fields.bits.IntraPicFlag =
      sh.slice_type == h265::SliceType::kI ? 1 : 0;

  pp.log2_max_pic_order_cnt_lsb_minus4 =
      static_cast<std::uint8_t>(sps_.log2_max_pic_order_cnt_lsb - 4);
  pp.num_short_term_ref_pic_sets =
      static_cast<std::uint8_t>(sps_.short_term_rps.size());
  pp.num_long_term_ref_pic_sps =
      static_cast<std::uint8_t>(sps_.num_long_term_ref_pics_sps);
  pp.num_ref_idx_l0_default_active_minus1 =
      static_cast<std::uint8_t>(pps_.num_ref_idx_l0_default_active_minus1);
  pp.num_ref_idx_l1_default_active_minus1 =
      static_cast<std::uint8_t>(pps_.num_ref_idx_l1_default_active_minus1);
  pp.pps_beta_offset_div2 = static_cast<std::int8_t>(pps_.pps_beta_offset_div2);
  pp.pps_tc_offset_div2 = static_cast<std::int8_t>(pps_.pps_tc_offset_div2);
  pp.num_extra_slice_header_bits =
      static_cast<std::uint8_t>(pps_.num_extra_slice_header_bits);
  pp.st_rps_bits = sh.short_term_ref_pic_set_bits;

  VASliceParameterBufferHEVC sp{};
  sp.slice_data_size = static_cast<std::uint32_t>(nal.raw.size());
  sp.slice_data_offset = 0;
  sp.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
  sp.slice_data_byte_offset = slice_data_byte_offset;
  sp.slice_segment_address = sh.slice_segment_address;
  for (auto& list : sp.RefPicList)
    for (auto& e : list) e = 0xFF;

  // Reference lists (clause 8.3.4). The temporary list cycles through the
  // current picture's short-term sets until it is at least as long as the
  // active count. Without reference-list modification RefPicListX[i] is
  // RefPicListTempX[i]; with it, RefPicListTempX[list_entry_lX[i]]. Each entry
  // is stored as its index into ReferenceFrames[].
  const bool is_p = sh.slice_type == h265::SliceType::kP;
  const bool is_b = sh.slice_type == h265::SliceType::kB;
  // A reference the DPB no longer holds maps to 0xFF (invalid); it never
  // aliases the picture that happens to occupy slot 0.
  auto ref_of = [&](const RpsEntry& e) -> std::uint8_t {
    return (e.found && e.slot < 64 && ref_index_by_slot[e.slot] >= 0)
               ? static_cast<std::uint8_t>(ref_index_by_slot[e.slot])
               : 0xFF;
  };
  auto build_list = [](const std::vector<std::uint8_t>& temp, bool modified,
                       const std::uint32_t* list_entry,
                       std::uint32_t active_minus1, std::uint8_t* out_list) {
    if (temp.empty()) return;
    for (std::uint32_t i = 0; i <= active_minus1 && i < 15; ++i) {
      std::uint32_t idx = modified ? list_entry[i] : i % temp.size();
      idx %= temp.size();  // the parser bounds list_entry; clamp defensively
      out_list[i] = temp[idx];
    }
  };
  if (is_p || is_b) {
    // L0: StCurrBefore, then StCurrAfter.
    std::vector<std::uint8_t> temp0;
    for (const auto& e : st_curr_before) temp0.push_back(ref_of(e));
    for (const auto& e : st_curr_after) temp0.push_back(ref_of(e));
    build_list(temp0, sh.ref_pic_list_modification_flag_l0, sh.list_entry_l0,
               sh.num_ref_idx_l0_active_minus1, sp.RefPicList[0]);
    if (is_b) {
      // L1: StCurrAfter, then StCurrBefore.
      std::vector<std::uint8_t> temp1;
      for (const auto& e : st_curr_after) temp1.push_back(ref_of(e));
      for (const auto& e : st_curr_before) temp1.push_back(ref_of(e));
      build_list(temp1, sh.ref_pic_list_modification_flag_l1, sh.list_entry_l1,
                 sh.num_ref_idx_l1_active_minus1, sp.RefPicList[1]);
    }
  }
  sp.LongSliceFlags.fields.LastSliceOfPic = 1;
  sp.LongSliceFlags.fields.dependent_slice_segment_flag =
      sh.dependent_slice_segment_flag;
  sp.LongSliceFlags.fields.slice_type =
      static_cast<std::uint32_t>(sh.slice_type);
  sp.LongSliceFlags.fields.color_plane_id = sh.colour_plane_id;
  sp.LongSliceFlags.fields.slice_sao_luma_flag = sh.slice_sao_luma_flag;
  sp.LongSliceFlags.fields.slice_sao_chroma_flag = sh.slice_sao_chroma_flag;
  sp.LongSliceFlags.fields.slice_temporal_mvp_enabled_flag =
      sh.slice_temporal_mvp_enabled_flag;
  sp.LongSliceFlags.fields.slice_deblocking_filter_disabled_flag =
      sh.slice_deblocking_filter_disabled_flag;
  sp.LongSliceFlags.fields.collocated_from_l0_flag = sh.collocated_from_l0_flag;
  sp.LongSliceFlags.fields.slice_loop_filter_across_slices_enabled_flag =
      sh.slice_loop_filter_across_slices_enabled_flag;
  // collocated_ref_idx indexes the collocated reference list; only meaningful
  // when temporal MVP is on (0xFF otherwise, e.g. an intra slice).
  sp.collocated_ref_idx = sh.slice_temporal_mvp_enabled_flag
                              ? static_cast<std::uint8_t>(sh.collocated_ref_idx)
                              : 0xFF;
  sp.num_ref_idx_l0_active_minus1 =
      static_cast<std::uint8_t>(sh.num_ref_idx_l0_active_minus1);
  sp.num_ref_idx_l1_active_minus1 =
      static_cast<std::uint8_t>(sh.num_ref_idx_l1_active_minus1);
  sp.slice_qp_delta = static_cast<std::int8_t>(sh.slice_qp_delta);
  sp.slice_cb_qp_offset = static_cast<std::int8_t>(sh.slice_cb_qp_offset);
  sp.slice_cr_qp_offset = static_cast<std::int8_t>(sh.slice_cr_qp_offset);
  sp.slice_beta_offset_div2 =
      static_cast<std::int8_t>(sh.slice_beta_offset_div2);
  sp.slice_tc_offset_div2 = static_cast<std::int8_t>(sh.slice_tc_offset_div2);
  sp.five_minus_max_num_merge_cand =
      static_cast<std::uint8_t>(sh.five_minus_max_num_merge_cand);
  sp.num_entry_point_offsets =
      static_cast<std::uint16_t>(sh.num_entry_point_offsets);
  sp.slice_data_num_emu_prevn_bytes = static_cast<std::uint16_t>(num_emu_bytes);

  // Weighted prediction (clause 7.4.7.3). VA takes the luma weight/offset
  // deltas as-is and the derived chroma offsets. An unflagged reference keeps
  // the default weight (delta 0) and zero offset. The common HLS stream signals
  // the table with default weights, so this is usually all zeros.
  if (is_p || is_b) {
    const auto& w = sh.pred_weight;
    sp.luma_log2_weight_denom =
        static_cast<std::uint8_t>(w.luma_log2_weight_denom);
    sp.delta_chroma_log2_weight_denom =
        static_cast<std::int8_t>(w.delta_chroma_log2_weight_denom);
    const int chroma_denom = static_cast<int>(w.luma_log2_weight_denom) +
                             w.delta_chroma_log2_weight_denom;
    constexpr int kWpOffsetHalfRangeC =
        128;  // 8-bit, no high-precision offsets
    auto fill = [&](int list, std::uint32_t active_minus1,
                    std::int8_t (&dlw)[15], std::int8_t (&lo)[15],
                    std::int8_t (&dcw)[15][2], std::int8_t (&co)[15][2]) {
      for (std::uint32_t i = 0; i <= active_minus1 && i < 15; ++i) {
        if (w.luma_weight_flag[list][i]) {
          dlw[i] = static_cast<std::int8_t>(w.delta_luma_weight[list][i]);
          lo[i] = static_cast<std::int8_t>(w.luma_offset[list][i]);
        }
        if (w.chroma_weight_flag[list][i]) {
          for (int j = 0; j < 2; ++j) {
            const int cweight =
                (1 << chroma_denom) + w.delta_chroma_weight[list][i][j];
            dcw[i][j] =
                static_cast<std::int8_t>(w.delta_chroma_weight[list][i][j]);
            int off = kWpOffsetHalfRangeC + w.delta_chroma_offset[list][i][j] -
                      ((kWpOffsetHalfRangeC * cweight) >> chroma_denom);
            if (off < -kWpOffsetHalfRangeC) off = -kWpOffsetHalfRangeC;
            if (off > kWpOffsetHalfRangeC - 1) off = kWpOffsetHalfRangeC - 1;
            co[i][j] = static_cast<std::int8_t>(off);
          }
        }
      }
    };
    fill(0, sh.num_ref_idx_l0_active_minus1, sp.delta_luma_weight_l0,
         sp.luma_offset_l0, sp.delta_chroma_weight_l0, sp.ChromaOffsetL0);
    if (is_b)
      fill(1, sh.num_ref_idx_l1_active_minus1, sp.delta_luma_weight_l1,
           sp.luma_offset_l1, sp.delta_chroma_weight_l1, sp.ChromaOffsetL1);
  }

  VABufferID b_pp = 0, b_sp = 0, b_sd = 0;
  auto ck = [&](VAStatus s, const char* what) {
    if (s != VA_STATUS_SUCCESS)
      V4L2WC_LOG(V4L2WC_ERROR)
          << "vaapi-h265: " << what << ": " << va_.ErrorStr(s);
    return s == VA_STATUS_SUCCESS;
  };
  if (!ck(va_.CreateBuffer(dpy_, context_, VAPictureParameterBufferType,
                           sizeof(pp), 1, &pp, &b_pp),
          "pic buf") ||
      !ck(va_.CreateBuffer(dpy_, context_, VASliceParameterBufferType,
                           sizeof(sp), 1, &sp, &b_sp),
          "slice buf") ||
      !ck(va_.CreateBuffer(dpy_, context_, VASliceDataBufferType,
                           static_cast<unsigned>(nal.raw.size()), 1,
                           const_cast<std::uint8_t*>(nal.raw.data()), &b_sd),
          "data buf"))
    return false;
  bool ok = ck(va_.BeginPicture(dpy_, context_, surf), "BeginPicture");
  ok =
      ok && ck(va_.RenderPicture(dpy_, context_, &b_pp, 1), "RenderPicture pp");
  VABufferID sl[2] = {b_sp, b_sd};
  ok =
      ok && ck(va_.RenderPicture(dpy_, context_, sl, 2), "RenderPicture slice");
  ok = ok && ck(va_.EndPicture(dpy_, context_), "EndPicture");
  ok = ok && ck(va_.SyncSurface(dpy_, surf), "SyncSurface");
  va_.DestroyBuffer(dpy_, b_pp);
  va_.DestroyBuffer(dpy_, b_sp);
  va_.DestroyBuffer(dpy_, b_sd);
  if (!ok) return false;

  // Park as ready (latest-wins). A previously parked frame that is not a
  // reference and not checked out frees for reuse once ready_slot_ moves off
  // it.
  ready_slot_ = static_cast<std::uint32_t>(slot);
  have_ready_ = true;
  slots_[slot].width = coded_w_;
  slots_[slot].height = coded_h_;

  // Add the decoded picture to the DPB as a short-term reference; a later
  // picture's RPS evicts it when it is no longer needed. Guard against
  // unbounded growth if a reference goes missing by dropping the lowest-POC
  // entry beyond the DPB size.
  slots_[slot].is_reference = true;
  dpb_.push_back(RefPic{static_cast<std::uint32_t>(slot), poc});
  const std::size_t max_dpb =
      static_cast<std::size_t>(sps_.sps_max_dec_pic_buffering_minus1) + 1;
  while (dpb_.size() > max_dpb) {
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < dpb_.size(); ++i)
      if (dpb_[i].poc < dpb_[oldest].poc) oldest = i;
    if (dpb_[oldest].slot != static_cast<std::uint32_t>(slot))
      slots_[dpb_[oldest].slot].is_reference = false;
    dpb_.erase(dpb_.begin() + oldest);
  }
  return true;
}

SubmitResult VaapiH265Decoder::SubmitBitstream(const std::uint8_t* data,
                                               std::size_t size,
                                               std::uint64_t timestamp) {
  auto nals = h265::ParseAnnexB(data, size);
  for (auto& n : nals) {
    if (n.type == h265::NalUnitType::kSpsNut) {
      h265::Sps s{};
      if (h265::ParseSps(n.rbsp.data(), n.rbsp.size(), &s)) {
        if (configured_ && (s.pic_width_in_luma_samples != coded_w_ ||
                            s.pic_height_in_luma_samples != coded_h_)) {
          V4L2WC_LOG(V4L2WC_INFO)
              << "vaapi-h265: stream changed to " << s.pic_width_in_luma_samples
              << "x" << s.pic_height_in_luma_samples << " from " << coded_w_
              << "x" << coded_h_;
          return SubmitResult::kSourceChange;
        }
        sps_ = s;
        have_sps_ = true;
      }
    } else if (n.type == h265::NalUnitType::kPpsNut) {
      h265::Pps p{};
      if (h265::ParsePps(n.rbsp.data(), n.rbsp.size(), &p)) {
        pps_ = p;
        have_pps_ = true;
      }
    } else if (h265::IsVcl(n.type) && have_sps_ && have_pps_) {
      if (!EnsureConfigured(sps_)) return SubmitResult::kError;
      if (DecodeSlice(n) && have_ready_)
        slots_[ready_slot_].timestamp = timestamp;
    }
  }
  return SubmitResult::kOk;
}

void VaapiH265Decoder::ExportSlot(std::uint32_t slot, std::uint64_t timestamp) {
  Slot& s = slots_[slot];
  s.timestamp = timestamp;
  // Export once and keep the fd for the pool's lifetime: consumers cache
  // dma-buf imports keyed on the fd, so re-exporting per frame would recycle fd
  // numbers across surfaces and alias those caches. The destructor closes them.
  if (s.fd >= 0) return;
  VADRMPRIMESurfaceDescriptor d{};
  VAStatus st = va_.ExportSurfaceHandle(
      dpy_, s.surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
      VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_COMPOSED_LAYERS, &d);
  if (st != VA_STATUS_SUCCESS) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "vaapi-h265: ExportSurfaceHandle: " << va_.ErrorStr(st);
    s.fd = -1;
    return;
  }
  s.fd = d.objects[0].fd;
  s.drm_fourcc = d.fourcc;
  s.modifier = d.objects[0].drm_format_modifier;
  s.num_planes = d.layers[0].num_planes;
  for (std::uint32_t p = 0; p < d.layers[0].num_planes && p < 4; ++p) {
    s.offsets[p] = d.layers[0].offset[p];
    s.pitches[p] = d.layers[0].pitch[p];
  }
  s.timestamp = timestamp;
  for (std::uint32_t i = 1; i < d.num_objects; ++i) ::close(d.objects[i].fd);
}

bool VaapiH265Decoder::Acquire(V4l2DmaFrame* out) {
  if (!have_ready_) return false;
  std::uint32_t slot = ready_slot_;
  ExportSlot(slot, slots_[slot].timestamp);
  if (slots_[slot].fd < 0) {
    have_ready_ = false;
    return false;
  }
  Slot& s = slots_[slot];
  s.checked_out = true;
  out->capture_index = slot;
  out->width = s.width;
  out->height = s.height;
  out->drm_fourcc = s.drm_fourcc;
  out->modifier = s.modifier;
  out->num_planes = s.num_planes;
  for (int p = 0; p < 4; ++p) {
    out->fds[p] = (p < static_cast<int>(s.num_planes)) ? s.fd : -1;
    out->offsets[p] = s.offsets[p];
    out->pitches[p] = s.pitches[p];
  }
  out->timestamp = s.timestamp;
  have_ready_ = false;
  return true;
}

void VaapiH265Decoder::Release(std::uint32_t slot) {
  if (slot >= slots_.size()) return;
  // The exported fd deliberately stays open for the pool's lifetime (see
  // ExportSlot); only the check-out is returned here.
  slots_[slot].checked_out = false;
}

void VaapiH265Decoder::Flush() {
  // Drop every reference and the pending frame, as a seek does: the next
  // keyframe rebuilds the DPB. Surfaces a consumer still holds stay checked out
  // until it releases them; everything else frees. The decoder stays
  // configured, so no VAAPI resources are torn down. POC restarts from the
  // post-seek keyframe.
  for (auto& r : dpb_) slots_[r.slot].is_reference = false;
  dpb_.clear();
  prev_poc_lsb_ = 0;
  prev_poc_msb_ = 0;
  seen_first_picture_ = false;
  have_ready_ = false;
}

void VaapiH265Decoder::Drain() {
  // VAAPI decodes synchronously: nothing is held back, so there is nothing to
  // flush out.
}

}  // namespace v4l2wc
