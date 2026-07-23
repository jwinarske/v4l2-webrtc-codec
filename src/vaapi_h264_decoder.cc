// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/vaapi_h264_decoder.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

#include "rtc_base/logging.h"
#include "va/va.h"
#include "va/va_drm.h"
#include "va/va_drmcommon.h"

namespace v4l2wc {
std::unique_ptr<VaapiH264Decoder> VaapiH264Decoder::Create(
    const char* render_node, std::uint32_t pool_size) {
  auto dec = std::unique_ptr<VaapiH264Decoder>(new VaapiH264Decoder());
  if (!va::VaLoad(&dec->va_)) {
    RTC_LOG(LS_WARNING) << "vaapi: libva unavailable (dlopen failed)";
    return nullptr;
  }
  dec->drm_fd_ = ::open(render_node, O_RDWR | O_CLOEXEC);
  if (dec->drm_fd_ < 0) {
    RTC_LOG(LS_ERROR) << "vaapi: open(" << render_node << ") failed";
    return nullptr;
  }
  dec->dpy_ = dec->va_.GetDisplayDRM(dec->drm_fd_);
  if (!dec->dpy_) {
    RTC_LOG(LS_ERROR) << "vaapi: vaGetDisplayDRM failed";
    return nullptr;
  }
  int major = 0, minor = 0;
  VAStatus s = dec->va_.Initialize(dec->dpy_, &major, &minor);
  if (s != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_ERROR) << "vaapi: vaInitialize: " << dec->va_.ErrorStr(s);
    return nullptr;
  }
  dec->pool_size_ = pool_size;
  RTC_LOG(LS_INFO) << "vaapi: VA-API " << major << "." << minor << " on "
                   << render_node;
  return dec;
}

VaapiH264Decoder::VaapiH264Decoder() = default;

VaapiH264Decoder::Slot::Slot() = default;
VaapiH264Decoder::Slot::~Slot() = default;
VaapiH264Decoder::Slot::Slot(const Slot&) = default;
VaapiH264Decoder::Slot& VaapiH264Decoder::Slot::operator=(const Slot&) =
    default;

// VAAPI decodes synchronously, so there is nothing to pump.
// Nothing to pump: libva decodes synchronously in SubmitBitstream.
DriveResult VaapiH264Decoder::Drive() { return DriveResult::kOk; }

VaapiH264Decoder::~VaapiH264Decoder() {
  for (auto& sl : slots_)
    if (sl.fd >= 0) ::close(sl.fd);
  if (dpy_) {
    if (context_) va_.DestroyContext(dpy_, context_);
    if (!slots_.empty()) {
      std::vector<VASurfaceID> surfs;
      for (auto& sl : slots_) surfs.push_back(sl.surface);
      va_.DestroySurfaces(dpy_, surfs.data(), (int)surfs.size());
    }
    if (config_) va_.DestroyConfig(dpy_, config_);
    va_.Terminate(dpy_);
  }
  if (drm_fd_ >= 0) ::close(drm_fd_);
}

bool VaapiH264Decoder::EnsureConfigured(const h264::Sps& sps) {
  if (configured_) return true;
  coded_w_ = sps.pic_width_in_mbs * 16;
  coded_h_ =
      sps.pic_height_in_map_units * 16 * (sps.frame_mbs_only_flag ? 1 : 2);
  VAConfigAttrib attr = {VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420};
  VAConfigID cfg = 0;
  VAStatus s = va_.CreateConfig(dpy_, VAProfileH264ConstrainedBaseline,
                                VAEntrypointVLD, &attr, 1, &cfg);
  if (s != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_ERROR) << "vaapi: CreateConfig: " << va_.ErrorStr(s);
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
    RTC_LOG(LS_ERROR) << "vaapi: CreateSurfaces: " << va_.ErrorStr(s);
    return false;
  }
  slots_.resize(pool_size_);
  for (std::uint32_t i = 0; i < pool_size_; ++i) slots_[i].surface = surfs[i];
  VAContextID ctx = 0;
  s = va_.CreateContext(dpy_, config_, coded_w_, coded_h_, VA_PROGRESSIVE,
                        surfs.data(), pool_size_, &ctx);
  if (s != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_ERROR) << "vaapi: CreateContext: " << va_.ErrorStr(s);
    return false;
  }
  context_ = ctx;
  configured_ = true;
  RTC_LOG(LS_INFO) << "vaapi: configured " << coded_w_ << "x" << coded_h_
                   << " pool=" << pool_size_
                   << " ref_frames=" << sps.max_num_ref_frames;
  return true;
}

int VaapiH264Decoder::PickFreeSlot() {
  for (std::uint32_t i = 0; i < slots_.size(); ++i) {
    const Slot& s = slots_[i];
    if (s.is_reference || s.checked_out) continue;
    if (have_ready_ && i == ready_slot_) continue;
    return static_cast<int>(i);
  }
  return -1;
}

int VaapiH264Decoder::ComputePoc(const h264::SliceHeader& sh, int ref_idc) {
  const int MaxFrameNum = 1 << sps_.log2_max_frame_num;
  int offset;
  if (sh.idr)
    offset = 0;
  else if (prev_frame_num_ > static_cast<int>(sh.frame_num))
    offset = prev_frame_num_offset_ + MaxFrameNum;
  else
    offset = prev_frame_num_offset_;
  int temp =
      2 * (offset + static_cast<int>(sh.frame_num)) - (ref_idc == 0 ? 1 : 0);
  prev_frame_num_offset_ = offset;
  prev_frame_num_ = sh.frame_num;
  return temp;
}

bool VaapiH264Decoder::DecodeSlice(const h264::Nal& nal) {
  // The slice parser is shared with the stateless path; it needs the SPS/PPS
  // state gathered into a context.
  h264::SliceContext ctx;
  ctx.log2_max_frame_num = sps_.log2_max_frame_num;
  ctx.pic_order_cnt_type = sps_.pic_order_cnt_type;
  ctx.log2_max_pic_order_cnt_lsb = sps_.log2_max_pic_order_cnt_lsb;
  ctx.delta_pic_order_always_zero_flag = sps_.delta_pic_order_always_zero_flag;
  ctx.frame_mbs_only_flag = sps_.frame_mbs_only_flag;
  ctx.separate_colour_plane_flag = sps_.separate_colour_plane_flag;
  ctx.chroma_array_type =
      sps_.separate_colour_plane_flag ? 0 : sps_.chroma_format_idc;
  ctx.entropy_coding_mode_flag = pps_.entropy_coding_mode_flag;
  ctx.deblocking_filter_control_present_flag =
      pps_.deblocking_filter_control_present_flag;
  ctx.num_slice_groups_minus1 = pps_.num_slice_groups_minus1;
  ctx.bottom_field_pic_order_in_frame_present_flag =
      pps_.bottom_field_pic_order_in_frame_present_flag;
  ctx.num_ref_idx_l0_default_active_minus1 =
      pps_.num_ref_idx_l0_default_active_minus1;
  ctx.num_ref_idx_l1_default_active_minus1 =
      pps_.num_ref_idx_l1_default_active_minus1;
  ctx.weighted_pred_flag = pps_.weighted_pred_flag;
  ctx.weighted_bipred_idc = pps_.weighted_bipred_idc;
  ctx.redundant_pic_cnt_present_flag = pps_.redundant_pic_cnt_present_flag;

  const bool idr = (nal.type == h264::NalUnitType::kSliceIdr);
  h264::SliceHeader sh{};
  if (!h264::ParseSliceHeader(nal.rbsp.data(), nal.rbsp.size(), nal.nal_ref_idc,
                              idr, ctx, &sh)) {
    RTC_LOG(LS_WARNING) << "vaapi: malformed slice header; dropping";
    return false;
  }
  // The hardware addresses slice data in raw-NAL bit space.
  std::uint32_t data_bit_offset = 0;
  if (!h264::RbspToRawBitOffset(nal, sh.slice_data_bit_offset_rbsp,
                                &data_bit_offset)) {
    RTC_LOG(LS_WARNING) << "vaapi: slice data offset outside NAL; dropping";
    return false;
  }
  const int st = sh.slice_type % 5;  // 0=P 2=I
  const int MaxFrameNum = 1 << sps_.log2_max_frame_num;
  if (sh.idr) {
    for (auto& r : dpb_) slots_[r.slot].is_reference = false;
    dpb_.clear();
    prev_frame_num_ = 0;
    prev_frame_num_offset_ = 0;
  }
  int poc = ComputePoc(sh, nal.nal_ref_idc);

  int slot = PickFreeSlot();
  if (slot < 0) {
    RTC_LOG(LS_ERROR) << "vaapi: no free surface";
    return false;
  }
  VASurfaceID surf = slots_[slot].surface;

  for (auto& r : dpb_)
    r.pic_num = (r.frame_num > static_cast<int>(sh.frame_num))
                    ? r.frame_num - MaxFrameNum
                    : r.frame_num;
  std::vector<RefEntry> list0 = dpb_;
  std::sort(list0.begin(), list0.end(),
            [](const RefEntry& a, const RefEntry& b) {
              return a.pic_num > b.pic_num;
            });

  VAPictureParameterBufferH264 pp{};
  pp.CurrPic.picture_id = surf;
  pp.CurrPic.frame_idx = sh.frame_num;
  pp.CurrPic.flags = nal.nal_ref_idc ? VA_PICTURE_H264_SHORT_TERM_REFERENCE : 0;
  pp.CurrPic.TopFieldOrderCnt = poc;
  pp.CurrPic.BottomFieldOrderCnt = poc;
  int nref = 0;
  for (auto& r : dpb_) {
    if (nref >= 16) break;
    pp.ReferenceFrames[nref].picture_id = slots_[r.slot].surface;
    pp.ReferenceFrames[nref].frame_idx = r.frame_num;
    pp.ReferenceFrames[nref].flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
    pp.ReferenceFrames[nref].TopFieldOrderCnt = r.poc;
    pp.ReferenceFrames[nref].BottomFieldOrderCnt = r.poc;
    ++nref;
  }
  for (int i = nref; i < 16; ++i) {
    pp.ReferenceFrames[i].picture_id = VA_INVALID_SURFACE;
    pp.ReferenceFrames[i].flags = VA_PICTURE_H264_INVALID;
  }
  pp.picture_width_in_mbs_minus1 = sps_.pic_width_in_mbs - 1;
  pp.picture_height_in_mbs_minus1 =
      sps_.pic_height_in_map_units * (sps_.frame_mbs_only_flag ? 1 : 2) - 1;
  pp.num_ref_frames = sps_.max_num_ref_frames;
  pp.seq_fields.bits.chroma_format_idc = sps_.chroma_format_idc;
  pp.seq_fields.bits.frame_mbs_only_flag = sps_.frame_mbs_only_flag;
  pp.seq_fields.bits.direct_8x8_inference_flag = sps_.direct_8x8_inference_flag;
  pp.seq_fields.bits.log2_max_frame_num_minus4 = sps_.log2_max_frame_num - 4;
  pp.seq_fields.bits.pic_order_cnt_type = sps_.pic_order_cnt_type;
  pp.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 =
      sps_.log2_max_pic_order_cnt_lsb - 4;
  pp.pic_fields.bits.entropy_coding_mode_flag = pps_.entropy_coding_mode_flag;
  pp.pic_fields.bits.weighted_pred_flag = pps_.weighted_pred_flag;
  pp.pic_fields.bits.weighted_bipred_idc = pps_.weighted_bipred_idc;
  pp.pic_fields.bits.transform_8x8_mode_flag = pps_.transform_8x8_mode_flag;
  pp.pic_fields.bits.constrained_intra_pred_flag =
      pps_.constrained_intra_pred_flag;
  pp.pic_fields.bits.pic_order_present_flag =
      pps_.bottom_field_pic_order_in_frame_present_flag;
  pp.pic_fields.bits.deblocking_filter_control_present_flag =
      pps_.deblocking_filter_control_present_flag;
  pp.pic_fields.bits.redundant_pic_cnt_present_flag =
      pps_.redundant_pic_cnt_present_flag;
  pp.pic_fields.bits.reference_pic_flag = nal.nal_ref_idc != 0;
  pp.frame_num = sh.frame_num;
  pp.pic_init_qp_minus26 = (pps_.pic_init_qp_minus26 + 26) - 26;
  pp.chroma_qp_index_offset = pps_.chroma_qp_index_offset;
  pp.second_chroma_qp_index_offset = pps_.second_chroma_qp_index_offset;

  VAIQMatrixBufferH264 iq{};
  memset(&iq, 16, sizeof(iq));

  VASliceParameterBufferH264 sp{};
  sp.slice_data_size = nal.raw.size();
  sp.slice_data_offset = 0;
  sp.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
  sp.slice_data_bit_offset = data_bit_offset;
  sp.first_mb_in_slice = sh.first_mb_in_slice;
  sp.slice_type = sh.slice_type;
  sp.num_ref_idx_l0_active_minus1 = sh.num_ref_idx_l0_active_minus1;
  sp.num_ref_idx_l1_active_minus1 = 0;
  sp.slice_qp_delta = sh.slice_qp_delta;
  for (int i = 0; i < 32; ++i) {
    sp.RefPicList0[i].picture_id = VA_INVALID_SURFACE;
    sp.RefPicList0[i].flags = VA_PICTURE_H264_INVALID;
    sp.RefPicList1[i].picture_id = VA_INVALID_SURFACE;
    sp.RefPicList1[i].flags = VA_PICTURE_H264_INVALID;
  }
  if (st == 0) {
    for (std::size_t i = 0; i < list0.size() && i < 32; ++i) {
      sp.RefPicList0[i].picture_id = slots_[list0[i].slot].surface;
      sp.RefPicList0[i].frame_idx = list0[i].frame_num;
      sp.RefPicList0[i].flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
      sp.RefPicList0[i].TopFieldOrderCnt = list0[i].poc;
      sp.RefPicList0[i].BottomFieldOrderCnt = list0[i].poc;
    }
  }

  VABufferID b_pp, b_iq, b_sp, b_sd;
  auto ck = [&](VAStatus s, const char* what) {
    if (s != VA_STATUS_SUCCESS)
      RTC_LOG(LS_ERROR) << "vaapi: " << what << ": " << va_.ErrorStr(s);
    return s == VA_STATUS_SUCCESS;
  };
  if (!ck(va_.CreateBuffer(dpy_, context_, VAPictureParameterBufferType,
                           sizeof(pp), 1, &pp, &b_pp),
          "pic buf") ||
      !ck(va_.CreateBuffer(dpy_, context_, VAIQMatrixBufferType, sizeof(iq), 1,
                           &iq, &b_iq),
          "iq buf") ||
      !ck(va_.CreateBuffer(dpy_, context_, VASliceParameterBufferType,
                           sizeof(sp), 1, &sp, &b_sp),
          "slice buf") ||
      !ck(va_.CreateBuffer(dpy_, context_, VASliceDataBufferType,
                           nal.raw.size(), 1, (void*)nal.raw.data(), &b_sd),
          "data buf"))
    return false;
  bool ok = ck(va_.BeginPicture(dpy_, context_, surf), "BeginPicture");
  VABufferID hdr[2] = {b_pp, b_iq};
  ok = ok && ck(va_.RenderPicture(dpy_, context_, hdr, 2), "RenderPicture hdr");
  VABufferID sl[2] = {b_sp, b_sd};
  ok =
      ok && ck(va_.RenderPicture(dpy_, context_, sl, 2), "RenderPicture slice");
  ok = ok && ck(va_.EndPicture(dpy_, context_), "EndPicture");
  ok = ok && ck(va_.SyncSurface(dpy_, surf), "SyncSurface");
  va_.DestroyBuffer(dpy_, b_pp);
  va_.DestroyBuffer(dpy_, b_iq);
  va_.DestroyBuffer(dpy_, b_sp);
  va_.DestroyBuffer(dpy_, b_sd);
  if (!ok) return false;

  // Park as ready (latest-wins): drop a previous un-acquired frame.
  if (have_ready_ && !slots_[ready_slot_].is_reference &&
      !slots_[ready_slot_].checked_out)
    slots_[ready_slot_].in_use = false;
  ready_slot_ = static_cast<std::uint32_t>(slot);
  have_ready_ = true;
  slots_[slot].width = coded_w_;
  slots_[slot].height = coded_h_;

  // dec_ref_pic_marking: sliding window (no MMCO in this subset).
  if (nal.nal_ref_idc) {
    const int active =
        std::max<int>(1, static_cast<int>(sps_.max_num_ref_frames));
    if (static_cast<int>(dpb_.size()) >= active) {
      std::size_t victim = 0;
      for (std::size_t i = 1; i < dpb_.size(); ++i)
        if (dpb_[i].pic_num < dpb_[victim].pic_num) victim = i;
      slots_[dpb_[victim].slot].is_reference = false;
      dpb_.erase(dpb_.begin() + victim);
    }
    slots_[slot].is_reference = true;
    dpb_.push_back(RefEntry{static_cast<std::uint32_t>(slot),
                            static_cast<int>(sh.frame_num),
                            static_cast<int>(sh.frame_num), poc});
  }
  return true;
}

SubmitResult VaapiH264Decoder::SubmitBitstream(const std::uint8_t* data,
                                               std::size_t size,
                                               std::uint64_t rtp_timestamp) {
  auto nals = h264::ParseAnnexB(data, size);
  for (auto& n : nals) {
    if (n.type == h264::NalUnitType::kSps) {
      h264::Sps s{};
      if (h264::ParseSps(n.rbsp.data(), n.rbsp.size(), &s)) {
        // Height as well as width: a stream that keeps its width and changes
        // aspect would otherwise decode into surfaces of the wrong size.
        const std::uint32_t w = s.pic_width_in_mbs * 16;
        const std::uint32_t h =
            s.pic_height_in_map_units * 16 * (s.frame_mbs_only_flag ? 1 : 2);
        if (configured_ && (w != coded_w_ || h != coded_h_)) {
          RTC_LOG(LS_INFO) << "vaapi: stream changed to " << w << "x" << h
                           << " from " << coded_w_ << "x" << coded_h_;
          return SubmitResult::kSourceChange;
        }
        sps_ = s;
        have_sps_ = true;
      }
    } else if (n.type == h264::NalUnitType::kPps) {
      h264::Pps p{};
      if (h264::ParsePps(n.rbsp.data(), n.rbsp.size(), &p)) {
        pps_ = p;
        have_pps_ = true;
      }
    } else if ((n.type == h264::NalUnitType::kSliceNonIdr ||
                n.type == h264::NalUnitType::kSliceIdr) &&
               have_sps_ && have_pps_) {
      if (!EnsureConfigured(sps_)) return SubmitResult::kError;
      if (!DecodeSlice(n)) return SubmitResult::kError;
      if (have_ready_) slots_[ready_slot_].rtp = rtp_timestamp;
    }
  }
  return SubmitResult::kOk;
}

void VaapiH264Decoder::ExportSlot(std::uint32_t slot, std::uint64_t rtp) {
  Slot& s = slots_[slot];
  s.rtp = rtp;
  // Export once and keep the fd for the pool's lifetime, the way the V4L2
  // engine hands out its VIDIOC_EXPBUF fds. Consumers cache dma-buf imports
  // keyed on the fd, so re-exporting per frame would recycle fd numbers across
  // surfaces and alias those caches. The destructor closes them.
  if (s.fd >= 0) {
    return;
  }
  VADRMPRIMESurfaceDescriptor d{};
  VAStatus st = va_.ExportSurfaceHandle(
      dpy_, s.surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
      VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_COMPOSED_LAYERS, &d);
  if (st != VA_STATUS_SUCCESS) {
    RTC_LOG(LS_ERROR) << "vaapi: ExportSurfaceHandle: " << va_.ErrorStr(st);
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
  s.rtp = rtp;
  // Close any extra objects (single-object NV12 expected; be safe).
  for (std::uint32_t i = 1; i < d.num_objects; ++i) ::close(d.objects[i].fd);
}

bool VaapiH264Decoder::Acquire(V4l2DmaFrame* out) {
  if (!have_ready_) return false;
  std::uint32_t slot = ready_slot_;
  ExportSlot(slot, slots_[slot].rtp);
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
  out->rtp_timestamp = s.rtp;
  have_ready_ = false;
  return true;
}

void VaapiH264Decoder::Release(std::uint32_t slot) {
  if (slot >= slots_.size()) return;
  Slot& s = slots_[slot];
  // The exported fd deliberately stays open for the pool's lifetime (see
  // ExportSlot); only the check-out is returned here.
  s.checked_out = false;
  // The surface frees for reuse once it is also no longer a reference (handled
  // by DPB eviction clearing is_reference).
}

}  // namespace v4l2wc
