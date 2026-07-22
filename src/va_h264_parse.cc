// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/va_h264_parse.h"

#include <dlfcn.h>
#include <stdio.h>
#include <string.h>

namespace v4l2wc::va {

// Out-of-line special members: these aggregates are wide enough that emitting
// their construction inline in every translation unit is wasteful.
VaApi::VaApi() = default;
VaApi::~VaApi() = default;
Sps::Sps() = default;
Sps::~Sps() = default;
Pps::Pps() = default;
Pps::~Pps() = default;
SliceHdr::SliceHdr() = default;
SliceHdr::~SliceHdr() = default;
Nal::Nal() = default;
Nal::~Nal() = default;
Nal::Nal(const Nal&) = default;
Nal& Nal::operator=(const Nal&) = default;
Nal::Nal(Nal&&) noexcept = default;
Nal& Nal::operator=(Nal&&) noexcept = default;

// Upper bound on ref_pic_list_modification entries; the syntax allows one per
// reference index, so anything beyond this is malformed.
constexpr uint32_t kMaxRefListMods = 64;

// Spec bound on num_ref_frames_in_pic_order_cnt_cycle.
constexpr uint32_t kMaxPocCycleLen = 255;

// Spec bound on log2_max_frame_num_minus4 / log2_max_pic_order_cnt_lsb_minus4.
constexpr uint32_t kMaxLog2Minus4 = 12;

// ---- bit reader (RBSP, MSB-first, Exp-Golomb), exposes bit position ----
namespace {
struct BR {
  const uint8_t* d;
  size_t n;        // bytes
  size_t pos = 0;  // bit position
  BR(const uint8_t* data, size_t bytes) : d(data), n(bytes) {}
  uint32_t u(uint32_t bits) {
    // Defense in depth: several bit counts derive from SPS fields, so clamp
    // rather than letting a crafted value drive the loop. More than 32 bits
    // cannot be represented in the result anyway.
    if (bits > 32) bits = 32;
    uint32_t v = 0;
    for (uint32_t i = 0; i < bits; ++i) {
      uint32_t bit = 0;
      if (pos < n * 8) bit = (d[pos >> 3] >> (7 - (pos & 7))) & 1u;
      v = (v << 1) | bit;
      ++pos;
    }
    return v;
  }
  uint32_t ue() {
    int zeros = 0;
    // Cap at 31: `1u << 32` is undefined, and no H.264 syntax element needs a
    // prefix that long. A longer run means malformed input, which the callers'
    // overrun()/range checks then reject.
    while (pos < n * 8 && u(1) == 0 && zeros < 31) ++zeros;
    uint32_t v = 0;
    if (zeros) v = u(zeros);
    return (1u << zeros) - 1 + v;
  }
  int32_t se() {
    uint32_t k = ue();
    uint32_t m = (k + 1) >> 1;
    return (k & 1) ? (int32_t)m : -(int32_t)m;
  }
  bool more_rbsp() const {
    // crude: any bits left beyond the rbsp_stop trailing bit
    return pos < n * 8;
  }
  // True once a read has run past the end. Reads past the end yield zero bits
  // while still advancing `pos`, so callers must treat this as malformed input
  // rather than trusting any value derived from `pos`.
  bool overrun() const { return pos > n * 8; }
};
}  // namespace

std::vector<Nal> SplitAnnexB(const uint8_t* data, size_t size) {
  std::vector<Nal> out;
  size_t i = 0;
  auto is_sc = [&](size_t p, int* len) {
    if (p + 3 <= size && data[p] == 0 && data[p + 1] == 0 && data[p + 2] == 1) {
      *len = 3;
      return true;
    }
    if (p + 4 <= size && data[p] == 0 && data[p + 1] == 0 && data[p + 2] == 0 &&
        data[p + 3] == 1) {
      *len = 4;
      return true;
    }
    return false;
  };
  int len = 0;
  while (i < size && !is_sc(i, &len)) ++i;
  while (i < size) {
    i += len;  // skip this start code
    size_t start = i;
    while (i < size && !is_sc(i, &len)) ++i;
    size_t end = i;  // [start,end) is one NAL (header+payload, with emulation)
    if (end <= start) continue;
    Nal nal;
    nal.ref_idc = (data[start] >> 5) & 3;
    nal.type = data[start] & 0x1f;
    nal.raw.assign(data + start, data + end);
    // strip emulation-prevention (00 00 03 -> 00 00) from the payload (byte 1+)
    nal.rbsp.reserve(end - start - 1);
    int zeros = 0;
    for (size_t p = start + 1; p < end; ++p) {
      uint8_t b = data[p];
      if (zeros >= 2 && b == 3) {
        zeros = 0;
        continue;  // drop the emulation byte
      }
      nal.rbsp.push_back(b);
      zeros = (b == 0) ? zeros + 1 : 0;
    }
    out.push_back(std::move(nal));
  }
  return out;
}

static void skip_scaling_lists(BR& br, int count) {
  for (int i = 0; i < count; ++i) {
    if (br.u(1)) {  // scaling_list_present
      int sz = i < 6 ? 16 : 64;
      int last = 8, next = 8;
      for (int j = 0; j < sz; ++j) {
        if (next != 0) {
          int delta = br.se();
          next = (last + delta + 256) % 256;
        }
        last = next == 0 ? last : next;
      }
    }
  }
}

bool ParseSps(const uint8_t* rbsp, size_t n, Sps* s) {
  BR br(rbsp, n);
  s->profile_idc = br.u(8);
  br.u(8);  // constraint flags + reserved
  s->level_idc = br.u(8);
  s->sps_id = br.ue();
  s->chroma_format_idc = 1;
  if (s->profile_idc == 100 || s->profile_idc == 110 || s->profile_idc == 122 ||
      s->profile_idc == 244 || s->profile_idc == 44 || s->profile_idc == 83 ||
      s->profile_idc == 86 || s->profile_idc == 118 || s->profile_idc == 128) {
    s->chroma_format_idc = br.ue();
    if (s->chroma_format_idc == 3) br.u(1);  // separate_colour_plane
    br.ue();                                 // bit_depth_luma_minus8
    br.ue();                                 // bit_depth_chroma_minus8
    br.u(1);                                 // qpprime_y_zero_transform_bypass
    if (br.u(1)) skip_scaling_lists(br, s->chroma_format_idc == 3 ? 12 : 8);
  }
  // Both log2 fields size later u(n) reads, so enforce the spec bound
  // (*_minus4 <= 12) before they are used; an unbounded value would otherwise
  // drive a huge bit-by-bit read for every slice header.
  const uint32_t log2_max_frame_num_minus4 = br.ue();
  if (log2_max_frame_num_minus4 > kMaxLog2Minus4) {
    return false;
  }
  s->log2_max_frame_num = log2_max_frame_num_minus4 + 4;
  s->pic_order_cnt_type = br.ue();
  if (s->pic_order_cnt_type == 0) {
    const uint32_t log2_max_poc_lsb_minus4 = br.ue();
    if (log2_max_poc_lsb_minus4 > kMaxLog2Minus4) {
      return false;
    }
    s->log2_max_poc_lsb = log2_max_poc_lsb_minus4 + 4;
  } else if (s->pic_order_cnt_type == 1) {
    s->delta_pic_order_always_zero = br.u(1);
    br.se();  // offset_for_non_ref_pic
    br.se();  // offset_for_top_to_bottom_field
    // num_ref_frames_in_pic_order_cnt_cycle is bounded to 255 by the spec;
    // without the check a crafted value drives billions of iterations.
    uint32_t cyc = br.ue();
    if (cyc > kMaxPocCycleLen) {
      return false;
    }
    for (uint32_t i = 0; i < cyc; ++i) br.se();
  }
  s->max_num_ref_frames = br.ue();
  br.u(1);  // gaps_in_frame_num_value_allowed
  const uint32_t width_mbs_minus1 = br.ue();
  const uint32_t height_map_units_minus1 = br.ue();
  // Bound before multiplying out: coded_w/coded_h are handed to libva as ints,
  // so an absurd SPS must be rejected here rather than narrowed later.
  if (width_mbs_minus1 >= kMaxDimension / 16 ||
      height_map_units_minus1 >= kMaxDimension / 16) {
    return false;
  }
  s->pic_width_in_mbs = width_mbs_minus1 + 1;
  s->pic_height_in_map_units = height_map_units_minus1 + 1;
  s->frame_mbs_only = br.u(1);
  if (!s->frame_mbs_only) s->mb_adaptive_frame_field = br.u(1);
  s->direct_8x8_inference = br.u(1);
  // frame_cropping + vui: not needed for VA params
  // Reads past the end yield zeros while advancing, so a truncated SPS would
  // otherwise produce plausible-looking but fabricated values.
  if (br.overrun()) {
    return false;
  }
  return true;
}

bool ParsePps(const uint8_t* rbsp, size_t n, const Sps* sps, Pps* p) {
  BR br(rbsp, n);
  p->pps_id = br.ue();
  p->sps_id = br.ue();
  p->entropy_coding_mode = br.u(1);
  p->bottom_field_pic_order_present = br.u(1);
  p->num_slice_groups = br.ue() + 1;
  if (p->num_slice_groups > 1) return false;  // slice groups: unsupported (CBP)
  p->num_ref_idx_l0_default = br.ue() + 1;
  p->num_ref_idx_l1_default = br.ue() + 1;
  p->weighted_pred = br.u(1);
  p->weighted_bipred_idc = br.u(2);
  p->pic_init_qp = br.se() + 26;
  br.se();  // pic_init_qs_minus26
  p->chroma_qp_index_offset = br.se();
  p->deblocking_filter_control_present = br.u(1);
  p->constrained_intra_pred = br.u(1);
  p->redundant_pic_cnt_present = br.u(1);
  p->second_chroma_qp_index_offset = p->chroma_qp_index_offset;
  if (br.more_rbsp() && (br.n * 8 - br.pos) > 8) {
    p->transform_8x8_mode = br.u(1);
    if (br.u(1)) skip_scaling_lists(br, 6 + (p->transform_8x8_mode ? 2 : 0));
    p->second_chroma_qp_index_offset = br.se();
  }
  (void)sps;
  if (br.overrun()) {
    return false;
  }
  return true;
}

bool ParseSliceHdr(const Nal& nal, const Sps& sps, const Pps& pps,
                   SliceHdr* sh) {
  BR br(nal.rbsp.data(), nal.rbsp.size());
  sh->idr = (nal.type == 5);
  sh->first_mb = br.ue();
  sh->slice_type = br.ue();
  sh->pps_id = br.ue();
  sh->frame_num = br.u(sps.log2_max_frame_num);
  // frame_mbs_only assumed (CBP): no field_pic_flag
  if (sh->idr) sh->idr_pic_id = br.ue();
  if (sps.pic_order_cnt_type == 0) {
    sh->poc_lsb = br.u(sps.log2_max_poc_lsb);
    if (pps.bottom_field_pic_order_present)
      br.se();  // delta_pic_order_cnt_bottom
  } else if (sps.pic_order_cnt_type == 1 && !sps.delta_pic_order_always_zero) {
    br.se();
    if (pps.bottom_field_pic_order_present) br.se();
  }
  if (pps.redundant_pic_cnt_present) br.ue();
  uint32_t st = sh->slice_type % 5;
  // st: 0=P 1=B 2=I 3=SP 4=SI
  if (st == 1) br.u(1);  // direct_spatial_mv_pred (B)
  sh->num_ref_idx_l0_active = pps.num_ref_idx_l0_default;
  if (st == 0 || st == 3 || st == 1) {  // P/SP/B
    if (br.u(1)) {                      // num_ref_idx_active_override
      sh->num_ref_idx_l0_active = br.ue() + 1;
      if (st == 1) br.ue();  // l1
    }
    // ref_pic_list_modification_l0. The terminator is idc == 3, but a
    // truncated or hostile bitstream makes ue() return 0 forever once the
    // reader is exhausted, so the loop must also stop on exhaustion and is
    // capped besides.
    if (br.u(1)) {
      uint32_t idc;
      uint32_t guard = 0;
      do {
        idc = br.ue();
        // 0/1: abs_diff_pic_num_minus1, 2: long_term_pic_num -- one ue() each.
        if (idc <= 2) br.ue();
      } while (idc != 3 && br.more_rbsp() && ++guard < kMaxRefListMods);
    }
    if (st == 1) {  // l1 mod (B)
      if (br.u(1)) {
        uint32_t idc;
        uint32_t guard = 0;
        do {
          idc = br.ue();
          if (idc <= 2) br.ue();
        } while (idc != 3 && br.more_rbsp() && ++guard < kMaxRefListMods);
      }
    }
  }
  // weighted pred table: CBP has weighted_pred=0, weighted_bipred_idc=0 -> none
  if (nal.ref_idc) {
    if (sh->idr) {
      br.u(1);
      br.u(1);
    }  // no_output, long_term_reference
    else if (br.u(1)) {  // adaptive_ref_pic_marking
      uint32_t op;
      do {
        op = br.ue();
        if (op == 1 || op == 3) br.ue();
        if (op == 2) br.ue();
        if (op == 3 || op == 6) br.ue();
        if (op == 4) br.ue();
      } while (op != 0);
    }
  }
  if (pps.entropy_coding_mode && st != 2 && st != 4) br.ue();  // cabac_init_idc
  sh->slice_qp_delta = br.se();
  if (pps.deblocking_filter_control_present) {
    uint32_t idc = br.ue();  // disable_deblocking_filter_idc
    if (idc != 1) {
      br.se();
      br.se();
    }
  }
  // first MB begins now (CAVLC: no alignment). Offset in RBSP space:
  uint32_t rbsp_bit = (uint32_t)br.pos;
  // Map to raw payload bit offset by counting emulation bytes removed before
  // the header end. (Header EPBs are rare; count precisely to be safe.)
  uint32_t rbsp_byte = rbsp_bit >> 3;
  uint32_t epb = 0, zeros = 0;
  const uint8_t* raw_payload = nal.raw.data() + 1;  // after NAL header byte
  size_t raw_n = nal.raw.size() - 1;
  for (size_t p = 0, kept = 0; p < raw_n && kept < rbsp_byte; ++p) {
    uint8_t b = raw_payload[p];
    if (zeros >= 2 && b == 3) {
      ++epb;
      zeros = 0;
      continue;
    }
    ++kept;
    zeros = (b == 0) ? zeros + 1 : 0;
  }
  // slice_data_buffer will be the raw NAL (header byte + payload); first MB bit
  // offset from the buffer start = 8 (NAL header) + payload bit offset.
  // A truncated header leaves the reader past the end with `pos` still
  // advanced, which would report a slice-data offset outside the NAL. That
  // offset is handed to the hardware verbatim as
  // VASliceParameterBufferH264.slice_data_bit_offset, so reject instead.
  if (br.overrun()) {
    return false;
  }
  sh->data_bit_offset = 8 + rbsp_bit + epb * 8;
  if (sh->data_bit_offset > nal.raw.size() * 8) {
    return false;
  }
  return true;
}

// ---- VA loader ----
bool VaLoad(VaApi* va) {
  va->lib = dlopen("libva.so.2", RTLD_NOW | RTLD_GLOBAL);
  va->lib_drm = dlopen("libva-drm.so.2", RTLD_NOW | RTLD_GLOBAL);
  if (!va->lib || !va->lib_drm) {
    fprintf(stderr, "dlopen libva: %s\n", dlerror());
    return false;
  }
#define S(h, f, name)                      \
  do {                                     \
    *(void**)(&va->f) = dlsym(h, name);    \
    if (!va->f) {                          \
      fprintf(stderr, "dlsym %s\n", name); \
      return false;                        \
    }                                      \
  } while (0)
  S(va->lib_drm, GetDisplayDRM, "vaGetDisplayDRM");
  S(va->lib, Initialize, "vaInitialize");
  S(va->lib, Terminate, "vaTerminate");
  S(va->lib, ErrorStr, "vaErrorStr");
  S(va->lib, CreateConfig, "vaCreateConfig");
  S(va->lib, CreateSurfaces, "vaCreateSurfaces");
  S(va->lib, CreateContext, "vaCreateContext");
  S(va->lib, CreateBuffer, "vaCreateBuffer");
  S(va->lib, BeginPicture, "vaBeginPicture");
  S(va->lib, RenderPicture, "vaRenderPicture");
  S(va->lib, EndPicture, "vaEndPicture");
  S(va->lib, SyncSurface, "vaSyncSurface");
  S(va->lib, DestroyBuffer, "vaDestroyBuffer");
  S(va->lib, DestroySurfaces, "vaDestroySurfaces");
  S(va->lib, DestroyContext, "vaDestroyContext");
  S(va->lib, DestroyConfig, "vaDestroyConfig");
  S(va->lib, DeriveImage, "vaDeriveImage");
  S(va->lib, MapBuffer, "vaMapBuffer");
  S(va->lib, UnmapBuffer, "vaUnmapBuffer");
  S(va->lib, DestroyImage, "vaDestroyImage");
  S(va->lib, ExportSurfaceHandle, "vaExportSurfaceHandle");
#undef S
  return true;
}

}  // namespace v4l2wc::va
