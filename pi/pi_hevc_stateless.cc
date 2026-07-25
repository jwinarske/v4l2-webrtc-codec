// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Standalone smoke test for the Raspberry Pi 4 stateless HEVC decoder
// (rpi-hevc-dec, /dev/video19). It reuses the webrtc-free parse/h265 layer to
// parse an Annex-B clip, marshals the SPS/PPS/slice/decode parameters into the
// V4L2 stateless HEVC control structs, and drives the decode through the V4L2
// request API. The point is to cross-validate the parser against a second,
// independent decoder (Broadcom, not AMD VAAPI) on the actual ARM target.
//
// Milestone 1: all-intra clips (IDR / CRA, I slices only) -- no reference
// lists or DPB management yet, which keeps the request-API plumbing the focus.
// A P/B slice is reported and the picture skipped.
//
// Cross-built for aarch64 with emb (see pi/.emb) and run on the device.

#include <fcntl.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "parse/h265/nal.h"
#include "parse/h265/pps.h"
#include "parse/h265/slice_header.h"
#include "parse/h265/sps.h"

using namespace v4l2wc::h265;

namespace {

#define LOGE(...) std::fprintf(stderr, __VA_ARGS__)

int Xioctl(int fd, unsigned long req, void* arg, const char* name) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  if (r == -1) {
    LOGE("ioctl %s failed: %s (errno %d)\n", name, std::strerror(errno), errno);
  }
  return r;
}
#define XIOCTL(fd, req, arg) Xioctl((fd), (req), (arg), #req)

std::vector<std::uint8_t> ReadFile(const char* path) {
  std::vector<std::uint8_t> out;
  FILE* f = std::fopen(path, "rb");
  if (!f) {
    LOGE("cannot open %s\n", path);
    return out;
  }
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (n > 0) {
    out.resize(static_cast<size_t>(n));
    if (std::fread(out.data(), 1, out.size(), f) != out.size()) out.clear();
  }
  std::fclose(f);
  return out;
}

// One coded picture: its VCL NALs in decode order (first has
// first_slice_segment_in_pic_flag set).
struct Picture {
  std::vector<const Nal*> slices;
};

// Opens the media device whose driver is `driver` (e.g. "rpi-hevc-dec"), used
// for MEDIA_IOC_REQUEST_ALLOC. Returns -1 if none matches.
int OpenMediaDevice(const char* driver) {
  for (int i = 0; i < 16; ++i) {
    char path[32];
    std::snprintf(path, sizeof(path), "/dev/media%d", i);
    int fd = ::open(path, O_RDWR | O_CLOEXEC);
    if (fd < 0) continue;
    media_device_info info{};
    if (Xioctl(fd, MEDIA_IOC_DEVICE_INFO, &info, "MEDIA_IOC_DEVICE_INFO") ==
            0 &&
        std::strncmp(info.driver, driver, sizeof(info.driver)) == 0) {
      LOGE("media device: %s (driver %s)\n", path, info.driver);
      return fd;
    }
    ::close(fd);
  }
  LOGE("no media device with driver %s\n", driver);
  return -1;
}

constexpr int kMaxDpb = V4L2_HEVC_DPB_ENTRIES_NUM_MAX;

void FillSps(const Sps& s, v4l2_ctrl_hevc_sps* o) {
  std::memset(o, 0, sizeof(*o));
  o->video_parameter_set_id = static_cast<__u8>(s.sps_video_parameter_set_id);
  o->seq_parameter_set_id = static_cast<__u8>(s.sps_seq_parameter_set_id);
  o->pic_width_in_luma_samples =
      static_cast<__u16>(s.pic_width_in_luma_samples);
  o->pic_height_in_luma_samples =
      static_cast<__u16>(s.pic_height_in_luma_samples);
  o->bit_depth_luma_minus8 = static_cast<__u8>(s.bit_depth_luma - 8);
  o->bit_depth_chroma_minus8 = static_cast<__u8>(s.bit_depth_chroma - 8);
  o->log2_max_pic_order_cnt_lsb_minus4 =
      static_cast<__u8>(s.log2_max_pic_order_cnt_lsb - 4);
  o->sps_max_dec_pic_buffering_minus1 =
      static_cast<__u8>(s.sps_max_dec_pic_buffering_minus1);
  o->sps_max_num_reorder_pics = 0;
  o->sps_max_latency_increase_plus1 = 0;
  o->log2_min_luma_coding_block_size_minus3 =
      static_cast<__u8>(s.log2_min_cb_size - 3);
  o->log2_diff_max_min_luma_coding_block_size =
      static_cast<__u8>(s.log2_ctb_size - s.log2_min_cb_size);
  o->log2_min_luma_transform_block_size_minus2 =
      static_cast<__u8>(s.log2_min_tb_size - 2);
  o->log2_diff_max_min_luma_transform_block_size =
      static_cast<__u8>(s.log2_diff_max_min_tb_size);
  o->max_transform_hierarchy_depth_inter =
      static_cast<__u8>(s.max_transform_hierarchy_depth_inter);
  o->max_transform_hierarchy_depth_intra =
      static_cast<__u8>(s.max_transform_hierarchy_depth_intra);
  o->num_short_term_ref_pic_sets = static_cast<__u8>(s.short_term_rps.size());
  o->num_long_term_ref_pics_sps =
      static_cast<__u8>(s.num_long_term_ref_pics_sps);
  o->chroma_format_idc = static_cast<__u8>(s.chroma_format_idc);
  o->sps_max_sub_layers_minus1 = static_cast<__u8>(s.sps_max_sub_layers_minus1);
  __u64 f = 0;
  if (s.separate_colour_plane_flag)
    f |= V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE;
  if (s.scaling_list_enabled_flag) f |= V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED;
  if (s.amp_enabled_flag) f |= V4L2_HEVC_SPS_FLAG_AMP_ENABLED;
  if (s.sample_adaptive_offset_enabled_flag)
    f |= V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET;
  if (s.pcm_enabled_flag) f |= V4L2_HEVC_SPS_FLAG_PCM_ENABLED;
  if (s.long_term_ref_pics_present_flag)
    f |= V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT;
  if (s.sps_temporal_mvp_enabled_flag)
    f |= V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED;
  if (s.strong_intra_smoothing_enabled_flag)
    f |= V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED;
  o->flags = f;
}

void FillPps(const Pps& p, v4l2_ctrl_hevc_pps* o) {
  std::memset(o, 0, sizeof(*o));
  o->pic_parameter_set_id = static_cast<__u8>(p.pps_pic_parameter_set_id);
  o->num_extra_slice_header_bits =
      static_cast<__u8>(p.num_extra_slice_header_bits);
  o->num_ref_idx_l0_default_active_minus1 =
      static_cast<__u8>(p.num_ref_idx_l0_default_active_minus1);
  o->num_ref_idx_l1_default_active_minus1 =
      static_cast<__u8>(p.num_ref_idx_l1_default_active_minus1);
  o->init_qp_minus26 = static_cast<__s8>(p.init_qp_minus26);
  o->diff_cu_qp_delta_depth = static_cast<__u8>(p.diff_cu_qp_delta_depth);
  o->pps_cb_qp_offset = static_cast<__s8>(p.pps_cb_qp_offset);
  o->pps_cr_qp_offset = static_cast<__s8>(p.pps_cr_qp_offset);
  o->num_tile_columns_minus1 = static_cast<__u8>(p.num_tile_columns_minus1);
  o->num_tile_rows_minus1 = static_cast<__u8>(p.num_tile_rows_minus1);
  for (size_t i = 0; i < p.column_width_minus1.size() && i < 20; ++i)
    o->column_width_minus1[i] = static_cast<__u8>(p.column_width_minus1[i]);
  for (size_t i = 0; i < p.row_height_minus1.size() && i < 22; ++i)
    o->row_height_minus1[i] = static_cast<__u8>(p.row_height_minus1[i]);
  o->pps_beta_offset_div2 = static_cast<__s8>(p.pps_beta_offset_div2);
  o->pps_tc_offset_div2 = static_cast<__s8>(p.pps_tc_offset_div2);
  o->log2_parallel_merge_level_minus2 =
      static_cast<__u8>(p.log2_parallel_merge_level_minus2);
  __u64 f = 0;
  if (p.dependent_slice_segments_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED;
  if (p.output_flag_present_flag) f |= V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT;
  if (p.sign_data_hiding_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED;
  if (p.cabac_init_present_flag) f |= V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT;
  if (p.constrained_intra_pred_flag)
    f |= V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED;
  if (p.transform_skip_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED;
  if (p.cu_qp_delta_enabled_flag) f |= V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED;
  if (p.pps_slice_chroma_qp_offsets_present_flag)
    f |= V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT;
  if (p.weighted_pred_flag) f |= V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED;
  if (p.weighted_bipred_flag) f |= V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED;
  if (p.transquant_bypass_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED;
  if (p.tiles_enabled_flag) f |= V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
  if (p.entropy_coding_sync_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED;
  if (p.loop_filter_across_tiles_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED;
  if (p.pps_loop_filter_across_slices_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED;
  if (p.deblocking_filter_override_enabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED;
  if (p.pps_deblocking_filter_disabled_flag)
    f |= V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
  if (p.deblocking_filter_control_present_flag)
    f |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;
  if (p.lists_modification_present_flag)
    f |= V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT;
  if (p.slice_segment_header_extension_present_flag)
    f |= V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT;
  if (p.uniform_spacing_flag) f |= V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING;
  o->flags = f;
}

void FillScalingMatrix(const ScalingListData& s,
                       v4l2_ctrl_hevc_scaling_matrix* o) {
  std::memcpy(o->scaling_list_4x4, s.list4x4, sizeof(o->scaling_list_4x4));
  std::memcpy(o->scaling_list_8x8, s.list8x8, sizeof(o->scaling_list_8x8));
  std::memcpy(o->scaling_list_16x16, s.list16x16,
              sizeof(o->scaling_list_16x16));
  std::memcpy(o->scaling_list_32x32, s.list32x32,
              sizeof(o->scaling_list_32x32));
  std::memcpy(o->scaling_list_dc_coef_16x16, s.dc16x16,
              sizeof(o->scaling_list_dc_coef_16x16));
  std::memcpy(o->scaling_list_dc_coef_32x32, s.dc32x32,
              sizeof(o->scaling_list_dc_coef_32x32));
}

// Detiles a Broadcom NV12_COL128 plane (NC12/Nc12) into a tightly packed
// raster plane. The tiled plane stores 128-byte-wide columns laid left to
// right, each `col_h` rows tall; byte (bx, y) lives at
// (bx/128)*128*col_h + y*128 + (bx%128). `w_bytes` is the valid byte width
// (luma: width; NV12 chroma: 2*chroma_width, i.e. the interleaved UV row).
void DetileCol128(const std::uint8_t* tiled, std::uint32_t stride,
                  std::uint32_t w_bytes, std::uint32_t h, std::uint32_t col_h,
                  std::vector<std::uint8_t>* out) {
  out->resize(static_cast<size_t>(w_bytes) * h);
  const std::uint32_t cols = stride / 128;
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t bx = 0; bx < w_bytes; ++bx) {
      const std::uint32_t col = bx / 128;
      const std::uint32_t within = bx % 128;
      std::uint32_t src = col * 128 * col_h + y * 128 + within;
      (void)cols;
      (*out)[static_cast<size_t>(y) * w_bytes + bx] = tiled[src];
    }
  }
}

// A managed mmap'd multi-planar buffer.
struct MappedBuf {
  void* start[VIDEO_MAX_PLANES] = {};
  size_t length[VIDEO_MAX_PLANES] = {};
  __u32 n = 0;
};

// S_FMT then G_FMT-back; returns the driver-adjusted format in *out.
bool SetFmt(int fd, v4l2_buf_type type, __u32 fourcc, __u32 w, __u32 h,
            __u32 sizeimage, v4l2_format* out) {
  v4l2_format fmt{};
  fmt.type = type;
  fmt.fmt.pix_mp.width = w;
  fmt.fmt.pix_mp.height = h;
  fmt.fmt.pix_mp.pixelformat = fourcc;
  fmt.fmt.pix_mp.num_planes = 1;
  fmt.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;
  fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
  if (XIOCTL(fd, VIDIOC_S_FMT, &fmt) < 0) return false;
  *out = fmt;
  return true;
}

// REQBUFS `count` buffers of `type` (each with `n_planes` planes) and mmap
// them.
bool ReqAndMap(int fd, v4l2_buf_type type, __u32 n_planes, int count,
               std::vector<MappedBuf>* bufs) {
  v4l2_requestbuffers rb{};
  rb.count = static_cast<__u32>(count);
  rb.type = type;
  rb.memory = V4L2_MEMORY_MMAP;
  if (XIOCTL(fd, VIDIOC_REQBUFS, &rb) < 0) return false;

  bufs->assign(rb.count, {});
  for (__u32 i = 0; i < rb.count; ++i) {
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    v4l2_buffer b{};
    b.type = type;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    b.length = n_planes;
    b.m.planes = planes;
    if (XIOCTL(fd, VIDIOC_QUERYBUF, &b) < 0) return false;
    MappedBuf& mb = (*bufs)[i];
    mb.n = n_planes;
    for (__u32 pl = 0; pl < n_planes; ++pl) {
      void* p = mmap(nullptr, planes[pl].length, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, planes[pl].m.mem_offset);
      if (p == MAP_FAILED) {
        LOGE("mmap failed: %s\n", std::strerror(errno));
        return false;
      }
      mb.start[pl] = p;
      mb.length[pl] = planes[pl].length;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    LOGE("usage: %s <clip.hevc> [video_node=/dev/video19] [out.yuv]\n",
         argv[0]);
    return 2;
  }
  const char* clip = argv[1];
  const char* node = argc > 2 ? argv[2] : "/dev/video19";
  const char* out_path = argc > 3 ? argv[3] : nullptr;

  std::vector<std::uint8_t> data = ReadFile(clip);
  if (data.empty()) return 1;
  std::vector<Nal> nals = ParseAnnexB(data.data(), data.size());
  LOGE("parsed %zu NAL units from %s (%zu bytes)\n", nals.size(), clip,
       data.size());

  // Latest parameter sets, and pictures grouped at first_slice boundaries.
  Sps sps{};
  Pps pps{};
  bool have_sps = false, have_pps = false;
  std::vector<Picture> pics;
  for (const Nal& n : nals) {
    if (n.type == NalUnitType::kSpsNut) {
      if (ParseSps(n.rbsp.data(), n.rbsp.size(), &sps)) have_sps = true;
    } else if (n.type == NalUnitType::kPpsNut) {
      if (ParsePps(n.rbsp.data(), n.rbsp.size(), &pps)) have_pps = true;
    } else if (IsVcl(n.type)) {
      const bool first = !n.rbsp.empty() && (n.rbsp[0] & 0x80);
      if (first || pics.empty()) pics.push_back(Picture{});
      pics.back().slices.push_back(&n);
    }
  }
  if (!have_sps || !have_pps) {
    LOGE("missing SPS/PPS\n");
    return 1;
  }
  LOGE("SPS %ux%u %u-bit chroma_idc=%u; %zu pictures\n",
       sps.pic_width_in_luma_samples, sps.pic_height_in_luma_samples,
       sps.bit_depth_luma, sps.chroma_format_idc, pics.size());

  // Build the slice-parse context once from SPS/PPS.
  SliceContext sctx;
  sctx.pic_size_in_ctbs = sps.pic_size_in_ctbs;
  sctx.log2_max_pic_order_cnt_lsb = sps.log2_max_pic_order_cnt_lsb;
  sctx.separate_colour_plane_flag = sps.separate_colour_plane_flag;
  sctx.chroma_array_type =
      sps.separate_colour_plane_flag ? 0 : sps.chroma_format_idc;
  sctx.sample_adaptive_offset_enabled_flag =
      sps.sample_adaptive_offset_enabled_flag;
  sctx.sps_temporal_mvp_enabled_flag = sps.sps_temporal_mvp_enabled_flag;
  sctx.long_term_ref_pics_present_flag = sps.long_term_ref_pics_present_flag;
  sctx.num_long_term_ref_pics_sps = sps.num_long_term_ref_pics_sps;
  sctx.used_by_curr_pic_lt_sps = sps.used_by_curr_pic_lt_sps;
  sctx.lt_ref_pic_poc_lsb_sps = sps.lt_ref_pic_poc_lsb_sps;
  sctx.short_term_rps = sps.short_term_rps;
  sctx.dependent_slice_segments_enabled_flag =
      pps.dependent_slice_segments_enabled_flag;
  sctx.num_extra_slice_header_bits = pps.num_extra_slice_header_bits;
  sctx.output_flag_present_flag = pps.output_flag_present_flag;
  sctx.num_ref_idx_l0_default_active_minus1 =
      pps.num_ref_idx_l0_default_active_minus1;
  sctx.num_ref_idx_l1_default_active_minus1 =
      pps.num_ref_idx_l1_default_active_minus1;
  sctx.cabac_init_present_flag = pps.cabac_init_present_flag;
  sctx.weighted_pred_flag = pps.weighted_pred_flag;
  sctx.weighted_bipred_flag = pps.weighted_bipred_flag;
  sctx.pps_slice_chroma_qp_offsets_present_flag =
      pps.pps_slice_chroma_qp_offsets_present_flag;
  sctx.deblocking_filter_override_enabled_flag =
      pps.deblocking_filter_override_enabled_flag;
  sctx.pps_deblocking_filter_disabled_flag =
      pps.pps_deblocking_filter_disabled_flag;
  sctx.pps_loop_filter_across_slices_enabled_flag =
      pps.pps_loop_filter_across_slices_enabled_flag;
  sctx.tiles_enabled_flag = pps.tiles_enabled_flag;
  sctx.entropy_coding_sync_enabled_flag = pps.entropy_coding_sync_enabled_flag;
  sctx.lists_modification_present_flag = pps.lists_modification_present_flag;
  sctx.slice_segment_header_extension_present_flag =
      pps.slice_segment_header_extension_present_flag;
  sctx.chroma_qp_offset_list_enabled_flag =
      pps.range_extension.chroma_qp_offset_list_enabled_flag;

  int fd = ::open(node, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    LOGE("open %s: %s\n", node, std::strerror(errno));
    return 1;
  }
  v4l2_capability cap{};
  if (XIOCTL(fd, VIDIOC_QUERYCAP, &cap) < 0) return 1;
  LOGE("driver=%s card=%s caps=0x%08x\n", cap.driver, cap.card,
       cap.device_caps);

  int media_fd = OpenMediaDevice("rpi-hevc-dec");
  if (media_fd < 0) return 1;

  // decode_mode = FRAME_BASED, start_code = NONE (set on the device, not a
  // request).
  {
    v4l2_ext_control c[2]{};
    c[0].id = V4L2_CID_STATELESS_HEVC_DECODE_MODE;
    c[0].value = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED;
    c[1].id = V4L2_CID_STATELESS_HEVC_START_CODE;
    c[1].value = V4L2_STATELESS_HEVC_START_CODE_NONE;
    v4l2_ext_controls h{};
    h.count = 2;
    h.controls = c;
    if (XIOCTL(fd, VIDIOC_S_EXT_CTRLS, &h) < 0) return 1;
  }

  const __u32 w = sps.pic_width_in_luma_samples;
  const __u32 h = sps.pic_height_in_luma_samples;
  const __u32 out_size = w * h * 2 + (1u << 20);  // generous coded-input size

  v4l2_format ofmt{};
  if (!SetFmt(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE, V4L2_PIX_FMT_HEVC_SLICE, w,
              h, out_size, &ofmt))
    return 1;
  std::vector<MappedBuf> out_bufs;
  if (!ReqAndMap(fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
                 ofmt.fmt.pix_mp.num_planes, 4, &out_bufs))
    return 1;

  // Learn the driver's CAPTURE pixel format (a Broadcom column-tiled NV12),
  // then set the coded dimensions from the SPS so it sizes the frame correctly
  // (a stateless decoder can't derive them itself).
  v4l2_format gfmt{};
  gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (XIOCTL(fd, VIDIOC_G_FMT, &gfmt) < 0) return 1;
  const __u32 cfourcc = gfmt.fmt.pix_mp.pixelformat;
  v4l2_format cfmt{};
  if (!SetFmt(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, cfourcc, w, h, 0, &cfmt))
    return 1;
  LOGE("CAPTURE fmt=%c%c%c%c %ux%u planes=%u\n", cfourcc & 0xff,
       (cfourcc >> 8) & 0xff, (cfourcc >> 16) & 0xff, (cfourcc >> 24) & 0xff,
       cfmt.fmt.pix_mp.width, cfmt.fmt.pix_mp.height,
       cfmt.fmt.pix_mp.num_planes);
  for (__u32 k = 0; k < cfmt.fmt.pix_mp.num_planes; ++k)
    LOGE("  plane %u: bytesperline=%u sizeimage=%u\n", k,
         cfmt.fmt.pix_mp.plane_fmt[k].bytesperline,
         cfmt.fmt.pix_mp.plane_fmt[k].sizeimage);
  const __u32 cap_planes = cfmt.fmt.pix_mp.num_planes;
  const __u32 cap_bpl = cfmt.fmt.pix_mp.plane_fmt[0].bytesperline;
  // Column height of each tiled plane = plane bytes / stride.
  const __u32 luma_col_h = cfmt.fmt.pix_mp.plane_fmt[0].sizeimage / cap_bpl;
  const __u32 chroma_col_h = cap_planes > 1
                                 ? cfmt.fmt.pix_mp.plane_fmt[1].sizeimage /
                                       cfmt.fmt.pix_mp.plane_fmt[1].bytesperline
                                 : 0;
  std::vector<MappedBuf> cbufs;
  if (!ReqAndMap(fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE, cap_planes, 6, &cbufs))
    return 1;

  v4l2_buf_type ot = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  v4l2_buf_type ct = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (XIOCTL(fd, VIDIOC_STREAMON, &ot) < 0) return 1;
  if (XIOCTL(fd, VIDIOC_STREAMON, &ct) < 0) return 1;

  FILE* fout = out_path ? std::fopen(out_path, "wb") : nullptr;

  int prev_poc_lsb = 0, prev_poc_msb = 0, decoded = 0;
  bool seen_first = false;
  for (size_t pi = 0; pi < pics.size(); ++pi) {
    const Picture& pic = pics[pi];
    const Nal& first_nal = *pic.slices[0];
    SliceHeader sh0;
    if (!ParseSliceHeader(first_nal.rbsp.data(), first_nal.rbsp.size(),
                          first_nal.type, sctx, &sh0)) {
      LOGE("pic %zu: slice header parse failed\n", pi);
      continue;
    }
    if (sh0.slice_type != SliceType::kI) {
      LOGE("pic %zu: non-intra slice (type %d) -- skipped (milestone 1)\n", pi,
           static_cast<int>(sh0.slice_type));
      continue;
    }

    // POC (clause 8.3.1), simplified: IRAP that starts a CVS resets to 0.
    const bool idr = IsIdr(first_nal.type);
    const bool irap = IsIrap(first_nal.type);
    const bool no_rasl = idr || IsBla(first_nal.type) || (irap && !seen_first);
    int poc;
    if (no_rasl) {
      poc = 0;
      prev_poc_lsb = 0;
      prev_poc_msb = 0;
    } else {
      const int max_lsb = 1 << sps.log2_max_pic_order_cnt_lsb;
      const int lsb = static_cast<int>(sh0.slice_pic_order_cnt_lsb);
      int msb;
      if (lsb < prev_poc_lsb && (prev_poc_lsb - lsb) >= max_lsb / 2)
        msb = prev_poc_msb + max_lsb;
      else if (lsb > prev_poc_lsb && (lsb - prev_poc_lsb) > max_lsb / 2)
        msb = prev_poc_msb - max_lsb;
      else
        msb = prev_poc_msb;
      poc = msb + lsb;
      prev_poc_lsb = lsb;
      prev_poc_msb = msb;
    }
    seen_first = true;

    // Concatenate the slice NALs (raw, no start code) into an OUTPUT buffer and
    // record each slice's data offset.
    std::vector<std::uint8_t> buf;
    std::vector<v4l2_ctrl_hevc_slice_params> sparams;
    for (const Nal* np : pic.slices) {
      SliceHeader sh;
      if (!ParseSliceHeader(np->rbsp.data(), np->rbsp.size(), np->type, sctx,
                            &sh))
        continue;
      const size_t slice_off = buf.size();
      buf.insert(buf.end(), np->raw.begin(), np->raw.end());

      std::uint32_t raw_bit = 0;
      RbspToRawBitOffset(*np, sh.slice_data_bit_offset_rbsp, &raw_bit);
      // data_byte_offset is the byte offset from the buffer start to this
      // slice's entropy-coded data; bit_size is the size of that data (from the
      // offset to the slice end) in bits -- NOT the whole NAL, which would make
      // the hardware read past the buffer and fail the decode.
      const __u32 dbo = static_cast<__u32>(slice_off + raw_bit / 8);
      v4l2_ctrl_hevc_slice_params sp{};
      sp.data_byte_offset = dbo;
      sp.bit_size = static_cast<__u32>((np->raw.size() - raw_bit / 8) * 8);
      sp.num_entry_point_offsets = sh.num_entry_point_offsets;
      sp.nal_unit_type = static_cast<__u8>(np->type);
      sp.nuh_temporal_id_plus1 = np->nuh_temporal_id_plus1;
      sp.slice_type = static_cast<__u8>(sh.slice_type);
      sp.colour_plane_id = static_cast<__u8>(sh.colour_plane_id);
      sp.slice_pic_order_cnt = poc;
      sp.num_ref_idx_l0_active_minus1 =
          static_cast<__u8>(sh.num_ref_idx_l0_active_minus1);
      sp.num_ref_idx_l1_active_minus1 =
          static_cast<__u8>(sh.num_ref_idx_l1_active_minus1);
      sp.collocated_ref_idx = static_cast<__u8>(sh.collocated_ref_idx);
      sp.five_minus_max_num_merge_cand =
          static_cast<__u8>(sh.five_minus_max_num_merge_cand);
      sp.slice_qp_delta = static_cast<__s8>(sh.slice_qp_delta);
      sp.slice_cb_qp_offset = static_cast<__s8>(sh.slice_cb_qp_offset);
      sp.slice_cr_qp_offset = static_cast<__s8>(sh.slice_cr_qp_offset);
      sp.slice_beta_offset_div2 = static_cast<__s8>(sh.slice_beta_offset_div2);
      sp.slice_tc_offset_div2 = static_cast<__s8>(sh.slice_tc_offset_div2);
      sp.slice_segment_addr = sh.slice_segment_address;
      sp.short_term_ref_pic_set_size =
          static_cast<__u16>(sh.short_term_ref_pic_set_bits);
      for (int k = 0; k < kMaxDpb; ++k) {
        sp.ref_idx_l0[k] = 0;
        sp.ref_idx_l1[k] = 0;
      }
      __u64 f = 0;
      if (sh.slice_sao_luma_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;
      if (sh.slice_sao_chroma_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;
      if (sh.slice_temporal_mvp_enabled_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
      if (sh.mvd_l1_zero_flag) f |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;
      if (sh.cabac_init_flag) f |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;
      if (sh.collocated_from_l0_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
      if (sh.slice_deblocking_filter_disabled_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
      if (sh.slice_loop_filter_across_slices_enabled_flag)
        f |=
            V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;
      if (sh.dependent_slice_segment_flag)
        f |= V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
      sp.flags = f;
      sparams.push_back(sp);
    }
    if (sparams.empty()) continue;

    v4l2_ctrl_hevc_decode_params dp{};
    dp.pic_order_cnt_val = poc;
    dp.num_active_dpb_entries = 0;  // intra: empty DPB
    __u64 df = 0;
    if (idr) df |= V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC;
    if (irap) df |= V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC;
    if (sh0.no_output_of_prior_pics_flag)
      df |= V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR;
    dp.flags = df;

    v4l2_ctrl_hevc_sps ksps;
    v4l2_ctrl_hevc_pps kpps;
    // Flat default (all coefficients 16) unless scaling lists are enabled; the
    // driver wants this control present either way.
    v4l2_ctrl_hevc_scaling_matrix ksm;
    std::memset(&ksm, 16, sizeof(ksm));
    FillSps(sps, &ksps);
    FillPps(pps, &kpps);
    const ScalingListData& sl = pps.pps_scaling_list_data_present_flag
                                    ? pps.scaling_list
                                    : sps.scaling_list;
    if (sps.scaling_list_enabled_flag) FillScalingMatrix(sl, &ksm);

    // Allocate a request and attach the controls to it.
    int req_fd = -1;
    if (Xioctl(media_fd, MEDIA_IOC_REQUEST_ALLOC, &req_fd,
               "MEDIA_IOC_REQUEST_ALLOC") < 0)
      break;

    v4l2_ext_control ctrls[5]{};
    int nc = 0;
    ctrls[nc].id = V4L2_CID_STATELESS_HEVC_SPS;
    ctrls[nc].ptr = &ksps;
    ctrls[nc++].size = sizeof(ksps);
    ctrls[nc].id = V4L2_CID_STATELESS_HEVC_PPS;
    ctrls[nc].ptr = &kpps;
    ctrls[nc++].size = sizeof(kpps);
    ctrls[nc].id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS;
    ctrls[nc].ptr = &dp;
    ctrls[nc++].size = sizeof(dp);
    ctrls[nc].id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS;
    ctrls[nc].ptr = sparams.data();
    ctrls[nc++].size = static_cast<__u32>(sparams.size() * sizeof(sparams[0]));
    ctrls[nc].id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX;
    ctrls[nc].ptr = &ksm;
    ctrls[nc++].size = sizeof(ksm);
    v4l2_ext_controls hc{};
    hc.which = V4L2_CTRL_WHICH_REQUEST_VAL;
    hc.request_fd = req_fd;
    hc.count = static_cast<__u32>(nc);
    hc.controls = ctrls;
    if (XIOCTL(fd, VIDIOC_S_EXT_CTRLS, &hc) < 0) {
      ::close(req_fd);
      break;
    }

    // Copy the coded frame into OUTPUT buffer 0 and queue it against the
    // request; queue a CAPTURE buffer to receive the decoded frame.
    const int obuf = static_cast<int>(pi % out_bufs.size());
    const int cbuf = static_cast<int>(pi % cbufs.size());
    std::memcpy(out_bufs[obuf].start[0], buf.data(),
                std::min(buf.size(), out_bufs[obuf].length[0]));
    {
      v4l2_plane pl{};
      pl.bytesused = static_cast<__u32>(buf.size());
      pl.length = static_cast<__u32>(out_bufs[obuf].length[0]);
      v4l2_buffer b{};
      b.type = ot;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = static_cast<__u32>(obuf);
      b.length = 1;
      b.m.planes = &pl;
      b.flags = V4L2_BUF_FLAG_REQUEST_FD;
      b.request_fd = req_fd;
      b.timestamp.tv_usec = static_cast<long>(pi + 1);  // unique tag
      if (XIOCTL(fd, VIDIOC_QBUF, &b) < 0) {
        ::close(req_fd);
        break;
      }
    }
    {
      v4l2_plane pl[VIDEO_MAX_PLANES]{};
      for (__u32 k = 0; k < cbufs[cbuf].n; ++k)
        pl[k].length = static_cast<__u32>(cbufs[cbuf].length[k]);
      v4l2_buffer b{};
      b.type = ct;
      b.memory = V4L2_MEMORY_MMAP;
      b.index = static_cast<__u32>(cbuf);
      b.length = cbufs[cbuf].n;
      b.m.planes = pl;
      if (XIOCTL(fd, VIDIOC_QBUF, &b) < 0) {
        ::close(req_fd);
        break;
      }
    }
    if (Xioctl(req_fd, MEDIA_REQUEST_IOC_QUEUE, nullptr,
               "MEDIA_REQUEST_IOC_QUEUE") < 0) {
      ::close(req_fd);
      break;
    }

    // Wait for completion, then dequeue both buffers.
    pollfd pfd{fd, POLLIN | POLLOUT, 0};
    poll(&pfd, 1, 2000);
    {
      v4l2_plane pl{};
      v4l2_buffer b{};
      b.type = ot;
      b.memory = V4L2_MEMORY_MMAP;
      b.length = 1;
      b.m.planes = &pl;
      XIOCTL(fd, VIDIOC_DQBUF, &b);
    }
    v4l2_plane cpl[VIDEO_MAX_PLANES]{};
    v4l2_buffer cb{};
    cb.type = ct;
    cb.memory = V4L2_MEMORY_MMAP;
    cb.length = cap_planes;
    cb.m.planes = cpl;
    if (XIOCTL(fd, VIDIOC_DQBUF, &cb) < 0) {
      ::close(req_fd);
      break;
    }
    __u32 used = 0;
    for (__u32 k = 0; k < cap_planes; ++k) used += cpl[k].bytesused;
    LOGE(
        "pic %zu: POC=%d slices=%zu -> CAPTURE idx=%u planes=%u bytesused=%u "
        "flags=0x%08x%s\n",
        pi, poc, sparams.size(), cb.index, cap_planes, used, cb.flags,
        (cb.flags & V4L2_BUF_FLAG_ERROR) ? " ERROR" : "");
    const bool err = (cb.flags & V4L2_BUF_FLAG_ERROR) != 0;
    if (used > 0 && !err) ++decoded;
    // Detile the tiled NV12 into planar I420 (Y, then U, then V) so the output
    // compares directly against an `ffmpeg -pix_fmt yuv420p` reference.
    if (fout && used > 0 && !err && cap_planes >= 2) {
      std::vector<std::uint8_t> y_plane, uv_plane;
      DetileCol128(static_cast<std::uint8_t*>(cbufs[cb.index].start[0]),
                   cap_bpl, w, h, luma_col_h, &y_plane);
      DetileCol128(static_cast<std::uint8_t*>(cbufs[cb.index].start[1]),
                   cap_bpl, w, h / 2, chroma_col_h,
                   &uv_plane);  // interleaved UV
      std::vector<std::uint8_t> u((w / 2) * (h / 2)), v((w / 2) * (h / 2));
      for (std::uint32_t cy = 0; cy < h / 2; ++cy)
        for (std::uint32_t cx = 0; cx < w / 2; ++cx) {
          u[cy * (w / 2) + cx] = uv_plane[cy * w + 2 * cx];
          v[cy * (w / 2) + cx] = uv_plane[cy * w + 2 * cx + 1];
        }
      std::fwrite(y_plane.data(), 1, y_plane.size(), fout);
      std::fwrite(u.data(), 1, u.size(), fout);
      std::fwrite(v.data(), 1, v.size(), fout);
    }

    ::close(req_fd);
  }

  if (fout) std::fclose(fout);
  XIOCTL(fd, VIDIOC_STREAMOFF, &ot);
  XIOCTL(fd, VIDIOC_STREAMOFF, &ct);
  for (auto& b : out_bufs)
    for (__u32 k = 0; k < b.n; ++k)
      if (b.start[k]) munmap(b.start[k], b.length[k]);
  for (auto& b : cbufs)
    for (__u32 k = 0; k < b.n; ++k)
      if (b.start[k]) munmap(b.start[k], b.length[k]);
  ::close(media_fd);
  ::close(fd);

  LOGE("=== decoded %d / %zu intra pictures ===\n", decoded, pics.size());
  return decoded > 0 ? 0 : 1;
}
