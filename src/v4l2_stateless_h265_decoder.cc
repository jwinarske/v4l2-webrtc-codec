// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_stateless_h265_decoder.h"

#include <fcntl.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>

#include "parse/h265/scaling_list.h"
#include "src/log.h"

namespace v4l2wc {
namespace {

constexpr int kMaxDpb = V4L2_HEVC_DPB_ENTRIES_NUM_MAX;

// DRM fourccs and the Broadcom SAND128 column-tiled modifier (uapi
// drm_fourcc.h), inlined so the core library needs no libdrm at build time.
// The V4L2 column-tiled CAPTURE formats map to a standard DRM format plus the
// SAND128 modifier, whose parameter carries the tile column height a consumer
// needs to import and detile the dma-buf.
constexpr std::uint32_t kDrmFormatNV12 = 0x3231564e;  // 'NV12' (8-bit 4:2:0)
constexpr std::uint32_t kDrmFormatP030 = 0x30333050;  // 'P030' (10-bit packed)
std::uint64_t Sand128ColHeight(std::uint32_t col_h) {
  constexpr std::uint64_t kVendorBroadcom = 7;
  // fourcc_mod_broadcom_code(4, v): the SAND128 (128-byte column) tiling. Code
  // 3 is SAND64 and code 5 is SAND256 -- the wrong one silently mis-tiles a KMS
  // plane or a GL import, so it must match the actual 128-byte column layout.
  constexpr std::uint64_t kSand128 = 4;
  return (kVendorBroadcom << 56) |
         ((static_cast<std::uint64_t>(col_h) << 8) | kSand128);
}

int Xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do {
    r = ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

// The V4L2 buffer timestamp is the reference key: a picture decoded from an
// OUTPUT buffer stamped `tag` is referenced later by v4l2_timeval_to_ns() of
// that stamp. Encode a tag as whole seconds so the ns value is tag * 1e9.
void SetTag(v4l2_buffer* b, std::uint64_t tag) {
  b->timestamp.tv_sec = static_cast<long>(tag);
  b->timestamp.tv_usec = 0;
}
std::uint64_t TagNs(std::uint64_t tag) { return tag * 1000000000ULL; }

void FillSps(const h265::Sps& s, v4l2_ctrl_hevc_sps* o) {
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
  o->sps_max_num_reorder_pics = static_cast<__u8>(s.sps_max_num_reorder_pics);
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

void FillPps(const h265::Pps& p, v4l2_ctrl_hevc_pps* o) {
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

void FillScalingMatrix(const h265::ScalingListData& s,
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

void BuildSliceContext(const h265::Sps& sps, const h265::Pps& pps,
                       h265::SliceContext* ctx) {
  ctx->pic_size_in_ctbs = sps.pic_size_in_ctbs;
  ctx->log2_max_pic_order_cnt_lsb = sps.log2_max_pic_order_cnt_lsb;
  ctx->separate_colour_plane_flag = sps.separate_colour_plane_flag;
  ctx->chroma_array_type =
      sps.separate_colour_plane_flag ? 0 : sps.chroma_format_idc;
  ctx->sample_adaptive_offset_enabled_flag =
      sps.sample_adaptive_offset_enabled_flag;
  ctx->sps_temporal_mvp_enabled_flag = sps.sps_temporal_mvp_enabled_flag;
  ctx->long_term_ref_pics_present_flag = sps.long_term_ref_pics_present_flag;
  ctx->num_long_term_ref_pics_sps = sps.num_long_term_ref_pics_sps;
  ctx->used_by_curr_pic_lt_sps = sps.used_by_curr_pic_lt_sps;
  ctx->lt_ref_pic_poc_lsb_sps = sps.lt_ref_pic_poc_lsb_sps;
  ctx->short_term_rps = sps.short_term_rps;
  ctx->dependent_slice_segments_enabled_flag =
      pps.dependent_slice_segments_enabled_flag;
  ctx->num_extra_slice_header_bits = pps.num_extra_slice_header_bits;
  ctx->output_flag_present_flag = pps.output_flag_present_flag;
  ctx->num_ref_idx_l0_default_active_minus1 =
      pps.num_ref_idx_l0_default_active_minus1;
  ctx->num_ref_idx_l1_default_active_minus1 =
      pps.num_ref_idx_l1_default_active_minus1;
  ctx->cabac_init_present_flag = pps.cabac_init_present_flag;
  ctx->weighted_pred_flag = pps.weighted_pred_flag;
  ctx->weighted_bipred_flag = pps.weighted_bipred_flag;
  ctx->pps_slice_chroma_qp_offsets_present_flag =
      pps.pps_slice_chroma_qp_offsets_present_flag;
  ctx->deblocking_filter_override_enabled_flag =
      pps.deblocking_filter_override_enabled_flag;
  ctx->pps_deblocking_filter_disabled_flag =
      pps.pps_deblocking_filter_disabled_flag;
  ctx->pps_loop_filter_across_slices_enabled_flag =
      pps.pps_loop_filter_across_slices_enabled_flag;
  ctx->tiles_enabled_flag = pps.tiles_enabled_flag;
  ctx->entropy_coding_sync_enabled_flag = pps.entropy_coding_sync_enabled_flag;
  ctx->lists_modification_present_flag = pps.lists_modification_present_flag;
  ctx->slice_segment_header_extension_present_flag =
      pps.slice_segment_header_extension_present_flag;
  ctx->chroma_qp_offset_list_enabled_flag =
      pps.range_extension.chroma_qp_offset_list_enabled_flag;
}

}  // namespace

std::unique_ptr<V4l2StatelessH265Decoder> V4l2StatelessH265Decoder::Create(
    const char* video_node, std::uint32_t pool_size) {
  auto dec =
      std::unique_ptr<V4l2StatelessH265Decoder>(new V4l2StatelessH265Decoder());
  dec->video_fd_ = ::open(video_node, O_RDWR | O_CLOEXEC);
  if (dec->video_fd_ < 0) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "v4l2-h265: open(" << video_node << ") failed";
    return nullptr;
  }
  // Confirm the device accepts the stateless HEVC slice format on its OUTPUT.
  bool hevc = false;
  for (std::uint32_t i = 0;; ++i) {
    v4l2_fmtdesc fd{};
    fd.index = i;
    fd.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    if (Xioctl(dec->video_fd_, VIDIOC_ENUM_FMT, &fd) < 0) break;
    if (fd.pixelformat == V4L2_PIX_FMT_HEVC_SLICE) hevc = true;
  }
  if (!hevc) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "v4l2-h265: " << video_node << " has no HEVC_SLICE output format";
    return nullptr;
  }
  // Find the request-API media device (matched by driver name).
  v4l2_capability cap{};
  if (Xioctl(dec->video_fd_, VIDIOC_QUERYCAP, &cap) < 0) return nullptr;
  for (int i = 0; i < 16; ++i) {
    char path[32];
    std::snprintf(path, sizeof(path), "/dev/media%d", i);
    int mfd = ::open(path, O_RDWR | O_CLOEXEC);
    if (mfd < 0) continue;
    media_device_info info{};
    if (Xioctl(mfd, MEDIA_IOC_DEVICE_INFO, &info) == 0 &&
        std::strncmp(info.driver, reinterpret_cast<const char*>(cap.driver),
                     sizeof(info.driver)) == 0) {
      dec->media_fd_ = mfd;
      break;
    }
    ::close(mfd);
  }
  if (dec->media_fd_ < 0) {
    V4L2WC_LOG(V4L2WC_WARNING) << "v4l2-h265: no matching media device";
    return nullptr;
  }
  dec->pool_size_ = pool_size;
  V4L2WC_LOG(V4L2WC_INFO) << "v4l2-h265: " << video_node << " (" << cap.driver
                          << ")";
  return dec;
}

V4l2StatelessH265Decoder::V4l2StatelessH265Decoder() = default;

V4l2StatelessH265Decoder::~V4l2StatelessH265Decoder() {
  if (video_fd_ >= 0) {
    v4l2_buf_type ot = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    v4l2_buf_type ct = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    Xioctl(video_fd_, VIDIOC_STREAMOFF, &ot);
    Xioctl(video_fd_, VIDIOC_STREAMOFF, &ct);
  }
  for (auto& b : output_bufs_)
    for (std::uint32_t k = 0; k < b.n; ++k)
      if (b.start[k]) munmap(b.start[k], b.length[k]);
  for (auto& c : captures_) {
    for (std::uint32_t k = 0; k < c.map.n; ++k) {
      if (c.map.start[k]) munmap(c.map.start[k], c.map.length[k]);
      if (c.fd[k] >= 0) ::close(c.fd[k]);
    }
  }
  if (media_fd_ >= 0) ::close(media_fd_);
  if (video_fd_ >= 0) ::close(video_fd_);
}

bool V4l2StatelessH265Decoder::EnsureConfigured(const h265::Sps& sps) {
  if (configured_) return true;
  if ((sps.bit_depth_luma != 8 && sps.bit_depth_luma != 10) ||
      sps.chroma_format_idc != 1) {
    V4L2WC_LOG(V4L2WC_WARNING)
        << "v4l2-h265: only 8/10-bit 4:2:0 configured (bd="
        << sps.bit_depth_luma << " cfmt=" << sps.chroma_format_idc << ")";
    return false;
  }
  coded_w_ = sps.pic_width_in_luma_samples;
  coded_h_ = sps.pic_height_in_luma_samples;
  coded_bit_depth_ = sps.bit_depth_luma;

  // Frame-based decode, slice data with no start codes.
  {
    v4l2_ext_control c[2]{};
    c[0].id = V4L2_CID_STATELESS_HEVC_DECODE_MODE;
    c[0].value = V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED;
    c[1].id = V4L2_CID_STATELESS_HEVC_START_CODE;
    c[1].value = V4L2_STATELESS_HEVC_START_CODE_NONE;
    v4l2_ext_controls h{};
    h.count = 2;
    h.controls = c;
    if (Xioctl(video_fd_, VIDIOC_S_EXT_CTRLS, &h) < 0) {
      V4L2WC_LOG(V4L2WC_ERROR)
          << "v4l2-h265: set decode-mode/start-code failed";
      return false;
    }
  }

  // OUTPUT (coded input) queue.
  v4l2_format ofmt{};
  ofmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  ofmt.fmt.pix_mp.width = coded_w_;
  ofmt.fmt.pix_mp.height = coded_h_;
  ofmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_HEVC_SLICE;
  ofmt.fmt.pix_mp.num_planes = 1;
  ofmt.fmt.pix_mp.plane_fmt[0].sizeimage = coded_w_ * coded_h_ * 2 + (1u << 20);
  ofmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
  if (Xioctl(video_fd_, VIDIOC_S_FMT, &ofmt) < 0) return false;

  v4l2_requestbuffers orb{};
  orb.count = 4;
  orb.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  orb.memory = V4L2_MEMORY_MMAP;
  if (Xioctl(video_fd_, VIDIOC_REQBUFS, &orb) < 0) return false;
  output_bufs_.resize(orb.count);
  for (std::uint32_t i = 0; i < orb.count; ++i) {
    v4l2_plane pl[VIDEO_MAX_PLANES]{};
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    b.length = 1;
    b.m.planes = pl;
    if (Xioctl(video_fd_, VIDIOC_QUERYBUF, &b) < 0) return false;
    void* p = mmap(nullptr, pl[0].length, PROT_READ | PROT_WRITE, MAP_SHARED,
                   video_fd_, pl[0].m.mem_offset);
    if (p == MAP_FAILED) return false;
    output_bufs_[i].n = 1;
    output_bufs_[i].start[0] = p;
    output_bufs_[i].length[0] = pl[0].length;
  }

  // CAPTURE (decoded) queue: the driver defaults to the 8-bit column-tiled
  // format, so a 10-bit stream needs its column-tiled counterpart requested
  // explicitly (the stateless driver cannot know the bit depth itself).
  v4l2_format gfmt{};
  gfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (Xioctl(video_fd_, VIDIOC_G_FMT, &gfmt) < 0) return false;
  const std::uint32_t nc12 = v4l2_fourcc('N', 'c', '1', '2');  // 8-bit col128
  const std::uint32_t nc30 = v4l2_fourcc('N', 'c', '3', '0');  // 10-bit col128
  const std::uint32_t want_fourcc =
      coded_bit_depth_ == 10
          ? nc30
          : (gfmt.fmt.pix_mp.pixelformat != 0 ? gfmt.fmt.pix_mp.pixelformat
                                              : nc12);
  v4l2_format cfmt{};
  cfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  cfmt.fmt.pix_mp.width = coded_w_;
  cfmt.fmt.pix_mp.height = coded_h_;
  cfmt.fmt.pix_mp.pixelformat = want_fourcc;
  cfmt.fmt.pix_mp.num_planes = 1;
  cfmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
  if (Xioctl(video_fd_, VIDIOC_S_FMT, &cfmt) < 0) return false;
  cap_fourcc_ = cfmt.fmt.pix_mp.pixelformat;
  cap_planes_ = cfmt.fmt.pix_mp.num_planes;
  cap_stride_ = cfmt.fmt.pix_mp.plane_fmt[0].bytesperline;
  luma_col_h_ = cap_stride_
                    ? cfmt.fmt.pix_mp.plane_fmt[0].sizeimage / cap_stride_
                    : coded_h_;
  chroma_col_h_ = (cap_planes_ > 1 && cfmt.fmt.pix_mp.plane_fmt[1].bytesperline)
                      ? cfmt.fmt.pix_mp.plane_fmt[1].sizeimage /
                            cfmt.fmt.pix_mp.plane_fmt[1].bytesperline
                      : coded_h_ / 2;

  v4l2_requestbuffers crb{};
  crb.count = pool_size_;
  crb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  crb.memory = V4L2_MEMORY_MMAP;
  if (Xioctl(video_fd_, VIDIOC_REQBUFS, &crb) < 0) return false;
  pool_size_ = crb.count;
  captures_.resize(crb.count);
  for (std::uint32_t i = 0; i < crb.count; ++i) {
    v4l2_plane pl[VIDEO_MAX_PLANES]{};
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = i;
    b.length = cap_planes_;
    b.m.planes = pl;
    if (Xioctl(video_fd_, VIDIOC_QUERYBUF, &b) < 0) return false;
    captures_[i].map.n = cap_planes_;
    for (std::uint32_t k = 0; k < cap_planes_; ++k) {
      void* p = mmap(nullptr, pl[k].length, PROT_READ | PROT_WRITE, MAP_SHARED,
                     video_fd_, pl[k].m.mem_offset);
      if (p == MAP_FAILED) return false;
      captures_[i].map.start[k] = p;
      captures_[i].map.length[k] = pl[k].length;
    }
    RequeueCapture(static_cast<int>(i));  // hand all capture buffers to driver
  }

  v4l2_buf_type ot = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
  v4l2_buf_type ct = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  if (Xioctl(video_fd_, VIDIOC_STREAMON, &ot) < 0) return false;
  if (Xioctl(video_fd_, VIDIOC_STREAMON, &ct) < 0) return false;
  reorder_depth_ = sps.sps_max_num_reorder_pics;
  configured_ = true;
  V4L2WC_LOG(V4L2WC_INFO) << "v4l2-h265: configured " << coded_w_ << "x"
                          << coded_h_ << " " << coded_bit_depth_
                          << "-bit pool=" << pool_size_
                          << " reorder=" << reorder_depth_;
  return true;
}

// Re-queues a capture buffer to the driver once it is no longer needed for
// reference, output, or hand-out.
void V4l2StatelessH265Decoder::MaybeRequeue(int index) {
  const Capture& c = captures_[index];
  if (c.queued || c.is_reference || c.checked_out || c.pending_output) return;
  for (int r : output_ready_)
    if (r == index) return;
  RequeueCapture(index);
}

void V4l2StatelessH265Decoder::RequeueCapture(int index) {
  Capture& c = captures_[index];
  if (c.queued) return;
  v4l2_plane pl[VIDEO_MAX_PLANES]{};
  for (std::uint32_t k = 0; k < cap_planes_; ++k)
    pl[k].length = static_cast<__u32>(c.map.length[k]);
  v4l2_buffer b{};
  b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  b.memory = V4L2_MEMORY_MMAP;
  b.index = static_cast<__u32>(index);
  b.length = cap_planes_;
  b.m.planes = pl;
  if (Xioctl(video_fd_, VIDIOC_QBUF, &b) == 0) c.queued = true;
}

int V4l2StatelessH265Decoder::ComputePoc(const h265::SliceHeader& sh,
                                         const h265::Nal& nal,
                                         bool no_rasl_output) {
  const int max_poc_lsb = 1 << sps_.log2_max_pic_order_cnt_lsb;
  const int poc_lsb = static_cast<int>(sh.slice_pic_order_cnt_lsb);
  int poc_msb;
  if (no_rasl_output) {
    poc_msb = 0;
  } else if (poc_lsb < prev_poc_lsb_ &&
             (prev_poc_lsb_ - poc_lsb) >= max_poc_lsb / 2) {
    poc_msb = prev_poc_msb_ + max_poc_lsb;
  } else if (poc_lsb > prev_poc_lsb_ &&
             (poc_lsb - prev_poc_lsb_) > max_poc_lsb / 2) {
    poc_msb = prev_poc_msb_ - max_poc_lsb;
  } else {
    poc_msb = prev_poc_msb_;
  }
  const int poc = poc_msb + poc_lsb;
  // RASL / RADL / sub-layer non-reference pictures do not advance the anchor.
  const bool is_rasl_radl = nal.type == h265::NalUnitType::kRaslN ||
                            nal.type == h265::NalUnitType::kRaslR ||
                            nal.type == h265::NalUnitType::kRadlN ||
                            nal.type == h265::NalUnitType::kRadlR;
  if (nal.nuh_temporal_id_plus1 == 1 && !is_rasl_radl) {
    prev_poc_lsb_ = poc_lsb;
    prev_poc_msb_ = poc_msb;
  }
  return poc;
}

// Included from a separate translation unit section below to keep this file
// focused; DecodePicture and the IDmaDecoder surface follow.
bool V4l2StatelessH265Decoder::DecodePicture(
    const std::vector<const h265::Nal*>& slices, std::uint64_t timestamp) {
  if (slices.empty()) return false;
  h265::SliceContext ctx;
  BuildSliceContext(sps_, pps_, &ctx);

  std::vector<h265::SliceHeader> headers(slices.size());
  for (std::size_t i = 0; i < slices.size(); ++i) {
    if (!h265::ParseSliceHeader(slices[i]->rbsp.data(), slices[i]->rbsp.size(),
                                slices[i]->type, ctx, &headers[i])) {
      V4L2WC_LOG(V4L2WC_WARNING) << "v4l2-h265: malformed slice header";
      return false;
    }
  }
  const h265::Nal& nal = *slices[0];
  const h265::SliceHeader& sh = headers[0];

  const bool no_rasl_output =
      h265::IsIdr(nal.type) || h265::IsBla(nal.type) ||
      (h265::IsIrap(nal.type) && (!seen_first_picture_ || eos_seen_));
  eos_seen_ = false;
  const int poc = ComputePoc(sh, nal, no_rasl_output);
  seen_first_picture_ = true;

  if (h265::IsIrap(nal.type) && no_rasl_output) {
    for (auto& r : dpb_) captures_[r.capture].is_reference = false;
    dpb_.clear();
  }

  // Reference-picture-set derivation (clause 8.3.2).
  struct RpsEntry {
    int poc;
    int capture;
    bool found;
  };
  std::vector<RpsEntry> st_curr_before, st_curr_after, lt_curr;
  std::vector<int> kept_pocs;
  auto find_ref = [&](int target) -> RpsEntry {
    for (const auto& r : dpb_)
      if (r.poc == target) return {target, r.capture, true};
    return {target, -1, false};
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

  const int max_poc_lsb = 1 << sps_.log2_max_pic_order_cnt_lsb;
  std::vector<int> lt_captures;
  const std::uint32_t num_long_term =
      sh.num_long_term_sps + sh.num_long_term_pics;
  for (std::uint32_t i = 0; i < num_long_term && i < h265::kMaxLongTermTotal;
       ++i) {
    const h265::LongTermRef& lt = sh.long_term_refs[i];
    RpsEntry e{0, -1, false};
    if (lt.delta_poc_msb_present) {
      const std::int64_t poc_lt =
          static_cast<std::int64_t>(lt.poc_lsb) + poc -
          static_cast<std::int64_t>(lt.delta_poc_msb_cycle) * max_poc_lsb -
          (poc & (max_poc_lsb - 1));
      e.poc = static_cast<int>(poc_lt);
      for (const auto& r : dpb_)
        if (r.poc == poc_lt) {
          e.capture = r.capture;
          e.found = true;
          break;
        }
    } else {
      for (const auto& r : dpb_)
        if ((r.poc & (max_poc_lsb - 1)) == static_cast<int>(lt.poc_lsb)) {
          e.poc = r.poc;
          e.capture = r.capture;
          e.found = true;
          break;
        }
    }
    if (e.found) {
      kept_pocs.push_back(e.poc);
      lt_captures.push_back(e.capture);
    }
    if (lt.used_by_curr) lt_curr.push_back(e);
  }

  // Evict DPB pictures the current set does not keep.
  {
    std::vector<RefPic> kept;
    for (const auto& r : dpb_) {
      bool keep = false;
      for (int p : kept_pocs)
        if (p == r.poc) {
          keep = true;
          break;
        }
      if (keep)
        kept.push_back(r);
      else
        captures_[r.capture].is_reference = false;
    }
    dpb_.swap(kept);
  }

  // The DPB array the driver sees; poc_st_curr_* index into it, and so do the
  // slice reference lists. Mark long-term entries.
  int dpb_index_by_capture[64];
  for (int& e : dpb_index_by_capture) e = -1;
  v4l2_ctrl_hevc_decode_params dp{};
  dp.pic_order_cnt_val = poc;
  dp.num_active_dpb_entries = static_cast<__u8>(dpb_.size());
  for (std::size_t j = 0; j < dpb_.size() && j < kMaxDpb; ++j) {
    const RefPic& r = dpb_[j];
    dp.dpb[j].timestamp = TagNs(r.timestamp);
    dp.dpb[j].pic_order_cnt_val = r.poc;
    dp.dpb[j].flags = 0;
    for (int c : lt_captures)
      if (c == r.capture)
        dp.dpb[j].flags |= V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE;
    if (r.capture < 64) dpb_index_by_capture[r.capture] = static_cast<int>(j);
  }
  auto dpb_index = [&](const RpsEntry& e) -> std::uint8_t {
    return (e.found && e.capture >= 0 && e.capture < 64 &&
            dpb_index_by_capture[e.capture] >= 0)
               ? static_cast<std::uint8_t>(dpb_index_by_capture[e.capture])
               : 0;
  };
  dp.num_poc_st_curr_before = static_cast<__u8>(st_curr_before.size());
  dp.num_poc_st_curr_after = static_cast<__u8>(st_curr_after.size());
  dp.num_poc_lt_curr = static_cast<__u8>(lt_curr.size());
  for (std::size_t i = 0; i < st_curr_before.size() && i < kMaxDpb; ++i)
    dp.poc_st_curr_before[i] = dpb_index(st_curr_before[i]);
  for (std::size_t i = 0; i < st_curr_after.size() && i < kMaxDpb; ++i)
    dp.poc_st_curr_after[i] = dpb_index(st_curr_after[i]);
  for (std::size_t i = 0; i < lt_curr.size() && i < kMaxDpb; ++i)
    dp.poc_lt_curr[i] = dpb_index(lt_curr[i]);
  dp.short_term_ref_pic_set_size =
      static_cast<__u16>(sh.short_term_ref_pic_set_bits);
  if (h265::IsIdr(nal.type)) dp.flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC;
  if (h265::IsIrap(nal.type)) dp.flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC;
  if (sh.no_output_of_prior_pics_flag)
    dp.flags |= V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR;

  // Build the concatenated slice buffer and per-slice params.
  std::vector<std::uint8_t> buf;
  std::vector<v4l2_ctrl_hevc_slice_params> sparams;
  for (std::size_t si = 0; si < slices.size(); ++si) {
    const h265::Nal* np = slices[si];
    const h265::SliceHeader& ssh = headers[si];
    const std::size_t slice_off = buf.size();
    buf.insert(buf.end(), np->raw.begin(), np->raw.end());
    std::uint32_t raw_bit = 0;
    h265::RbspToRawBitOffset(*np, ssh.slice_data_bit_offset_rbsp, &raw_bit);

    v4l2_ctrl_hevc_slice_params sp{};
    sp.data_byte_offset = static_cast<__u32>(slice_off + raw_bit / 8);
    sp.bit_size = static_cast<__u32>((np->raw.size() - raw_bit / 8) * 8);
    sp.num_entry_point_offsets = ssh.num_entry_point_offsets;
    sp.nal_unit_type = static_cast<__u8>(np->type);
    sp.nuh_temporal_id_plus1 = np->nuh_temporal_id_plus1;
    sp.slice_type = static_cast<__u8>(ssh.slice_type);
    sp.colour_plane_id = static_cast<__u8>(ssh.colour_plane_id);
    sp.slice_pic_order_cnt = poc;
    sp.num_ref_idx_l0_active_minus1 =
        static_cast<__u8>(ssh.num_ref_idx_l0_active_minus1);
    sp.num_ref_idx_l1_active_minus1 =
        static_cast<__u8>(ssh.num_ref_idx_l1_active_minus1);
    sp.collocated_ref_idx = static_cast<__u8>(ssh.collocated_ref_idx);
    sp.five_minus_max_num_merge_cand =
        static_cast<__u8>(ssh.five_minus_max_num_merge_cand);
    sp.slice_qp_delta = static_cast<__s8>(ssh.slice_qp_delta);
    sp.slice_cb_qp_offset = static_cast<__s8>(ssh.slice_cb_qp_offset);
    sp.slice_cr_qp_offset = static_cast<__s8>(ssh.slice_cr_qp_offset);
    sp.slice_beta_offset_div2 = static_cast<__s8>(ssh.slice_beta_offset_div2);
    sp.slice_tc_offset_div2 = static_cast<__s8>(ssh.slice_tc_offset_div2);
    sp.slice_segment_addr = ssh.slice_segment_address;
    sp.short_term_ref_pic_set_size =
        static_cast<__u16>(ssh.short_term_ref_pic_set_bits);

    const bool is_p = ssh.slice_type == h265::SliceType::kP;
    const bool is_b = ssh.slice_type == h265::SliceType::kB;
    for (int k = 0; k < kMaxDpb; ++k) {
      sp.ref_idx_l0[k] = 0;
      sp.ref_idx_l1[k] = 0;
    }
    auto build_list = [&](const std::vector<std::uint8_t>& temp, bool modified,
                          const std::uint32_t* list_entry,
                          std::uint32_t active_minus1, __u8* out) {
      if (temp.empty()) return;
      for (std::uint32_t i = 0; i <= active_minus1 && i < kMaxDpb; ++i) {
        std::uint32_t idx = modified ? list_entry[i] : i % temp.size();
        idx %= temp.size();
        out[i] = temp[idx];
      }
    };
    if (is_p || is_b) {
      std::vector<std::uint8_t> t0;
      for (const auto& e : st_curr_before) t0.push_back(dpb_index(e));
      for (const auto& e : st_curr_after) t0.push_back(dpb_index(e));
      for (const auto& e : lt_curr) t0.push_back(dpb_index(e));
      build_list(t0, ssh.ref_pic_list_modification_flag_l0, ssh.list_entry_l0,
                 ssh.num_ref_idx_l0_active_minus1, sp.ref_idx_l0);
      if (is_b) {
        std::vector<std::uint8_t> t1;
        for (const auto& e : st_curr_after) t1.push_back(dpb_index(e));
        for (const auto& e : st_curr_before) t1.push_back(dpb_index(e));
        for (const auto& e : lt_curr) t1.push_back(dpb_index(e));
        build_list(t1, ssh.ref_pic_list_modification_flag_l1, ssh.list_entry_l1,
                   ssh.num_ref_idx_l1_active_minus1, sp.ref_idx_l1);
      }
    }

    // Weighted prediction: VA-style derivation reduced to the int8 fields.
    if (is_p || is_b) {
      const auto& w = ssh.pred_weight;
      auto& pw = sp.pred_weight_table;
      pw.luma_log2_weight_denom = static_cast<__u8>(w.luma_log2_weight_denom);
      pw.delta_chroma_log2_weight_denom =
          static_cast<__s8>(w.delta_chroma_log2_weight_denom);
      const int cdenom = static_cast<int>(w.luma_log2_weight_denom) +
                         w.delta_chroma_log2_weight_denom;
      const int half = h265::WpOffsetHalfRange(
          static_cast<int>(coded_bit_depth_),
          sps_.range_extension.high_precision_offsets_enabled_flag);
      auto fill = [&](int list, std::uint32_t active_m1, __s8* dlw, __s8* lo,
                      __s8(*dcw)[2], __s8(*co)[2]) {
        for (std::uint32_t i = 0; i <= active_m1 && i < kMaxDpb; ++i) {
          if (w.luma_weight_flag[list][i]) {
            dlw[i] = static_cast<__s8>(w.delta_luma_weight[list][i]);
            lo[i] = static_cast<__s8>(w.luma_offset[list][i]);
          }
          if (w.chroma_weight_flag[list][i]) {
            for (int j = 0; j < 2; ++j) {
              const int cw = (1 << cdenom) + w.delta_chroma_weight[list][i][j];
              dcw[i][j] = static_cast<__s8>(w.delta_chroma_weight[list][i][j]);
              co[i][j] = static_cast<__s8>(h265::DeriveChromaOffset(
                  half, w.delta_chroma_offset[list][i][j], cw, cdenom));
            }
          }
        }
      };
      fill(0, ssh.num_ref_idx_l0_active_minus1, pw.delta_luma_weight_l0,
           pw.luma_offset_l0, pw.delta_chroma_weight_l0, pw.chroma_offset_l0);
      if (is_b)
        fill(1, ssh.num_ref_idx_l1_active_minus1, pw.delta_luma_weight_l1,
             pw.luma_offset_l1, pw.delta_chroma_weight_l1, pw.chroma_offset_l1);
    }

    __u64 f = 0;
    if (ssh.slice_sao_luma_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;
    if (ssh.slice_sao_chroma_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;
    if (ssh.slice_temporal_mvp_enabled_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
    if (ssh.mvd_l1_zero_flag) f |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;
    if (ssh.cabac_init_flag) f |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;
    if (ssh.collocated_from_l0_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
    if (ssh.slice_deblocking_filter_disabled_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
    if (ssh.slice_loop_filter_across_slices_enabled_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;
    if (ssh.dependent_slice_segment_flag)
      f |= V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;
    sp.flags = f;
    sparams.push_back(sp);
  }

  v4l2_ctrl_hevc_sps ksps;
  v4l2_ctrl_hevc_pps kpps;
  v4l2_ctrl_hevc_scaling_matrix ksm;
  std::memset(&ksm, 16, sizeof(ksm));
  FillSps(sps_, &ksps);
  FillPps(pps_, &kpps);
  const h265::ScalingListData& sl = pps_.pps_scaling_list_data_present_flag
                                        ? pps_.scaling_list
                                        : sps_.scaling_list;
  if (sps_.scaling_list_enabled_flag) FillScalingMatrix(sl, &ksm);

  // Pick a free OUTPUT buffer.
  int obuf = -1;
  for (std::uint32_t i = 0; i < output_bufs_.size(); ++i) {
    if (!output_bufs_[i].start[0]) continue;
    obuf = static_cast<int>(i);
    break;
  }
  if (obuf < 0) return false;
  std::memcpy(output_bufs_[obuf].start[0], buf.data(),
              std::min(buf.size(), output_bufs_[obuf].length[0]));

  const std::uint64_t tag = next_tag_++;
  int req_fd = -1;
  if (Xioctl(media_fd_, MEDIA_IOC_REQUEST_ALLOC, &req_fd) < 0) return false;

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
  if (Xioctl(video_fd_, VIDIOC_S_EXT_CTRLS, &hc) < 0) {
    ::close(req_fd);
    return false;
  }

  {
    v4l2_plane pl{};
    pl.bytesused = static_cast<__u32>(buf.size());
    pl.length = static_cast<__u32>(output_bufs_[obuf].length[0]);
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = static_cast<__u32>(obuf);
    b.length = 1;
    b.m.planes = &pl;
    b.flags = V4L2_BUF_FLAG_REQUEST_FD;
    b.request_fd = req_fd;
    SetTag(&b, tag);
    if (Xioctl(video_fd_, VIDIOC_QBUF, &b) < 0) {
      ::close(req_fd);
      return false;
    }
  }
  if (Xioctl(req_fd, MEDIA_REQUEST_IOC_QUEUE, nullptr) < 0) {
    ::close(req_fd);
    return false;
  }

  pollfd pfd{video_fd_, POLLIN | POLLOUT, 0};
  poll(&pfd, 1, 2000);
  {
    v4l2_plane pl{};
    v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    b.memory = V4L2_MEMORY_MMAP;
    b.length = 1;
    b.m.planes = &pl;
    Xioctl(video_fd_, VIDIOC_DQBUF, &b);  // frees the OUTPUT buffer
  }
  v4l2_plane cpl[VIDEO_MAX_PLANES]{};
  v4l2_buffer cb{};
  cb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  cb.memory = V4L2_MEMORY_MMAP;
  cb.length = cap_planes_;
  cb.m.planes = cpl;
  const bool dq_ok = Xioctl(video_fd_, VIDIOC_DQBUF, &cb) == 0;
  ::close(req_fd);
  if (!dq_ok) return false;
  const int cap = static_cast<int>(cb.index);
  captures_[cap].queued = false;
  if (cb.flags & V4L2_BUF_FLAG_ERROR) {
    V4L2WC_LOG(V4L2WC_WARNING) << "v4l2-h265: decode error for POC " << poc;
    RequeueCapture(cap);
    return false;
  }

  // The decoded picture becomes a short-term reference and the newest output.
  captures_[cap].is_reference = true;
  captures_[cap].timestamp = timestamp;
  dpb_.push_back(RefPic{poc, cap, tag, false});
  const std::size_t max_dpb =
      static_cast<std::size_t>(sps_.sps_max_dec_pic_buffering_minus1) + 1;
  while (dpb_.size() > max_dpb) {
    std::size_t oldest = 0;
    for (std::size_t i = 1; i < dpb_.size(); ++i)
      if (dpb_[i].poc < dpb_[oldest].poc) oldest = i;
    if (dpb_[oldest].capture != cap)
      captures_[dpb_[oldest].capture].is_reference = false;
    dpb_.erase(dpb_.begin() + oldest);
  }

  // Mark the picture needed for output (unless suppressed), then run the
  // bumping process so display order comes out in increasing POC.
  captures_[cap].pending_output = sh.pic_output_flag;
  captures_[cap].poc = poc;
  Bump(/*flush=*/false);
  for (std::uint32_t i = 0; i < captures_.size(); ++i)
    MaybeRequeue(static_cast<int>(i));
  return true;
}

void V4l2StatelessH265Decoder::Bump(bool flush) {
  const std::size_t limit = flush ? 0 : reorder_depth_;
  while (true) {
    int min_poc = 0;
    int min_cap = -1;
    std::size_t pending = 0;
    for (std::uint32_t i = 0; i < captures_.size(); ++i) {
      if (!captures_[i].pending_output) continue;
      ++pending;
      if (min_cap < 0 || captures_[i].poc < min_poc) {
        min_poc = captures_[i].poc;
        min_cap = static_cast<int>(i);
      }
    }
    if (min_cap < 0 || pending <= limit) break;
    captures_[min_cap].pending_output = false;
    output_ready_.push_back(min_cap);  // pushed in increasing POC
  }
}

SubmitResult V4l2StatelessH265Decoder::SubmitBitstream(
    const std::uint8_t* data, std::size_t size, std::uint64_t timestamp) {
  auto nals = h265::ParseAnnexB(data, size);
  std::vector<std::vector<const h265::Nal*>> pics;
  for (auto& n : nals) {
    if (n.type == h265::NalUnitType::kSpsNut) {
      h265::Sps s{};
      if (h265::ParseSps(n.rbsp.data(), n.rbsp.size(), &s)) {
        if (configured_ && (s.pic_width_in_luma_samples != coded_w_ ||
                            s.pic_height_in_luma_samples != coded_h_)) {
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
    } else if (n.type == h265::NalUnitType::kEosNut ||
               n.type == h265::NalUnitType::kEobNut) {
      eos_seen_ = true;
    } else if (h265::IsVcl(n.type)) {
      const bool first = !n.rbsp.empty() && (n.rbsp[0] & 0x80);
      if (first || pics.empty()) pics.emplace_back();
      pics.back().push_back(&n);
    }
  }
  if (!have_sps_ || !have_pps_) return SubmitResult::kOk;
  if (!EnsureConfigured(sps_)) return SubmitResult::kError;
  for (auto& pic : pics) DecodePicture(pic, timestamp);
  return SubmitResult::kOk;
}

DriveResult V4l2StatelessH265Decoder::Drive() { return DriveResult::kOk; }

bool V4l2StatelessH265Decoder::ExportCapture(int index) {
  Capture& c = captures_[index];
  if (c.fd[0] >= 0) return true;  // export once, keep for the pool's lifetime
  for (std::uint32_t k = 0; k < cap_planes_; ++k) {
    v4l2_exportbuffer eb{};
    eb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    eb.index = static_cast<__u32>(index);
    eb.plane = k;
    eb.flags = O_RDONLY | O_CLOEXEC;
    if (Xioctl(video_fd_, VIDIOC_EXPBUF, &eb) < 0) return false;
    c.fd[k] = eb.fd;
    c.pitches[k] = cap_stride_;
    c.offsets[k] = 0;
  }
  return true;
}

bool V4l2StatelessH265Decoder::Acquire(V4l2DmaFrame* out) {
  if (output_ready_.empty()) return false;
  const int cap = output_ready_.front();  // smallest POC first
  output_ready_.erase(output_ready_.begin());
  if (!ExportCapture(cap)) {
    MaybeRequeue(cap);
    return false;
  }
  Capture& c = captures_[cap];
  c.checked_out = true;
  out->capture_index = static_cast<std::uint32_t>(cap);
  out->width = coded_w_;
  out->height = coded_h_;
  // Present a standard DRM format plus the SAND128 modifier (carrying the tile
  // column height) so a GL/KMS consumer can import and detile the dma-buf.
  out->drm_fourcc = coded_bit_depth_ == 10 ? kDrmFormatP030 : kDrmFormatNV12;
  out->modifier = Sand128ColHeight(luma_col_h_);
  out->num_planes = cap_planes_;
  for (int p = 0; p < V4l2DmaFrame::kMaxPlanes; ++p) {
    out->fds[p] = (p < static_cast<int>(cap_planes_)) ? c.fd[p] : -1;
    out->offsets[p] = c.offsets[p];
    out->pitches[p] = c.pitches[p];
  }
  out->timestamp = c.timestamp;
  return true;
}

void V4l2StatelessH265Decoder::Release(std::uint32_t capture_index) {
  if (capture_index >= captures_.size()) return;
  captures_[capture_index].checked_out = false;
  MaybeRequeue(static_cast<int>(capture_index));
}

void V4l2StatelessH265Decoder::Flush() {
  for (auto& r : dpb_) captures_[r.capture].is_reference = false;
  dpb_.clear();
  output_ready_.clear();
  for (auto& c : captures_) c.pending_output = false;
  prev_poc_lsb_ = 0;
  prev_poc_msb_ = 0;
  seen_first_picture_ = false;
  eos_seen_ = false;
  for (std::uint32_t i = 0; i < captures_.size(); ++i)
    if (!captures_[i].checked_out) RequeueCapture(static_cast<int>(i));
}

// End of sequence: flush every buffered picture to the output queue in POC
// order so Acquire can drain them.
void V4l2StatelessH265Decoder::Drain() { Bump(/*flush=*/true); }

}  // namespace v4l2wc
