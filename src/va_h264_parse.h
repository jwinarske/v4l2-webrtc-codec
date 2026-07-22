// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// VA-oriented H.264 parsing plus the dlopen'd libva entry table, used by the
// VAAPI decode engine.
//
// This is deliberately separate from the stateless path's parse/h264/ layer:
// VAAPI needs VASliceParameterBufferH264.slice_data_bit_offset, a bit offset
// into the RAW NAL (emulation-prevention bytes intact), whereas parse/h264/
// consumes emulation-stripped RBSP and instead carries the reference-list and
// MMCO detail the stateless path needs. Converging the two requires teaching
// parse/h264/ raw-bitstream positions; until then both are fuzzed (see
// test/fuzz/).
#ifndef V4L2WC_SRC_VA_H264_PARSE_H_
#define V4L2WC_SRC_VA_H264_PARSE_H_

#include <stdint.h>
#include <va/va.h>

#include <vector>

namespace v4l2wc::va {

// Largest coded dimension accepted from an SPS, mirroring the stateless
// parser's bound (parse/h264/sps.h). Dimensions are multiplied out and handed
// to the driver as ints, so they are rejected here rather than overflowing.
inline constexpr uint32_t kMaxDimension = 16384;

// dlopen'd libva entry table.
struct VaApi {
  VaApi();
  ~VaApi();

  void* lib = nullptr;
  void* lib_drm = nullptr;
  VADisplay (*GetDisplayDRM)(int) = nullptr;
  VAStatus (*Initialize)(VADisplay, int*, int*) = nullptr;
  VAStatus (*Terminate)(VADisplay) = nullptr;
  const char* (*ErrorStr)(VAStatus) = nullptr;
  VAStatus (*CreateConfig)(VADisplay, VAProfile, VAEntrypoint, VAConfigAttrib*,
                           int, VAConfigID*) = nullptr;
  VAStatus (*CreateSurfaces)(VADisplay, unsigned, unsigned, unsigned,
                             VASurfaceID*, unsigned, VASurfaceAttrib*,
                             unsigned) = nullptr;
  VAStatus (*CreateContext)(VADisplay, VAConfigID, int, int, int, VASurfaceID*,
                            int, VAContextID*) = nullptr;
  VAStatus (*CreateBuffer)(VADisplay, VAContextID, VABufferType, unsigned,
                           unsigned, void*, VABufferID*) = nullptr;
  VAStatus (*BeginPicture)(VADisplay, VAContextID, VASurfaceID) = nullptr;
  VAStatus (*RenderPicture)(VADisplay, VAContextID, VABufferID*, int) = nullptr;
  VAStatus (*EndPicture)(VADisplay, VAContextID) = nullptr;
  VAStatus (*SyncSurface)(VADisplay, VASurfaceID) = nullptr;
  VAStatus (*DestroyBuffer)(VADisplay, VABufferID) = nullptr;
  VAStatus (*DestroySurfaces)(VADisplay, VASurfaceID*, int) = nullptr;
  VAStatus (*DestroyContext)(VADisplay, VAContextID) = nullptr;
  VAStatus (*DestroyConfig)(VADisplay, VAConfigID) = nullptr;
  VAStatus (*DeriveImage)(VADisplay, VASurfaceID, VAImage*) = nullptr;
  VAStatus (*MapBuffer)(VADisplay, VABufferID, void**) = nullptr;
  VAStatus (*UnmapBuffer)(VADisplay, VABufferID) = nullptr;
  VAStatus (*DestroyImage)(VADisplay, VAImageID) = nullptr;
  VAStatus (*ExportSurfaceHandle)(VADisplay, VASurfaceID, uint32_t, uint32_t,
                                  void*) = nullptr;
};
bool VaLoad(VaApi* va);  // dlopen + dlsym; false on failure

// ---- minimal H.264 parse (VA-oriented; enough for constrained baseline) ----

struct Sps {
  Sps();
  ~Sps();

  uint32_t sps_id = 0;
  uint32_t profile_idc = 0, level_idc = 0;
  uint32_t chroma_format_idc = 1;
  uint32_t log2_max_frame_num = 4;  // minus4 + 4
  uint32_t pic_order_cnt_type = 0;
  uint32_t log2_max_poc_lsb = 4;  // minus4 + 4
  bool delta_pic_order_always_zero = false;
  uint32_t max_num_ref_frames = 0;
  uint32_t pic_width_in_mbs = 0;
  uint32_t pic_height_in_map_units = 0;
  bool frame_mbs_only = true;
  bool mb_adaptive_frame_field = false;
  bool direct_8x8_inference = false;
};

struct Pps {
  Pps();
  ~Pps();

  uint32_t pps_id = 0, sps_id = 0;
  bool entropy_coding_mode = false;
  bool bottom_field_pic_order_present = false;
  uint32_t num_slice_groups = 1;
  uint32_t num_ref_idx_l0_default = 1;  // active_minus1 + 1
  uint32_t num_ref_idx_l1_default = 1;
  bool weighted_pred = false;
  uint32_t weighted_bipred_idc = 0;
  int32_t pic_init_qp = 26;  // minus26 + 26
  int32_t chroma_qp_index_offset = 0;
  bool deblocking_filter_control_present = false;
  bool constrained_intra_pred = false;
  bool redundant_pic_cnt_present = false;
  bool transform_8x8_mode = false;
  int32_t second_chroma_qp_index_offset = 0;
};

struct SliceHdr {
  SliceHdr();
  ~SliceHdr();

  uint32_t first_mb = 0;
  uint32_t slice_type = 0;  // %5: 0=P 1=B 2=I ...
  uint32_t pps_id = 0;
  uint32_t frame_num = 0;
  bool idr = false;
  uint32_t idr_pic_id = 0;
  uint32_t poc_lsb = 0;
  uint32_t num_ref_idx_l0_active = 1;
  int32_t slice_qp_delta = 0;
  // Bit offset, in the RAW NAL bitstream, of the first macroblock (i.e. the
  // slice-header length). This is
  // VASliceParameterBufferH264.slice_data_bit_offset.
  uint32_t data_bit_offset = 0;
};

// One NAL as it appears in the stream: raw bytes (with the NAL header, no start
// code, emulation-prevention intact) + the emulation-stripped RBSP for parsing.
struct Nal {
  Nal();
  ~Nal();
  Nal(const Nal&);
  Nal& operator=(const Nal&);
  Nal(Nal&&) noexcept;
  Nal& operator=(Nal&&) noexcept;

  uint32_t type = 0;
  uint32_t ref_idc = 0;
  std::vector<uint8_t> raw;   // NAL header + payload, emulation bytes intact
  std::vector<uint8_t> rbsp;  // payload only, emulation stripped
};

std::vector<Nal> SplitAnnexB(const uint8_t* data, size_t size);

bool ParseSps(const uint8_t* rbsp, size_t n, Sps* out);
bool ParsePps(const uint8_t* rbsp, size_t n, const Sps* sps, Pps* out);
// Parses the slice header from the RAW NAL (header stripped by caller: pass the
// NAL payload raw bytes + emulation-stripped rbsp). Fills data_bit_offset in
// raw-bitstream space.
bool ParseSliceHdr(const Nal& nal, const Sps& sps, const Pps& pps,
                   SliceHdr* out);

}  // namespace v4l2wc::va

#endif  // V4L2WC_SRC_VA_H264_PARSE_H_
