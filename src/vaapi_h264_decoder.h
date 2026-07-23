// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// VAAPI H.264 decoder engine: the amdgpu/VAAPI sibling of V4l2M2mDecoder. It
// decodes constrained-baseline H.264 to NV12 on a VAAPI device and exports each
// decoded surface as a dma-buf, presenting the same V4l2DmaFrame + Acquire /
// Release contract the V4L2 engine does -- so the webrtc VideoDecoder layer and
// everything downstream are shared. libva is loaded at runtime (dlopen), so a
// single libwebrtc.so serves both the Pi (V4L2) and AMD (VAAPI) without a
// libva link dependency.
//
// NOTE: compiled only when absorbed in-tree into libwebrtc.so
// (lw_enable_v4l2_codec), alongside the V4L2 engine.
#ifndef V4L2WC_SRC_VAAPI_H264_DECODER_H_
#define V4L2WC_SRC_VAAPI_H264_DECODER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "parse/h264/nal.h"
#include "parse/h264/pps.h"
#include "parse/h264/slice_header.h"
#include "parse/h264/sps.h"
#include "src/dma_decoder.h"  // V4l2DmaFrame, SubmitResult, IDmaDecoder
#include "src/va_loader.h"

namespace v4l2wc {

class VaapiH264Decoder : public IDmaDecoder {
 public:
  // Opens `render_node` (e.g. /dev/dri/renderD128), dlopen's + initializes
  // libva. The H.264 VLD config, the NV12 surface pool (`pool_size` surfaces),
  // and the decode context are created lazily on the first SPS (which carries
  // the coded dimensions), mirroring V4L2's lazy CAPTURE setup. Returns nullptr
  // if libva is unavailable or the device has no H.264 VLD entrypoint.
  static std::unique_ptr<VaapiH264Decoder> Create(const char* render_node,
                                                  std::uint32_t pool_size);
  ~VaapiH264Decoder() override;

  VaapiH264Decoder(const VaapiH264Decoder&) = delete;
  VaapiH264Decoder& operator=(const VaapiH264Decoder&) = delete;

  // Decodes one Annex-B access unit (VAAPI is synchronous, so the frame is
  // ready on return). Parses NALs, updates SPS/PPS, builds the VA buffers,
  // drives the decode, manages the sliding-window DPB, and parks the decoded
  // frame (latest-wins). kSourceChange when the resolution changes mid-stream.
  SubmitResult SubmitBitstream(const std::uint8_t* data, std::size_t size,
                               std::uint64_t rtp_timestamp) override;

  // VAAPI decodes synchronously, so there is nothing to pump; always true.
  DriveResult Drive() override;

  // Hands out the parked decoded frame as borrowed dma-buf fds (exported via
  // vaExportSurfaceHandle). capture_index is the surface slot, returned via
  // Release. False when no frame is ready.
  bool Acquire(V4l2DmaFrame* out) override;

  // Returns a surface slot after the consumer is done: closes its exported
  // dma-buf fd and frees the surface for reuse (unless it is still a
  // reference).
  void Release(std::uint32_t slot) override;
  std::uint32_t PoolSize() const override { return pool_size_; }

 private:
  VaapiH264Decoder();
  bool EnsureConfigured(const h264::Sps& sps);
  bool DecodeSlice(const h264::Nal& nal);
  int ComputePoc(const h264::SliceHeader& sh, int ref_idc);
  int PickFreeSlot();
  void ExportSlot(std::uint32_t slot, std::uint64_t rtp);

  struct Slot {
    Slot();
    ~Slot();
    Slot(const Slot&);
    Slot& operator=(const Slot&);

    std::uint32_t surface = 0;  // VASurfaceID
    bool in_use = false;        // decoding / reference / checked-out
    bool is_reference = false;  // held by the DPB
    bool checked_out = false;   // Acquire'd, awaiting Release
    // exported dma-buf (valid while checked_out)
    int fd = -1;
    std::uint32_t drm_fourcc = 0, num_planes = 0;
    std::uint64_t modifier = 0;
    std::uint32_t offsets[4] = {0, 0, 0, 0};
    std::uint32_t pitches[4] = {0, 0, 0, 0};
    std::uint32_t width = 0, height = 0;
    std::uint64_t rtp = 0;
  };
  struct RefEntry {
    std::uint32_t slot;
    int frame_num;
    int pic_num;
    int poc;
  };

  va::VaApi va_{};
  int drm_fd_ = -1;
  void* dpy_ = nullptr;   // VADisplay
  unsigned config_ = 0;   // VAConfigID
  unsigned context_ = 0;  // VAContextID
  bool configured_ = false;
  std::uint32_t coded_w_ = 0, coded_h_ = 0;
  std::uint32_t pool_size_ = 0;

  h264::Sps sps_{};
  h264::Pps pps_{};
  bool have_sps_ = false, have_pps_ = false;

  std::vector<Slot> slots_;
  std::vector<RefEntry> dpb_;
  int prev_frame_num_ = 0, prev_frame_num_offset_ = 0;

  bool have_ready_ = false;
  std::uint32_t ready_slot_ = 0;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_VAAPI_H264_DECODER_H_
