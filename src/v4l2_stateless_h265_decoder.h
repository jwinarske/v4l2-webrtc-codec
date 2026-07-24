// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// V4L2 stateless HEVC / H.265 decoder engine: the Raspberry Pi sibling of
// VaapiH265Decoder. It drives a V4L2 stateless decoder (e.g. rpi-hevc-dec on
// the Pi 4 / Pi 5) through the request API, reusing the webrtc-free parse/h265
// layer, and presents the same V4l2DmaFrame + Acquire / Release contract every
// other engine does -- so everything downstream is shared.
//
// It parses each access unit, derives POC (clause 8.3.1), the reference-picture
// set (8.3.2) and the reference lists (8.3.4), marshals the SPS / PPS / slice /
// decode parameters into the v4l2_ctrl_hevc_* controls, and submits one request
// per picture. Reference pictures are addressed by the timestamp carried on the
// OUTPUT buffer they were decoded from; a CAPTURE buffer holding a reference is
// kept out of the free pool until the DPB evicts it.
//
// Intra and inter (P / B) HEVC Main 4:2:0 is supported. Long-term references,
// reference-list modification, tiles and scaling lists are marshalled the same
// way the VAAPI engine marshals them.
#ifndef V4L2WC_SRC_V4L2_STATELESS_H265_DECODER_H_
#define V4L2WC_SRC_V4L2_STATELESS_H265_DECODER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "parse/h265/nal.h"
#include "parse/h265/pps.h"
#include "parse/h265/slice_header.h"
#include "parse/h265/sps.h"
#include "src/dma_decoder.h"

namespace v4l2wc {

class V4l2StatelessH265Decoder : public IDmaDecoder {
 public:
  // Opens `video_node` (e.g. /dev/video19) and finds its request-API media
  // device by driver name. The OUTPUT (coded) and CAPTURE (decoded) queues and
  // the buffer pools are created lazily on the first SPS. Returns nullptr if
  // the device is not a stateless HEVC decoder.
  static std::unique_ptr<V4l2StatelessH265Decoder> Create(
      const char* video_node, std::uint32_t pool_size);
  ~V4l2StatelessH265Decoder() override;

  V4l2StatelessH265Decoder(const V4l2StatelessH265Decoder&) = delete;
  V4l2StatelessH265Decoder& operator=(const V4l2StatelessH265Decoder&) = delete;

  SubmitResult SubmitBitstream(const std::uint8_t* data, std::size_t size,
                               std::uint64_t timestamp) override;
  DriveResult Drive() override;
  bool Acquire(V4l2DmaFrame* out) override;
  void Release(std::uint32_t capture_index) override;
  void Flush() override;
  void Drain() override;
  std::uint32_t PoolSize() const override { return pool_size_; }

 private:
  V4l2StatelessH265Decoder();
  bool EnsureConfigured(const h265::Sps& sps);
  bool DecodePicture(const std::vector<const h265::Nal*>& slices,
                     std::uint64_t timestamp);
  int ComputePoc(const h265::SliceHeader& sh, const h265::Nal& nal,
                 bool no_rasl_output);
  void RequeueCapture(int index);
  void MaybeRequeue(int index);
  bool ExportCapture(int index);
  // Output-order "bumping" (clause C.5.2.2): move pictures marked needed for
  // output to the ready queue in increasing POC while more than the reorder
  // depth are held back; `flush` empties it (drain / end of sequence).
  void Bump(bool flush);

  // An mmap'd multi-planar V4L2 buffer.
  struct MappedBuf {
    void* start[V4l2DmaFrame::kMaxPlanes] = {nullptr, nullptr, nullptr,
                                             nullptr};
    std::size_t length[V4l2DmaFrame::kMaxPlanes] = {0, 0, 0, 0};
    std::uint32_t n = 0;
  };

  // A CAPTURE buffer and its decode / reference / output state.
  struct Capture {
    MappedBuf map;
    bool queued = false;          // handed to the driver, awaiting decode
    bool is_reference = false;    // held by the DPB
    bool checked_out = false;     // Acquire'd, awaiting Release
    bool pending_output = false;  // decoded, not yet handed out (needs output)
    int poc = 0;                  // output order key
    std::uint64_t timestamp = 0;  // passthrough presentation timestamp
    // Exported dma-buf, one fd per plane, valid once ExportCapture ran.
    int fd[V4l2DmaFrame::kMaxPlanes] = {-1, -1, -1, -1};
    std::uint32_t offsets[V4l2DmaFrame::kMaxPlanes] = {0, 0, 0, 0};
    std::uint32_t pitches[V4l2DmaFrame::kMaxPlanes] = {0, 0, 0, 0};
  };

  // A decoded picture the DPB still holds as a reference.
  struct RefPic {
    int poc;
    int capture;              // index into captures_
    std::uint64_t timestamp;  // matches the OUTPUT buffer it was decoded from
    bool long_term = false;
  };

  int video_fd_ = -1;
  int media_fd_ = -1;
  bool configured_ = false;
  std::uint32_t coded_w_ = 0, coded_h_ = 0, coded_bit_depth_ = 8;
  std::uint32_t pool_size_ = 0;

  std::uint32_t cap_fourcc_ = 0, cap_stride_ = 0;
  std::uint32_t cap_planes_ = 0;
  std::uint32_t luma_col_h_ = 0, chroma_col_h_ = 0;

  h265::Sps sps_{};
  h265::Pps pps_{};
  bool have_sps_ = false, have_pps_ = false;

  std::vector<MappedBuf> output_bufs_;  // coded-input OUTPUT queue
  std::vector<Capture> captures_;       // decoded CAPTURE queue
  std::vector<RefPic> dpb_;
  // Capture indices ready to hand out, kept in increasing POC (front smallest).
  std::vector<int> output_ready_;
  std::uint32_t reorder_depth_ = 0;  // sps_max_num_reorder_pics

  // POC derivation state (clause 8.3.1).
  int prev_poc_lsb_ = 0, prev_poc_msb_ = 0;
  bool seen_first_picture_ = false;
  bool eos_seen_ = false;

  // A monotonically increasing tag put on each OUTPUT buffer so references
  // resolve by timestamp regardless of the passthrough presentation timestamp.
  std::uint64_t next_tag_ = 1;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_V4L2_STATELESS_H265_DECODER_H_
