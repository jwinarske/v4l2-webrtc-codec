// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Self-contained stateful V4L2 M2M video decoder.
//
// Drives a stateful decoder (bcm2835-codec, Venus, and similar) end to end
// using only the V4L2 UAPI: it feeds a coded bitstream on the OUTPUT queue,
// handles the V4L2_EVENT_SOURCE_CHANGE reconfiguration that establishes the
// CAPTURE format, and exports every decoded CAPTURE buffer as a DMA-BUF fd via
// VIDIOC_EXPBUF. It deliberately does NO DRM/KMS work -- there is no drm fd, no
// DRM master, no framebuffer import. Decoded frames leave as borrowed dmabuf
// fds for a downstream presenter to import however it likes (KMS or EGL).
//
// This replaces the scanout-oriented drm-cxx V4l2DecoderSource, whose KMS
// coupling and DRM-master handling are irrelevant to a decode-to-dmabuf
// pipeline.

#ifndef V4L2WC_SRC_V4L2_M2M_DECODER_H_
#define V4L2WC_SRC_V4L2_M2M_DECODER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "src/dma_decoder.h"  // V4l2DmaFrame, SubmitResult, IDmaDecoder

namespace v4l2wc {

class V4l2M2mDecoder : public IDmaDecoder {
 public:
  // Opens `device`, negotiates `codec_fourcc` on OUTPUT and `capture_fourcc`
  // on CAPTURE, allocates + mmaps OUTPUT buffers, subscribes SOURCE_CHANGE, and
  // streams OUTPUT on. CAPTURE is set up lazily on the first SOURCE_CHANGE.
  // Returns nullptr on failure.
  static std::unique_ptr<V4l2M2mDecoder> Create(
      const char* device, std::uint32_t codec_fourcc,
      std::uint32_t capture_fourcc, std::uint32_t coded_width,
      std::uint32_t coded_height, std::uint32_t output_buffer_count,
      std::uint32_t capture_buffer_count, std::size_t output_buffer_size);

  ~V4l2M2mDecoder() override;

  V4l2M2mDecoder(const V4l2M2mDecoder&) = delete;
  V4l2M2mDecoder& operator=(const V4l2M2mDecoder&) = delete;

  // Copies a coded access unit into a free OUTPUT buffer and queues it.
  SubmitResult SubmitBitstream(const std::uint8_t* data, std::size_t size,
                               std::uint64_t rtp_timestamp) override;

  // Pumps events and dequeues completed buffers without blocking: reclaims
  // OUTPUT buffers, sets up CAPTURE on the first SOURCE_CHANGE, and parks the
  // newest decoded CAPTURE buffer (latest-wins). Returns false on a fatal
  // error or a mid-stream SOURCE_CHANGE (caller recreates the decoder).
  bool Drive() override;

  // Returns the newest ready CAPTURE frame, if any, transferring the caller a
  // borrowed reference that must be returned with Release(). Returns false when
  // nothing is ready.
  bool Acquire(V4l2DmaFrame* out) override;

  // Re-queues a previously acquired CAPTURE buffer for reuse.
  void Release(std::uint32_t capture_index) override;
  // The pool exists only after the CAPTURE queue is set up, so this reports
  // what was actually allocated rather than what was asked for.
  std::uint32_t PoolSize() const override {
    return static_cast<std::uint32_t>(capture_buffers_.size());
  }

 private:
  V4l2M2mDecoder();

  struct OutputBuffer {
    void* ptr = nullptr;
    std::size_t length = 0;
  };
  struct CaptureBuffer {
    int dmabuf_fd = -1;        // exported once, owned, reused
    std::uint32_t length = 0;  // plane 0 length
    bool queued = false;
  };

  bool SetupCapture();  // on SOURCE_CHANGE
  void TeardownCapture();

  int fd_ = -1;
  bool mplane_ = true;
  std::uint32_t codec_fourcc_ = 0;
  std::uint32_t capture_fourcc_ = 0;

  std::vector<OutputBuffer> output_buffers_;
  std::vector<std::uint32_t> output_free_;
  bool output_streaming_ = false;

  std::vector<CaptureBuffer> capture_buffers_;
  bool capture_streaming_ = false;
  std::uint32_t capture_count_req_ = 4;
  std::uint32_t cap_width_ = 0;
  std::uint32_t cap_height_ = 0;
  std::uint32_t cap_stride_ = 0;
  std::uint32_t cap_uv_offset_ = 0;  // byte offset of the UV plane
  std::uint64_t cap_modifier_ = 0;   // DRM_FORMAT_MOD_* (0 = LINEAR, else SAND)

  bool have_ready_ = false;
  std::uint32_t ready_index_ = 0;
  std::uint64_t ready_rtp_ = 0;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_V4L2_M2M_DECODER_H_
