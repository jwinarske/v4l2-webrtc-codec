// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// The device-agnostic decoder seam. Both the V4L2 M2M engine (V4l2M2mDecoder,
// the Pi path) and the VAAPI engine (VaapiH264Decoder, the AMD path) implement
// IDmaDecoder, so the webrtc VideoDecoder wrapper drives either one uniformly:
// feed an access unit, pump, acquire a decoded dma-buf frame, release it.
#ifndef V4L2WC_SRC_DMA_DECODER_H_
#define V4L2WC_SRC_DMA_DECODER_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace v4l2wc {

// A decoded frame exposed as borrowed DMA-BUF fds. The fds are owned by the
// engine; they stay valid until Release(capture_index). Planes are already
// mapped to the DRM layout (e.g. NV12 as two planes referencing one buffer).
struct V4l2DmaFrame {
  static constexpr int kMaxPlanes = 4;

  // Out-of-line so the default construction is not emitted inline in every
  // translation unit that builds a frame.
  V4l2DmaFrame();

  std::uint32_t capture_index = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t drm_fourcc = 0;
  std::uint64_t modifier = 0;
  std::uint32_t num_planes = 0;
  std::array<int, kMaxPlanes> fds{-1, -1, -1, -1};
  std::array<std::uint32_t, kMaxPlanes> offsets{0, 0, 0, 0};
  std::array<std::uint32_t, kMaxPlanes> pitches{0, 0, 0, 0};
  std::uint64_t rtp_timestamp = 0;  // opaque passthrough token (frame RTP ts)
};

// What a Drive() pass found. A resolution change is not an error: the stream
// is fine, the decoder is simply configured for the wrong geometry and has to
// be rebuilt, which the caller does. Reporting both as false left the caller
// unable to tell a recoverable reconfiguration from a dead device.
enum class DriveResult {
  kOk,            // nothing to report
  kSourceChange,  // mid-stream format change; recreate the decoder
  kError,         // fatal
};

enum class SubmitResult {
  kOk,            // queued
  kTryAgain,      // no free OUTPUT buffer; Drive() then retry
  kSourceChange,  // decoder needs reconfiguration; recreate the decoder
  kError,         // fatal
};

class IDmaDecoder {
 public:
  virtual ~IDmaDecoder() = default;

  // Feed one coded access unit.
  virtual SubmitResult SubmitBitstream(const std::uint8_t* data,
                                       std::size_t size,
                                       std::uint64_t rtp_timestamp) = 0;
  // Pump without blocking.
  virtual DriveResult Drive() = 0;
  // Hand out the newest ready frame as borrowed dma-buf fds; false if none.
  virtual bool Acquire(V4l2DmaFrame* out) = 0;
  // Return a previously acquired frame's capture slot for reuse.
  virtual void Release(std::uint32_t capture_index) = 0;

  // How many buffers the engine decodes into, or 0 before the pool exists.
  // Passed on to consumers so they can bound what they hold: a consumer that
  // holds a large fraction of the pool starves the decoder waiting for a
  // buffer, and the two engines' pools differ by an order of magnitude.
  virtual std::uint32_t PoolSize() const = 0;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_DMA_DECODER_H_
