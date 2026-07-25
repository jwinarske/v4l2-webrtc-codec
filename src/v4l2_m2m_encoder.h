// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// V4L2 M2M stateful H.264 encoder -- the inverse of V4l2M2mDecoder. OUTPUT
// takes raw NV12 (imported zero-copy as a dma-buf when the producer exports
// one, else copied into an MMAP buffer); CAPTURE produces the H.264 access
// unit, read out on the CPU for RTP. Targets bcm2835-codec's encode node
// (/dev/video11 on the Pi) and similar stateful M2M encoders.
//
// No libdrm dependency: the fourcc/modifier constants it needs are defined
// locally, matching the decoder engine.
#ifndef V4L2WC_SRC_V4L2_M2M_ENCODER_H_
#define V4L2WC_SRC_V4L2_M2M_ENCODER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace v4l2wc {

class V4l2M2mEncoder {
 public:
  // Open `device`, negotiate OUTPUT = NV12(width x height) and CAPTURE = H.264,
  // program bitrate/framerate/gop, and stream both queues on. Returns nullptr
  // on any failure (no device, unsupported format, ioctl error).
  static std::unique_ptr<V4l2M2mEncoder> Create(const char* device,
                                                uint32_t width, uint32_t height,
                                                uint32_t bitrate_bps,
                                                uint32_t framerate,
                                                uint32_t gop);
  ~V4l2M2mEncoder();

  V4l2M2mEncoder(const V4l2M2mEncoder&) = delete;
  V4l2M2mEncoder& operator=(const V4l2M2mEncoder&) = delete;

  // Encode one NV12 frame delivered as dma-buf planes, imported zero-copy into
  // the OUTPUT queue (V4L2_MEMORY_DMABUF). `fds`/`offsets`/`strides` describe
  // `num_planes` planes sharing the buffer (NV12: two planes, offsets 0 and the
  // luma size, one fd). On success appends the coded access unit to `out` and
  // sets `keyframe`. Synchronous: the input dma-buf is consumed (DQBUF'd)
  // before returning, so the caller may drop the frame afterward.
  bool EncodeDmabuf(const int* fds, const uint32_t* offsets,
                    const uint32_t* strides, uint32_t num_planes,
                    uint64_t timestamp, bool force_keyframe,
                    std::vector<uint8_t>* out, bool* keyframe);

  // CPU fallback: copy an NV12 buffer into an MMAP OUTPUT buffer and encode.
  // `size` must be width*height*3/2.
  bool EncodeCpu(const uint8_t* nv12, size_t size, uint64_t timestamp,
                 bool force_keyframe, std::vector<uint8_t>* out,
                 bool* keyframe);

  void SetBitrate(uint32_t bitrate_bps);
  void SetFramerate(uint32_t framerate);

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

 private:
  // Out-of-line (defined in the .cc) so the vector members' construction is not
  // emitted inline in every TU, matching the repo's chromium-style convention.
  V4l2M2mEncoder();

  // Shared tail of both Encode paths: pump the queues, read out one coded
  // buffer, reclaim the consumed OUTPUT buffer.
  bool DriveAndCollect(std::vector<uint8_t>* out, bool* keyframe);
  bool SetCtrl(uint32_t id, int32_t value);

  int fd_ = -1;
  bool mplane_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;

  // OUTPUT (raw NV12) queue. Two memory models: MMAP buffers for the CPU path,
  // DMABUF import for the zero-copy path. Only one is populated per Create.
  struct OutputBuffer {
    void* mmap_addr = nullptr;  // CPU path only
    size_t length = 0;
    bool queued = false;
  };
  std::vector<OutputBuffer> output_buffers_;
  bool output_dmabuf_ = false;  // true => OUTPUT imports dma-bufs, no mmap

  // CAPTURE (H.264) queue: MMAP, read coded bytes out via DQBUF bytesused.
  struct CaptureBuffer {
    void* mmap_addr = nullptr;
    size_t length = 0;
  };
  std::vector<CaptureBuffer> capture_buffers_;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_V4L2_M2M_ENCODER_H_
