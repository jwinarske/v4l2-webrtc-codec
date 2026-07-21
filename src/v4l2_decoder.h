// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Stateful hardware decoder: a webrtc::VideoDecoder that reuses drm-cxx's
// V4l2DecoderSource (stateful V4L2 M2M decode + DMA-BUF export) and hands each
// decoded NV12 frame to webrtc as an LwNativeVideoFrameBuffer carrying an
// LwDmabufDescriptor. The KMS/wl import happens later, in the presenter (the
// lw_video_sink_v1 consumer) -- this decoder only produces borrowed dmabuf fds.
//
// NOTE: this is compiled only when absorbed in-tree into libwebrtc.so
// (lw_enable_v4l2_codec), where the webrtc, drm-cxx, and fork headers resolve.
// It has NOT been built or run yet; it is written against the confirmed
// webrtc m144 / drm-cxx / lw_video_sink.h APIs and is pending verification on
// target hardware (bcm2835 on the Raspberry Pi validated the raw V4L2 path).
#ifndef V4L2WC_SRC_V4L2_DECODER_H_
#define V4L2WC_SRC_V4L2_DECODER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_decoder.h"
#include "api/video_codecs/video_decoder_factory.h"
#include "v4l2wc/v4l2wc.h"

namespace drm {
class Device;
namespace scene {
class V4l2DecoderSource;
}  // namespace scene
}  // namespace drm

namespace v4l2wc {

// Serialized access to the V4L2 decoder source. A decoded frame's release
// callback can fire on the presenter thread while Decode() runs on the webrtc
// decoder thread, so every source call goes through this holder's mutex, and
// in-flight frames keep it alive (shared_ptr) past decoder Release().
class SourceHolder {
 public:
  explicit SourceHolder(std::unique_ptr<drm::scene::V4l2DecoderSource> src,
                        std::unique_ptr<drm::Device> dev);
  ~SourceHolder();

  std::mutex& mutex() { return m_; }
  drm::scene::V4l2DecoderSource* src() { return src_.get(); }

 private:
  std::mutex m_;
  std::unique_ptr<drm::scene::V4l2DecoderSource> src_;
  std::unique_ptr<drm::Device>
      dev_;  // borrowed by src_ for its internal import
};

class V4l2Decoder : public webrtc::VideoDecoder {
 public:
  explicit V4l2Decoder(V4l2WcConfig config);
  ~V4l2Decoder() override;

  bool Configure(const Settings& settings) override;
  int32_t Decode(const webrtc::EncodedImage& input_image, bool missing_frames,
                 int64_t render_time_ms) override;
  int32_t RegisterDecodeCompleteCallback(
      webrtc::DecodedImageCallback* callback) override;
  int32_t Release() override;

 private:
  // Lazily create the source once the first SPS gives us coded dimensions.
  bool EnsureSource(const uint8_t* data, size_t size);
  // Drain any ready CAPTURE buffers and deliver them to `callback_`.
  void DeliverReadyFrames(int64_t render_time_ms, uint32_t rtp_timestamp);

  V4l2WcConfig config_;
  webrtc::DecodedImageCallback* callback_ = nullptr;
  std::shared_ptr<SourceHolder> holder_;  // shared with in-flight frames
  uint32_t pool_generation_ = 0;
  uint64_t frame_seq_ = 0;
};

class V4l2DecoderFactory : public webrtc::VideoDecoderFactory {
 public:
  explicit V4l2DecoderFactory(V4l2WcConfig config);

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
  std::unique_ptr<webrtc::VideoDecoder> Create(
      const webrtc::Environment& env,
      const webrtc::SdpVideoFormat& format) override;

 private:
  V4l2WcConfig config_;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_V4L2_DECODER_H_
