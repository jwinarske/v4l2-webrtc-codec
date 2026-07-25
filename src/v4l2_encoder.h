// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Stateful hardware encoder: a webrtc::VideoEncoder that drives a V4L2 M2M
// encoder (V4l2M2mEncoder) -- the inverse of V4l2Decoder. It takes each raw
// frame webrtc hands it (a zero-copy NV12 dma-buf from a GPU producer, wrapped
// as an LwNativeVideoFrameBuffer, or a CPU buffer it converts) and produces an
// H.264 access unit, which it returns through the EncodedImageCallback. Rate
// control, pacing, and RTP stay in libwebrtc; this only turns frames into NALs.
//
// NOTE: compiled only when absorbed in-tree into libwebrtc.so
// (lw_enable_v4l2_codec), where the webrtc and fork headers resolve.
#ifndef V4L2WC_SRC_V4L2_ENCODER_H_
#define V4L2WC_SRC_V4L2_ENCODER_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "api/video_codecs/video_encoder_factory.h"
#include "src/v4l2_m2m_encoder.h"
#include "v4l2wc/v4l2wc.h"

namespace v4l2wc {

class V4l2Encoder : public webrtc::VideoEncoder {
 public:
  explicit V4l2Encoder(V4l2WcEncoderConfig config);
  ~V4l2Encoder() override;

  int InitEncode(const webrtc::VideoCodec* codec_settings,
                 const webrtc::VideoEncoder::Settings& settings) override;
  int32_t RegisterEncodeCompleteCallback(
      webrtc::EncodedImageCallback* callback) override;
  int32_t Release() override;
  int32_t Encode(
      const webrtc::VideoFrame& frame,
      const std::vector<webrtc::VideoFrameType>* frame_types) override;
  void SetRates(const RateControlParameters& parameters) override;
  webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

 private:
  // Hand one coded access unit back to webrtc, tagged key/delta and stamped
  // with the source frame's timestamps so RTP and the receiver line up.
  int32_t Deliver(const std::vector<uint8_t>& au, bool keyframe,
                  const webrtc::VideoFrame& frame);

  V4l2WcEncoderConfig config_;
  webrtc::EncodedImageCallback* callback_ = nullptr;
  std::unique_ptr<V4l2M2mEncoder> engine_;  // created in InitEncode
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t bitrate_bps_ = 0;
  uint32_t framerate_ = 30;
  // Reused scratch so a steady stream allocates nothing per frame: `au_` holds
  // the coded output, `nv12_` the I420->NV12 conversion for the CPU fallback.
  std::vector<uint8_t> au_;
  std::vector<uint8_t> nv12_;
};

class V4l2EncoderFactory : public webrtc::VideoEncoderFactory {
 public:
  explicit V4l2EncoderFactory(V4l2WcEncoderConfig config);

  std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
  std::unique_ptr<webrtc::VideoEncoder> Create(
      const webrtc::Environment& env,
      const webrtc::SdpVideoFormat& format) override;

 private:
  V4l2WcEncoderConfig config_;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_V4L2_ENCODER_H_
