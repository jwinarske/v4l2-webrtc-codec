// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_encoder.h"

#include <cstdlib>
#include <cstring>

#include "absl/strings/match.h"
#include "api/units/time_delta.h"
#include "api/video/encoded_image.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "api/video/video_timing.h"
#include "api/video_codecs/video_codec.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "src/log.h"

// The fork's native dma-buf buffer + descriptor ABI (resolved via the
// //libwebrtc include dirs in BUILD.gn, exactly as the decoder does).
#include "c/lw_video_sink.h"
#include "src/internal/lw_native_video_frame_buffer.h"

namespace v4l2wc {
namespace {

// The Pi encode node; the decoder's /dev/video10 inverse.
constexpr char kDefaultEncodeDevice[] = "/dev/video11";

bool WantsKeyframe(const std::vector<webrtc::VideoFrameType>* frame_types) {
  if (frame_types == nullptr) {
    return false;
  }
  for (const auto t : *frame_types) {
    if (t == webrtc::VideoFrameType::kVideoFrameKey) {
      return true;
    }
  }
  return false;
}

}  // namespace

V4l2Encoder::V4l2Encoder(V4l2WcEncoderConfig config) : config_(config) {}
V4l2Encoder::~V4l2Encoder() { Release(); }

int V4l2Encoder::InitEncode(
    const webrtc::VideoCodec* codec_settings,
    const webrtc::VideoEncoder::Settings& /*settings*/) {
  if (codec_settings == nullptr || codec_settings->width == 0 ||
      codec_settings->height == 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  width_ = codec_settings->width;
  height_ = codec_settings->height;
  framerate_ =
      codec_settings->maxFramerate != 0 ? codec_settings->maxFramerate : 30;
  bitrate_bps_ =
      codec_settings->startBitrate != 0
          ? codec_settings->startBitrate * 1000
          : (config_.default_bitrate_bps != 0 ? config_.default_bitrate_bps
                                              : 2'000'000);
  // IVI_ENC_BITRATE=<bps> overrides the initial bitrate and, in SetRates below,
  // acts as a floor webrtc's rate control cannot drop under. Useful when the
  // negotiated rate is too low for the content -- e.g. smooth gradients band at
  // the ~2 Mbps default for 1920x720.
  if (const char* br = std::getenv("IVI_ENC_BITRATE")) {
    const long v = std::atol(br);
    if (v > 0) {
      bitrate_bps_ = static_cast<uint32_t>(v);
    }
  }
  const uint32_t gop = framerate_ * 2;  // ~2s GOP; SetRates can request sooner.
  const char* device =
      (config_.video_device != nullptr && config_.video_device[0] != '\0')
          ? config_.video_device
          : kDefaultEncodeDevice;

  engine_ = V4l2M2mEncoder::Create(device, width_, height_, bitrate_bps_,
                                   framerate_, gop);
  if (!engine_) {
    V4L2WC_LOG(V4L2WC_ERROR) << "V4l2Encoder: failed to open encoder " << device
                             << " (" << width_ << "x" << height_ << ")";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4l2Encoder::RegisterEncodeCompleteCallback(
    webrtc::EncodedImageCallback* callback) {
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4l2Encoder::Release() {
  engine_.reset();
  callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4l2Encoder::Encode(
    const webrtc::VideoFrame& frame,
    const std::vector<webrtc::VideoFrameType>* frame_types) {
  if (!engine_ || callback_ == nullptr) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  const bool force_key = WantsKeyframe(frame_types);
  const uint64_t ts = frame.rtp_timestamp();

  au_.clear();
  bool keyframe = false;
  bool ok = false;

  webrtc::scoped_refptr<webrtc::VideoFrameBuffer> buffer =
      frame.video_frame_buffer();
  if (buffer->type() == webrtc::VideoFrameBuffer::Type::kNative) {
    // Zero-copy: the producer exported a dma-buf; encode it in place.
    auto* native =
        static_cast<libwebrtc::LwNativeVideoFrameBuffer*>(buffer.get());
    const LwDmabufDescriptor& d = native->descriptor();
    int fds[LW_MAX_PLANES];
    uint32_t offsets[LW_MAX_PLANES];
    uint32_t strides[LW_MAX_PLANES];
    const uint32_t n =
        d.num_planes <= LW_MAX_PLANES ? d.num_planes : LW_MAX_PLANES;
    for (uint32_t p = 0; p < n; ++p) {
      fds[p] = d.planes[p].fd;
      offsets[p] = d.planes[p].offset;
      strides[p] = d.planes[p].pitch;
    }
    ok = engine_->EncodeDmabuf(fds, offsets, strides, n, ts, force_key, &au_,
                               &keyframe);
  } else {
    // CPU fallback: convert I420 -> NV12 into a scratch, then encode. The GPU
    // producer path above is the common case; this covers a software source.
    webrtc::scoped_refptr<webrtc::I420BufferInterface> i420 = buffer->ToI420();
    if (!i420) {
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    const int w = i420->width();
    const int h = i420->height();
    const size_t nv12_size = static_cast<size_t>(w) * h * 3 / 2;
    nv12_.resize(nv12_size);
    uint8_t* dst = nv12_.data();
    // Y plane, row by row (source stride may exceed width).
    for (int row = 0; row < h; ++row) {
      std::memcpy(dst + static_cast<size_t>(row) * w,
                  i420->DataY() + static_cast<size_t>(row) * i420->StrideY(),
                  w);
    }
    // Interleaved CbCr at half resolution.
    uint8_t* uv = dst + static_cast<size_t>(w) * h;
    for (int row = 0; row < h / 2; ++row) {
      const uint8_t* su =
          i420->DataU() + static_cast<size_t>(row) * i420->StrideU();
      const uint8_t* sv =
          i420->DataV() + static_cast<size_t>(row) * i420->StrideV();
      uint8_t* du = uv + static_cast<size_t>(row) * w;
      for (int col = 0; col < w / 2; ++col) {
        du[2 * col] = su[col];
        du[2 * col + 1] = sv[col];
      }
    }
    ok = engine_->EncodeCpu(nv12_.data(), nv12_size, ts, force_key, &au_,
                            &keyframe);
  }

  if (!ok) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  if (au_.empty()) {
    return WEBRTC_VIDEO_CODEC_OK;  // encoder buffered it; nothing to emit yet
  }
  return Deliver(au_, keyframe, frame);
}

int32_t V4l2Encoder::Deliver(const std::vector<uint8_t>& au, bool keyframe,
                             const webrtc::VideoFrame& frame) {
  webrtc::EncodedImage image;
  image.SetEncodedData(
      webrtc::EncodedImageBuffer::Create(au.data(), au.size()));
  image._encodedWidth = width_;
  image._encodedHeight = height_;
  image.SetRtpTimestamp(frame.rtp_timestamp());
  image.capture_time_ms_ = frame.render_time_ms();
  image._frameType = keyframe ? webrtc::VideoFrameType::kVideoFrameKey
                              : webrtc::VideoFrameType::kVideoFrameDelta;

  // Latency knob: attach a playout-delay hint so the receiver minimizes its
  // jitter buffer (the dominant latency term). It rides the playout-delay RTP
  // header extension, negotiated by default. IVI_ENC_PLAYOUT_MS=<max_ms> sets
  // the delay to {0, max_ms}; 0 asks the receiver to render as fast as it can.
  // Unset leaves webrtc's default adaptive buffering.
  static const char* pd_env = std::getenv("IVI_ENC_PLAYOUT_MS");
  static const int pd_max_ms = pd_env != nullptr ? std::atoi(pd_env) : -1;
  if (pd_max_ms >= 0) {
    image.SetPlayoutDelay(webrtc::VideoPlayoutDelay(
        webrtc::TimeDelta::Millis(0), webrtc::TimeDelta::Millis(pd_max_ms)));
  }

  webrtc::CodecSpecificInfo codec_specific;
  codec_specific.codecType = webrtc::kVideoCodecH264;

  const webrtc::EncodedImageCallback::Result result =
      callback_->OnEncodedImage(image, &codec_specific);
  if (result.error != webrtc::EncodedImageCallback::Result::OK) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  return WEBRTC_VIDEO_CODEC_OK;
}

void V4l2Encoder::SetRates(const RateControlParameters& parameters) {
  if (!engine_) {
    return;
  }
  // IVI_ENC_BITRATE (see InitEncode) is a floor here so webrtc's BWE cannot
  // drive the rate back below it mid-stream.
  static const uint32_t bps_floor = [] {
    const char* br = std::getenv("IVI_ENC_BITRATE");
    const long v = br != nullptr ? std::atol(br) : 0;
    return v > 0 ? static_cast<uint32_t>(v) : 0u;
  }();
  uint32_t bps = parameters.bitrate.get_sum_bps();
  if (bps_floor != 0 && bps < bps_floor) {
    bps = bps_floor;
  }
  if (bps != 0 && bps != bitrate_bps_) {
    bitrate_bps_ = bps;
    engine_->SetBitrate(bps);
  }
  const uint32_t fps = static_cast<uint32_t>(parameters.framerate_fps + 0.5);
  if (fps != 0 && fps != framerate_) {
    framerate_ = fps;
    engine_->SetFramerate(fps);
  }
}

webrtc::VideoEncoder::EncoderInfo V4l2Encoder::GetEncoderInfo() const {
  webrtc::VideoEncoder::EncoderInfo info;
  info.implementation_name = "v4l2wc-h264";
  info.is_hardware_accelerated = true;
  // The encoder rate-controls internally (V4L2 bitrate control), so webrtc's
  // software rate allocator is not needed on top.
  info.has_trusted_rate_controller = true;
  return info;
}

// ---- factory ----------------------------------------------------------------

V4l2EncoderFactory::V4l2EncoderFactory(V4l2WcEncoderConfig config)
    : config_(config) {}

std::vector<webrtc::SdpVideoFormat> V4l2EncoderFactory::GetSupportedFormats()
    const {
  return webrtc::SupportedH264Codecs(/*add_scalability_modes=*/false);
}

std::unique_ptr<webrtc::VideoEncoder> V4l2EncoderFactory::Create(
    const webrtc::Environment& /*env*/, const webrtc::SdpVideoFormat& format) {
  if (!absl::EqualsIgnoreCase(format.name, webrtc::kH264CodecName)) {
    return nullptr;
  }
  return std::make_unique<V4l2Encoder>(config_);
}

}  // namespace v4l2wc
