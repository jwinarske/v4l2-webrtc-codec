// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_decoder.h"

#include <linux/videodev2.h>

#include <utility>

#include "absl/strings/match.h"
#include "api/make_ref_counted.h"
#include "api/video/video_frame.h"
#include "media/base/media_constants.h"

// drm-cxx: the stateful V4L2 decode + DMA-BUF export (reused, not
// reimplemented).
#include <drm-cxx/scene/buffer_source.hpp>  // DmaBufDesc, AcquiredBuffer
#include <drm-cxx/scene/v4l2_decoder_source.hpp>

// The shared native-buffer class and the descriptor ABI live in the libwebrtc
// fork; absorbed in-tree, these resolve against the wrapper's include dirs.
#include "c/lw_video_sink.h"
#include "src/internal/lw_native_video_frame_buffer.h"

// Bitstream sizing (SPS -> coded dimensions) uses the pure-logic parse layer.
#include "parse/h264/nal.h"
#include "parse/h264/sps.h"

namespace v4l2wc {
namespace {

// Context handed to LwNativeVideoFrameBuffer as (LwFrameRelease, void*). It
// keeps the SourceHolder alive (shared_ptr) and owns the AcquiredBuffer whose
// borrowed dmabuf fds the descriptor references, so both outlive the frame.
struct FrameReleaseCtx {
  std::shared_ptr<SourceHolder> holder;
  drm::scene::AcquiredBuffer acquired;
};

void ReleaseFrame(void* ctx) {
  auto* c = static_cast<FrameReleaseCtx*>(ctx);
  {
    std::lock_guard<std::mutex> lock(c->holder->mutex());
    if (c->holder->src()) {
      c->holder->src()->release(std::move(c->acquired));  // re-QBUF CAPTURE
    }
  }
  delete c;
}

// Opens a DRM device for the decoder. V4l2DecoderSource requires one for its
// internal KMS import (unused on this path); a render node is sufficient.
// TODO(hw-verify): use drm-cxx's exact Device open API once building in-tree.
std::unique_ptr<drm::Device> OpenDrmDevice() {
  // return drm::Device::open("/dev/dri/renderD128");  // exact API pending
  return nullptr;
}

}  // namespace

SourceHolder::SourceHolder(std::unique_ptr<drm::scene::V4l2DecoderSource> src,
                           std::unique_ptr<drm::Device> dev)
    : src_(std::move(src)), dev_(std::move(dev)) {}
SourceHolder::~SourceHolder() = default;

V4l2Decoder::V4l2Decoder(V4l2WcConfig config) : config_(config) {}
V4l2Decoder::~V4l2Decoder() = default;

bool V4l2Decoder::Configure(const Settings& /*settings*/) {
  return true;  // the source is created lazily on the first SPS (for dims)
}

int32_t V4l2Decoder::RegisterDecodeCompleteCallback(
    webrtc::DecodedImageCallback* callback) {
  callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

bool V4l2Decoder::EnsureSource(const uint8_t* data, size_t size) {
  if (holder_) {
    return true;
  }
  // Find coded dimensions from the first SPS.
  auto nals = h264::ParseAnnexB(data, size);
  h264::Sps sps;
  bool have_sps = false;
  for (const auto& nal : nals) {
    if (nal.type == h264::NalUnitType::kSps &&
        h264::ParseSps(nal.rbsp.data(), nal.rbsp.size(), &sps)) {
      have_sps = true;
      break;
    }
  }
  if (!have_sps) {
    return false;  // wait for a keyframe carrying an SPS
  }

  auto dev = OpenDrmDevice();
  if (!dev) {
    return false;
  }
  drm::scene::V4l2DecoderConfig cfg;
  cfg.codec_fourcc = V4L2_PIX_FMT_H264;
  cfg.capture_fourcc = V4L2_PIX_FMT_NV12;
  cfg.coded_width = sps.width;
  cfg.coded_height = sps.height;
  cfg.output_buffer_count = 4;
  cfg.capture_buffer_count = 4;
  cfg.output_buffer_size = static_cast<size_t>(sps.width) * sps.height;

  const char* path = (config_.video_device && config_.video_device[0])
                         ? config_.video_device
                         : "/dev/video10";
  auto r = drm::scene::V4l2DecoderSource::create(*dev, path, cfg);
  if (!r) {
    return false;
  }
  ++pool_generation_;
  holder_ = std::make_shared<SourceHolder>(std::move(*r), std::move(dev));
  return true;
}

void V4l2Decoder::DeliverReadyFrames(int64_t render_time_ms,
                                     uint32_t rtp_timestamp) {
  if (!callback_ || !holder_) {
    return;
  }
  for (;;) {
    drm::scene::AcquiredBuffer acquired;
    DmaBufDesc dma;
    {
      std::lock_guard<std::mutex> lock(holder_->mutex());
      auto* src = holder_->src();
      auto a = src->acquire();
      if (!a) {
        break;  // nothing ready
      }
      acquired = std::move(*a);
      auto d = src->export_dma_buf();
      if (!d) {
        src->release(std::move(acquired));
        break;
      }
      dma = *d;
    }

    LwDmabufDescriptor desc{};
    desc.size = sizeof(desc);
    desc.fourcc = dma.drm_fourcc;
    desc.modifier = dma.modifier;
    desc.width = dma.width;
    desc.height = dma.height;
    desc.num_planes = dma.n_planes;
    for (uint32_t p = 0; p < dma.n_planes && p < LW_MAX_PLANES; ++p) {
      desc.planes[p].fd = dma.fds[p];
      desc.planes[p].offset = dma.offsets[p];
      desc.planes[p].pitch = dma.pitches[p];
    }
    desc.acquire_fence_fd = -1;
    desc.rtp_timestamp_us = static_cast<int64_t>(rtp_timestamp);
    desc.frame_seq = frame_seq_++;
    desc.pool_generation = pool_generation_;

    auto* ctx = new FrameReleaseCtx{holder_, std::move(acquired)};
    auto buffer = webrtc::make_ref_counted<libwebrtc::LwNativeVideoFrameBuffer>(
        desc, &ReleaseFrame, ctx);

    webrtc::VideoFrame frame = webrtc::VideoFrame::Builder()
                                   .set_video_frame_buffer(buffer)
                                   .set_timestamp_rtp(rtp_timestamp)
                                   .set_timestamp_us(render_time_ms * 1000)
                                   .build();
    callback_->Decoded(frame);
  }
}

int32_t V4l2Decoder::Decode(const webrtc::EncodedImage& input_image,
                            bool /*missing_frames*/, int64_t render_time_ms) {
  if (!EnsureSource(input_image.data(), input_image.size())) {
    // No source yet (waiting for an SPS keyframe): request one.
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Feed the coded frame; on a full OUTPUT queue, drive() to reclaim and retry.
  drm::span<const std::uint8_t> coded(input_image.data(), input_image.size());
  for (int attempt = 0; attempt < 64; ++attempt) {
    std::lock_guard<std::mutex> lock(holder_->mutex());
    auto* src = holder_->src();
    auto s = src->submit_bitstream(coded, input_image.RtpTimestamp());
    if (s) {
      break;
    }
    // Any error other than "try again" is fatal (e.g. SOURCE_CHANGE).
    if (s.error() != std::errc::resource_unavailable_try_again) {
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    src->drive();  // reclaim OUTPUT buffers, then retry
  }

  {
    std::lock_guard<std::mutex> lock(holder_->mutex());
    holder_->src()->drive();
  }
  DeliverReadyFrames(render_time_ms, input_image.RtpTimestamp());
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4l2Decoder::Release() {
  // In-flight frames hold their own shared_ptr to the source, so it survives
  // until the last one is released. Dropping ours stops new decodes.
  holder_.reset();
  callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

// ---- factory ----

V4l2DecoderFactory::V4l2DecoderFactory(V4l2WcConfig config) : config_(config) {}

std::vector<webrtc::SdpVideoFormat> V4l2DecoderFactory::GetSupportedFormats()
    const {
  return {webrtc::SdpVideoFormat(webrtc::kH264CodecName)};
}

std::unique_ptr<webrtc::VideoDecoder> V4l2DecoderFactory::Create(
    const webrtc::Environment& /*env*/, const webrtc::SdpVideoFormat& format) {
  if (absl::EqualsIgnoreCase(format.name, webrtc::kH264CodecName)) {
    return std::make_unique<V4l2Decoder>(config_);
  }
  return nullptr;
}

}  // namespace v4l2wc
