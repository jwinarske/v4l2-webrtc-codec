// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_decoder.h"

#include <linux/videodev2.h>

#include <algorithm>
#include <utility>

#include "absl/strings/match.h"
#include "api/make_ref_counted.h"
#include "api/video/video_frame.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/logging.h"

// The shared native-buffer class and the descriptor ABI live in the libwebrtc
// fork; absorbed in-tree, these resolve against the wrapper's include dirs.
#include "c/lw_video_sink.h"
#include "src/internal/lw_native_video_frame_buffer.h"

// The two decode engines behind IDmaDecoder (V4L2 M2M, VAAPI).
#include "src/v4l2_m2m_decoder.h"
#include "src/vaapi_h264_decoder.h"

// Bitstream sizing (SPS -> coded dimensions) uses the pure-logic parse layer.
#include "parse/h264/nal.h"
#include "parse/h264/sps.h"

namespace v4l2wc {
namespace {

// Context handed to LwNativeVideoFrameBuffer as (LwFrameRelease, void*). It
// keeps the SourceHolder alive (shared_ptr) so the decoder -- and the dmabuf
// fds the descriptor borrows -- outlive the frame, and remembers which CAPTURE
// buffer to re-queue on release.
struct FrameReleaseCtx {
  std::shared_ptr<SourceHolder> holder;
  std::uint32_t capture_index;
};

void ReleaseFrame(void* ctx) {
  auto* c = static_cast<FrameReleaseCtx*>(ctx);
  {
    std::lock_guard<std::mutex> lock(c->holder->mutex());
    if (c->holder->dec()) {
      c->holder->dec()->Release(c->capture_index);  // re-QBUF CAPTURE
    }
  }
  delete c;
}

}  // namespace

SourceHolder::SourceHolder(std::unique_ptr<IDmaDecoder> dec)
    : dec_(std::move(dec)) {}
SourceHolder::~SourceHolder() = default;

V4l2Decoder::V4l2Decoder(V4l2WcConfig config) : config_(config) {}
V4l2Decoder::~V4l2Decoder() = default;

bool V4l2Decoder::Configure(const Settings& /*settings*/) {
  return true;  // the decoder is created lazily on the first SPS (for dims)
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
  // Dropping the previous holder does not free the decoder while frames are
  // still out: each in-flight frame holds a shared reference, so the dmabuf
  // fds its descriptor borrows stay valid until it is released. The new pool
  // gets a new generation, which is how a consumer knows not to carry an
  // import cache across.
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
    RTC_LOG(LS_WARNING) << "v4l2wc: no SPS yet; waiting for a keyframe";
    return false;  // wait for a keyframe carrying an SPS
  }

  const char* path = (config_.video_device && config_.video_device[0])
                         ? config_.video_device
                         : "/dev/video10";
  // OUTPUT holds a single compressed access unit. width*height (the luma size)
  // is not a safe bound -- a high-bitrate keyframe can exceed it -- so floor it
  // at 1 MiB, which comfortably holds an SD/HD H.264 frame.
  const std::size_t output_buffer_size = std::max<std::size_t>(
      static_cast<std::size_t>(sps.width) * sps.height, std::size_t{1} << 20);

  // Prefer the V4L2 M2M stateful decoder (the Pi path). Linear NV12: the
  // software floor (SHM) reads it correctly and is tear-free.
  std::unique_ptr<IDmaDecoder> dec = V4l2M2mDecoder::Create(
      path, V4L2_PIX_FMT_H264, V4L2_PIX_FMT_NV12, sps.width, sps.height,
      /*output_buffer_count=*/4,
      /*capture_buffer_count=*/16, output_buffer_size);
  if (dec) {
    RTC_LOG(LS_INFO) << "v4l2wc: V4L2 M2M decoder on " << path << " "
                     << sps.width << "x" << sps.height;
  } else {
    // No V4L2 M2M device (e.g. an amdgpu host / Steam Deck): fall back to the
    // VAAPI engine, which decodes to an NV12 dma-buf via libva (dlopen'd). The
    // pool covers the max H.264 DPB (16) plus frames in flight to the
    // compositor; the engine re-reads the SPS to size its own state.
    dec = VaapiH264Decoder::Create("/dev/dri/renderD128", /*pool_size=*/28);
    if (dec) {
      RTC_LOG(LS_INFO) << "v4l2wc: VAAPI decoder " << sps.width << "x"
                       << sps.height;
    }
  }
  if (!dec) {
    RTC_LOG(LS_ERROR) << "v4l2wc: no decode backend (V4L2 " << path
                      << " and VAAPI both unavailable)";
    return false;
  }
  ++pool_generation_;
  holder_ = std::make_shared<SourceHolder>(std::move(dec));
  return true;
}

void V4l2Decoder::DeliverReadyFrames() {
  if (!callback_ || !holder_) {
    return;
  }
  for (;;) {
    V4l2DmaFrame f;
    std::uint32_t pool_size = 0;
    {
      std::lock_guard<std::mutex> lock(holder_->mutex());
      if (!holder_->dec()->Acquire(&f)) {
        break;  // nothing ready
      }
      // Read under the same lock as the acquire: a reconfiguration reallocates
      // the pool, and a size from the other side of that would not describe
      // the buffer just acquired.
      pool_size = holder_->dec()->PoolSize();
    }

    // V4L2 copies the OUTPUT buffer timestamp onto the matching CAPTURE buffer,
    // so rtp_timestamp carries the RTP timestamp we stored when this frame's
    // bitstream was submitted -- the identity of the frame that actually popped
    // out, which (with pipeline depth) is not the just-submitted one.
    const uint32_t rtp_timestamp = static_cast<uint32_t>(f.rtp_timestamp);
    int64_t render_time_ms = 0;
    if (auto it = render_time_by_rtp_.find(rtp_timestamp);
        it != render_time_by_rtp_.end()) {
      render_time_ms = it->second;
      // Drop this and any earlier (smaller-rtp) entries: frames leave the
      // decoder in submission order, so nothing older will be looked up again.
      render_time_by_rtp_.erase(render_time_by_rtp_.begin(), std::next(it));
    }

    LwDmabufDescriptor desc{};
    desc.size = sizeof(desc);
    desc.fourcc = f.drm_fourcc;
    desc.modifier = f.modifier;
    desc.width = f.width;
    desc.height = f.height;
    desc.num_planes = f.num_planes;
    for (uint32_t p = 0; p < f.num_planes && p < LW_MAX_PLANES; ++p) {
      desc.planes[p].fd = f.fds[p];
      desc.planes[p].offset = f.offsets[p];
      desc.planes[p].pitch = f.pitches[p];
    }
    desc.acquire_fence_fd = -1;
    // rtp_timestamp_us is microseconds per the sink ABI; the RTP media clock is
    // 90 kHz, so convert. Consumers pace on the (relative) value.
    desc.rtp_timestamp_us =
        static_cast<int64_t>(rtp_timestamp) * 1000000 / 90000;
    desc.frame_seq = frame_seq_++;
    desc.pool_generation = pool_generation_;
    desc.pool_size = pool_size;

    RTC_LOG(LS_INFO) << "v4l2wc: delivering native frame " << f.width << "x"
                     << f.height << " fourcc=" << f.drm_fourcc
                     << " planes=" << f.num_planes;
    auto* ctx = new FrameReleaseCtx{holder_, f.capture_index};
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
    // No decoder yet (waiting for an SPS keyframe): request one.
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  // Remember this frame's render time keyed by its RTP timestamp, to reattach
  // when the (pipelined) frame pops out of CAPTURE. Bound the map so a decoder
  // that never emits some frames can't leak; 64 covers the deepest pipeline.
  render_time_by_rtp_[input_image.RtpTimestamp()] = render_time_ms;
  while (render_time_by_rtp_.size() > 64) {
    render_time_by_rtp_.erase(render_time_by_rtp_.begin());
  }

  // Feed the coded frame; on a full OUTPUT queue, drive() to reclaim and retry.
  for (int attempt = 0; attempt < 64; ++attempt) {
    std::lock_guard<std::mutex> lock(holder_->mutex());
    auto* dec = holder_->dec();
    const SubmitResult s = dec->SubmitBitstream(
        input_image.data(), input_image.size(), input_image.RtpTimestamp());
    if (s == SubmitResult::kOk) {
      break;
    }
    if (s == SubmitResult::kSourceChange) {
      // The engine recognised a stream that no longer matches the decoder it
      // built. Same handling as a format change reported by Drive: rebuild
      // rather than fail, since the stream is fine.
      return Reconfigure(DriveResult::kSourceChange);
    }
    if (s != SubmitResult::kTryAgain) {
      RTC_LOG(LS_ERROR) << "v4l2wc: submit failed (" << static_cast<int>(s)
                        << ")";
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    const DriveResult d = dec->Drive();  // reclaim OUTPUT buffers, then retry
    if (d != DriveResult::kOk) {
      return Reconfigure(d);
    }
  }

  DriveResult drive = DriveResult::kOk;
  {
    std::lock_guard<std::mutex> lock(holder_->mutex());
    drive = holder_->dec()->Drive();
  }
  if (drive != DriveResult::kOk) {
    // Deliver whatever the old pool already produced before dropping it.
    DeliverReadyFrames();
    return Reconfigure(drive);
  }
  DeliverReadyFrames();
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t V4l2Decoder::Reconfigure(DriveResult reason) {
  if (reason != DriveResult::kSourceChange) {
    RTC_LOG(LS_ERROR) << "v4l2wc: decoder failed";
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  // The stream is fine; this decoder is configured for the wrong geometry.
  // Drop it and rebuild from the next keyframe's SPS, which is what carries
  // the new dimensions. Frames already handed out keep the old decoder alive
  // through their release contexts.
  RTC_LOG(LS_INFO) << "v4l2wc: rebuilding the decoder for the new format";
  holder_.reset();
  render_time_by_rtp_.clear();
  // Asking for a keyframe rather than an error: the caller should send one,
  // and the next SubmitBitstream will carry the SPS the rebuild needs.
  return WEBRTC_VIDEO_CODEC_ERROR;
}

int32_t V4l2Decoder::Release() {
  // In-flight frames hold their own shared_ptr to the decoder, so it survives
  // until the last one is released. Dropping ours stops new decodes.
  holder_.reset();
  callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

// ---- factory ----

V4l2DecoderFactory::V4l2DecoderFactory(V4l2WcConfig config) : config_(config) {}

std::vector<webrtc::SdpVideoFormat> V4l2DecoderFactory::GetSupportedFormats()
    const {
  // Advertise the standard H.264 decoder profiles (profile-level-id,
  // packetization-mode, level-asymmetry-allowed) so the generated answer's
  // fmtp matches a peer's offer. A bare format without these parameters is
  // rejected by strict peers when they apply the answer.
  return webrtc::SupportedH264DecoderCodecs();
}

std::unique_ptr<webrtc::VideoDecoder> V4l2DecoderFactory::Create(
    const webrtc::Environment& /*env*/, const webrtc::SdpVideoFormat& format) {
  if (absl::EqualsIgnoreCase(format.name, webrtc::kH264CodecName)) {
    return std::make_unique<V4l2Decoder>(config_);
  }
  return nullptr;
}

}  // namespace v4l2wc
