// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_m2m_decoder.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "rtc_base/logging.h"

namespace v4l2wc {
namespace {

// DRM_FORMAT_MOD_LINEAR; kept local to avoid a libdrm dependency.
constexpr std::uint64_t kModifierLinear = 0;

int xioctl(int fd, unsigned long request, void* arg) {
  int r = 0;
  do {
    r = ioctl(fd, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

std::uint32_t OutputType(bool mplane) {
  return mplane ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
                : V4L2_BUF_TYPE_VIDEO_OUTPUT;
}
std::uint32_t CaptureType(bool mplane) {
  return mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

bool StreamOn(int fd, std::uint32_t type) {
  return xioctl(fd, VIDIOC_STREAMON, &type) == 0;
}
void StreamOff(int fd, std::uint32_t type) {
  xioctl(fd, VIDIOC_STREAMOFF, &type);
}

}  // namespace

std::unique_ptr<V4l2M2mDecoder> V4l2M2mDecoder::Create(
    const char* device, std::uint32_t codec_fourcc,
    std::uint32_t capture_fourcc, std::uint32_t coded_width,
    std::uint32_t coded_height, std::uint32_t output_buffer_count,
    std::uint32_t capture_buffer_count, std::size_t output_buffer_size) {
  if (!device || !device[0] || coded_width == 0 || coded_height == 0) {
    return nullptr;
  }

  std::unique_ptr<V4l2M2mDecoder> dec(new V4l2M2mDecoder());
  dec->codec_fourcc_ = codec_fourcc;
  dec->capture_fourcc_ = capture_fourcc;
  dec->capture_count_req_ = capture_buffer_count;

  dec->fd_ = ::open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (dec->fd_ < 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: open(" << device
                      << ") failed: " << std::strerror(errno);
    return nullptr;
  }

  v4l2_capability cap{};
  if (xioctl(dec->fd_, VIDIOC_QUERYCAP, &cap) != 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: QUERYCAP failed";
    return nullptr;
  }
  const std::uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS)
                                 ? cap.device_caps
                                 : cap.capabilities;
  if (caps & V4L2_CAP_VIDEO_M2M_MPLANE) {
    dec->mplane_ = true;
  } else if (caps & V4L2_CAP_VIDEO_M2M) {
    dec->mplane_ = false;
  } else {
    RTC_LOG(LS_ERROR) << "v4l2wc: device is not an M2M decoder (caps=" << caps
                      << ")";
    return nullptr;
  }

  // S_FMT OUTPUT (coded input). Deliberately leave width/height at 0: a
  // stateful decoder detects the resolution from the stream and fires the
  // initial V4L2_EVENT_SOURCE_CHANGE. Pinning the coded dimensions here makes
  // bcm2835-codec treat the resolution as already known and skip the event, so
  // CAPTURE never gets set up. Only the codec and a generous OUTPUT sizeimage
  // are set. (coded_width/height are still used to floor output_buffer_size.)
  (void)coded_width;
  (void)coded_height;
  v4l2_format ofmt{};
  ofmt.type = OutputType(dec->mplane_);
  if (dec->mplane_) {
    ofmt.fmt.pix_mp.pixelformat = codec_fourcc;
    ofmt.fmt.pix_mp.num_planes = 1;
    if (output_buffer_size > 0) {
      ofmt.fmt.pix_mp.plane_fmt[0].sizeimage =
          static_cast<std::uint32_t>(output_buffer_size);
    }
  } else {
    ofmt.fmt.pix.pixelformat = codec_fourcc;
    if (output_buffer_size > 0) {
      ofmt.fmt.pix.sizeimage = static_cast<std::uint32_t>(output_buffer_size);
    }
  }
  if (xioctl(dec->fd_, VIDIOC_S_FMT, &ofmt) != 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: S_FMT OUTPUT failed: "
                      << std::strerror(errno);
    return nullptr;
  }

  // Do NOT pre-set the CAPTURE format: a stateful decoder only fires the
  // initial V4L2_EVENT_SOURCE_CHANGE when the CAPTURE format still needs to be
  // resolved. Pinning CAPTURE to the coded dimensions up front makes some
  // drivers (bcm2835-codec) suppress the event. CAPTURE is negotiated in
  // SetupCapture() when the event arrives.

  // Allocate + mmap OUTPUT buffers.
  v4l2_requestbuffers oreq{};
  oreq.count = output_buffer_count;
  oreq.type = OutputType(dec->mplane_);
  oreq.memory = V4L2_MEMORY_MMAP;
  if (xioctl(dec->fd_, VIDIOC_REQBUFS, &oreq) != 0 || oreq.count == 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: REQBUFS OUTPUT failed";
    return nullptr;
  }
  dec->output_buffers_.resize(oreq.count);
  for (std::uint32_t i = 0; i < oreq.count; ++i) {
    v4l2_buffer buf{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buf.type = OutputType(dec->mplane_);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (dec->mplane_) {
      buf.length = VIDEO_MAX_PLANES;
      buf.m.planes = planes;
    }
    if (xioctl(dec->fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      RTC_LOG(LS_ERROR) << "v4l2wc: QUERYBUF OUTPUT " << i << " failed";
      return nullptr;
    }
    const std::size_t len = dec->mplane_ ? planes[0].length : buf.length;
    const off_t off = dec->mplane_ ? planes[0].m.mem_offset : buf.m.offset;
    void* ptr =
        ::mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, dec->fd_, off);
    if (ptr == MAP_FAILED) {
      RTC_LOG(LS_ERROR) << "v4l2wc: mmap OUTPUT " << i << " failed";
      return nullptr;
    }
    dec->output_buffers_[i] = OutputBuffer{ptr, len};
    dec->output_free_.push_back(i);
  }

  // Subscribe to the reconfiguration event, then stream OUTPUT on.
  v4l2_event_subscription sub{};
  sub.type = V4L2_EVENT_SOURCE_CHANGE;
  xioctl(dec->fd_, VIDIOC_SUBSCRIBE_EVENT, &sub);

  if (!StreamOn(dec->fd_, OutputType(dec->mplane_))) {
    RTC_LOG(LS_ERROR) << "v4l2wc: STREAMON OUTPUT failed";
    return nullptr;
  }
  dec->output_streaming_ = true;
  return dec;
}

V4l2M2mDecoder::~V4l2M2mDecoder() {
  if (fd_ < 0) {
    return;
  }
  if (capture_streaming_) {
    StreamOff(fd_, CaptureType(mplane_));
  }
  if (output_streaming_) {
    StreamOff(fd_, OutputType(mplane_));
  }
  for (auto& b : capture_buffers_) {
    if (b.dmabuf_fd >= 0) {
      ::close(b.dmabuf_fd);
    }
  }
  for (auto& b : output_buffers_) {
    if (b.ptr && b.ptr != MAP_FAILED) {
      ::munmap(b.ptr, b.length);
    }
  }
  ::close(fd_);
}

SubmitResult V4l2M2mDecoder::SubmitBitstream(const std::uint8_t* data,
                                             std::size_t size,
                                             std::uint64_t timestamp_ns) {
  if (!data || size == 0) {
    return SubmitResult::kError;
  }
  if (output_free_.empty()) {
    return SubmitResult::kTryAgain;
  }
  const std::uint32_t idx = output_free_.back();
  OutputBuffer& slot = output_buffers_[idx];
  if (size > slot.length) {
    RTC_LOG(LS_ERROR) << "v4l2wc: coded frame " << size
                      << " exceeds OUTPUT buffer " << slot.length;
    return SubmitResult::kError;
  }
  std::memcpy(slot.ptr, data, size);
  output_free_.pop_back();

  v4l2_buffer buf{};
  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  buf.type = OutputType(mplane_);
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = idx;
  buf.timestamp.tv_sec = static_cast<long>(timestamp_ns / 1000000000ULL);
  buf.timestamp.tv_usec =
      static_cast<long>((timestamp_ns % 1000000000ULL) / 1000ULL);
  if (mplane_) {
    buf.length = 1;
    buf.m.planes = planes;
    // Set BOTH bytesused and length: bcm2835-codec rejects the QBUF with
    // EMSGSIZE when the plane length is left zero.
    planes[0].bytesused = static_cast<std::uint32_t>(size);
    planes[0].length = static_cast<std::uint32_t>(slot.length);
  } else {
    buf.bytesused = static_cast<std::uint32_t>(size);
    buf.length = static_cast<std::uint32_t>(slot.length);
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buf) != 0) {
    output_free_.push_back(idx);
    RTC_LOG(LS_ERROR) << "v4l2wc: QBUF OUTPUT failed: " << std::strerror(errno);
    return SubmitResult::kError;
  }
  return SubmitResult::kOk;
}

bool V4l2M2mDecoder::SetupCapture() {
  // Read the decoder-detected CAPTURE format.
  v4l2_format cfmt{};
  cfmt.type = CaptureType(mplane_);
  if (xioctl(fd_, VIDIOC_G_FMT, &cfmt) != 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: G_FMT CAPTURE failed";
    return false;
  }
  if (mplane_) {
    cfmt.fmt.pix_mp.pixelformat = capture_fourcc_;
    cap_width_ = cfmt.fmt.pix_mp.width;
    cap_height_ = cfmt.fmt.pix_mp.height;
  } else {
    cfmt.fmt.pix.pixelformat = capture_fourcc_;
    cap_width_ = cfmt.fmt.pix.width;
    cap_height_ = cfmt.fmt.pix.height;
  }
  xioctl(fd_, VIDIOC_S_FMT, &cfmt);  // pin the CAPTURE pixel format
  if (mplane_) {
    cap_stride_ = cfmt.fmt.pix_mp.plane_fmt[0].bytesperline;
  } else {
    cap_stride_ = cfmt.fmt.pix.bytesperline;
  }
  if (cap_stride_ == 0) {
    cap_stride_ = cap_width_;  // NV12 luma stride fallback
  }

  v4l2_requestbuffers creq{};
  creq.count = capture_count_req_;
  creq.type = CaptureType(mplane_);
  creq.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &creq) != 0 || creq.count == 0) {
    RTC_LOG(LS_ERROR) << "v4l2wc: REQBUFS CAPTURE failed";
    return false;
  }
  capture_buffers_.assign(creq.count, CaptureBuffer{});
  for (std::uint32_t i = 0; i < creq.count; ++i) {
    v4l2_buffer buf{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buf.type = CaptureType(mplane_);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (mplane_) {
      buf.length = VIDEO_MAX_PLANES;
      buf.m.planes = planes;
    }
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      RTC_LOG(LS_ERROR) << "v4l2wc: QUERYBUF CAPTURE " << i << " failed";
      return false;
    }
    capture_buffers_[i].length = mplane_ ? planes[0].length : buf.length;

    // Export the CAPTURE buffer (plane 0) as a DMA-BUF fd, held for reuse.
    v4l2_exportbuffer expbuf{};
    expbuf.type = CaptureType(mplane_);
    expbuf.index = i;
    expbuf.plane = 0;
    expbuf.flags = O_RDONLY | O_CLOEXEC;
    if (xioctl(fd_, VIDIOC_EXPBUF, &expbuf) != 0) {
      RTC_LOG(LS_ERROR) << "v4l2wc: EXPBUF CAPTURE " << i << " failed";
      return false;
    }
    capture_buffers_[i].dmabuf_fd = expbuf.fd;

    // Queue the buffer so the decoder can fill it.
    v4l2_buffer qbuf{};
    v4l2_plane qplanes[VIDEO_MAX_PLANES]{};
    qbuf.type = CaptureType(mplane_);
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.index = i;
    if (mplane_) {
      qbuf.length = 1;
      qbuf.m.planes = qplanes;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &qbuf) != 0) {
      RTC_LOG(LS_ERROR) << "v4l2wc: QBUF CAPTURE " << i << " failed";
      return false;
    }
    capture_buffers_[i].queued = true;
  }

  if (!StreamOn(fd_, CaptureType(mplane_))) {
    RTC_LOG(LS_ERROR) << "v4l2wc: STREAMON CAPTURE failed";
    return false;
  }
  capture_streaming_ = true;
  RTC_LOG(LS_INFO) << "v4l2wc: CAPTURE up " << cap_width_ << "x" << cap_height_
                   << " stride=" << cap_stride_
                   << " buffers=" << capture_buffers_.size();
  return true;
}

void V4l2M2mDecoder::TeardownCapture() {
  if (capture_streaming_) {
    StreamOff(fd_, CaptureType(mplane_));
    capture_streaming_ = false;
  }
}

bool V4l2M2mDecoder::Drive() {
  // bcm2835-codec only surfaces V4L2 events (SOURCE_CHANGE) and buffer
  // completions through poll(); a bare DQEVENT/DQBUF spin never sees them. A
  // short poll lets the driver deliver without stalling the decode thread --
  // free OUTPUT buffers keep POLLOUT ready, so it usually returns immediately.
  pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN | POLLOUT | POLLPRI;
  poll(&pfd, 1, 12);

  // Drain events first: the initial SOURCE_CHANGE establishes CAPTURE.
  for (;;) {
    v4l2_event ev{};
    if (xioctl(fd_, VIDIOC_DQEVENT, &ev) != 0) {
      break;  // no more events
    }
    if (ev.type == V4L2_EVENT_SOURCE_CHANGE &&
        (ev.u.src_change.changes & V4L2_EVENT_SRC_CH_RESOLUTION)) {
      if (!capture_streaming_) {
        if (!SetupCapture()) {
          return false;
        }
      } else {
        // Mid-stream resolution change: the caller recreates the decoder.
        RTC_LOG(LS_INFO) << "v4l2wc: mid-stream SOURCE_CHANGE; recreate";
        return false;
      }
    }
  }

  // Reclaim completed OUTPUT buffers.
  for (;;) {
    v4l2_buffer buf{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buf.type = OutputType(mplane_);
    buf.memory = V4L2_MEMORY_MMAP;
    if (mplane_) {
      buf.length = 1;
      buf.m.planes = planes;
    }
    if (xioctl(fd_, VIDIOC_DQBUF, &buf) != 0) {
      break;  // EAGAIN: nothing to reclaim
    }
    output_free_.push_back(buf.index);
  }

  // Dequeue decoded CAPTURE buffers, keeping only the newest (latest-wins).
  if (capture_streaming_) {
    for (;;) {
      v4l2_buffer buf{};
      v4l2_plane planes[VIDEO_MAX_PLANES]{};
      buf.type = CaptureType(mplane_);
      buf.memory = V4L2_MEMORY_MMAP;
      if (mplane_) {
        buf.length = 1;
        buf.m.planes = planes;
      }
      if (xioctl(fd_, VIDIOC_DQBUF, &buf) != 0) {
        break;  // EAGAIN
      }
      if (buf.index < capture_buffers_.size()) {
        capture_buffers_[buf.index].queued = false;
      }
      // Drop any prior ready-but-unacquired buffer back to the decoder.
      if (have_ready_ && ready_index_ != buf.index &&
          ready_index_ < capture_buffers_.size() &&
          !capture_buffers_[ready_index_].queued) {
        Release(ready_index_);
      }
      have_ready_ = true;
      ready_index_ = buf.index;
      ready_ts_ns_ =
          static_cast<std::uint64_t>(buf.timestamp.tv_sec) * 1000000000ULL +
          static_cast<std::uint64_t>(buf.timestamp.tv_usec) * 1000ULL;
    }
  }
  return true;
}

bool V4l2M2mDecoder::Acquire(V4l2DmaFrame* out) {
  if (!have_ready_ || !out) {
    return false;
  }
  const std::uint32_t idx = ready_index_;
  have_ready_ = false;
  if (idx >= capture_buffers_.size()) {
    return false;
  }
  const int fd = capture_buffers_[idx].dmabuf_fd;

  *out = V4l2DmaFrame{};
  out->capture_index = idx;
  out->width = cap_width_;
  out->height = cap_height_;
  out->drm_fourcc = capture_fourcc_;  // V4L2 NV12 fourcc == DRM_FORMAT_NV12
  out->modifier = kModifierLinear;
  out->timestamp_ns = ready_ts_ns_;

  if (capture_fourcc_ == V4L2_PIX_FMT_NV12) {
    // One contiguous buffer, two DRM planes: Y then interleaved UV.
    out->num_planes = 2;
    out->fds[0] = fd;
    out->offsets[0] = 0;
    out->pitches[0] = cap_stride_;
    out->fds[1] = fd;
    out->offsets[1] = cap_stride_ * cap_height_;
    out->pitches[1] = cap_stride_;
  } else {
    out->num_planes = 1;
    out->fds[0] = fd;
    out->offsets[0] = 0;
    out->pitches[0] = cap_stride_;
  }
  return true;
}

void V4l2M2mDecoder::Release(std::uint32_t capture_index) {
  if (capture_index >= capture_buffers_.size()) {
    return;
  }
  if (capture_buffers_[capture_index].queued) {
    return;  // already back with the decoder
  }
  v4l2_buffer buf{};
  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  buf.type = CaptureType(mplane_);
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = capture_index;
  if (mplane_) {
    buf.length = 1;
    buf.m.planes = planes;
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buf) == 0) {
    capture_buffers_[capture_index].queued = true;
  }
}

}  // namespace v4l2wc
