// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/v4l2_m2m_encoder.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include "src/log.h"

// First-cut V4L2 M2M H.264 encoder. Structurally the inverse of
// V4l2M2mDecoder; validated on hardware is a follow-up (needs the Pi's
// /dev/video11). OUTPUT carries raw NV12, CAPTURE the coded H.264.

namespace v4l2wc {
namespace {

constexpr uint32_t kCaptureBufferCount = 4;
constexpr uint32_t kOutputBufferCount = 4;

int xioctl(int fd, unsigned long req, void* arg) {
  int r;
  do {
    r = ::ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

uint32_t OutputType(bool mplane) {
  return mplane ? V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE
                : V4L2_BUF_TYPE_VIDEO_OUTPUT;
}
uint32_t CaptureType(bool mplane) {
  return mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

}  // namespace

V4l2M2mEncoder::V4l2M2mEncoder() = default;

std::unique_ptr<V4l2M2mEncoder> V4l2M2mEncoder::Create(
    const char* device, uint32_t width, uint32_t height, uint32_t bitrate_bps,
    uint32_t framerate, uint32_t gop) {
  std::unique_ptr<V4l2M2mEncoder> enc(new V4l2M2mEncoder());
  enc->width_ = width;
  enc->height_ = height;

  enc->fd_ = ::open(device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (enc->fd_ < 0) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "encoder open " << device << " failed: " << std::strerror(errno);
    return nullptr;
  }

  v4l2_capability cap{};
  if (xioctl(enc->fd_, VIDIOC_QUERYCAP, &cap) < 0) {
    return nullptr;
  }
  enc->mplane_ = (cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) != 0;

  // CAPTURE = H.264. Set first so the encoder sizes its coded buffer.
  v4l2_format cfmt{};
  cfmt.type = CaptureType(enc->mplane_);
  if (enc->mplane_) {
    cfmt.fmt.pix_mp.width = width;
    cfmt.fmt.pix_mp.height = height;
    cfmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
    cfmt.fmt.pix_mp.num_planes = 1;
    cfmt.fmt.pix_mp.plane_fmt[0].sizeimage = width * height;  // generous
  } else {
    cfmt.fmt.pix.width = width;
    cfmt.fmt.pix.height = height;
    cfmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    cfmt.fmt.pix.sizeimage = width * height;
  }
  if (xioctl(enc->fd_, VIDIOC_S_FMT, &cfmt) < 0) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "S_FMT CAPTURE H264 failed: " << std::strerror(errno);
    return nullptr;
  }

  // OUTPUT = NV12 at the real geometry.
  v4l2_format ofmt{};
  ofmt.type = OutputType(enc->mplane_);
  if (enc->mplane_) {
    ofmt.fmt.pix_mp.width = width;
    ofmt.fmt.pix_mp.height = height;
    ofmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
    ofmt.fmt.pix_mp.num_planes = 1;  // NV12 as a single contiguous plane
  } else {
    ofmt.fmt.pix.width = width;
    ofmt.fmt.pix.height = height;
    ofmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  }
  if (xioctl(enc->fd_, VIDIOC_S_FMT, &ofmt) < 0) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "S_FMT OUTPUT NV12 failed: " << std::strerror(errno);
    return nullptr;
  }

  // Frame rate (drives the encoder's rate control cadence).
  v4l2_streamparm parm{};
  parm.type = OutputType(enc->mplane_);
  parm.parm.output.timeperframe.numerator = 1;
  parm.parm.output.timeperframe.denominator = framerate != 0 ? framerate : 30;
  xioctl(enc->fd_, VIDIOC_S_PARM, &parm);  // best-effort

  enc->SetCtrl(
      V4L2_CID_MPEG_VIDEO_BITRATE,
      static_cast<int32_t>(bitrate_bps != 0 ? bitrate_bps : 2'000'000));
  enc->SetCtrl(V4L2_CID_MPEG_VIDEO_GOP_SIZE,
               static_cast<int32_t>(gop != 0 ? gop : 60));
  // Low-latency: no B-frames, so no reorder delay.
  enc->SetCtrl(V4L2_CID_MPEG_VIDEO_B_FRAMES, 0);
  enc->SetCtrl(V4L2_CID_MPEG_VIDEO_H264_PROFILE,
               V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE);

  // CAPTURE buffers: MMAP, read coded bytes out on the CPU.
  v4l2_requestbuffers creq{};
  creq.count = kCaptureBufferCount;
  creq.type = CaptureType(enc->mplane_);
  creq.memory = V4L2_MEMORY_MMAP;
  if (xioctl(enc->fd_, VIDIOC_REQBUFS, &creq) < 0 || creq.count == 0) {
    return nullptr;
  }
  enc->capture_buffers_.resize(creq.count);
  for (uint32_t i = 0; i < creq.count; ++i) {
    v4l2_buffer buf{};
    v4l2_plane planes[VIDEO_MAX_PLANES]{};
    buf.type = CaptureType(enc->mplane_);
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (enc->mplane_) {
      buf.length = 1;
      buf.m.planes = planes;
    }
    if (xioctl(enc->fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      return nullptr;
    }
    const uint32_t len = enc->mplane_ ? planes[0].length : buf.length;
    const uint32_t off = enc->mplane_ ? planes[0].m.mem_offset : buf.m.offset;
    void* addr =
        ::mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, enc->fd_, off);
    if (addr == MAP_FAILED) {
      return nullptr;
    }
    enc->capture_buffers_[i] = {addr, len};
    if (xioctl(enc->fd_, VIDIOC_QBUF, &buf) < 0) {
      return nullptr;
    }
  }
  uint32_t ctype = CaptureType(enc->mplane_);
  if (xioctl(enc->fd_, VIDIOC_STREAMON, &ctype) < 0) {
    return nullptr;
  }

  // OUTPUT queue is set up lazily on the first Encode, once we know whether the
  // frames arrive as dma-bufs (import) or CPU buffers (mmap + copy).
  return enc;
}

V4l2M2mEncoder::~V4l2M2mEncoder() {
  if (fd_ < 0) {
    return;
  }
  uint32_t otype = OutputType(mplane_);
  uint32_t ctype = CaptureType(mplane_);
  xioctl(fd_, VIDIOC_STREAMOFF, &otype);
  xioctl(fd_, VIDIOC_STREAMOFF, &ctype);
  for (auto& b : output_buffers_) {
    if (b.mmap_addr != nullptr) {
      ::munmap(b.mmap_addr, b.length);
    }
  }
  for (auto& b : capture_buffers_) {
    if (b.mmap_addr != nullptr) {
      ::munmap(b.mmap_addr, b.length);
    }
  }
  ::close(fd_);
  fd_ = -1;
}

bool V4l2M2mEncoder::SetCtrl(uint32_t id, int32_t value) {
  v4l2_control ctrl{};
  ctrl.id = id;
  ctrl.value = value;
  return xioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

void V4l2M2mEncoder::SetBitrate(uint32_t bitrate_bps) {
  SetCtrl(V4L2_CID_MPEG_VIDEO_BITRATE, static_cast<int32_t>(bitrate_bps));
}

void V4l2M2mEncoder::SetFramerate(uint32_t framerate) {
  v4l2_streamparm parm{};
  parm.type = OutputType(mplane_);
  parm.parm.output.timeperframe.numerator = 1;
  parm.parm.output.timeperframe.denominator = framerate != 0 ? framerate : 30;
  xioctl(fd_, VIDIOC_S_PARM, &parm);
}

bool V4l2M2mEncoder::DriveAndCollect(std::vector<uint8_t>* out,
                                     bool* keyframe) {
  // Wait for a coded buffer, then dequeue it.
  pollfd pfd{fd_, POLLIN, 0};
  if (::poll(&pfd, 1, 200) <= 0) {
    return false;  // timed out; caller treats as no output this call
  }
  v4l2_buffer buf{};
  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  buf.type = CaptureType(mplane_);
  buf.memory = V4L2_MEMORY_MMAP;
  if (mplane_) {
    buf.length = 1;
    buf.m.planes = planes;
  }
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
    return false;
  }
  const uint32_t bytes = mplane_ ? planes[0].bytesused : buf.bytesused;
  const void* src = capture_buffers_[buf.index].mmap_addr;
  out->insert(out->end(), static_cast<const uint8_t*>(src),
              static_cast<const uint8_t*>(src) + bytes);
  *keyframe = (buf.flags & V4L2_BUF_FLAG_KEYFRAME) != 0;
  // Re-queue the coded buffer for the next frame.
  xioctl(fd_, VIDIOC_QBUF, &buf);
  // Reclaim every consumed OUTPUT (raw) buffer so its slot -- and the imported
  // dma-buf -- is free to queue again.
  for (;;) {
    v4l2_buffer obuf{};
    v4l2_plane oplanes[VIDEO_MAX_PLANES]{};
    obuf.type = OutputType(mplane_);
    obuf.memory = output_dmabuf_ ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
    if (mplane_) {
      obuf.length = 1;
      obuf.m.planes = oplanes;
    }
    if (xioctl(fd_, VIDIOC_DQBUF, &obuf) < 0) {
      break;  // EAGAIN: nothing more to reclaim this pass
    }
    output_free_.push_back(obuf.index);
  }
  return true;
}

bool V4l2M2mEncoder::EnsureOutput(bool dmabuf) {
  if (output_ready_) {
    return true;
  }
  v4l2_requestbuffers req{};
  req.count = kOutputBufferCount;
  req.type = OutputType(mplane_);
  req.memory = dmabuf ? V4L2_MEMORY_DMABUF : V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0 || req.count == 0) {
    return false;
  }
  output_buffers_.assign(req.count, OutputBuffer{});
  output_free_.clear();
  for (uint32_t i = 0; i < req.count; ++i) {
    if (!dmabuf) {
      // CPU path: map each OUTPUT buffer so a frame can be copied into it.
      v4l2_buffer buf{};
      v4l2_plane planes[VIDEO_MAX_PLANES]{};
      buf.type = OutputType(mplane_);
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (mplane_) {
        buf.length = 1;
        buf.m.planes = planes;
      }
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        return false;
      }
      const uint32_t len = mplane_ ? planes[0].length : buf.length;
      const uint32_t off = mplane_ ? planes[0].m.mem_offset : buf.m.offset;
      void* addr =
          ::mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, off);
      if (addr == MAP_FAILED) {
        return false;
      }
      output_buffers_[i] = {addr, len};
    }
    output_free_.push_back(i);
  }
  uint32_t otype = OutputType(mplane_);
  if (xioctl(fd_, VIDIOC_STREAMON, &otype) < 0) {
    return false;
  }
  output_dmabuf_ = dmabuf;
  output_ready_ = true;
  return true;
}

int V4l2M2mEncoder::QueueOutputDmabuf(const int* fds, const uint32_t* strides,
                                      uint32_t num_planes, uint64_t timestamp) {
  if (output_free_.empty() || num_planes == 0 || fds == nullptr) {
    return -1;
  }
  const uint32_t index = output_free_.back();
  // NV12 is imported as one contiguous plane referencing the producer's fd: the
  // Y plane then interleaved CbCr, stride*height*3/2 bytes. (The descriptor may
  // list two planes, but both share fds[0] and the packed layout is implicit.)
  const uint32_t stride =
      (strides != nullptr && strides[0] != 0) ? strides[0] : width_;
  const uint32_t frame_bytes = stride * height_ * 3 / 2;
  v4l2_buffer buf{};
  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  buf.type = OutputType(mplane_);
  buf.memory = V4L2_MEMORY_DMABUF;
  buf.index = index;
  buf.timestamp.tv_sec = static_cast<long>(timestamp / 1000000);
  buf.timestamp.tv_usec = static_cast<long>(timestamp % 1000000);
  if (mplane_) {
    buf.length = 1;
    buf.m.planes = planes;
    planes[0].m.fd = fds[0];
    planes[0].bytesused = frame_bytes;
    planes[0].length = frame_bytes;
    planes[0].data_offset = 0;
  } else {
    buf.m.fd = fds[0];
    buf.bytesused = frame_bytes;
    buf.length = frame_bytes;
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
    return -1;  // slot stays free
  }
  output_free_.pop_back();
  return static_cast<int>(index);
}

int V4l2M2mEncoder::QueueOutputCpu(const uint8_t* nv12, size_t size,
                                   uint64_t timestamp) {
  if (output_free_.empty()) {
    return -1;
  }
  const uint32_t index = output_free_.back();
  const OutputBuffer& ob = output_buffers_[index];
  if (ob.mmap_addr == nullptr || size > ob.length) {
    return -1;
  }
  std::memcpy(ob.mmap_addr, nv12, size);
  v4l2_buffer buf{};
  v4l2_plane planes[VIDEO_MAX_PLANES]{};
  buf.type = OutputType(mplane_);
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = index;
  buf.timestamp.tv_sec = static_cast<long>(timestamp / 1000000);
  buf.timestamp.tv_usec = static_cast<long>(timestamp % 1000000);
  if (mplane_) {
    buf.length = 1;
    buf.m.planes = planes;
    planes[0].bytesused = static_cast<uint32_t>(size);
    planes[0].length = static_cast<uint32_t>(ob.length);
  } else {
    buf.bytesused = static_cast<uint32_t>(size);
    buf.length = static_cast<uint32_t>(ob.length);
  }
  if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
    return -1;
  }
  output_free_.pop_back();
  return static_cast<int>(index);
}

bool V4l2M2mEncoder::EncodeDmabuf(const int* fds, const uint32_t* offsets,
                                  const uint32_t* strides, uint32_t num_planes,
                                  uint64_t timestamp, bool force_keyframe,
                                  std::vector<uint8_t>* out, bool* keyframe) {
  (void)offsets;  // packed NV12: plane offsets are implicit in the format
  if (num_planes == 0 || fds == nullptr) {
    return false;
  }
  if (!EnsureOutput(/*dmabuf=*/true)) {
    return false;
  }
  if (force_keyframe) {
    SetCtrl(V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
  }
  if (QueueOutputDmabuf(fds, strides, num_planes, timestamp) < 0) {
    return false;
  }
  return DriveAndCollect(out, keyframe);
}

bool V4l2M2mEncoder::EncodeCpu(const uint8_t* nv12, size_t size,
                               uint64_t timestamp, bool force_keyframe,
                               std::vector<uint8_t>* out, bool* keyframe) {
  if (nv12 == nullptr || size == 0) {
    return false;
  }
  if (!EnsureOutput(/*dmabuf=*/false)) {
    return false;
  }
  if (force_keyframe) {
    SetCtrl(V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1);
  }
  if (QueueOutputCpu(nv12, size, timestamp) < 0) {
    return false;
  }
  return DriveAndCollect(out, keyframe);
}

}  // namespace v4l2wc
