// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// The dlopen'd libva entry table used by the VAAPI decode engine. libva is
// resolved at runtime so nothing links against it; only its headers are needed
// at build time (from the sysroot, or libva-devel for a host build).
#ifndef V4L2WC_SRC_VA_LOADER_H_
#define V4L2WC_SRC_VA_LOADER_H_

#include <stdint.h>
#include <va/va.h>

namespace v4l2wc::va {

// dlopen'd libva entry table.
struct VaApi {
  VaApi();
  ~VaApi();

  void* lib = nullptr;
  void* lib_drm = nullptr;
  VADisplay (*GetDisplayDRM)(int) = nullptr;
  VAStatus (*Initialize)(VADisplay, int*, int*) = nullptr;
  VAStatus (*Terminate)(VADisplay) = nullptr;
  const char* (*ErrorStr)(VAStatus) = nullptr;
  VAStatus (*CreateConfig)(VADisplay, VAProfile, VAEntrypoint, VAConfigAttrib*,
                           int, VAConfigID*) = nullptr;
  VAStatus (*CreateSurfaces)(VADisplay, unsigned, unsigned, unsigned,
                             VASurfaceID*, unsigned, VASurfaceAttrib*,
                             unsigned) = nullptr;
  VAStatus (*CreateContext)(VADisplay, VAConfigID, int, int, int, VASurfaceID*,
                            int, VAContextID*) = nullptr;
  VAStatus (*CreateBuffer)(VADisplay, VAContextID, VABufferType, unsigned,
                           unsigned, void*, VABufferID*) = nullptr;
  VAStatus (*BeginPicture)(VADisplay, VAContextID, VASurfaceID) = nullptr;
  VAStatus (*RenderPicture)(VADisplay, VAContextID, VABufferID*, int) = nullptr;
  VAStatus (*EndPicture)(VADisplay, VAContextID) = nullptr;
  VAStatus (*SyncSurface)(VADisplay, VASurfaceID) = nullptr;
  VAStatus (*DestroyBuffer)(VADisplay, VABufferID) = nullptr;
  VAStatus (*DestroySurfaces)(VADisplay, VASurfaceID*, int) = nullptr;
  VAStatus (*DestroyContext)(VADisplay, VAContextID) = nullptr;
  VAStatus (*DestroyConfig)(VADisplay, VAConfigID) = nullptr;
  VAStatus (*DeriveImage)(VADisplay, VASurfaceID, VAImage*) = nullptr;
  VAStatus (*MapBuffer)(VADisplay, VABufferID, void**) = nullptr;
  VAStatus (*UnmapBuffer)(VADisplay, VABufferID) = nullptr;
  VAStatus (*DestroyImage)(VADisplay, VAImageID) = nullptr;
  VAStatus (*ExportSurfaceHandle)(VADisplay, VASurfaceID, uint32_t, uint32_t,
                                  void*) = nullptr;
};
bool VaLoad(VaApi* va);

}  // namespace v4l2wc::va

#endif  // V4L2WC_SRC_VA_LOADER_H_
