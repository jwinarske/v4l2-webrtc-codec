// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/va_loader.h"

#include <dlfcn.h>
#include <stdio.h>

namespace v4l2wc::va {

VaApi::VaApi() = default;
VaApi::~VaApi() = default;

bool VaLoad(VaApi* va) {
  va->lib = dlopen("libva.so.2", RTLD_NOW | RTLD_GLOBAL);
  va->lib_drm = dlopen("libva-drm.so.2", RTLD_NOW | RTLD_GLOBAL);
  if (!va->lib || !va->lib_drm) {
    fprintf(stderr, "dlopen libva: %s\n", dlerror());
    return false;
  }
#define S(h, f, name)                      \
  do {                                     \
    *(void**)(&va->f) = dlsym(h, name);    \
    if (!va->f) {                          \
      fprintf(stderr, "dlsym %s\n", name); \
      return false;                        \
    }                                      \
  } while (0)
  S(va->lib_drm, GetDisplayDRM, "vaGetDisplayDRM");
  S(va->lib, Initialize, "vaInitialize");
  S(va->lib, Terminate, "vaTerminate");
  S(va->lib, ErrorStr, "vaErrorStr");
  S(va->lib, CreateConfig, "vaCreateConfig");
  S(va->lib, CreateSurfaces, "vaCreateSurfaces");
  S(va->lib, CreateContext, "vaCreateContext");
  S(va->lib, CreateBuffer, "vaCreateBuffer");
  S(va->lib, BeginPicture, "vaBeginPicture");
  S(va->lib, RenderPicture, "vaRenderPicture");
  S(va->lib, EndPicture, "vaEndPicture");
  S(va->lib, SyncSurface, "vaSyncSurface");
  S(va->lib, DestroyBuffer, "vaDestroyBuffer");
  S(va->lib, DestroySurfaces, "vaDestroySurfaces");
  S(va->lib, DestroyContext, "vaDestroyContext");
  S(va->lib, DestroyConfig, "vaDestroyConfig");
  S(va->lib, DeriveImage, "vaDeriveImage");
  S(va->lib, MapBuffer, "vaMapBuffer");
  S(va->lib, UnmapBuffer, "vaUnmapBuffer");
  S(va->lib, DestroyImage, "vaDestroyImage");
  S(va->lib, ExportSurfaceHandle, "vaExportSurfaceHandle");
#undef S
  return true;
}

}  // namespace v4l2wc::va
