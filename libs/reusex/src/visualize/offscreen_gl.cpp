// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "visualize/offscreen_gl.hpp"

#include <dlfcn.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace reusex::visualize {

namespace {

// ── A minimal EGL ABI, declared here rather than included ────────────────────
//
// The probe deliberately has no build-time dependency on EGL: it must compile
// and run on a machine with no EGL development package, because "is EGL usable
// here" is a runtime question about the machine, not about the build. What is
// needed is five core entry points and two extension ones, all of them frozen
// ABI since EGL 1.4 / EGL_EXT_platform_base. Every constant below is quoted
// from the Khronos registry headers.

using EglDisplay = void *;
using EglDeviceExt = void *;
using EglInt = std::int32_t;
using EglEnum = unsigned int;
using EglBoolean = unsigned int;
/// EGL's __eglMustCastToProperFunctionPointerType.
using EglProcFn = void (*)();

constexpr EglDisplay kEglNoDisplay = nullptr;
constexpr void *kEglDefaultDisplay = nullptr;
constexpr EglBoolean kEglTrue = 1;
constexpr EglInt kEglSuccess = 0x3000;
constexpr EglInt kEglVendor = 0x3053;
constexpr EglInt kEglExtensions = 0x3055;
constexpr EglEnum kEglPlatformDeviceExt = 0x313F;

/// The subset of libEGL this file calls, resolved by name at runtime.
struct EglApi {
  EglDisplay (*get_display)(void *native_display) = nullptr;
  EglBoolean (*initialize)(EglDisplay, EglInt *major, EglInt *minor) = nullptr;
  EglInt (*get_error)() = nullptr;
  const char *(*query_string)(EglDisplay, EglInt name) = nullptr;
  EglProcFn (*get_proc_address)(const char *name) = nullptr;

  // EGL_EXT_device_enumeration / EGL_EXT_platform_base. Absent on old or
  // cut-down drivers, which is not fatal — the EGL_DEFAULT_DISPLAY path still
  // reaches a verdict.
  EglBoolean (*query_devices)(EglInt max, EglDeviceExt *devices,
                              EglInt *count) = nullptr;
  EglDisplay (*get_platform_display)(EglEnum platform, void *native,
                                     const EglInt *attribs) = nullptr;
};

/// Look up @p name in @p handle as a function pointer.
///
/// The integer round-trip is the standard way to launder dlsym's `void *`:
/// casting an object pointer straight to a function pointer is ill-formed in
/// ISO C++ even though POSIX requires it to work.
template <typename Fn> Fn symbol(void *handle, const char *name) {
  return reinterpret_cast<Fn>(
      reinterpret_cast<std::uintptr_t>(dlsym(handle, name)));
}

/// Load libEGL and resolve the entry points.
/// @return nullptr when EGL is absent or missing a core entry point, with the
///         reason in @p why.
void *load_egl(EglApi &api, std::string &why) {
  // libEGL.so.1 is the versioned SONAME every runtime ships (Mesa, libglvnd,
  // the NVIDIA driver); libEGL.so is the development symlink and only exists
  // where the -dev package is installed.
  void *handle = dlopen("libEGL.so.1", RTLD_LAZY | RTLD_LOCAL);
  if (handle == nullptr)
    handle = dlopen("libEGL.so", RTLD_LAZY | RTLD_LOCAL);
  if (handle == nullptr) {
    const char *err = dlerror();
    why = std::string("libEGL could not be loaded (") +
          (err != nullptr ? err : "no dlerror") + ")";
    return nullptr;
  }

  api.get_display = symbol<decltype(api.get_display)>(handle, "eglGetDisplay");
  api.initialize = symbol<decltype(api.initialize)>(handle, "eglInitialize");
  api.get_error = symbol<decltype(api.get_error)>(handle, "eglGetError");
  api.query_string =
      symbol<decltype(api.query_string)>(handle, "eglQueryString");
  api.get_proc_address =
      symbol<decltype(api.get_proc_address)>(handle, "eglGetProcAddress");

  if (api.get_display == nullptr || api.initialize == nullptr ||
      api.get_error == nullptr) {
    why = "libEGL loaded but is missing core entry points "
          "(eglGetDisplay / eglInitialize / eglGetError)";
    dlclose(handle);
    return nullptr;
  }

  if (api.get_proc_address != nullptr) {
    api.query_devices = reinterpret_cast<decltype(api.query_devices)>(
        api.get_proc_address("eglQueryDevicesEXT"));
    api.get_platform_display =
        reinterpret_cast<decltype(api.get_platform_display)>(
            api.get_proc_address("eglGetPlatformDisplayEXT"));
  }
  return handle;
}

/// EGL's last error as hex, which is how the registry documents the codes.
std::string egl_error_string(const EglApi &api) {
  const EglInt code = api.get_error();
  if (code == kEglSuccess)
    return "EGL_SUCCESS";
  char buffer[16];
  std::snprintf(buffer, sizeof(buffer), "0x%04X",
                static_cast<unsigned>(code) & 0xFFFFU);
  return buffer;
}

/// A breadcrumb for the failure message: DRM render nodes, if any.
///
/// Decides nothing — a render node is neither necessary (software rasteriser)
/// nor sufficient (no driver) — but it is the first thing anyone diagnosing
/// this will want to know.
std::string render_node_summary() {
  std::error_code ec;
  int nodes = 0;
  for (const auto &entry :
       std::filesystem::directory_iterator("/dev/dri", ec)) {
    if (entry.path().filename().string().starts_with("renderD"))
      ++nodes;
  }
  if (ec)
    return "/dev/dri is not readable";
  return std::to_string(nodes) + " DRM render node(s) in /dev/dri";
}

/// True when @p name is set to something non-empty.
bool env_set(const char *name) {
  const char *value = std::getenv(name);
  return value != nullptr && value[0] != '\0';
}

/// Try to initialise every enumerated EGL device, newest extension path first.
/// @return true on the first device that initialises, filling @p result.
bool try_enumerated_devices(const EglApi &api, OffscreenGlProbe &result) {
  if (api.query_devices == nullptr || api.get_platform_display == nullptr)
    return false;

  EglInt count = 0;
  if (api.query_devices(0, nullptr, &count) != kEglTrue || count <= 0) {
    result.devices = 0;
    return false;
  }
  result.devices = static_cast<int>(count);

  std::vector<EglDeviceExt> devices(static_cast<std::size_t>(count), nullptr);
  EglInt returned = 0;
  if (api.query_devices(count, devices.data(), &returned) != kEglTrue)
    return false;

  for (EglInt i = 0; i < returned; ++i) {
    EglDeviceExt device = devices[static_cast<std::size_t>(i)];
    EglDisplay display =
        api.get_platform_display(kEglPlatformDeviceExt, device, nullptr);
    if (display == kEglNoDisplay)
      continue;

    EglInt major = 0;
    EglInt minor = 0;
    if (api.initialize(display, &major, &minor) != kEglTrue)
      continue;

    const char *vendor = api.query_string != nullptr
                             ? api.query_string(display, kEglVendor)
                             : nullptr;
    result.status = OffscreenGlStatus::usable;
    result.detail = "EGL " + std::to_string(major) + "." +
                    std::to_string(minor) + " initialised on device " +
                    std::to_string(i + 1) + "/" + std::to_string(returned) +
                    " (vendor " + (vendor != nullptr ? vendor : "unknown") +
                    ")";
    return true;
  }
  return false;
}

} // namespace

bool display_configured() noexcept {
  return env_set("DISPLAY") || env_set("WAYLAND_DISPLAY");
}

OffscreenGlProbe probe_offscreen_gl() noexcept try {
  OffscreenGlProbe result;

  if (env_set("REUSEX_SKIP_EGL_PROBE")) {
    result.status = OffscreenGlStatus::unknown;
    result.detail = "probe disabled by REUSEX_SKIP_EGL_PROBE";
    return result;
  }

  EglApi api;
  std::string why;
  // Deliberately never dlclose()d on the success path: unloading a GL driver
  // that has registered atexit handlers and TLS destructors is a well-known
  // way to crash at shutdown, and VTK is about to load the same library.
  if (load_egl(api, why) == nullptr) {
    result.status = OffscreenGlStatus::unknown;
    result.detail = why;
    return result;
  }

  const char *client_extensions =
      api.query_string != nullptr
          ? api.query_string(kEglNoDisplay, kEglExtensions)
          : nullptr;

  // The enumeration path covers a headless GPU and a software rasteriser
  // alike: Mesa reports a software device here when there is no DRM node, and
  // vendor drivers report their devices whether or not a node is visible.
  if (try_enumerated_devices(api, result))
    return result;

  // Fall back to the default display — reached when the enumeration extension
  // is missing, or when every enumerated device refused. Some vendor drivers,
  // and forwarded/remote setups, only answer here.
  EglDisplay display = api.get_display(kEglDefaultDisplay);
  if (display != kEglNoDisplay) {
    EglInt major = 0;
    EglInt minor = 0;
    if (api.initialize(display, &major, &minor) == kEglTrue) {
      result.status = OffscreenGlStatus::usable;
      result.detail = "EGL " + std::to_string(major) + "." +
                      std::to_string(minor) +
                      " initialised on EGL_DEFAULT_DISPLAY";
      return result;
    }
  }

  result.status = OffscreenGlStatus::unusable;
  result.detail =
      "EGL is present but no display could be initialised: " +
      (result.devices >= 0 ? std::to_string(result.devices) + " EGL device(s)"
                           : std::string("device enumeration unavailable")) +
      ", last error " + egl_error_string(api) + ", " + render_node_summary() +
      ", client extensions: " +
      (client_extensions != nullptr && client_extensions[0] != '\0'
           ? client_extensions
           : "(none)");
  return result;

} catch (...) {
  // noexcept: a probe that cannot answer must say so, never propagate.
  OffscreenGlProbe result;
  result.status = OffscreenGlStatus::unknown;
  result.detail = "the EGL probe itself failed unexpectedly";
  return result;
}

} // namespace reusex::visualize
