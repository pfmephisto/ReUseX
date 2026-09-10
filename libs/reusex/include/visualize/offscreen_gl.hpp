// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Can this machine create an offscreen OpenGL context? (#313)
//
// VTK's headless path ends in vtkEGLRenderWindow, and on a machine where EGL
// has nothing to render on — a GitHub-hosted CI runner, a nix build sandbox —
// that render window crashes on its first Render() instead of reporting the
// failure. Everything downstream of it (the SupportsOpenGL() guard in
// render_view(), the "blank frame" check, the caller's try/catch) is therefore
// unreachable, and STANDARDS §5 is served by a segfault.
//
// This header answers the question *before* any VTK object is touched, and it
// answers it by asking EGL rather than by looking at the filesystem. That
// distinction is the whole point: the obvious probe — "is there a
// /dev/dri/renderD* node?" — reports failure on two configurations that render
// perfectly well.
//
//   - A software rasteriser (Mesa llvmpipe) has no DRM node at all. It still
//     exposes an EGL device, and EGL still initialises on it.
//   - A remote or containerised GPU can be reachable through a vendor EGL
//     driver with no local render node bind-mounted.
//
// Asking EGL to enumerate its devices and initialise one covers both, and
// fails only when there is genuinely nothing to render on.

#include <string>

namespace reusex::visualize {

/// What the probe could establish about offscreen OpenGL on this machine.
enum class OffscreenGlStatus {
  /// EGL enumerated a device and initialised a display on it. Rendering can
  /// proceed.
  usable,
  /// EGL is present and was asked, and reports nothing it can render on.
  /// This is the condition that used to crash.
  unusable,
  /// The probe could not reach a verdict — libEGL is absent, its entry points
  /// are missing, or the probe was switched off. Never a reason to refuse to
  /// render: a VTK built against OSMesa, or one that will use GLX on an X
  /// display, is perfectly capable and simply does not go through EGL.
  unknown,
};

/// The outcome of one probe, with enough detail to put in an error message.
struct OffscreenGlProbe {
  OffscreenGlStatus status = OffscreenGlStatus::unknown;
  /// Human-readable explanation, always non-empty: which device initialised,
  /// or which step failed and with what EGL error.
  std::string detail;
  /// EGL devices enumerated, or -1 when enumeration was not available.
  int devices = -1;
};

/// Ask EGL whether an offscreen context is possible here.
///
/// Loads libEGL at runtime (`dlopen`), enumerates devices via
/// `eglQueryDevicesEXT` and tries `eglInitialize` on each, falling back to
/// `EGL_DEFAULT_DISPLAY`. A display it initialises is left initialised rather
/// than terminated: an EGL display is a process-global singleton that VTK's
/// own `eglInitialize` then simply shares, whereas `eglTerminate` tears down
/// everything on that display and is the less well-behaved half of the pair
/// across drivers. The probe must not be able to break the render it is
/// clearing the way for.
///
/// Runtime loading rather than a link-time dependency is deliberate: whether
/// EGL exists at all is exactly what is being asked, the answer is a property
/// of the machine and not of the build, and it keeps a build with no EGL
/// development package from losing the diagnosis it needs most.
///
/// Costs one dlopen and a few EGL calls (single-digit milliseconds). Callers
/// that render in a loop should cache the result; it cannot change within a
/// process.
///
/// Setting the environment variable `REUSEX_SKIP_EGL_PROBE` to a non-empty
/// value makes this return OffscreenGlStatus::unknown without touching EGL —
/// an escape hatch for a configuration the probe judges wrongly, which lets
/// the old (crash-prone) behaviour be reached deliberately rather than by
/// accident.
///
/// Never throws: a probe that fails is a verdict, not an error.
OffscreenGlProbe probe_offscreen_gl() noexcept;

/// True when a window-system display is configured (`DISPLAY` or
/// `WAYLAND_DISPLAY`).
///
/// VTK prefers a windowed GL path when one of these is set, and only falls
/// back to EGL if it cannot be reached — so on such a machine an unusable EGL
/// says nothing about whether rendering will work, and must not be treated as
/// fatal.
bool display_configured() noexcept;

} // namespace reusex::visualize
