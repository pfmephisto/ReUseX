// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>

// Forward-declare to avoid pulling PCL/VTK into the math-only header.
namespace rux {
class VizualizationObserver;
} // namespace rux

namespace rux::view {

/// Shared state for the turntable orbit mode.
struct TurntableState {
  bool active = false;           ///< Turntable mode is enabled
  bool spinning = true;          ///< Camera is rotating (can be paused)
  bool tick_running = false;     ///< Whether the per-tick task chain is live
  double azimuth = 0.0;          ///< Current angle around world Z, radians
  double angular_velocity = 0.3; ///< rad/s
  double radius = 5.0;           ///< Camera distance from scene centre
  double elevation_deg = 20.0;   ///< Camera elevation above horizontal plane
  std::array<double, 3> center = {0.0, 0.0, 0.0}; ///< Scene focal point
  std::chrono::steady_clock::time_point last_tick{};
};

/// Compute the camera world position for one turntable orbit step.
///
/// Pure function — no display or state side-effects; directly unit-testable.
/// The camera looks toward @p center from the returned position with Z up.
[[nodiscard]] inline std::array<double, 3>
orbit_camera_position(const std::array<double, 3> &center, double radius,
                      double azimuth_rad, double elevation_deg) noexcept {
  const double el = elevation_deg * (std::numbers::pi / 180.0);
  return {center[0] + radius * std::cos(azimuth_rad) * std::cos(el),
          center[1] + radius * std::sin(azimuth_rad) * std::cos(el),
          center[2] + radius * std::sin(el)};
}

/// Register turntable keyboard callbacks and wire up the per-tick orbit loop.
///
/// Keys (chosen to avoid PCL and VTK base-class reserved bindings):
///   k       — toggle turntable on/off
///             ('o'/'O' is PCL-reserved for projection toggle;
///              't'/'T' falls through to VTK Superclass::OnChar joystick mode)
///   Space   — pause/resume spinning while turntable is active
///   ,       — decrease angular velocity
///   .       — increase angular velocity
void register_turntable_callbacks(std::shared_ptr<TurntableState> state,
                                  rux::VizualizationObserver &observer);

} // namespace rux::view
