// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "view/turntable.hpp"

#include "processing_observer.hpp"
#include "view/panorama_handler.hpp" // viewport_renderer()

#include <pcl/visualization/keyboard_event.h>
#include <spdlog/spdlog.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>

#include <chrono>
#include <cmath>
#include <memory>

namespace rux::view {

namespace {

using ViewerPtr = rux::VizualizationObserver::ViewerPtr;

// Forward declaration — defined below.
void start_turntable_tick(std::shared_ptr<TurntableState> state,
                          rux::VizualizationObserver &observer);

/// One iteration of the camera orbit loop.
///
/// Updates the camera position, then re-enqueues itself so the next spin
/// cycle (≈100 ms) carries it forward.  The chain stops naturally when
/// state->active becomes false.
void turntable_tick(std::shared_ptr<TurntableState> state,
                    rux::VizualizationObserver &observer, const ViewerPtr &v,
                    const std::vector<int> &vp) {
  if (!state->active) {
    state->tick_running = false;
    return;
  }

  auto now = std::chrono::steady_clock::now();
  if (state->spinning) {
    double elapsed =
        std::chrono::duration<double>(now - state->last_tick).count();
    // Cap the first tick or any tick after a long pause so the camera
    // doesn't lurch.
    if (elapsed > 0.5)
      elapsed = 0.05;
    state->azimuth += elapsed * state->angular_velocity;

    const auto pos = orbit_camera_position(
        state->center, state->radius, state->azimuth, state->elevation_deg);
    vtkRenderer *renderer = viewport_renderer(v, vp[0]);
    vtkCamera *cam = renderer->GetActiveCamera();
    cam->SetPosition(pos[0], pos[1], pos[2]);
    cam->SetFocalPoint(state->center[0], state->center[1], state->center[2]);
    cam->SetViewUp(0.0, 0.0, 1.0);
    renderer->ResetCameraClippingRange();
  }
  state->last_tick = now;

  start_turntable_tick(state, observer);
}

/// Enqueue the next turntable tick into the viewer task queue.
void start_turntable_tick(std::shared_ptr<TurntableState> state,
                          rux::VizualizationObserver &observer) {
  observer.viewer_enqueue_task(
      [state, &observer](const ViewerPtr &v, const std::vector<int> &vp) {
        turntable_tick(state, observer, v, vp);
      });
}

} // namespace

void register_turntable_callbacks(std::shared_ptr<TurntableState> state,
                                  rux::VizualizationObserver &observer) {
  // 'k' — toggle turntable on/off.
  // 'o'/'O' is PCL-reserved (perspective/parallel projection toggle);
  // 't'/'T' falls through to VTK's Superclass::OnChar (joystick/trackball).
  observer.viewer_enqueue_task([state, &observer](const ViewerPtr &viewer,
                                                  const std::vector<int> &) {
    viewer->registerKeyboardCallback(
        [state, &observer](const pcl::visualization::KeyboardEvent &event) {
          if (event.getKeySym() != "k" || !event.keyDown())
            return;

          state->active = !state->active;

          if (state->active) {
            // Initialise orbit parameters inside a viewer task so we have
            // access to the renderer and can read the scene bounds.
            observer.viewer_enqueue_task([state, &observer](
                                             const ViewerPtr &v,
                                             const std::vector<int> &vp) {
              vtkRenderer *renderer = viewport_renderer(v, vp[0]);

              // Compute the scene bounding box without moving the camera.
              double bounds[6];
              renderer->ComputeVisiblePropBounds(bounds);

              // Guard against an empty scene (all-infinity bounds).
              const bool valid = bounds[0] < bounds[1] &&
                                 bounds[2] < bounds[3] && bounds[4] < bounds[5];

              if (valid) {
                state->center = {(bounds[0] + bounds[1]) / 2.0,
                                 (bounds[2] + bounds[3]) / 2.0,
                                 (bounds[4] + bounds[5]) / 2.0};
                const double dx = bounds[1] - bounds[0];
                const double dy = bounds[3] - bounds[2];
                const double dz = bounds[5] - bounds[4];
                const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
                state->radius = std::max(diag * 0.7, 1.0);
              }

              // Seed the starting azimuth from the current camera
              // position so the orbit begins from where the user was.
              vtkCamera *cam = renderer->GetActiveCamera();
              double pos[3];
              cam->GetPosition(pos);
              state->azimuth = std::atan2(pos[1] - state->center[1],
                                          pos[0] - state->center[0]);
              state->spinning = true;
              state->last_tick = std::chrono::steady_clock::now();

              if (!state->tick_running) {
                state->tick_running = true;
                start_turntable_tick(state, observer);
              }

              spdlog::info("Turntable: on  (k=off  Space=pause  ,/.=speed)");
            });
          } else {
            spdlog::info("Turntable: off");
          }
        });
  });

  // Space — pause/resume spinning while turntable is active.
  observer.viewer_enqueue_task(
      [state](const ViewerPtr &viewer, const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [state](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() != "space" || !event.keyDown())
                return;
              if (!state->active)
                return;

              state->spinning = !state->spinning;
              if (state->spinning) {
                state->last_tick = std::chrono::steady_clock::now();
                spdlog::info("Turntable: spinning");
              } else {
                spdlog::info("Turntable: paused");
              }
            });
      });

  // ',' — decrease angular velocity.
  observer.viewer_enqueue_task(
      [state](const ViewerPtr &viewer, const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [state](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() != "comma" || !event.keyDown())
                return;
              if (!state->active)
                return;
              state->angular_velocity =
                  std::max(0.05, state->angular_velocity * 0.8);
              spdlog::info("Turntable speed: {:.2f} rad/s",
                           state->angular_velocity);
            });
      });

  // '.' — increase angular velocity.
  observer.viewer_enqueue_task(
      [state](const ViewerPtr &viewer, const std::vector<int> &) {
        viewer->registerKeyboardCallback(
            [state](const pcl::visualization::KeyboardEvent &event) {
              if (event.getKeySym() != "period" || !event.keyDown())
                return;
              if (!state->active)
                return;
              state->angular_velocity =
                  std::min(5.0, state->angular_velocity * 1.25);
              spdlog::info("Turntable speed: {:.2f} rad/s",
                           state->angular_velocity);
            });
      });
}

} // namespace rux::view
