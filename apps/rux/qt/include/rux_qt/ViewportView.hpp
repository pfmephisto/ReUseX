// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A 3D pane over a project's point cloud, in one of two modes:
//
//  - snapshot (default): VTK renders offscreen — through EGL when there is
//    no display — via reusex::visualize::render_view(), and the image is
//    shown in the pane. This is the only mode that works under
//    QT_QPA_PLATFORM=offscreen, where QVTKOpenGLNativeWidget has no GL
//    context and grabs blank. It is what every gallery screenshot uses.
//  - interactive (`gl`): a real QVTKOpenGLNativeWidget. Needs a display;
//    `qt_shot.sh --gl` runs it under xvfb-run.
//
// The canvas colour is --color-canvas in both modes and both themes.
//
// Q0 scope: the interactive mode draws the cloud's points with a reset
// camera. Q3 extracts render_view's scene builder (populate_scene) so both
// modes draw the same layers from the same code.

#include <QWidget>

#include <memory>
#include <string>

namespace reusex {
class ProjectDB;
}

namespace rux::qt {

class ViewportView : public QWidget {
  Q_OBJECT
    public:
  /// @param db  the open project, or nullptr for the empty state. Not owned;
  ///            must outlive the view.
  ViewportView(const reusex::ProjectDB *db, bool interactive,
               QWidget *parent = nullptr);
  ~ViewportView() override;

  /// The named layer to draw: "cloud" (stored RGB) or a Label cloud name
  /// ("planes", "rooms", "instances", "labels").
  void set_layer(const QString &layer);

  /// Milliseconds the last snapshot render took (0 in interactive mode).
  qint64 last_render_ms() const;
  /// Why there is nothing to show; empty when the view rendered.
  QString empty_reason() const;

    signals:
  void rendered();

    protected:
  void resizeEvent(QResizeEvent *) override;

    private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rux::qt
