// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewportView.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/types/point_types.hpp>
#include <reusex/visualize/render_view.hpp>

#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QResizeEvent>
#include <QStackedLayout>
#include <QTimer>
#include <QVTKOpenGLNativeWidget.h>

#include <opencv2/core.hpp>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkUnsignedCharArray.h>

#include <array>
#include <cstdio>
#include <exception>
#include <optional>

namespace rux::qt {

namespace vis = reusex::visualize;

struct ViewportView::Impl {
  const reusex::ProjectDB *db = nullptr;
  bool interactive = false;
  QString layer = "cloud";
  QString empty;
  qint64 render_ms = 0;
  QSize rendered_size;

  QStackedLayout *stack = nullptr;
  QLabel *image = nullptr;
  QLabel *message = nullptr;
  QVTKOpenGLNativeWidget *gl = nullptr;
  vtkRenderer *renderer = nullptr; // owned by the render window
  QTimer *debounce = nullptr;
};

namespace {

std::array<double, 3> canvas_rgb() {
  const QColor c = theme().color("--color-canvas");
  return {c.redF(), c.greenF(), c.blueF()};
}

std::optional<vis::Layer> to_layer(const QString &name) {
  return vis::layer_from_string(name.toStdString());
}

/// The interactive scene: the geometry cloud's points in their stored RGB.
/// Q3 replaces this with render_view's own scene builder.
void build_points_scene(const reusex::ProjectDB &db, vtkRenderer *r) {
  const reusex::CloudPtr cloud = db.point_cloud_xyzrgb("cloud");
  vtkNew<vtkPoints> pts;
  vtkNew<vtkUnsignedCharArray> rgb;
  rgb->SetNumberOfComponents(3);
  vtkNew<vtkCellArray> verts;
  pts->SetNumberOfPoints(static_cast<vtkIdType>(cloud->size()));
  rgb->SetNumberOfTuples(static_cast<vtkIdType>(cloud->size()));
  for (vtkIdType i = 0; i < static_cast<vtkIdType>(cloud->size()); ++i) {
    const auto &p = (*cloud)[static_cast<std::size_t>(i)];
    pts->SetPoint(i, p.x, p.y, p.z);
    const unsigned char c[3] = {p.r, p.g, p.b};
    rgb->SetTypedTuple(i, c);
    verts->InsertNextCell(1, &i);
  }
  vtkNew<vtkPolyData> poly;
  poly->SetPoints(pts);
  poly->SetVerts(verts);
  poly->GetPointData()->SetScalars(rgb);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(poly);
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(2.0);
  r->AddActor(actor);
  r->ResetCamera();
  vtkCamera *cam = r->GetActiveCamera();
  cam->SetViewUp(0, 0, 1);
  cam->Elevation(-60);
  cam->Azimuth(30);
  r->ResetCamera();
}

} // namespace

ViewportView::ViewportView(const reusex::ProjectDB *db, bool interactive,
                           QWidget *parent)
    : QWidget(parent), impl_(std::make_unique<Impl>()) {
  setObjectName("viewport");
  setAttribute(Qt::WA_StyledBackground);
  impl_->db = db;
  impl_->interactive = interactive;
  impl_->stack = new QStackedLayout(this);
  impl_->stack->setContentsMargins(0, 0, 0, 0);

  impl_->message = new QLabel;
  impl_->message->setObjectName("viewportEmpty");
  impl_->message->setAlignment(Qt::AlignCenter);
  impl_->message->setWordWrap(true);
  impl_->stack->addWidget(impl_->message);

  if (!db) {
    impl_->empty = "Intet projekt åbent";
  } else if (!db->has_point_cloud("cloud")) {
    impl_->empty = "Ingen punktsky endnu — kør rux create clouds";
  }

  if (interactive && impl_->empty.isEmpty()) {
    impl_->gl = new QVTKOpenGLNativeWidget;
    vtkNew<vtkGenericOpenGLRenderWindow> win;
    impl_->gl->setRenderWindow(win);
    vtkNew<vtkRenderer> r;
    const auto bg = canvas_rgb();
    r->SetBackground(bg[0], bg[1], bg[2]);
    build_points_scene(*db, r);
    win->AddRenderer(r);
    impl_->renderer = r;
    impl_->stack->addWidget(impl_->gl);
    impl_->stack->setCurrentWidget(impl_->gl);
  } else {
    impl_->image = new QLabel;
    impl_->image->setObjectName("viewportImage");
    impl_->image->setAlignment(Qt::AlignCenter);
    impl_->image->setMinimumSize(1, 1);
    impl_->image->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    impl_->stack->addWidget(impl_->image);
    impl_->debounce = new QTimer(this);
    impl_->debounce->setSingleShot(true);
    impl_->debounce->setInterval(0);
    connect(impl_->debounce, &QTimer::timeout, this, [this] {
      const QSize logical = size();
      if (logical.isEmpty() || !impl_->empty.isEmpty())
        return;
      const qreal dpr = devicePixelRatioF();
      vis::RenderOptions o;
      if (auto l = to_layer(impl_->layer))
        o.layers = {*l};
      o.view = vis::ViewPreset::orbit;
      o.orbit_index = 0;
      o.cut = true;
      o.width = static_cast<int>(logical.width() * dpr);
      o.height = static_cast<int>(logical.height() * dpr);
      o.background = canvas_rgb();
      o.point_size = 1.5 * dpr;
      o.margin = 0.72;
      QElapsedTimer t;
      t.start();
      try {
        const cv::Mat bgr = vis::render_view(*impl_->db, o);
        QImage img(bgr.data, bgr.cols, bgr.rows, static_cast<int>(bgr.step),
                   QImage::Format_BGR888);
        QPixmap pm = QPixmap::fromImage(img.copy());
        pm.setDevicePixelRatio(dpr);
        impl_->image->setPixmap(pm);
        impl_->stack->setCurrentWidget(impl_->image);
        impl_->rendered_size = logical;
      } catch (const vis::OffscreenGlUnavailable &e) {
        impl_->empty = "3D kræver OpenGL — ingen offscreen-kontekst her";
        std::fprintf(stderr, "rux-qt: ERROR %s\n", e.what());
      } catch (const std::exception &e) {
        impl_->empty = QString("Kunne ikke tegne: %1").arg(e.what());
        std::fprintf(stderr, "rux-qt: ERROR render: %s\n", e.what());
      }
      impl_->render_ms = t.elapsed();
      if (!impl_->empty.isEmpty()) {
        impl_->message->setText(impl_->empty);
        impl_->stack->setCurrentWidget(impl_->message);
      }
      emit rendered();
    });
    // The canvas colour comes from a token: re-render when it changes.
    connect(&theme(), &Theme::changed, this, [this] {
      impl_->rendered_size = {};
      impl_->debounce->start();
    });
  }

  if (!impl_->empty.isEmpty()) {
    impl_->message->setText(impl_->empty);
    impl_->stack->setCurrentWidget(impl_->message);
  }
}

ViewportView::~ViewportView() = default;

void ViewportView::set_layer(const QString &layer) {
  impl_->layer = layer;
  impl_->rendered_size = {};
  if (impl_->debounce)
    impl_->debounce->start();
}

qint64 ViewportView::last_render_ms() const { return impl_->render_ms; }
QString ViewportView::empty_reason() const { return impl_->empty; }

void ViewportView::resizeEvent(QResizeEvent *e) {
  QWidget::resizeEvent(e);
  if (impl_->debounce && e->size() != impl_->rendered_size)
    impl_->debounce->start();
}

} // namespace rux::qt
