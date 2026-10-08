// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// "3D" placeholder: the project's cloud on the near-black canvas, rendered by
// VTK offscreen (EGL) into the pane, with the layer panel the Q3 workspace
// will grow. Proves the 3D half of the screenshot loop.

#include "demo_pages.hpp"

#include <rux_qt/Theme.hpp>
#include <rux_qt/ViewportView.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace rux::qt::gallery {
namespace {

QWidget *segmented(const QStringList &options, int checked) {
  auto *w = new QFrame;
  w->setObjectName("segmented");
  auto *l = new QHBoxLayout(w);
  l->setContentsMargins(0, 0, 0, 0);
  l->setSpacing(0);
  auto *group = new QButtonGroup(w);
  for (int i = 0; i < options.size(); ++i) {
    auto *b = new QPushButton(NavItem::escape_mnemonic(options[i]));
    b->setObjectName("segment");
    b->setCheckable(true);
    b->setChecked(i == checked);
    b->setProperty("position", i == 0                    ? "first"
                               : i == options.size() - 1 ? "last"
                                                         : "middle");
    group->addButton(b, i);
    l->addWidget(b);
  }
  return w;
}

QWidget *layer_panel(const PageContext &ctx) {
  auto *w = new QFrame;
  w->setObjectName("inspector");
  auto *l = new QVBoxLayout(w);
  const int pad = theme().px("--space-4");
  l->setContentsMargins(pad, pad, pad, pad);
  l->setSpacing(theme().px("--space-4"));
  l->addWidget(new CapsLabel("Lag", "eyebrowSurface", "--tracking-wide"));

  auto has = [&](const char *n) {
    return ctx.db && ctx.db->has_point_cloud(n);
  };
  auto *layers = new QVBoxLayout;
  layers->setSpacing(theme().px("--space-2"));
  struct Row {
    const char *cloud;
    const char *label;
  };
  for (const Row r : {Row{"cloud", "Punktsky"}, Row{"planes", "Planer"},
                      Row{"rooms", "Rum"}, Row{"instances", "Instanser"}}) {
    auto *row = new QHBoxLayout;
    row->setSpacing(theme().px("--space-2"));
    auto *c = new QCheckBox(r.label);
    c->setEnabled(has(r.cloud));
    c->setChecked(QString(r.cloud) == "cloud" && has(r.cloud));
    row->addWidget(c);
    if (!has(r.cloud)) {
      // Say why it is disabled, not just that it is.
      auto *why = new QLabel("ikke kørt endnu");
      why->setObjectName("layerHint");
      row->addWidget(why);
    }
    row->addStretch(1);
    layers->addLayout(row);
  }
  l->addLayout(layers);

  auto *colour = new QVBoxLayout;
  colour->setSpacing(theme().px("--space-1"));
  colour->addWidget(new CapsLabel("Farv efter", "fieldLabel"));
  auto *by = new QComboBox;
  by->addItems({"Lagret RGB", "Planer", "Rum", "Instanser"});
  colour->addWidget(by);
  l->addLayout(colour);

  auto *cut = new QVBoxLayout;
  cut->setSpacing(theme().px("--space-1"));
  cut->addWidget(new CapsLabel("Snitplan", "fieldLabel"));
  auto *cutOn = new QCheckBox("Skjul over 1,2 m (loft)");
  cutOn->setChecked(true);
  cut->addWidget(cutOn);
  l->addLayout(cut);

  auto *info = new Panel("Sky", nullptr, "well");
  auto *props = new PropertyList;
  if (has("cloud")) {
    for (const auto &c : ctx.db->project_summary().clouds)
      if (c.name == "cloud") {
        props->add("Punkter", format_count(c.point_count));
        props->add("Orden", c.storage_order.empty()
                                ? QString("sekventiel")
                                : QString::fromStdString(c.storage_order));
      }
  } else {
    props->add("Punkter", "—");
  }
  info->body()->addWidget(props);
  l->addWidget(info);
  l->addStretch(1);
  return w;
}

} // namespace

QWidget *make_viewport_page(const PageContext &ctx) {
  auto *content = new QWidget;
  content->setObjectName("content");
  auto *v = new QVBoxLayout(content);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  // Toolbar over the canvas: view presets + actions.
  auto *bar = new QFrame;
  bar->setObjectName("toolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(theme().px("--space-4"), theme().px("--space-2"),
                         theme().px("--space-4"), theme().px("--space-2"));
  bl->setSpacing(theme().px("--space-3"));
  bl->addWidget(new CapsLabel("3D", "toolbarTitle", QString()));
  bl->addWidget(segmented({"Bane", "Plan", "Top", "Front"}, 0));
  bl->addStretch(1);
  auto *reset = new QPushButton("Nulstil kamera");
  reset->setProperty("kind", "ghost");
  bl->addWidget(reset);
  auto *shot = new QPushButton("Gem billede");
  shot->setProperty("kind", "secondary");
  bl->addWidget(shot);
  v->addWidget(bar);

  auto *view = new ViewportView(ctx.db, ctx.interactive_3d);
  v->addWidget(view, 1);

  auto *status = new QFrame;
  status->setObjectName("statusBar");
  auto *sl = new QHBoxLayout(status);
  sl->setContentsMargins(theme().px("--space-4"), theme().px("--space-1"),
                         theme().px("--space-4"), theme().px("--space-1"));
  sl->setSpacing(theme().px("--space-3"));
  auto *left = new QLabel;
  left->setObjectName("statusText");
  auto *right = new QLabel(ctx.interactive_3d ? "QVTKOpenGLNativeWidget"
                                              : "VTK · EGL offscreen");
  right->setObjectName("statusMono");
  sl->addWidget(left);
  sl->addStretch(1);
  sl->addWidget(right);
  v->addWidget(status);

  QObject::connect(view, &ViewportView::rendered, left, [view, left] {
    left->setText(view->empty_reason().isEmpty()
                      ? QString("Bane 1/8 · snit 1,2 m · tegnet på %1 ms")
                            .arg(view->last_render_ms())
                      : view->empty_reason());
  });
  if (!view->empty_reason().isEmpty())
    left->setText(view->empty_reason());
  else if (ctx.interactive_3d)
    left->setText("Interaktiv · træk for at dreje, scroll for at zoome");

  return demo_frame(ctx, 1, content, layer_panel(ctx));
}

void register_demo_pages() {
  register_page({"components", "Alle delte komponenter med projektdata",
                 make_components_page});
  register_page({"viewport", "3D-pladsholder: punktskyen via VTK EGL offscreen",
                 make_viewport_page});
}

} // namespace rux::qt::gallery
