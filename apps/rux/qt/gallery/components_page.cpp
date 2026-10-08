// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// "Komponenter": every shared building block on one screen, filled from the
// project copy — the page a design change is checked against first.

#include "demo_pages.hpp"

#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/types/point_types.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTableView>
#include <QVBoxLayout>

#include <algorithm>
#include <map>

namespace rux::qt::gallery {
namespace {

QPushButton *button(const QString &text, const QString &kind) {
  auto *b = new QPushButton(NavItem::escape_mnemonic(text));
  b->setProperty("kind", kind);
  b->setCursor(Qt::PointingHandCursor);
  return b;
}

QWidget *view_head(const QString &title, const QString &sub,
                   QList<QWidget *> actions) {
  auto *w = new QWidget;
  auto *l = new QHBoxLayout(w);
  l->setContentsMargins(0, 0, 0, 0);
  l->setSpacing(theme().px("--space-3"));
  auto *t = new CapsLabel(title, "viewTitle", QString());
  l->addWidget(t, 0, Qt::AlignBottom);
  auto *s = new QLabel(sub);
  s->setObjectName("viewSub");
  l->addWidget(s, 0, Qt::AlignBottom);
  l->addStretch(1);
  for (QWidget *a : actions)
    l->addWidget(a, 0, Qt::AlignVCenter);
  return w;
}

QString type_name(const std::string &t) {
  if (t == "PointXYZRGB")
    return "Farvet punkt";
  if (t == "Normal")
    return "Normal";
  if (t == "Label")
    return "Mærkat";
  if (t == "PointXYZ")
    return "Punkt";
  return QString::fromStdString(t);
}

QWidget *clouds_table(const reusex::ProjectDB *db) {
  auto *model = new QStandardItemModel(0, 3);
  model->setHorizontalHeaderLabels({"NAVN", "TYPE", "PUNKTER"});
  const QFont mono = theme().font(FontRole::mono, "--font-size-sm");
  if (db) {
    for (const auto &c : db->project_summary().clouds) {
      auto *name = new QStandardItem(QString::fromStdString(c.name));
      auto *type = new QStandardItem(type_name(c.type));
      auto *n = new QStandardItem(format_count(c.point_count));
      n->setFont(mono);
      n->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
      model->appendRow({name, type, n});
    }
  }
  auto *view = new QTableView;
  view->setObjectName("dataTable");
  view->setModel(model);
  model->setParent(view);
  view->verticalHeader()->hide();
  view->setShowGrid(false);
  view->setSelectionBehavior(QAbstractItemView::SelectRows);
  view->setSelectionMode(QAbstractItemView::SingleSelection);
  view->setEditTriggers(QAbstractItemView::NoEditTriggers);
  view->setFocusPolicy(Qt::NoFocus);
  view->horizontalHeader()->setHighlightSections(false);
  view->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft |
                                                Qt::AlignVCenter);
  model->horizontalHeaderItem(2)->setTextAlignment(Qt::AlignRight |
                                                   Qt::AlignVCenter);
  const int row_h = theme().px("--space-6") + theme().px("--space-1");
  view->verticalHeader()->setDefaultSectionSize(row_h);
  auto *h = view->horizontalHeader();
  // QSS has no letter-spacing: the caps tracking goes on the header font,
  // which the stylesheet's size/weight resolve onto.
  QFont hf = h->font();
  hf.setLetterSpacing(QFont::AbsoluteSpacing,
                      theme().em("--tracking-caps") *
                          theme().px("--font-size-2xs"));
  h->setFont(hf);
  h->setSectionResizeMode(0, QHeaderView::Stretch);
  for (int c = 1; c < 3; ++c)
    h->setSectionResizeMode(c, QHeaderView::ResizeToContents);
  view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  if (model->rowCount() > 0)
    view->selectRow(0);
  // Tall enough for every row: a short table should not scroll.
  view->setMinimumHeight(view->horizontalHeader()->sizeHint().height() +
                         row_h * model->rowCount() + 2);
  return view;
}

/// The planes label cloud's eight largest classes on --label-0..7, plus the
/// unlabeled points — real data for the legend.
QVector<LegendEntry> plane_legend(const reusex::ProjectDB *db) {
  QVector<LegendEntry> out;
  if (!db || !db->has_point_cloud("planes"))
    return out;
  const auto labels = db->point_cloud_label("planes");
  std::map<std::uint32_t, qulonglong> counts;
  for (const auto &p : *labels)
    ++counts[p.label];
  qulonglong unlabeled = 0;
  std::vector<std::pair<qulonglong, std::uint32_t>> by_size;
  for (const auto &[label, n] : counts) {
    if (label == 0)
      unlabeled = n;
    else
      by_size.emplace_back(n, label);
  }
  std::sort(by_size.rbegin(), by_size.rend());
  const int shown = std::min<int>(8, static_cast<int>(by_size.size()));
  for (int i = 0; i < shown; ++i)
    out.push_back({QString("Plan %1").arg(by_size[i].second),
                   QString("--label-%1").arg(i),
                   format_count(by_size[i].first)});
  out.push_back({"Umærket", "--label-unlabeled", format_count(unlabeled)});
  return out;
}

QWidget *controls_panel() {
  auto *p = new Panel("Knapper og felter");
  auto *buttons = new QHBoxLayout;
  buttons->setSpacing(theme().px("--space-2"));
  buttons->addWidget(button("Kør pipeline", "primary"));
  buttons->addWidget(button("Eksportér", "secondary"));
  buttons->addWidget(button("Annullér", "ghost"));
  buttons->addWidget(button("Slet kant", "danger"));
  auto *disabled = button("Gem ændringer", "primary");
  disabled->setEnabled(false);
  buttons->addWidget(disabled);
  buttons->addStretch(1);
  p->body()->addLayout(buttons);

  auto *form = new QGridLayout;
  form->setHorizontalSpacing(theme().px("--space-3"));
  form->setVerticalSpacing(theme().px("--space-2"));
  auto add = [&](int row, int col, const QString &label, QWidget *field) {
    auto *box = new QVBoxLayout;
    box->setSpacing(theme().px("--space-1"));
    box->addWidget(new CapsLabel(label, "fieldLabel"));
    box->addWidget(field);
    form->addLayout(box, row, col);
  };
  auto *name = new QLineEdit("Kontor, 2. sal");
  add(0, 0, "Navn", name);
  auto *filter = new QLineEdit;
  filter->setPlaceholderText("Filtrér skyer …");
  add(0, 1, "Søg", filter);
  auto *layer = new QComboBox;
  layer->addItems({"Punktsky (RGB)", "Planer", "Rum", "Instanser"});
  add(1, 0, "Farv efter", layer);
  auto *voxel = new QSpinBox;
  voxel->setRange(5, 200);
  voxel->setValue(20);
  voxel->setSuffix(" mm");
  add(1, 1, "Voxelgitter", voxel);
  form->setColumnStretch(0, 1);
  form->setColumnStretch(1, 1);
  p->body()->addLayout(form);

  auto *checks = new QHBoxLayout;
  checks->setSpacing(theme().px("--space-4"));
  auto *c1 = new QCheckBox("Vis kun mærkede punkter");
  c1->setChecked(true);
  checks->addWidget(c1);
  checks->addWidget(new QCheckBox("Snit i 1,2 m"));
  checks->addStretch(1);
  p->body()->addLayout(checks);

  auto *pills = new QHBoxLayout;
  pills->setSpacing(theme().px("--space-2"));
  pills->addWidget(new Pill("Færdig", "good"));
  pills->addWidget(new Pill("Kører", "accent"));
  pills->addWidget(new Pill("I kø", "wait"));
  pills->addWidget(new Pill("Advarsel", "warn"));
  pills->addWidget(new Pill("Fejlet", "crit"));
  pills->addWidget(new Pill("Udkast", "outline"));
  pills->addStretch(1);
  p->body()->addLayout(pills);
  p->body()->addStretch(1);
  return p;
}

/// The project's pipeline log as job rows (newest last), then the stages
/// that have not run yet — real data, every progress state that exists.
QWidget *jobs_panel(const reusex::ProjectDB *db) {
  auto *p = new Panel("Pipeline-log");
  struct Job {
    QString name, meta, tone, pill, state;
    int pct;
  };
  QVector<Job> jobs;
  QStringList done;
  if (db) {
    auto log = db->pipeline_log(); // newest first
    std::reverse(log.begin(), log.end());
    for (const auto &e : log) {
      const QString st = QString::fromStdString(e.status);
      const QString stage = QString::fromStdString(e.stage);
      done << stage;
      const QString when = QString::fromStdString(e.started_at).mid(11, 5);
      if (st == "success")
        jobs.push_back(
            {stage, "kl. " + when, "good", "Færdig", "succeeded", 100});
      else if (st == "failed")
        jobs.push_back({stage, QString::fromStdString(e.error_msg), "crit",
                        "Fejlet", "failed", 100});
      else
        jobs.push_back(
            {stage, "startet kl. " + when, "accent", "Kører", "running", 50});
    }
  }
  for (const char *next : {"segment_rooms", "mesh_generation"})
    if (!done.contains(next))
      jobs.push_back({next, "ikke kørt", "wait", "Venter", "queued", 0});

  auto *grid = new QGridLayout;
  grid->setHorizontalSpacing(theme().px("--space-3"));
  grid->setVerticalSpacing(theme().px("--space-1"));
  int row = 0;
  for (const auto &j : jobs) {
    auto *name = new QLabel(j.name);
    name->setObjectName("jobName");
    grid->addWidget(name, row, 0);
    auto *meta = new QLabel(j.meta);
    meta->setObjectName("jobMeta");
    grid->addWidget(meta, row, 1);
    grid->addWidget(new Pill(j.pill, j.tone), row, 2, Qt::AlignRight);
    auto *bar = new QProgressBar;
    bar->setRange(0, 100);
    bar->setValue(j.pct);
    bar->setTextVisible(false);
    bar->setProperty("state", j.state);
    grid->addWidget(bar, row + 1, 0, 1, 3);
    grid->setRowMinimumHeight(row + 2, theme().px("--space-2"));
    row += 3;
  }
  grid->setColumnStretch(1, 1);
  p->body()->addLayout(grid);
  p->body()->addStretch(1);
  return p;
}

QWidget *inspector(const PageContext &ctx) {
  auto *w = new QFrame;
  w->setObjectName("inspector");
  auto *l = new QVBoxLayout(w);
  const int pad = theme().px("--space-4");
  l->setContentsMargins(pad, pad, pad, pad);
  l->setSpacing(theme().px("--space-4"));
  l->addWidget(new CapsLabel("Inspektør", "eyebrowSurface", "--tracking-wide"));

  auto *props = new PropertyList;
  if (ctx.db) {
    const auto s = ctx.db->project_summary();
    props->add("Fil", QFileInfo(ctx.project_path).fileName(), false);
    props->add("Skema", QString("v%1").arg(s.schema_version));
    props->add("Billeder", format_count(static_cast<qulonglong>(
                               s.sensor_frames.total_count)));
    props->add("Opløsning", QString("%1 × %2")
                                .arg(s.sensor_frames.width)
                                .arg(s.sensor_frames.height));
    props->add("Skyer", QString::number(s.clouds.size()));
    props->add("Mesh", QString::number(s.meshes.size()));
    props->add("Panoramaer", QString::number(s.panoramic_images.total_count));
  } else {
    props->add("Fil", "—", false);
  }
  l->addWidget(props);

  auto *sel = new Panel("Valgt sky", nullptr, "well");
  auto *sp = new PropertyList;
  if (ctx.db && ctx.db->has_point_cloud("cloud")) {
    const auto s = ctx.db->project_summary();
    for (const auto &c : s.clouds)
      if (c.name == "cloud") {
        sp->add("Navn", "cloud");
        sp->add("Punkter", format_count(c.point_count));
        sp->add("Organiseret", c.organized ? "ja" : "nej", false);
      }
  } else {
    sp->add("Navn", "—");
  }
  sel->body()->addWidget(sp);
  l->addWidget(sel);
  l->addStretch(1);
  auto *open = button("Åbn i 3D", "secondary");
  l->addWidget(open);
  return w;
}

} // namespace

QWidget *make_components_page(const PageContext &ctx) {
  auto *content = new QWidget;
  content->setObjectName("content");
  auto *page = new QVBoxLayout(content);
  const int pad = theme().px("--space-5");
  page->setContentsMargins(pad, pad, pad, pad);
  page->setSpacing(theme().px("--space-4"));

  page->addWidget(view_head(
      "Komponenter",
      "Byggeklodserne i Qt-klienten — samme tokens som web-GUI'en",
      {button("Miljø & prøver", "ghost"), button("Eksportér", "secondary"),
       button("Kør pipeline", "primary")}));

  // KPI row
  qulonglong points = 0, planes = 0;
  int frames = 0, clouds = 0;
  if (ctx.db) {
    const auto s = ctx.db->project_summary();
    frames = s.sensor_frames.total_count;
    clouds = static_cast<int>(s.clouds.size());
    for (const auto &c : s.clouds) {
      if (c.name == "cloud")
        points = c.point_count;
      if (c.name == "plane_centroids")
        planes = c.point_count;
    }
  }
  auto *kpis = new QHBoxLayout;
  kpis->setSpacing(theme().px("--space-3"));
  // No project: a dash, not a misleading zero.
  auto fig = [&](qulonglong n) {
    return ctx.db ? format_count(n) : QString("—");
  };
  kpis->addWidget(
      new StatCard("Punkter", fig(points), {}, "i skyen \"cloud\""));
  kpis->addWidget(new StatCard("Billeder", fig(static_cast<qulonglong>(frames)),
                               {}, "RGB-D-billeder med pose"));
  kpis->addWidget(new StatCard("Planer", fig(planes), {}, "fra create planes"));
  kpis->addWidget(new StatCard("Skyer", fig(static_cast<qulonglong>(clouds)),
                               {}, "navngivne punktskyer"));
  page->addLayout(kpis);

  auto *grid = new QGridLayout;
  grid->setHorizontalSpacing(theme().px("--space-4"));
  grid->setVerticalSpacing(theme().px("--space-4"));
  auto *table = new Panel("Punktskyer");
  table->add_header_widget(
      new Pill(ctx.db ? "Skrivebeskyttet" : "Intet projekt", "outline"));
  table->body()->addWidget(clouds_table(ctx.db));
  grid->addWidget(table, 0, 0);
  grid->addWidget(controls_panel(), 0, 1);
  auto *legend = new Panel("Mærkater · planes");
  const auto entries = plane_legend(ctx.db);
  if (entries.isEmpty()) {
    auto *empty = new QLabel("Ingen planer endnu — kør rux create planes");
    empty->setObjectName("emptyText");
    legend->body()->addWidget(empty);
  } else {
    legend->body()->addWidget(new LabelLegend(entries));
  }
  legend->body()->addStretch(1);
  grid->addWidget(legend, 1, 0);
  grid->addWidget(jobs_panel(ctx.db), 1, 1);
  grid->setColumnStretch(0, 5);
  grid->setColumnStretch(1, 6);
  grid->setRowStretch(0, 1);
  grid->setRowStretch(1, 1);
  page->addLayout(grid, 1);

  return demo_frame(ctx, 5, content, inspector(ctx));
}

} // namespace rux::qt::gallery
