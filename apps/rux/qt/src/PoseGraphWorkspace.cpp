// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/PoseGraphWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/widgets.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QApplication>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLocale>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QPushButton>
#include <QScrollBar>
#include <QStackedLayout>
#include <QStyleOptionGraphicsItem>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <map>
#include <thread>

namespace rux::qt {

namespace {

const QLocale &da() {
  static const QLocale l(QLocale::Danish, QLocale::Denmark);
  return l;
}

/// Screen pixels of a node's radius and of the click tolerance.
constexpr double kNodePx = 1.6;
constexpr double kHitPx = 7.0;

QColor with_alpha(QColor c, double a) {
  c.setAlphaF(static_cast<float>(a));
  return c;
}

int label_slots() {
  bool ok = false;
  const int n = theme().value("--label-count").toInt(&ok);
  return ok && n > 0 ? n : 8;
}

/// Every node in one item: thousands of QGraphicsEllipseItems would make
/// the scene slow to build and to hit-test, and the dots must keep their
/// size on screen at every zoom.
class NodesItem : public QGraphicsItem {
    public:
  NodesItem(const std::vector<GraphNode> &nodes, const std::vector<int> &scans)
      : nodes_(nodes), scans_(scans) {
    for (const auto &n : nodes) {
      const QPointF p(n.x, -n.y);
      bounds_ = bounds_.isNull() ? QRectF(p, QSizeF(0, 0))
                                 : bounds_.united(QRectF(p, QSizeF(0, 0)));
    }
    setZValue(2);
  }
  QRectF boundingRect() const override {
    const double pad = 1.0; // a metre of slack for the screen-sized dots
    return bounds_.adjusted(-pad, -pad, pad, pad);
  }
  void paint(QPainter *p, const QStyleOptionGraphicsItem *option,
             QWidget *) override {
    const double scale =
        option->levelOfDetailFromTransform(p->worldTransform());
    const double r = kNodePx / std::max(scale, 1e-9);
    p->setPen(Qt::NoPen);
    const int nslots = label_slots();
    int last = -1;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      const int scan = scans_.empty() ? 0 : scans_[i];
      if (scan != last) {
        // Scans take the --label-* scale in turn, so two captures read apart.
        // The first scan in the chrome's muted ink, so the coloured edges
        // read on top; further scans take the --label-* scale in turn.
        p->setBrush(scan <= 1 ? theme().color("--color-on-chrome-muted")
                              : theme().color(
                                    QString("--label-%1").arg(scan % nslots)));
        last = scan;
      }
      p->drawEllipse(QPointF(nodes_[i].x, -nodes_[i].y), r, r);
    }
  }

    private:
  std::vector<GraphNode> nodes_;
  std::vector<int> scans_;
  QRectF bounds_;
};

/// The A or B ring, constant size on screen.
class MarkerItem : public QGraphicsItem {
    public:
  explicit MarkerItem(QString letter, QString token)
      : letter_(std::move(letter)), token_(std::move(token)) {
    setFlag(ItemIgnoresTransformations);
    setZValue(5);
  }
  QRectF boundingRect() const override {
    const double r = theme().px("--space-4");
    return {-r, -r * 2.6, r * 2.6, r * 3.6};
  }
  void paint(QPainter *p, const QStyleOptionGraphicsItem *,
             QWidget *) override {
    const Theme &t = theme();
    const double r = t.px("--space-2");
    const QColor c = t.color(token_);
    p->setRenderHint(QPainter::Antialiasing);
    p->setPen(QPen(c, 2.0));
    p->setBrush(Qt::NoBrush);
    p->drawEllipse(QPointF(0, 0), r, r);
    // The letter in a chip above the ring.
    QFont f = t.font(FontRole::mono, "--font-size-xs", "--font-weight-bold");
    p->setFont(f);
    const QRectF chip(-r, -r * 2.0 - t.px("--space-4"), t.px("--space-4"),
                      t.px("--space-4"));
    p->setPen(Qt::NoPen);
    p->setBrush(c);
    p->drawRoundedRect(chip, t.px("--radius-sm"), t.px("--radius-sm"));
    p->setPen(t.color("--color-on-accent"));
    p->drawText(chip, Qt::AlignCenter, letter_);
  }

    private:
  QString letter_;
  QString token_;
};

QString edge_type_da(const std::string &type) {
  if (type == "odometry")
    return "Odometri";
  if (type == "loop_closure")
    return "Løkkelukning";
  if (type == "panorama")
    return "Panorama";
  return QString::fromStdString(type);
}

QString edge_token(const std::string &type) {
  if (type == "loop_closure")
    return "--color-accent";
  if (type == "panorama")
    return "--color-star";
  return "--color-text-faint";
}

} // namespace

// ----------------------------------------------------------- PoseGraphView --

PoseGraphView::PoseGraphView(QWidget *parent) : QGraphicsView(parent) {
  setObjectName("poseGraphView");
  setRenderHint(QPainter::Antialiasing);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  setFrameShape(QFrame::NoFrame);
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setFocusPolicy(Qt::StrongFocus);
}

void PoseGraphView::fit() {
  if (!scene() || content_.isNull())
    return;
  QRectF r = content_;
  const double pad = std::max(r.width(), r.height()) * 0.06 + 0.5;
  r.adjust(-pad, -pad, pad, pad);
  // Let the view pan past the graph's edge a little.
  scene()->setSceneRect(
      r.adjusted(-r.width(), -r.height(), r.width(), r.height()));
  fitInView(r, Qt::KeepAspectRatio);
  user_view_ = false;
}

void PoseGraphView::resizeEvent(QResizeEvent *e) {
  QGraphicsView::resizeEvent(e);
  if (!user_view_)
    fit();
}

double PoseGraphView::units_per_pixel() const {
  const double m = transform().m11();
  return m > 0 ? 1.0 / m : 1.0;
}

void PoseGraphView::wheelEvent(QWheelEvent *e) {
  const double f = std::pow(1.0015, e->angleDelta().y());
  scale(f, f);
  user_view_ = true;
  e->accept();
}

void PoseGraphView::mousePressEvent(QMouseEvent *e) {
  press_ = e->position().toPoint();
  QGraphicsView::mousePressEvent(e);
}

void PoseGraphView::mouseReleaseEvent(QMouseEvent *e) {
  QGraphicsView::mouseReleaseEvent(e);
  const bool click = (e->position().toPoint() - press_).manhattanLength() <=
                     theme().px("--space-1");
  if (!click)
    user_view_ = true; // a drag panned the view
  else if (e->button() == Qt::LeftButton)
    emit clicked_at(mapToScene(e->position().toPoint()), e->modifiers());
}

void PoseGraphView::keyPressEvent(QKeyEvent *e) {
  if (e->key() == Qt::Key_F && e->modifiers() == Qt::NoModifier) {
    fit();
    return;
  }
  QGraphicsView::keyPressEvent(e);
}

// ------------------------------------------------------ PoseGraphWorkspace --

struct PoseGraphWorkspace::Loaded {
  unsigned generation = 0;
  std::vector<GraphNode> nodes;
  std::vector<int> scans;
  int unposed = 0;
  QString error;
};

PoseGraphWorkspace::PoseGraphWorkspace(ProjectSession &session,
                                       EdgeEditor &editor, QWidget *parent)
    : QWidget(parent), session_(session), editor_(editor) {
  setObjectName("poseGraphWorkspace");
  const Theme &t = theme();
  auto *v = new QVBoxLayout(this);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  auto *bar = new QFrame;
  bar->setObjectName("dbToolbar");
  auto *bl = new QHBoxLayout(bar);
  bl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                         t.px("--space-4"), t.px("--space-2"));
  bl->setSpacing(t.px("--space-3"));
  bl->addWidget(new CapsLabel("Posegraf", "panelTitle"));
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  // Edge legend: the stroke colours of the canvas.
  legend_ = new QWidget;
  auto *lg = new QHBoxLayout(legend_);
  lg->setContentsMargins(0, 0, 0, 0);
  lg->setSpacing(t.px("--space-3"));
  for (const auto &[token, name] : std::vector<std::pair<QString, QString>>{
           {"--color-text-faint", "Odometri"},
           {"--color-accent", "Løkkelukning"},
           {"--color-star", "Panorama"},
           {"--color-status-failed", "Slettes"}}) {
    auto *item = new QWidget;
    auto *il = new QHBoxLayout(item);
    il->setContentsMargins(0, 0, 0, 0);
    il->setSpacing(t.px("--space-1"));
    il->addWidget(new Swatch(token));
    auto *l = new QLabel(name);
    l->setObjectName("legendText");
    il->addWidget(l);
    lg->addWidget(item);
  }
  bl->addWidget(legend_);
  auto *fit = new QPushButton(NavItem::escape_mnemonic("Tilpas"));
  fit->setProperty("kind", "ghost");
  fit->setCursor(Qt::PointingHandCursor);
  fit->setToolTip("Vis hele grafen (F)");
  bl->addWidget(fit);
  v->addWidget(bar);

  auto *stack_host = new QWidget;
  auto *stack = new QStackedLayout(stack_host);
  view_ = new PoseGraphView;
  view_->setScene(new QGraphicsScene(view_));
  view_->setBackgroundBrush(t.color("--color-canvas"));
  stack->addWidget(view_);
  empty_ = new QLabel;
  empty_->setObjectName("viewportEmpty");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->setWordWrap(true);
  stack->addWidget(empty_);
  v->addWidget(stack_host, 1);

  auto *status = new QFrame;
  status->setObjectName("statusBar");
  auto *sl = new QHBoxLayout(status);
  sl->setContentsMargins(t.px("--space-4"), t.px("--space-1"),
                         t.px("--space-4"), t.px("--space-1"));
  auto *hint = new QLabel("Klik på en node: billede A · Shift+klik: billede B "
                          "· klik på en kant: A og B · hjul: zoom · træk: "
                          "panorér");
  hint->setObjectName("statusText");
  sl->addWidget(hint, 1);
  v->addWidget(status);

  connect(fit, &QPushButton::clicked, view_, &PoseGraphView::fit);
  connect(view_, &PoseGraphView::clicked_at, this,
          &PoseGraphWorkspace::on_click);
  connect(&editor_, &EdgeEditor::changed, this, [this] {
    rebuild_edges();
    update_meta();
  });
  connect(&session_, &ProjectSession::state_changed, this,
          &PoseGraphWorkspace::reset);
  connect(&theme(), &Theme::changed, this, [this] {
    view_->setBackgroundBrush(theme().color("--color-canvas"));
    rebuild_scene();
  });
  reset();
}

PoseGraphWorkspace::~PoseGraphWorkspace() = default;

void PoseGraphWorkspace::reset() {
  ++generation_;
  loading_ = false;
  nodes_.clear();
  scan_of_.clear();
  selection_ = {};
  rebuild_scene();
  update_meta();
  if (active_)
    activate();
}

void PoseGraphWorkspace::activate() {
  active_ = true;
  if (!session_.is_open() || loading_ || !nodes_.empty())
    return;
  loading_ = true;
  update_meta();
  const std::string path = session_.path().toStdString();
  const unsigned gen = generation_;
  auto work = std::make_shared<BackgroundWork>();
  QPointer<PoseGraphWorkspace> guard(this);
  std::thread([path, gen, guard, work]() mutable {
    auto d = std::make_shared<Loaded>();
    d->generation = gen;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/true);
      auto ids = db.sensor_frame_ids_with_scan();
      std::sort(ids.begin(), ids.end());
      for (const auto &[id, scan] : ids) {
        if (!db.has_sensor_frame_pose(id)) {
          ++d->unposed;
          continue;
        }
        const auto pose = db.sensor_frame_pose(id);
        d->nodes.push_back({id, pose[3], pose[7]});
        d->scans.push_back(scan);
      }
    } catch (const std::exception &e) {
      d->error = QString::fromUtf8(e.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, d] {
          if (guard)
            guard->loaded(d);
        },
        Qt::QueuedConnection);
    work.reset();
  }).detach();
}

void PoseGraphWorkspace::loaded(std::shared_ptr<Loaded> d) {
  if (d->generation != generation_)
    return;
  loading_ = false;
  nodes_ = std::move(d->nodes);
  scan_of_ = std::move(d->scans);
  rebuild_scene();
  if (!d->error.isEmpty())
    empty_->setText(QString("Poserne kunne ikke læses.\n%1").arg(d->error));
  update_meta();
  view_->fit();
}

void PoseGraphWorkspace::rebuild_scene() {
  QGraphicsScene *sc = view_->scene();
  sc->clear();
  edge_items_.clear();
  nodes_item_ = nullptr;
  marker_a_ = marker_b_ = nullptr;
  auto *stack = static_cast<QStackedLayout *>(view_->parentWidget()->layout());
  if (nodes_.empty()) {
    empty_->setText(!session_.is_open() ? QString("Intet projekt åbent")
                    : loading_          ? QString("Læser poserne …")
                                        : QString("Ingen billeder med pose — "
                                                  "importér en scanning"));
    stack->setCurrentWidget(empty_);
    return;
  }
  stack->setCurrentWidget(view_);
  const Theme &t = theme();

  // The capture path, one polyline per scan, under everything.
  QPainterPath path;
  int scan = -1;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const QPointF p(nodes_[i].x, -nodes_[i].y);
    if (scan_of_[i] != scan) {
      path.moveTo(p);
      scan = scan_of_[i];
    } else {
      path.lineTo(p);
    }
  }
  QPen trail(with_alpha(t.color("--color-on-chrome-muted"), 0.25), 1.0);
  trail.setCosmetic(true);
  auto *trail_item = sc->addPath(path, trail);
  trail_item->setZValue(0);

  nodes_item_ = new NodesItem(nodes_, scan_of_);
  sc->addItem(nodes_item_);
  view_->set_content_rect(path.boundingRect());
  marker_a_ = new MarkerItem("A", "--color-accent");
  marker_b_ = new MarkerItem("B", "--label-6");
  sc->addItem(marker_a_);
  sc->addItem(marker_b_);
  rebuild_edges();
  place_markers();
}

void PoseGraphWorkspace::rebuild_edges() {
  QGraphicsScene *sc = view_->scene();
  for (QGraphicsItem *it : edge_items_) {
    sc->removeItem(it);
    delete it;
  }
  edge_items_.clear();
  edges_.clear();
  const auto &edits = editor_.edits();
  for (const auto &e : edits.base()) {
    DrawnEdge d;
    d.edge = {e.key.from, e.key.to};
    d.type = e.key.type;
    d.residual = e.residual;
    d.weight = e.weight;
    edges_.push_back(d);
  }
  for (const auto &op : edits.ops()) {
    if (op.kind == PendingEdgeEdits::Op::Kind::add) {
      DrawnEdge d;
      d.edge = {op.edge.key.from, op.edge.key.to};
      d.type = op.edge.key.type;
      d.pending_add = true;
      d.weight = op.edge.weight;
      edges_.push_back(d);
    } else {
      for (auto &d : edges_)
        if (!d.pending_add && d.edge.from == op.edge.key.from &&
            d.edge.to == op.edge.key.to && d.type == op.edge.key.type)
          d.pending_delete = true;
    }
  }
  if (nodes_.empty())
    return;
  std::map<int, QPointF> at;
  for (const auto &n : nodes_)
    at[n.id] = QPointF(n.x, -n.y);
  const Theme &t = theme();
  for (const auto &d : edges_) {
    const auto a = at.find(d.edge.from), b = at.find(d.edge.to);
    if (a == at.end() || b == at.end())
      continue;
    QPen pen(d.pending_delete ? t.color("--color-status-failed")
                              : t.color(edge_token(d.type)),
             d.type == "odometry" ? 1.0 : 2.0);
    pen.setCosmetic(true);
    if (d.pending_add || d.pending_delete)
      pen.setStyle(Qt::DashLine);
    auto *line = view_->scene()->addLine(QLineF(a->second, b->second), pen);
    line->setZValue(d.type == "odometry" ? 1 : 3);
    edge_items_.push_back(line);
  }
}

void PoseGraphWorkspace::place_markers() {
  auto place = [this](QGraphicsItem *m, int id) {
    if (!m)
      return;
    const auto it =
        std::find_if(nodes_.begin(), nodes_.end(),
                     [id](const GraphNode &n) { return n.id == id; });
    m->setVisible(it != nodes_.end());
    if (it != nodes_.end())
      m->setPos(it->x, -it->y);
  };
  place(marker_a_, a_);
  place(marker_b_, b_);
}

void PoseGraphWorkspace::set_pair(int a_id, int b_id) {
  a_ = a_id;
  b_ = b_id;
  place_markers();
}

void PoseGraphWorkspace::on_click(const QPointF &pos,
                                  Qt::KeyboardModifiers mods) {
  const double tol = kHitPx * view_->units_per_pixel();
  const int n = nearest_node(nodes_, pos.x(), -pos.y(), tol);
  if (n >= 0) {
    click_node(nodes_[static_cast<std::size_t>(n)].id,
               mods.testFlag(Qt::ShiftModifier));
    return;
  }
  std::vector<GraphEdge> plain;
  for (const auto &d : edges_)
    plain.push_back(d.edge);
  const int e = nearest_edge(nodes_, plain, pos.x(), -pos.y(), tol);
  if (e >= 0) {
    const auto &d = edges_[static_cast<std::size_t>(e)];
    click_edge(d.edge.from, d.edge.to);
  }
}

void PoseGraphWorkspace::click_node(int id, bool as_b) {
  const auto it = std::find_if(nodes_.begin(), nodes_.end(),
                               [id](const GraphNode &n) { return n.id == id; });
  if (it == nodes_.end())
    return;
  Selection s;
  s.kind = "Node";
  s.title = QString("Billede %1").arg(id);
  s.subtitle = as_b ? QString("Valgt som B i Database")
                    : QString("Valgt som A i Database");
  SelectionSection p{"Pose", {}, {}};
  p.rows.push_back({"X", da().toString(it->x, 'f', 2) + " m"});
  p.rows.push_back({"Y", da().toString(it->y, 'f', 2) + " m"});
  p.rows.push_back(
      {"Scanning",
       QString::number(
           scan_of_[static_cast<std::size_t>(it - nodes_.begin())])});
  p.rows.push_back({"Kanter", QString::number(editor_.edits().degree(id))});
  s.sections.push_back(p);
  selection_ = s;
  emit selection_changed(selection_);
  emit frame_requested(id, as_b);
}

void PoseGraphWorkspace::click_edge(int from, int to) {
  for (const auto &d : edges_) {
    if (d.edge.from != from || d.edge.to != to)
      continue;
    Selection s;
    s.kind = "Kant";
    s.title = QString("%1 → %2").arg(from).arg(to);
    s.subtitle = edge_type_da(d.type);
    if (d.pending_add) {
      s.pill = "Ny · ikke gemt";
      s.pill_tone = "warn";
    } else if (d.pending_delete) {
      s.pill = "Slettes";
      s.pill_tone = "crit";
    }
    SelectionSection p{"Kant", {}, {}};
    p.rows.push_back(
        {"Type", edge_type_da(d.type), SelectionRow::Style::value});
    p.rows.push_back({"Residual", da().toString(d.residual, 'g', 4)});
    p.rows.push_back({"Vægt", std::isnan(d.weight)
                                  ? QString("—")
                                  : da().toString(d.weight, 'g', 4)});
    s.sections.push_back(p);
    selection_ = s;
    emit selection_changed(selection_);
    emit pair_requested(from, to);
    return;
  }
}

void PoseGraphWorkspace::update_meta() {
  if (!session_.is_open()) {
    meta_->setText(QString());
    return;
  }
  if (loading_) {
    meta_->setText("Læser poserne …");
    return;
  }
  int stored = 0, adds = 0, dels = 0;
  for (const auto &d : edges_) {
    stored += !d.pending_add;
    adds += d.pending_add;
    dels += d.pending_delete;
  }
  QString m = QString("%1 billeder · %2 kanter")
                  .arg(format_count(nodes_.size()))
                  .arg(format_count(static_cast<qulonglong>(stored)));
  if (adds || dels)
    m += QString(" · %1 ny, %2 slettes (ikke gemt)").arg(adds).arg(dels);
  meta_->setText(m);
}

} // namespace rux::qt
