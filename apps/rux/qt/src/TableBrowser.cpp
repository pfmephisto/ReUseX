// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/pipeline_ui.hpp>
#include <rux_qt/widgets.hpp>

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QDateTime>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QPainter>
#include <QPushButton>
#include <QScrollBar>
#include <QTableView>
#include <QTimeZone>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace rux::qt {
namespace {

using Cell = reusex::ProjectDB::TableCell;

QString qs(const std::string &s) { return QString::fromStdString(s); }

QString dec(double v, int decimals) {
  return qs(format_decimal_da(v, decimals));
}

/// A real in the shortest form that round-trips the shown digits, with a
/// Danish comma: 1773125870,8564 / 0,25 / 1e-07.
QString real_text(double v) {
  QString s = QString::number(v, 'g', 15);
  return s.replace('.', ',');
}

/// First line of a text cell, with an ellipsis when cut.
QString text_preview(const Cell &c) {
  QString s =
      QString::fromUtf8(c.text.data(), static_cast<qsizetype>(c.text.size()));
  const qsizetype nl = s.indexOf('\n');
  const bool cut = nl >= 0 || c.truncated;
  if (nl >= 0)
    s.truncate(nl);
  return cut ? s + " …" : s;
}

QHeaderView *setup_header(QTableView *view, bool caps) {
  auto *h = view->horizontalHeader();
  h->setHighlightSections(false);
  h->setDefaultAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  QFont hf = h->font();
  if (caps) {
    // QSS has neither text-transform nor letter-spacing.
    hf.setCapitalization(QFont::AllUppercase);
    hf.setLetterSpacing(QFont::AbsoluteSpacing,
                        theme().em("--tracking-caps") *
                            theme().px("--font-size-2xs"));
  }
  h->setFont(hf);
  return h;
}

} // namespace

void style_table(QTableView *view) {
  view->setObjectName("dataTable");
  view->verticalHeader()->hide();
  view->setShowGrid(false);
  view->setSelectionBehavior(QAbstractItemView::SelectRows);
  view->setSelectionMode(QAbstractItemView::SingleSelection);
  view->setEditTriggers(QAbstractItemView::NoEditTriggers);
  view->setWordWrap(false);
  view->setTextElideMode(Qt::ElideRight);
  view->setFrameShape(QFrame::NoFrame);
  view->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
  view->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  view->verticalHeader()->setDefaultSectionSize(theme().px("--space-6") +
                                                theme().px("--space-1"));
}

// ------------------------------------------------------------ DbTableModel --

DbTableModel::DbTableModel(QObject *parent) : QAbstractTableModel(parent) {}

void DbTableModel::set_table(const reusex::ProjectDB *db, const QString &table,
                             qint64 row_count) {
  beginResetModel();
  db_ = db;
  table_ = table;
  rows_.clear();
  columns_.clear();
  error_.clear();
  paging_ = Paging{0, 0, 200};
  if (db_ && !table_.isEmpty()) {
    try {
      columns_ = db_->table_columns(table_.toStdString());
      // Counted now, not taken from the tree's snapshot: rows added since
      // (a saved edge) must show.
      paging_.total = db_->table_row_count(table_.toStdString());
      (void)row_count;
    } catch (const std::exception &e) {
      error_ = QString::fromUtf8(e.what());
    }
  }
  endResetModel();
  if (paging_.can_fetch_more())
    fetch(paging_.next_count());
}

void DbTableModel::fetch(qint64 count) {
  if (!db_ || count <= 0)
    return;
  std::vector<std::vector<Cell>> page;
  try {
    page = db_->table_rows(table_.toStdString(), paging_.loaded, count);
  } catch (const std::exception &e) {
    error_ = QString::fromUtf8(e.what());
    paging_.total = paging_.loaded; // stop asking
    return;
  }
  if (page.empty()) {
    paging_.total = paging_.loaded; // the table shrank under us
    return;
  }
  const int first = static_cast<int>(rows_.size());
  beginInsertRows({}, first, first + static_cast<int>(page.size()) - 1);
  for (auto &r : page)
    rows_.push_back(std::move(r));
  paging_.loaded = static_cast<qint64>(rows_.size());
  endInsertRows();
}

void DbTableModel::ensure_loaded(qint64 row) {
  fetch(paging_.count_to_reach(row));
}

int DbTableModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(rows_.size());
}

int DbTableModel::columnCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(columns_.size());
}

bool DbTableModel::canFetchMore(const QModelIndex &parent) const {
  return !parent.isValid() && paging_.can_fetch_more();
}

void DbTableModel::fetchMore(const QModelIndex &parent) {
  if (!parent.isValid())
    fetch(paging_.next_count());
}

QString DbTableModel::display(const Cell &c, int column) const {
  switch (c.kind) {
  case Cell::Kind::null:
    return "NULL";
  case Cell::Kind::integer: {
    // Counts are grouped the Danish way (1.212.572, as the tree shows them);
    // keys and ids stay bare — "node_id 1.177" reads as a decimal.
    const auto &col = columns_[static_cast<std::size_t>(column)];
    const std::string &n = col.name;
    const bool id_like =
        col.primary_key || n == "id" ||
        (n.size() > 3 && n.compare(n.size() - 3, 3, "_id") == 0) ||
        n == "version" || n.find("timestamp") != std::string::npos;
    if (id_like || (c.integer > -1000 && c.integer < 1000))
      return QString::number(c.integer);
    return QLocale(QLocale::Danish, QLocale::Denmark)
        .toString(static_cast<qlonglong>(c.integer));
  }
  case Cell::Kind::real:
    return real_text(c.real);
  case Cell::Kind::text:
    return text_preview(c);
  case Cell::Kind::blob:
    return qs(describe_blob(table_.toStdString(),
                            columns_[static_cast<std::size_t>(column)].name,
                            c.text, c.size));
  }
  return {};
}

QVariant DbTableModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() >= rowCount() ||
      index.column() >= columnCount())
    return {};
  const Cell &c = rows_[static_cast<std::size_t>(index.row())]
                       [static_cast<std::size_t>(index.column())];
  switch (role) {
  case Qt::DisplayRole:
    return display(c, index.column());
  case KindRole:
    return static_cast<int>(c.kind);
  case Qt::TextAlignmentRole:
    return c.kind == Cell::Kind::integer || c.kind == Cell::Kind::real
               ? QVariant(Qt::AlignRight | Qt::AlignVCenter)
               : QVariant(Qt::AlignLeft | Qt::AlignVCenter);
  case Qt::FontRole:
    // Figures, ids, blobs and NULL in mono; prose in the UI face.
    return c.kind == Cell::Kind::text
               ? theme().font(FontRole::sans, "--font-size-sm")
               : theme().font(FontRole::mono, "--font-size-xs");
  case Qt::ForegroundRole:
    if (c.kind == Cell::Kind::null)
      return theme().color("--color-text-faint");
    if (c.kind == Cell::Kind::blob)
      return theme().color("--color-text-muted");
    return {};
  case Qt::ToolTipRole:
    if (c.kind == Cell::Kind::text && (c.truncated || c.text.size() > 60))
      return QString::fromUtf8(c.text.data(),
                               static_cast<qsizetype>(c.text.size())) +
             (c.truncated
                  ? QString("\n… (%1 i alt)").arg(qs(format_bytes_da(c.size)))
                  : QString());
    return {};
  default:
    return {};
  }
}

QVariant DbTableModel::headerData(int section, Qt::Orientation o,
                                  int role) const {
  if (o != Qt::Horizontal || section < 0 || section >= columnCount())
    return {};
  const auto &col = columns_[static_cast<std::size_t>(section)];
  if (role == Qt::DisplayRole)
    return qs(col.name);
  if (role == Qt::ToolTipRole)
    return QString("%1 · %2%3")
        .arg(qs(col.name),
             col.declared_type.empty() ? QString("ingen type")
                                       : qs(col.declared_type),
             col.primary_key ? QString(" · primærnøgle") : QString());
  return {};
}

Selection DbTableModel::row_selection(int row) const {
  Selection s;
  if (row < 0 || row >= rowCount())
    return s;
  // The selected row again, with whole text cells (up to 1 MiB each): the
  // page cache holds previews only, and a cut JSON value cannot be pretty-
  // printed. Same order as the pages, so the offset names the same row.
  std::vector<Cell> cells = rows_[static_cast<std::size_t>(row)];
  if (db_) {
    try {
      auto full = db_->table_rows(table_.toStdString(), row, 1, 1u << 20);
      if (full.size() == 1 && full.front().size() == cells.size())
        cells = std::move(full.front());
    } catch (const std::exception &) {
    }
  }

  // Lead with the primary key ("node_id 1177"), not the position.
  QString key;
  for (std::size_t c = 0; c < columns_.size(); ++c)
    if (columns_[c].primary_key) {
      key =
          QString("%1 %2").arg(qs(columns_[c].name), display(cells[c], int(c)));
      break;
    }
  s.kind = "Række";
  s.title =
      key.isEmpty() ? QString("Række %1").arg(format_count(row + 1)) : key;
  s.subtitle = QString("%1 · række %2 af %3")
                   .arg(table_, format_count(static_cast<qulonglong>(row + 1)),
                        format_count(static_cast<qulonglong>(
                            std::max<qint64>(0, paging_.total))));
  SelectionSection fields{"Felter", {}, {}};
  QVector<SelectionSection> blocks;
  for (std::size_t c = 0; c < columns_.size(); ++c) {
    const Cell &cell = cells[c];
    QString value = display(cell, static_cast<int>(c));
    if (cell.kind == Cell::Kind::text &&
        (cell.truncated || cell.text.size() > 32 ||
         cell.text.find('\n') != std::string::npos)) {
      // Long text (JSON parameters, provenance) gets a block of its own.
      QString full = QString::fromUtf8(
          cell.text.data(), static_cast<qsizetype>(cell.text.size()));
      const QJsonDocument doc = QJsonDocument::fromJson(full.toUtf8());
      if (!doc.isNull())
        full = QString::fromUtf8(doc.toJson(QJsonDocument::Indented)).trimmed();
      if (cell.truncated)
        full += QString("\n… (%1 i alt)").arg(qs(format_bytes_da(cell.size)));
      blocks.push_back({qs(columns_[c].name), {}, full});
      value = qs(format_bytes_da(cell.size)) + " · se nedenfor";
    }
    fields.rows.push_back(
        {qs(columns_[c].name), value, SelectionRow::Style::name});
  }
  s.sections.push_back(fields);
  s.sections += blocks;
  return s;
}

int DbTableModel::find_row(const QString &column, const QString &value) {
  int c = -1;
  for (std::size_t i = 0; i < columns_.size(); ++i)
    if (qs(columns_[i].name) == column)
      c = static_cast<int>(i);
  if (c < 0)
    return -1;
  // Look through what is loaded, fetching further pages as needed (small
  // tables — clouds, meshes — fit in the first page anyway).
  for (int r = 0;; ++r) {
    if (r >= rowCount()) {
      if (!paging_.can_fetch_more())
        return -1;
      fetch(paging_.next_count());
      if (r >= rowCount())
        return -1;
    }
    const Cell &cell =
        rows_[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
    if (cell.kind == Cell::Kind::text &&
        QString::fromUtf8(cell.text.data(),
                          static_cast<qsizetype>(cell.text.size())) == value)
      return r;
    if (cell.kind == Cell::Kind::integer &&
        QString::number(cell.integer) == value)
      return r;
  }
}

// ------------------------------------------------------------ TableBrowser --

TableBrowser::TableBrowser(ProjectSession &session, QWidget *parent)
    : QWidget(parent), session_(session) {
  setObjectName("tableBrowser");
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
  title_ = new QLabel;
  title_->setObjectName("tableName");
  bl->addWidget(title_);
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  auto *ro = new Pill("Kun læsning", "outline");
  ro->setToolTip("Tabelvisningen ændrer aldrig projektet.");
  bl->addWidget(ro);
  v->addWidget(bar);

  model_ = new DbTableModel(this);
  view_ = new QTableView;
  style_table(view_);
  view_->setModel(model_);
  setup_header(view_, /*caps=*/false)->setObjectName("rawHeader");
  view_->horizontalHeader()->setStretchLastSection(true);
  v->addWidget(view_, 1);

  empty_ = new QLabel;
  empty_->setObjectName("emptyText");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->setWordWrap(true);
  empty_->hide();
  v->addWidget(empty_, 1);

  connect(view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, [this](const QModelIndex &cur) {
            if (cur.isValid())
              emit selection_changed(model_->row_selection(cur.row()));
          });
  connect(&session_, &ProjectSession::closed, this,
          [this] { model_->set_table(nullptr, {}, 0); });
}

void TableBrowser::show_table(const QString &table, qint64 row_count,
                              const QString &title) {
  model_->set_table(session_.db(), table, row_count);
  title_->setText(title.isEmpty() ? table : title);
  meta_->setText(QString("%1 · %2 rækker · %3 kolonner")
                     .arg(table)
                     .arg(format_count(static_cast<qulonglong>(
                         std::max<qint64>(0, row_count))))
                     .arg(model_->columnCount()));
  // Columns sized to the first page, capped so one long text column
  // cannot push the rest off screen.
  view_->resizeColumnsToContents();
  const int cap = theme().px("--layout-panel-width");
  for (int c = 0; c < model_->columnCount(); ++c)
    if (view_->columnWidth(c) > cap)
      view_->setColumnWidth(c, cap);
  view_->scrollToTop();
  const bool none = model_->rowCount() == 0;
  view_->setVisible(!none);
  empty_->setVisible(none);
  empty_->setText(!model_->error().isEmpty()
                      ? "Tabellen kunne ikke læses.\n" + model_->error()
                      : QString("Tabellen %1 er tom.").arg(table));
}

void TableBrowser::select_where(const QString &column, const QString &value) {
  const int r = model_->find_row(column, value);
  if (r < 0)
    return;
  view_->selectRow(r);
  view_->scrollTo(model_->index(r, 0), QAbstractItemView::PositionAtCenter);
}

// ------------------------------------------------------- StatusPillDelegate --

QString log_status_da(const std::string &status, bool finished) {
  if (status == "success")
    return "Gennemført";
  if (status == "failed")
    return "Fejlet";
  if (status == "running")
    return finished ? "Kørte" : "Ikke afsluttet";
  return qs(status);
}

QString log_status_tone(const std::string &status, bool finished) {
  if (status == "success")
    return "good";
  if (status == "failed")
    return "crit";
  if (status == "running" && !finished)
    return "wait";
  return "outline";
}

void StatusPillDelegate::paint(QPainter *p, const QStyleOptionViewItem &option,
                               const QModelIndex &index) const {
  QStyleOptionViewItem opt(option);
  initStyleOption(&opt, index);
  const QString text = opt.text;
  opt.text.clear();
  QStyledItemDelegate::paint(p, opt, index); // background, selection
  const Theme &t = theme();
  const QString tone = index.data(PipelineLogModel::ToneRole).toString();
  const QFont f =
      t.font(FontRole::sans, "--font-size-xs", "--font-weight-bold");
  const QFontMetrics fm(f);
  const int padx = t.px("--space-2"), pady = t.px("--space-1");
  QRect pill(0, 0, fm.horizontalAdvance(text) + 2 * padx, fm.height() + pady);
  pill.moveCenter(
      QPoint(option.rect.left() + t.px("--space-3") + pill.width() / 2,
             option.rect.center().y()));
  p->save();
  p->setRenderHint(QPainter::Antialiasing);
  p->setPen(Qt::NoPen);
  if (tone == "outline") {
    p->setPen(QPen(t.color("--color-border-strong"), 1));
    p->setBrush(Qt::NoBrush);
  } else {
    p->setBrush(t.color(QString("--tone-%1-bg").arg(tone)));
  }
  p->drawRoundedRect(QRectF(pill).adjusted(0.5, 0.5, -0.5, -0.5),
                     t.px("--radius-sm"), t.px("--radius-sm"));
  p->setPen(tone == "outline" ? t.color("--color-text-muted")
                              : t.color(QString("--tone-%1-ink").arg(tone)));
  p->setFont(f);
  p->drawText(pill, Qt::AlignCenter, text);
  p->restore();
}

// ---------------------------------------------------------- PipelineLogModel
// --

PipelineLogModel::PipelineLogModel(QObject *parent)
    : QAbstractTableModel(parent) {}

void PipelineLogModel::set_entries(
    std::vector<reusex::ProjectDB::PipelineLogEntry> entries) {
  beginResetModel();
  entries_ = std::move(entries);
  // Newest first: the last run is what one looks for.
  std::sort(entries_.begin(), entries_.end(),
            [](const auto &a, const auto &b) { return a.id > b.id; });
  endResetModel();
}

int PipelineLogModel::rowCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : static_cast<int>(entries_.size());
}

int PipelineLogModel::columnCount(const QModelIndex &parent) const {
  return parent.isValid() ? 0 : Column::count;
}

namespace {

/// sqlite datetime('now') text ("2026-04-24 08:36:54", UTC) -> QDateTime.
QDateTime parse_utc(const std::string &s) {
  QDateTime dt = QDateTime::fromString(qs(s), "yyyy-MM-dd HH:mm:ss");
  dt.setTimeZone(QTimeZone::UTC);
  return dt;
}

QString duration_text(const reusex::ProjectDB::PipelineLogEntry &e) {
  if (e.finished_at.empty())
    return "—";
  const qint64 s = parse_utc(e.started_at).secsTo(parse_utc(e.finished_at));
  if (s < 0)
    return "—";
  if (s < 60)
    return QString("%1 s").arg(s);
  if (s < 3600)
    return QString("%1 min %2 s").arg(s / 60).arg(s % 60);
  return QString("%1 t %2 min").arg(s / 3600).arg((s % 3600) / 60);
}

QString local_time(const std::string &utc) {
  const QDateTime dt = parse_utc(utc);
  if (!dt.isValid())
    return qs(utc);
  return QLocale(QLocale::Danish, QLocale::Denmark)
      .toString(dt.toLocalTime(), "d. MMM yyyy HH:mm");
}

} // namespace

QVariant PipelineLogModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() >= rowCount())
    return {};
  const auto &e = entries_[static_cast<std::size_t>(index.row())];
  const bool finished = !e.finished_at.empty();
  if (role == ToneRole)
    return log_status_tone(e.status, finished);
  if (role == Qt::DisplayRole) {
    switch (index.column()) {
    case Column::id:
      return e.id;
    case Column::stage:
      return qs(e.stage);
    case Column::status:
      return log_status_da(e.status, finished);
    case Column::started:
      return local_time(e.started_at);
    case Column::duration:
      return duration_text(e);
    case Column::error: {
      QString m = qs(e.error_msg);
      return m.section('\n', 0, 0);
    }
    default:
      return {};
    }
  }
  if (role == Qt::FontRole) {
    if (index.column() == Column::id || index.column() == Column::stage ||
        index.column() == Column::duration)
      return theme().font(FontRole::mono, "--font-size-xs");
    return theme().font(FontRole::sans, "--font-size-sm");
  }
  if (role == Qt::ForegroundRole && index.column() == Column::id)
    return theme().color("--color-text-faint");
  if (role == Qt::TextAlignmentRole &&
      (index.column() == Column::id || index.column() == Column::duration))
    return QVariant(Qt::AlignRight | Qt::AlignVCenter);
  if (role == Qt::ToolTipRole && index.column() == Column::status &&
      e.status == "running" && !finished)
    return "Trinnet blev aldrig afsluttet i loggen — det blev afbrudt, "
           "eller kører stadig i en anden proces.";
  return {};
}

QVariant PipelineLogModel::headerData(int section, Qt::Orientation o,
                                      int role) const {
  if (o != Qt::Horizontal || role != Qt::DisplayRole)
    return {};
  static const char *names[] = {"#",       "Trin",     "Status",
                                "Startet", "Varighed", "Fejl"};
  return section >= 0 && section < Column::count
             ? QString(names[section]).toUpper()
             : QVariant();
}

// ------------------------------------------------------------ PipelineLogView
// --

void LogFilterProxy::set_filter(LogFilter filter) {
  filter_ = std::move(filter);
  invalidateFilter();
}

bool LogFilterProxy::filterAcceptsRow(int row, const QModelIndex &) const {
  const auto *m = static_cast<const PipelineLogModel *>(sourceModel());
  if (!m || row >= m->rowCount())
    return false;
  const auto &e = m->entry(row);
  return log_row_matches(
      {e.stage, e.status, !e.finished_at.empty(), e.error_msg, e.parameters},
      filter_);
}

PipelineLogView::PipelineLogView(ProjectSession &session, bool filters,
                                 QWidget *parent)
    : QWidget(parent), session_(session) {
  setObjectName("pipelineLog");
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
  bl->addWidget(new CapsLabel("Pipeline-log", "panelTitle"));
  meta_ = new QLabel;
  meta_->setObjectName("toolbarMeta");
  bl->addWidget(meta_, 1);
  copy_ = new QPushButton(NavItem::escape_mnemonic("Kopiér som rux-kommando"));
  copy_->setProperty("kind", "secondary");
  copy_->setCursor(Qt::PointingHandCursor);
  copy_->setEnabled(false);
  copy_->setToolTip("Vælg en kørsel af et trin, rux kan gentage");
  bl->addWidget(copy_);
  auto *cli = new QLabel("rux log");
  cli->setObjectName("codeChip");
  cli->setToolTip("Samme log i terminalen");
  cli->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bl->addWidget(cli);
  v->addWidget(bar);

  if (filters) {
    auto *fbar = new QFrame;
    fbar->setObjectName("filterBar");
    auto *fl = new QHBoxLayout(fbar);
    fl->setContentsMargins(t.px("--space-4"), t.px("--space-2"),
                           t.px("--space-4"), t.px("--space-2"));
    fl->setSpacing(t.px("--space-3"));
    fl->addWidget(new CapsLabel("Trin", "fieldLabel"));
    stage_ = new QComboBox;
    stage_->setObjectName("logStage");
    fl->addWidget(stage_);
    fl->addWidget(new CapsLabel("Status", "fieldLabel"));
    status_ = new QFrame;
    status_->setObjectName("segmented");
    auto *sl = new QHBoxLayout(status_);
    sl->setContentsMargins(0, 0, 0, 0);
    sl->setSpacing(0);
    const char *names[] = {"Alle", "Gennemført", "Fejlet", "Ikke afsluttet"};
    for (int i = 0; i < 4; ++i) {
      auto *b = new QPushButton(NavItem::escape_mnemonic(names[i]));
      b->setObjectName("segment");
      b->setCheckable(true);
      b->setAutoExclusive(true);
      b->setChecked(i == 0);
      b->setCursor(Qt::PointingHandCursor);
      b->setProperty("position", i == 0 ? "first" : i == 3 ? "last" : "middle");
      sl->addWidget(b);
      connect(b, &QPushButton::clicked, this, [this, i] {
        LogFilter f = proxy_->filter();
        f.status = static_cast<LogStatusFilter>(i);
        proxy_->set_filter(f);
        update_meta();
      });
    }
    fl->addWidget(status_);
    search_ = new QLineEdit;
    search_->setObjectName("logSearch");
    search_->setPlaceholderText("Søg i trin, fejl og parametre");
    search_->setClearButtonEnabled(true);
    fl->addWidget(search_, 1);
    v->addWidget(fbar);
    connect(stage_, &QComboBox::currentIndexChanged, this, [this](int) {
      LogFilter f = proxy_->filter();
      f.stage = stage_->currentData().toString().toStdString();
      proxy_->set_filter(f);
      update_meta();
    });
    connect(search_, &QLineEdit::textChanged, this, [this](const QString &q) {
      LogFilter f = proxy_->filter();
      f.text = q.toStdString();
      proxy_->set_filter(f);
      update_meta();
    });
  }

  model_ = new PipelineLogModel(this);
  proxy_ = new LogFilterProxy(this);
  proxy_->setSourceModel(model_);
  view_ = new QTableView;
  style_table(view_);
  view_->setModel(proxy_);
  view_->setItemDelegateForColumn(PipelineLogModel::status,
                                  new StatusPillDelegate(view_));
  auto *h = setup_header(view_, /*caps=*/true);
  h->setSectionResizeMode(QHeaderView::ResizeToContents);
  h->setSectionResizeMode(PipelineLogModel::error, QHeaderView::Stretch);
  v->addWidget(view_, 1);
  empty_ = new QLabel("Intet er kørt på projektet endnu.");
  empty_->setObjectName("emptyText");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->hide();
  v->addWidget(empty_, 1);

  connect(view_->selectionModel(), &QItemSelectionModel::currentRowChanged,
          this, [this](const QModelIndex &cur) {
            if (!cur.isValid())
              return;
            const int src = proxy_->mapToSource(cur).row();
            const Selection sel = entry_selection(src);
            copy_->setEnabled(!command_.isEmpty());
            copy_->setToolTip(command_.isEmpty()
                                  ? QString("Dette trin har ingen rux-kommando "
                                            "her")
                                  : command_);
            emit selection_changed(sel);
          });
  connect(copy_, &QPushButton::clicked, this, [this] {
    if (!command_.isEmpty())
      QApplication::clipboard()->setText(command_);
  });
}

void PipelineLogView::reload() {
  std::vector<reusex::ProjectDB::PipelineLogEntry> entries;
  if (const reusex::ProjectDB *db = session_.db()) {
    try {
      entries = db->pipeline_log();
    } catch (const std::exception &e) {
      empty_->setText(QString("Loggen kunne ikke læses.\n%1")
                          .arg(QString::fromUtf8(e.what())));
    }
  }
  if (stage_) {
    std::vector<LogRow> rows;
    for (const auto &e : entries)
      rows.push_back({e.stage, e.status, !e.finished_at.empty(), {}, {}});
    const QSignalBlocker block(stage_);
    const QString current = stage_->currentData().toString();
    stage_->clear();
    stage_->addItem("Alle trin", QString());
    for (const auto &st : log_stages(rows))
      stage_->addItem(QString::fromStdString(st), QString::fromStdString(st));
    const int i = stage_->findData(current);
    stage_->setCurrentIndex(i < 0 ? 0 : i);
  }
  model_->set_entries(std::move(entries));
  proxy_->invalidate();
  copy_->setEnabled(false);
  command_.clear();
  update_meta();
}

void PipelineLogView::update_meta() {
  int ok = 0, failed = 0, open = 0;
  for (int r = 0; r < model_->rowCount(); ++r) {
    const auto &e = model_->entry(r);
    ok += e.status == "success";
    failed += e.status == "failed";
    open += e.status == "running" && e.finished_at.empty();
  }
  QString m = QString("%1 kørsler · %2 gennemført · %3 fejlet · %4 ikke "
                      "afsluttet")
                  .arg(model_->rowCount())
                  .arg(ok)
                  .arg(failed)
                  .arg(open);
  if (proxy_->rowCount() != model_->rowCount())
    m = QString("%1 af %2 vist · ")
            .arg(proxy_->rowCount())
            .arg(model_->rowCount()) +
        m;
  meta_->setText(m);
  const bool any = model_->rowCount() > 0;
  const bool shown = proxy_->rowCount() > 0;
  if (any && !shown)
    empty_->setText("Ingen kørsler passer til filteret.");
  else if (!any)
    empty_->setText("Intet er kørt på projektet endnu.");
  view_->setVisible(shown);
  empty_->setVisible(!shown);
}

void PipelineLogView::set_filter(const LogFilter &filter) {
  proxy_->set_filter(filter);
  sync_filter_controls();
  update_meta();
}

void PipelineLogView::sync_filter_controls() {
  const LogFilter &f = proxy_->filter();
  if (stage_) {
    const QSignalBlocker b(stage_);
    const int i = stage_->findData(QString::fromStdString(f.stage));
    stage_->setCurrentIndex(i < 0 ? 0 : i);
  }
  if (search_) {
    const QSignalBlocker b(search_);
    search_->setText(QString::fromStdString(f.text));
  }
  if (status_) {
    const auto buttons = status_->findChildren<QPushButton *>("segment");
    const int i = static_cast<int>(f.status);
    if (i >= 0 && i < buttons.size())
      buttons[i]->setChecked(true);
  }
}

int PipelineLogView::visible_rows() const { return proxy_->rowCount(); }

void PipelineLogView::select_row(int row) {
  if (row >= 0 && row < proxy_->rowCount())
    view_->selectRow(row);
}

Selection PipelineLogView::entry_selection(int row) const {
  Selection s;
  if (row < 0 || row >= model_->rowCount())
    return s;
  const auto &e = model_->entry(row);
  const bool finished = !e.finished_at.empty();
  s.kind = "Kørsel";
  s.title = qs(e.stage);
  s.subtitle = QString("Kørsel #%1").arg(e.id);
  s.pill = log_status_da(e.status, finished);
  s.pill_tone = log_status_tone(e.status, finished);
  SelectionSection when{"Tid", {}, {}};
  when.rows.push_back({"Startet", local_time(e.started_at)});
  when.rows.push_back(
      {"Afsluttet", finished ? local_time(e.finished_at) : QString("—")});
  when.rows.push_back({"Varighed", duration_text(e)});
  s.sections.push_back(when);
  if (!e.error_msg.empty())
    s.sections.push_back({"Fejl", {}, qs(e.error_msg)});
  QString params = qs(e.parameters);
  const QJsonDocument doc = QJsonDocument::fromJson(params.toUtf8());
  if (!doc.isNull())
    params = QString::fromUtf8(doc.toJson(QJsonDocument::Indented)).trimmed();
  s.sections.push_back(
      {"Parametre", {}, params.isEmpty() ? QString("Ingen") : params});
  // The same run from the terminal, when it is a stage rux can repeat.
  QString &cmd = command_;
  cmd.clear();
  if (const auto stage = stage_from_log_name(e.stage)) {
    const CliCommand c =
        cli_for_parameters(*stage, qs(e.parameters), session_.path());
    if (c.supported) {
      cmd = QString::fromStdString(c.text);
      SelectionSection sec{"Som rux-kommando", {}, cmd};
      sec.command = true;
      s.sections.push_back(sec);
    }
  }
  return s;
}

} // namespace rux::qt
