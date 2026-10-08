// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/TableBrowser.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/widgets.hpp>

#include <QDateTime>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonDocument>
#include <QLabel>
#include <QPainter>
#include <QScrollBar>
#include <QTableView>
#include <QVBoxLayout>

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
  QString s = QString::number(v, 'g', 12);
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
      paging_.total = row_count;
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
  case Cell::Kind::integer:
    return QString::number(c.integer);
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
  s.kind = "Række";
  s.title = QString("%1 · række %2").arg(table_).arg(row + 1);
  s.subtitle = QString("af %1").arg(format_count(
      static_cast<qulonglong>(std::max<qint64>(0, paging_.total))));
  SelectionSection fields{"Felter", {}, {}};
  QString long_text;
  for (std::size_t c = 0; c < columns_.size(); ++c) {
    const Cell &cell = rows_[static_cast<std::size_t>(row)][c];
    QString value = display(cell, static_cast<int>(c));
    if (cell.kind == Cell::Kind::text &&
        (cell.truncated || value.size() > 40)) {
      // Long text (JSON parameters, provenance) goes in a block below.
      QString full = QString::fromUtf8(
          cell.text.data(), static_cast<qsizetype>(cell.text.size()));
      const QJsonDocument doc = QJsonDocument::fromJson(full.toUtf8());
      if (!doc.isNull())
        full = QString::fromUtf8(doc.toJson(QJsonDocument::Indented)).trimmed();
      long_text +=
          QString("%1\n%2%3\n\n")
              .arg(qs(columns_[c].name), full,
                   cell.truncated ? QString("\n… (%1 i alt)")
                                        .arg(qs(format_bytes_da(cell.size)))
                                  : QString());
      value = cell.truncated ? qs(format_bytes_da(cell.size)) + " tekst"
                             : QString("se nedenfor");
    }
    fields.rows.push_back(
        {qs(columns_[c].name), value, SelectionRow::Style::name});
  }
  s.sections.push_back(fields);
  if (!long_text.isEmpty())
    s.sections.push_back({"Lange felter", {}, long_text.trimmed()});
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

PipelineLogView::PipelineLogView(ProjectSession &session, QWidget *parent)
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
  auto *cli = new QLabel("rux log");
  cli->setObjectName("codeChip");
  cli->setToolTip("Samme log i terminalen");
  cli->setTextInteractionFlags(Qt::TextSelectableByMouse);
  bl->addWidget(cli);
  v->addWidget(bar);

  model_ = new PipelineLogModel(this);
  view_ = new QTableView;
  style_table(view_);
  view_->setModel(model_);
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
            if (cur.isValid())
              emit selection_changed(entry_selection(cur.row()));
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
  int ok = 0, failed = 0, open = 0;
  for (const auto &e : entries) {
    ok += e.status == "success";
    failed += e.status == "failed";
    open += e.status == "running" && e.finished_at.empty();
  }
  model_->set_entries(std::move(entries));
  meta_->setText(QString("%1 kørsler · %2 gennemført · %3 fejlet · %4 ikke "
                         "afsluttet")
                     .arg(model_->rowCount())
                     .arg(ok)
                     .arg(failed)
                     .arg(open));
  view_->setVisible(model_->rowCount() > 0);
  empty_->setVisible(model_->rowCount() == 0);
}

void PipelineLogView::select_row(int row) {
  if (row >= 0 && row < model_->rowCount())
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
  return s;
}

} // namespace rux::qt
