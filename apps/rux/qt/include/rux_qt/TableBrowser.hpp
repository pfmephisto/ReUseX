// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Read-only views of the project file for the Database workspace: any sqlite
// table, paged lazily (ProjectDB::table_rows, which never reads a blob's
// bytes — a blob shows what it is and its size), and the pipeline log.

#include <rux_qt/database_logic.hpp>
#include <rux_qt/selection.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QAbstractTableModel>
#include <QSortFilterProxyModel>
#include <QStyledItemDelegate>
#include <QWidget>

#include <vector>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableView;

namespace rux::qt {

class ProjectSession;

/// One sqlite table, fetched a page at a time as the view scrolls.
class DbTableModel : public QAbstractTableModel {
  Q_OBJECT
    public:
  enum Role { KindRole = Qt::UserRole + 1 }; ///< TableCell::Kind as int

  explicit DbTableModel(QObject *parent = nullptr);
  /// Show @p table of @p db (nullptr or empty name = nothing).
  void set_table(const reusex::ProjectDB *db, const QString &table,
                 qint64 row_count);
  QString table() const { return table_; }
  qint64 total_rows() const { return paging_.total; }
  const std::vector<reusex::ProjectDB::TableColumn> &columns() const {
    return columns_;
  }
  /// Load rows until @p row is in memory (for "select row with id …").
  void ensure_loaded(qint64 row);
  /// The row as an inspector selection.
  Selection row_selection(int row) const;
  /// First loaded-or-loadable row whose column @p column equals @p value.
  int find_row(const QString &column, const QString &value);
  QString error() const { return error_; }

  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;
  bool canFetchMore(const QModelIndex &parent) const override;
  void fetchMore(const QModelIndex &parent) override;

    private:
  void fetch(qint64 count);
  QString display(const reusex::ProjectDB::TableCell &c, int column) const;
  const reusex::ProjectDB *db_ = nullptr;
  QString table_;
  std::vector<reusex::ProjectDB::TableColumn> columns_;
  std::vector<std::vector<reusex::ProjectDB::TableCell>> rows_;
  Paging paging_;
  QString error_;
};

/// The generic table viewer: a head (name, rows, columns) and the table.
class TableBrowser : public QWidget {
  Q_OBJECT
    public:
  explicit TableBrowser(ProjectSession &session, QWidget *parent = nullptr);
  void show_table(const QString &table, qint64 row_count,
                  const QString &title = {});
  /// Select the first row whose @p column is @p value (a cloud by name).
  void select_where(const QString &column, const QString &value);
  QString table() const { return model_->table(); }

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    private:
  ProjectSession &session_;
  DbTableModel *model_ = nullptr;
  QTableView *view_ = nullptr;
  QLabel *title_ = nullptr;
  QLabel *meta_ = nullptr;
  QLabel *empty_ = nullptr;
};

/// Status pills in a table cell (the pipeline log's status column).
class StatusPillDelegate : public QStyledItemDelegate {
  Q_OBJECT
    public:
  using QStyledItemDelegate::QStyledItemDelegate;
  void paint(QPainter *p, const QStyleOptionViewItem &option,
             const QModelIndex &index) const override;
};

class PipelineLogModel : public QAbstractTableModel {
  Q_OBJECT
    public:
  enum Column { id = 0, stage, status, started, duration, error, count };
  enum Role { ToneRole = Qt::UserRole + 1 };
  explicit PipelineLogModel(QObject *parent = nullptr);
  void set_entries(std::vector<reusex::ProjectDB::PipelineLogEntry> entries);
  const reusex::ProjectDB::PipelineLogEntry &entry(int row) const {
    return entries_[static_cast<std::size_t>(row)];
  }
  int rowCount(const QModelIndex &parent = {}) const override;
  int columnCount(const QModelIndex &parent = {}) const override;
  QVariant data(const QModelIndex &index, int role) const override;
  QVariant headerData(int section, Qt::Orientation o, int role) const override;

    private:
  std::vector<reusex::ProjectDB::PipelineLogEntry> entries_;
};

/// The pipeline log filtered by stage, status and text (LogFilter).
class LogFilterProxy : public QSortFilterProxyModel {
  Q_OBJECT
    public:
  using QSortFilterProxyModel::QSortFilterProxyModel;
  void set_filter(LogFilter filter);
  const LogFilter &filter() const { return filter_; }

    protected:
  bool filterAcceptsRow(int row, const QModelIndex &parent) const override;

    private:
  LogFilter filter_;
};

/// The pipeline log: every stage run on the project, newest first. With
/// @p filters (the Log workspace) a bar filters it by stage, status and text.
/// A run of a stage the CLI can repeat carries its `rux` command.
class PipelineLogView : public QWidget {
  Q_OBJECT
    public:
  explicit PipelineLogView(ProjectSession &session, bool filters = false,
                           QWidget *parent = nullptr);
  void reload();
  /// Select the entry at visible @p row (and show it in the inspector).
  void select_row(int row);
  /// Apply a filter (the gallery; the bar's controls follow).
  void set_filter(const LogFilter &filter);
  /// Visible rows after filtering.
  int visible_rows() const;

    signals:
  void selection_changed(const rux::qt::Selection &selection);

    private:
  Selection entry_selection(int source_row) const;
  void sync_filter_controls();
  void update_meta();
  ProjectSession &session_;
  PipelineLogModel *model_ = nullptr;
  LogFilterProxy *proxy_ = nullptr;
  QTableView *view_ = nullptr;
  QLabel *meta_ = nullptr;
  QLabel *empty_ = nullptr;
  QComboBox *stage_ = nullptr;
  QLineEdit *search_ = nullptr;
  QWidget *status_ = nullptr;
  QPushButton *copy_ = nullptr;
  /// The selected entry's `rux` line (entry_selection() sets it).
  mutable QString command_;
};

/// Danish label and pill tone of a pipeline_log status.
QString log_status_da(const std::string &status, bool finished);
QString log_status_tone(const std::string &status, bool finished);

/// Header font with the caps tracking (QSS cannot letter-space).
void style_table(QTableView *view);

} // namespace rux::qt
