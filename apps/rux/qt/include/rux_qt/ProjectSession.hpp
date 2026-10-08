// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The open project of the Qt client: owns the one ProjectDB and tells the
// shell (and, from Q2, every workspace) what happened to it through signals.
//
// Opening runs on a detached worker thread — a big project's first open
// migrates its schema, and a locked one waits out the 5 s sqlite busy
// timeout — while the session reports State::loading. The session never
// joins it, so destroying the session (closing the window) never blocks. The
// finished ProjectDB is handed to the GUI thread and used only there from then
// on (ProjectDB is not thread-safe; handing a connection over between uses is
// fine for sqlite). Every signal is emitted on the GUI thread.
//
// A file is vetted with ProjectDB::probe (read-only, silent) before the
// read-write open, so a foreign sqlite file is reported as "not a ReUseX
// project" instead of being given ReUseX's tables. A file or directory the user
// cannot write is opened read-only, and the session says so.

#include <rux_qt/launch.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QObject>
#include <QString>

#include <memory>

namespace rux::qt {

class ProjectSession : public QObject {
  Q_OBJECT
    public:
  enum class State { empty, loading, open, failed };

  struct Error {
    OpenErrorKind kind = OpenErrorKind::none;
    QString title;  ///< Danish headline
    QString hint;   ///< Danish, what to do
    QString detail; ///< the raw exception text
  };

  explicit ProjectSession(QObject *parent = nullptr);
  ~ProjectSession() override;

  /// Open @p path on a worker thread. A call while another open runs is
  /// queued: the newest request wins and the older result is discarded.
  void open(const QString &path, bool read_only = false);
  /// Open on the calling thread (the gallery, tests). Returns success.
  bool open_blocking(const QString &path, bool read_only = false);
  /// Re-open the current (or last failed) path.
  void reload();
  void close();

  State state() const { return state_; }
  bool is_open() const { return state_ == State::open; }
  /// The open project, the one being opened, or the one that failed.
  QString path() const { return path_; }
  /// Display name: the project's own name, else the file's base name.
  QString display_name() const;

  bool is_read_only() const { return read_only_; }
  /// Why the project is read-only (Danish); empty when it is not.
  QString read_only_reason() const { return read_only_reason_; }

  /// nullptr unless open. GUI thread only.
  reusex::ProjectDB *db() const { return db_.get(); }
  /// Valid while open (taken at open time; reload() refreshes it).
  const reusex::ProjectDB::ProjectSummary &summary() const { return summary_; }
  const Error &error() const { return error_; }
  /// Milliseconds the last open took.
  qint64 load_ms() const { return load_ms_; }

  /// The result of one open attempt, built off the GUI thread.
  struct Result;

  /// Opens still running on worker threads, across all sessions (a session
  /// destroyed mid-open leaves its worker to finish alone).
  static int opens_in_flight();
  /// Wait up to @p timeout_ms for them; true if none is left. run_app calls
  /// it after the window has closed, so a quit never looks hung.
  static bool wait_for_opens(int timeout_ms);

    signals:
  void state_changed(rux::qt::ProjectSession::State state);
  void opened();
  void closed();
  void open_failed();

    private:
  void start_worker(const QString &path, bool read_only);
  void finish(std::shared_ptr<Result> r);
  void apply(const std::shared_ptr<Result> &r);
  struct Mailbox;
  void set_state(State s);

  State state_ = State::empty;
  QString path_;
  bool read_only_ = false;
  bool requested_read_only_ = false;
  QString read_only_reason_;
  std::unique_ptr<reusex::ProjectDB> db_;
  reusex::ProjectDB::ProjectSummary summary_{};
  Error error_;
  qint64 load_ms_ = 0;

  std::shared_ptr<Mailbox> mailbox_;
  bool busy_ = false; ///< a worker is running for this session
  unsigned generation_ = 0;
  bool has_pending_ = false;
  QString pending_path_;
  bool pending_read_only_ = false;
};

} // namespace rux::qt
