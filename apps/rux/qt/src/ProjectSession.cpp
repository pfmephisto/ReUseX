// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>

#include <reusex/core/logging.hpp>

#include <QElapsedTimer>
#include <QFileInfo>
#include <QThread>

namespace rux::qt {

struct ProjectSession::Result {
  QString path;
  bool requested_read_only = false;
  bool read_only = false;
  QString read_only_reason;
  std::unique_ptr<reusex::ProjectDB> db;
  reusex::ProjectDB::ProjectSummary summary{};
  Error error;
  qint64 ms = 0;
  unsigned generation = 0;
};

namespace {

ProjectSession::Error make_error(OpenErrorKind kind, const QString &detail) {
  return {kind, QString::fromStdString(open_error_title_da(kind)),
          QString::fromStdString(open_error_hint_da(kind)), detail};
}

/// Everything an open does, on whichever thread calls it. Never throws.
std::shared_ptr<ProjectSession::Result> do_open(const QString &path,
                                                bool read_only) {
  auto r = std::make_shared<ProjectSession::Result>();
  r->path = path;
  r->requested_read_only = read_only;
  QElapsedTimer clock;
  clock.start();

  const QFileInfo fi(path);
  if (!fi.exists()) {
    r->error = make_error(OpenErrorKind::not_found, path);
    return r;
  }
  if (!fi.isFile()) {
    r->error = make_error(OpenErrorKind::not_a_file, path);
    return r;
  }
  if (!fi.isReadable()) {
    r->error = make_error(OpenErrorKind::permission, path);
    return r;
  }

  bool ro = read_only;
  if (read_only) {
    r->read_only_reason = "Åbnet skrivebeskyttet efter eget valg.";
  } else if (!fi.isWritable() || !QFileInfo(fi.absolutePath()).isWritable()) {
    // sqlite's WAL needs the -wal/-shm files next to the project, so a
    // read-only directory is as good as a read-only file.
    ro = true;
    r->read_only_reason =
        fi.isWritable() ? "Mappen, projektet ligger i, er skrivebeskyttet."
                        : "Filen er skrivebeskyttet.";
  }

  const std::filesystem::path fs_path = path.toStdString();
  try {
    // Probe read-only first: a read-write open of a foreign sqlite file (or
    // an empty file) would create every ReUseX table in it.
    if (!ro) {
      try {
        // The probe's read-only open warns that an older schema was not
        // migrated — true of the probe, misleading in the log, since the
        // real open right after migrates. Quiet it for the probe alone.
        // (The level is process-global; a line another thread logs in
        // these milliseconds at warn level would be dropped.)
        const auto level = reusex::core::get_log_level();
        if (level < reusex::core::LogLevel::error)
          reusex::core::set_log_level(reusex::core::LogLevel::error);
        struct Restore {
          reusex::core::LogLevel l;
          ~Restore() { reusex::core::set_log_level(l); }
        } restore{level};
        reusex::ProjectDB probe(fs_path, /*readOnly=*/true);
      } catch (const std::exception &e) {
        if (classify_open_error(e.what()) == OpenErrorKind::not_a_project)
          throw;
        // Anything else (a WAL quirk of read-only opens) is the real open's
        // to report.
      }
    }
    try {
      r->db = std::make_unique<reusex::ProjectDB>(fs_path, ro);
    } catch (const std::exception &e) {
      if (ro || classify_open_error(e.what()) != OpenErrorKind::permission)
        throw;
      // sqlite refused to write after all (a read-only mount): read it.
      ro = true;
      r->read_only_reason =
          "Projektet kan ikke skrives (skrivebeskyttet drev?).";
      r->db = std::make_unique<reusex::ProjectDB>(fs_path, true);
    }
    r->read_only = ro;
    r->summary = r->db->project_summary();
  } catch (const std::exception &e) {
    r->db.reset();
    r->error = make_error(classify_open_error(e.what()), e.what());
  }
  if (!r->db)
    r->read_only_reason.clear();
  r->ms = clock.elapsed();
  return r;
}

} // namespace

ProjectSession::ProjectSession(QObject *parent) : QObject(parent) {}

ProjectSession::~ProjectSession() {
  if (worker_) {
    // A ProjectDB constructor cannot be interrupted; it ends within the busy
    // timeout at worst. Its result is dropped with this object.
    worker_->wait();
  }
}

QString ProjectSession::display_name() const {
  if (state_ == State::open && !summary_.projects.empty() &&
      !summary_.projects.front().name.empty())
    return QString::fromStdString(summary_.projects.front().name);
  return path_.isEmpty() ? QString() : QFileInfo(path_).completeBaseName();
}

void ProjectSession::set_state(State s) {
  state_ = s;
  emit state_changed(s);
}

void ProjectSession::open(const QString &path, bool read_only) {
  const QString abs =
      path.isEmpty() ? path : QFileInfo(path).absoluteFilePath();
  if (worker_) {
    has_pending_ = true;
    pending_path_ = abs;
    pending_read_only_ = read_only;
    path_ = abs;
    set_state(State::loading);
    return;
  }
  start_worker(abs, read_only);
}

void ProjectSession::start_worker(const QString &path, bool read_only) {
  // Release the current project first: a reopen of the same file must not
  // race its own connection, and the workspaces drop their views on closed().
  if (db_) {
    db_.reset();
    emit closed();
  }
  path_ = path;
  requested_read_only_ = read_only;
  error_ = {};
  set_state(State::loading);
  const unsigned gen = ++generation_;
  // `this` outlives the worker: the destructor waits for it. A result
  // posted just before destruction is discarded with the object's events.
  worker_ = QThread::create([this, path, read_only, gen] {
    auto r = do_open(path, read_only);
    r->generation = gen;
    QMetaObject::invokeMethod(
        this, [this, r] { finish(r); }, Qt::QueuedConnection);
  });
  worker_->setObjectName("rux-project-open");
  connect(worker_, &QThread::finished, worker_, &QObject::deleteLater);
  connect(worker_, &QThread::finished, this, [this, w = worker_] {
    if (worker_ == w)
      worker_ = nullptr;
    if (has_pending_ && !worker_) {
      has_pending_ = false;
      start_worker(pending_path_, pending_read_only_);
    }
  });
  worker_->start();
}

bool ProjectSession::open_blocking(const QString &path, bool read_only) {
  if (worker_)
    worker_->wait();
  if (db_) {
    db_.reset();
    emit closed();
  }
  path_ = QFileInfo(path).absoluteFilePath();
  requested_read_only_ = read_only;
  set_state(State::loading);
  auto r = do_open(path_, read_only);
  r->generation = ++generation_;
  finish(r);
  return state_ == State::open;
}

void ProjectSession::finish(std::shared_ptr<Result> r) {
  if (r->generation != generation_ || has_pending_)
    return; // superseded by a newer open
  load_ms_ = r->ms;
  if (r->db) {
    db_ = std::move(r->db);
    summary_ = std::move(r->summary);
    read_only_ = r->read_only;
    read_only_reason_ = r->read_only_reason;
    error_ = {};
    reusex::info("Qt client: opened {} in {} ms{}", r->path.toStdString(),
                 r->ms, read_only_ ? " (read-only)" : "");
    set_state(State::open);
    emit opened();
  } else {
    summary_ = {};
    read_only_ = false;
    read_only_reason_.clear();
    error_ = r->error;
    reusex::warn("Qt client: cannot open {}: {}", r->path.toStdString(),
                 r->error.detail.toStdString());
    set_state(State::failed);
    emit open_failed();
  }
}

void ProjectSession::reload() {
  if (!path_.isEmpty())
    open(path_, requested_read_only_);
}

void ProjectSession::close() {
  ++generation_; // drop an open still in flight
  has_pending_ = false;
  const bool had = db_ != nullptr;
  db_.reset();
  summary_ = {};
  path_.clear();
  read_only_ = false;
  read_only_reason_.clear();
  error_ = {};
  set_state(State::empty);
  if (had)
    emit closed();
}

} // namespace rux::qt
