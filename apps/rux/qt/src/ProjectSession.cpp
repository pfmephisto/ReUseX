// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/ProjectSession.hpp>

#include <reusex/core/logging.hpp>

#include <QElapsedTimer>
#include <QFileInfo>
#include <QThread>

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <thread>

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
      // ProjectDB::probe opens read-only, writes nothing and logs nothing.
      // Only "not a project" stops here; a lock or a read-only-WAL quirk is
      // the real open's to report.
      const auto probe = reusex::ProjectDB::probe(fs_path);
      if (!probe.is_project &&
          classify_open_error(probe.error) == OpenErrorKind::not_a_project)
        throw std::runtime_error(probe.error);
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

namespace {
std::atomic<int> g_in_flight{0};
} // namespace

/// Where a worker posts its result. The worker posts under the mutex, and the
/// session's destructor clears `owner` under it, so a post never races the
/// session's deletion; one already queued is dropped by ~QObject.
struct ProjectSession::Mailbox {
  std::mutex m;
  ProjectSession *owner = nullptr;
};

ProjectSession::ProjectSession(QObject *parent)
    : QObject(parent), mailbox_(std::make_shared<Mailbox>()) {
  mailbox_->owner = this;
}

ProjectSession::~ProjectSession() {
  // Never wait for a worker: an open stuck on a locked migration would hold
  // the window's close for the whole busy timeout. The worker finishes on
  // its own, finds no owner and closes its ProjectDB itself.
  std::lock_guard<std::mutex> lock(mailbox_->m);
  mailbox_->owner = nullptr;
}

int ProjectSession::opens_in_flight() { return g_in_flight.load(); }

bool ProjectSession::wait_for_opens(int timeout_ms) {
  QElapsedTimer t;
  t.start();
  while (g_in_flight.load() > 0) {
    if (t.elapsed() >= timeout_ms)
      return false;
    QThread::msleep(20);
  }
  return true;
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
  if (busy_) {
    has_pending_ = true;
    pending_path_ = abs;
    pending_read_only_ = read_only;
    path_ = abs;
    requested_read_only_ = read_only;
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
  busy_ = true;
  const unsigned gen = ++generation_;
  // A detached std::thread, not a QThread: nothing has to join it, so the
  // session (and the window) can go away while a locked open still waits.
  ++g_in_flight;
  std::thread([box = mailbox_, path, read_only, gen] {
    auto r = do_open(path, read_only);
    r->generation = gen;
    {
      std::lock_guard<std::mutex> lock(box->m);
      if (ProjectSession *owner = box->owner)
        QMetaObject::invokeMethod(
            owner, [owner, r] { owner->finish(r); }, Qt::QueuedConnection);
    }
    r.reset(); // without an owner the ProjectDB closes here, on the worker
    --g_in_flight;
  }).detach();
}

bool ProjectSession::open_blocking(const QString &path, bool read_only) {
  // An async open still running is superseded by the generation bump below.
  if (db_) {
    db_.reset();
    emit closed();
  }
  path_ = QFileInfo(path).absoluteFilePath();
  requested_read_only_ = read_only;
  set_state(State::loading);
  auto r = do_open(path_, read_only);
  r->generation = ++generation_;
  apply(r);
  return state_ == State::open;
}

void ProjectSession::finish(std::shared_ptr<Result> r) {
  busy_ = false;
  if (has_pending_) { // a newer open was asked for while this one ran
    has_pending_ = false;
    start_worker(pending_path_, pending_read_only_);
    return;
  }
  if (r->generation != generation_)
    return; // superseded (close(), open_blocking())
  apply(r);
}

void ProjectSession::apply(const std::shared_ptr<Result> &r) {
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
