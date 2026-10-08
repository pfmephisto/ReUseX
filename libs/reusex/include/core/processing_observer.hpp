// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// PCL-free core observer interface (STANDARDS §1: core/ must not pull PCL).
// The progress-reporting interface lives here; the PCL/Eigen-typed
// visualization payloads live in core/visual_observer.hpp, which core headers
// never include. The visual-observer registry below only traffics in an opaque
// IVisualObserver* (forward-declared), so this header stays PCL-free.

#include "reusex/core/stages.hpp"

#include <cstddef>

namespace reusex::core {

// Forward declaration: the full definition (with PCL/Eigen payloads) lives in
// core/visual_observer.hpp. Consumers that emit visualization events include
// that header; the registry here only stores/returns the pointer.
class IVisualObserver;

enum class EventType {
  process,
  progress,
  visualization
  //
};

class IObserver {};

class IProgressObserver;

/// One stage's progress report, RAII: started on construction, finished on
/// destruction, update() in between.
///
/// The observer it reports to is chosen ONCE, at construction, from the
/// constructing thread (current_progress_observer()). That is what makes
/// progress attributable per job when several stages run at once in one
/// process: a stage constructs its ProgressObserver on the thread that runs
/// it, under that job's ScopedProgressObserver, and every later update() —
/// including the ones TBB or OpenMP worker threads make from inside a parallel
/// loop, which carry no thread-local state of their own — still lands on that
/// job's observer.
class ProgressObserver {
    public:
  explicit ProgressObserver(Stage stage, size_t total = 0);
  ~ProgressObserver();

  ProgressObserver(const ProgressObserver &) = delete;
  ProgressObserver &operator=(const ProgressObserver &) = delete;

  void update(size_t progress = 1);

  inline void operator++() { update(1); };
  inline void operator+=(size_t increment) { update(increment); };

    private:
  Stage stage_;
  size_t total_ = 0;
  IProgressObserver *observer_ = nullptr; ///< Captured at construction.
};

class IProgressObserver : IObserver {
    public:
  virtual ~IProgressObserver() = default;

  // Progress bar callbacks
  virtual void on_process_started(Stage, size_t) {}
  virtual void on_process_finished(Stage) {}
  virtual void on_process_updated(Stage, size_t) {}
};

// Register a global processing observer. The caller retains ownership and must
// keep the observer alive until reset or replacement. Passing nullptr clears
// it.
//
// The global observer is the fallback for a thread with no
// ScopedProgressObserver installed — the rux CLI's progress bar. A server that
// runs several stages at once (ruxd's job scheduler) must not use it: it would
// interleave every job's progress into one observer. It installs a
// ScopedProgressObserver per job instead.

void set_visual_observer(IVisualObserver *observer);
void set_progress_observer(IProgressObserver *observer);

void reset_visual_observer();
void reset_progress_observer();

auto get_visual_observer() -> IVisualObserver *;
auto get_progress_observer() -> IProgressObserver *;

/// The observer progress reported from THIS thread goes to: the innermost
/// ScopedProgressObserver installed on it, else the global one (which may be
/// null).
auto current_progress_observer() -> IProgressObserver *;

/// Route progress started on this thread to @p observer for the lifetime of
/// the scope, without touching the process-global observer. Scopes nest; the
/// destructor reinstates whatever was current before. Must be destroyed on the
/// thread that created it.
///
/// Per-job progress in a process that runs several stages concurrently: each
/// job's worker thread installs its own scope, so two jobs never see each
/// other's events (see ProgressObserver for why updates made from pool threads
/// still arrive).
class ScopedProgressObserver {
    public:
  explicit ScopedProgressObserver(IProgressObserver *observer);
  ~ScopedProgressObserver();

  ScopedProgressObserver(const ScopedProgressObserver &) = delete;
  ScopedProgressObserver &operator=(const ScopedProgressObserver &) = delete;

    private:
  IProgressObserver *previous_ = nullptr;
  bool previous_set_ = false;
};

} // namespace reusex::core
