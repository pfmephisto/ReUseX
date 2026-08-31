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

class ProgressObserver {
    public:
  explicit ProgressObserver(Stage stage, size_t total = 0);
  ~ProgressObserver();

  void update(size_t progress = 1);

  inline void operator++() { update(1); };
  inline void operator+=(size_t increment) { update(increment); };

    private:
  Stage stage_;
  size_t total_ = 0;
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

void set_visual_observer(IVisualObserver *observer);
void set_progress_observer(IProgressObserver *observer);

void reset_visual_observer();
void reset_progress_observer();

auto get_visual_observer() -> IVisualObserver *;
auto get_progress_observer() -> IProgressObserver *;

} // namespace reusex::core
