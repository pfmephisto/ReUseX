// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/processing_observer.hpp"
#include "reusex/core/logging.hpp"

#include <atomic>

namespace reusex::core {
namespace {
// Global observer pointer. ReUseX does not own this object; callers must ensure
// the observer outlives all in-flight processing that may read it. Acquire/
// release semantics provide thread-safe publication of pointer updates, while
// observer implementations themselves must be thread-safe if shared.
std::atomic<IProgressObserver *> g_progress_observer{nullptr};
std::atomic<IVisualObserver *> g_visual_observer{nullptr};

// Per-thread override installed by ScopedProgressObserver. `t_scoped_set`
// distinguishes "a scope routes to nullptr" (progress deliberately dropped)
// from "no scope" (fall back to the global observer).
thread_local IProgressObserver *t_scoped_observer = nullptr;
thread_local bool t_scoped_set = false;
} // namespace

void set_progress_observer(IProgressObserver *observer) {
  g_progress_observer.store(observer, std::memory_order_release);
}

void reset_progress_observer() { set_progress_observer(nullptr); }

auto get_progress_observer() -> IProgressObserver * {
  return g_progress_observer.load(std::memory_order_acquire);
}

auto current_progress_observer() -> IProgressObserver * {
  return t_scoped_set ? t_scoped_observer : get_progress_observer();
}

ScopedProgressObserver::ScopedProgressObserver(IProgressObserver *observer)
    : previous_(t_scoped_observer), previous_set_(t_scoped_set) {
  t_scoped_observer = observer;
  t_scoped_set = true;
}

ScopedProgressObserver::~ScopedProgressObserver() {
  t_scoped_observer = previous_;
  t_scoped_set = previous_set_;
}

void set_visual_observer(IVisualObserver *observer) {
  g_visual_observer.store(observer, std::memory_order_release);
}

void reset_visual_observer() { set_visual_observer(nullptr); }

auto get_visual_observer() -> IVisualObserver * {
  return g_visual_observer.load(std::memory_order_acquire);
}

ProgressObserver::ProgressObserver(Stage stage, size_t total)
    : stage_(stage), total_(total), observer_(current_progress_observer()) {
  if (observer_ != nullptr)
    observer_->on_process_started(stage, total);
}

ProgressObserver::~ProgressObserver() {
  if (observer_ != nullptr)
    observer_->on_process_finished(stage_);
}

void ProgressObserver::update(size_t progress) {
  if (observer_ != nullptr)
    observer_->on_process_updated(stage_, progress);
}

} // namespace reusex::core
