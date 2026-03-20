// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/processing_observer.hpp"
#include "reusex/core/logging.hpp"

#include <atomic>

namespace ReUseX::core {
namespace {
// Global observer pointer. ReUseX does not own this object; callers must ensure
// the observer outlives all in-flight processing that may read it. Acquire/
// release semantics provide thread-safe publication of pointer updates, while
// observer implementations themselves must be thread-safe if shared.
std::atomic<IProcessingObserver *> g_processing_observer{nullptr};
} // namespace

void set_processing_observer(IProcessingObserver *observer) {
  g_processing_observer.store(observer, std::memory_order_release);
}

void reset_processing_observer() { set_processing_observer(nullptr); }

auto get_processing_observer() -> IProcessingObserver * {
  return g_processing_observer.load(std::memory_order_acquire);
}

ProgressObserver::ProgressObserver(Stage stage, size_t total)
    : stage_(stage), total_(total) {
  if (auto *observer = get_processing_observer())
    observer->on_process_started(stage, total);
}

ProgressObserver::~ProgressObserver() {
  if (auto *observer = get_processing_observer())
    observer->on_process_finished(stage_);
}

void ProgressObserver::update(size_t progress) {
  if (auto *observer = get_processing_observer())
    observer->on_process_updated(stage_, progress);
};

} // namespace ReUseX::core
