// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/processing_observer.hpp>

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

} // namespace ReUseX::core
