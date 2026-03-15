// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "processing_observer.hpp"

#include <atomic>
#include <spdlog/spdlog.h>

namespace rux {
namespace {

class SpdlogProcessingObserver final : public ReUseX::core::IProcessingObserver {
public:
  void on_stage_started(std::string_view stage) override {
    spdlog::info("Processing stage: {}", stage);
  }

  void on_progress(std::string_view stage, float progress) override {
    spdlog::info("Processing progress [{}]: {:.0f}%", stage, progress * 100.0F);
  }

  void on_warning(std::string_view message) override {
    spdlog::warn("Processing warning: {}", message);
  }

  void on_error(std::string_view message) override {
    spdlog::error("Processing error: {}", message);
  }
};

std::atomic_bool g_processing_observer_enabled{false};
SpdlogProcessingObserver g_processing_observer;

} // namespace

void enable_processing_observer(bool enabled) {
  g_processing_observer_enabled.store(enabled, std::memory_order_release);
}

auto processing_observer() -> ReUseX::core::IProcessingObserver * {
  return g_processing_observer_enabled.load(std::memory_order_acquire)
             ? &g_processing_observer
             : nullptr;
}

} // namespace rux
