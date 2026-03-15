// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "processing_observer.hpp"

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

SpdlogProcessingObserver g_processing_observer;

} // namespace

void enable_processing_observer(bool enabled) {
  if (enabled) {
    ReUseX::core::set_processing_observer(&g_processing_observer);
    return;
  }
  ReUseX::core::reset_processing_observer();
}

} // namespace rux
