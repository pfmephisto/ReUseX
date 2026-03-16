// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string_view>

namespace ReUseX::core {

class IProcessingObserver {
public:
  virtual ~IProcessingObserver() = default;

  /// Called whenever processing enters a new stage.
  /// Stage identifiers are stable string keys intended for UI state mapping.
  /// The same stage may be reported more than once in future pipelines.
  virtual void on_stage_started(std::string_view stage) {}
  /// Called with stage-local progress in the range [0.0, 1.0].
  /// This callback may be invoked multiple times for the same stage.
  /// Implementations should be thread-safe if shared across worker threads.
  virtual void on_progress(std::string_view stage, float progress) {}
  /// Called for recoverable warnings that do not abort processing.
  virtual void on_warning(std::string_view message) {}
  /// Called right before a non-recoverable error is propagated to the caller.
  /// For current synchronous pipelines, `on_error(...)` is terminal.
  virtual void on_error(std::string_view message) {}
};

class NullProcessingObserver final : public IProcessingObserver {};

// Register a global processing observer. The caller retains ownership and must
// keep the observer alive until reset or replacement. Passing nullptr clears it.
void set_processing_observer(IProcessingObserver *observer);
void reset_processing_observer();
auto get_processing_observer() -> IProcessingObserver *;

} // namespace ReUseX::core
