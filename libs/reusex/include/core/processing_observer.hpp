// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/types.hpp"

#include <string_view>

namespace ReUseX::core {

enum class EventType {
  Process,
  Progress,
  Visualization
  //
};

class IProcessingObserver {
    public:
  virtual ~IProcessingObserver() = default;

  virtual void on_process_started(std::string_view, size_t) {}
  virtual void on_process_finished(std::string_view) {}
  virtual void on_process_updated(std::string_view, size_t) {}
};

class NullProcessingObserver final : public IProcessingObserver {};

// Register a global processing observer. The caller retains ownership and must
// keep the observer alive until reset or replacement. Passing nullptr clears
// it.
void set_processing_observer(IProcessingObserver *observer);
void reset_processing_observer();
auto get_processing_observer() -> IProcessingObserver *;

class ProgressObserver {
    public:
  ProgressObserver(std::string_view stage, size_t total = 0);
  ~ProgressObserver();

  void update(size_t progress = 1);

  inline void operator++() { update(1); };
  inline void operator+=(size_t increment) { update(increment); };

    private:
  std::string_view stage_;
  size_t total_ = 0;
};

// void emit_visualization(std::string_view stage, CloudConstPtr cloud);

} // namespace ReUseX::core
