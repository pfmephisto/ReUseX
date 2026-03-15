// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string_view>

namespace ReUseX::core {

class IProcessingObserver {
public:
  virtual ~IProcessingObserver() = default;

  virtual void on_stage_started(std::string_view stage) {}
  virtual void on_progress(std::string_view stage, float progress) {}
  virtual void on_warning(std::string_view message) {}
  virtual void on_error(std::string_view message) {}
};

class NullProcessingObserver final : public IProcessingObserver {};

} // namespace ReUseX::core
