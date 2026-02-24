// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <ReUseX/vision/IData.hpp>
#include <filesystem>
#include <sqlite3.h>
#include <vector>

namespace ReUseX::vision {
class IDataset {
    public:
  IDataset(std::filesystem::path dbPath);
  ~IDataset();

  size_t size() const;

  virtual std::unique_ptr<IData> get(const std::size_t index) const = 0;

    protected:
  sqlite3 *db_ = nullptr;
  std::vector<int> ids_ = {};
};
} // namespace ReUseX::vision
