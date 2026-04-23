// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <filesystem>

namespace reusex::vision {
auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath,
              bool use_cuda = false) -> int;
} // namespace reusex::vision
