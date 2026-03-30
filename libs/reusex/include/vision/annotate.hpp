// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <filesystem>

namespace ReUseX::vision {
auto annotate(const std::filesystem::path &dbPath,
              const std::filesystem::path &modelPath) -> int;
} // namespace ReUseX::vision
