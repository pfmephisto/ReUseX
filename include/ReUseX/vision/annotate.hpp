// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <filesystem>

// #include "ReUseX/rtabmapTorchDataset.hpp"
// #include "ReUseX/Yolo.hpp"

// #include <spdlog/spdlog.h>
// #include <spdlog/stopwatch.h>
//
// #include <fmt/format.h>
// #include <fmt/std.h>

// #include "spdmon/spdmon.hpp"

// #include <rtabmap/core/DBDriver.h>
// #include <rtabmap/core/DBDriverSqlite3.h>
// #include <rtabmap/core/Graph.h>
// #include <rtabmap/core/Parameters.h>
// #include <rtabmap/core/Rtabmap.h>

namespace fs = std::filesystem;

namespace ReUseX::vision {
auto annotateRTABMap(const fs::path &dbPath, const fs::path &modelPath,
                     bool isCuda) -> int;
}
