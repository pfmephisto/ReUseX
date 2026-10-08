// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "create/stage_bridge.hpp"

#include <mutex>
#include <utility>

namespace rux {
namespace {
std::mutex g_sink_mutex;
StageParamsSink g_sink;
} // namespace

void set_stage_params_sink(StageParamsSink sink) {
  std::lock_guard lock(g_sink_mutex);
  g_sink = std::move(sink);
}

bool capture_stage_params(std::string_view stage, const std::string &params) {
  StageParamsSink sink;
  {
    std::lock_guard lock(g_sink_mutex);
    sink = g_sink;
  }
  if (!sink)
    return false;
  sink(stage, params);
  return true;
}

} // namespace rux
