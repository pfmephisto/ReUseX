// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/SensorIntrinsics.hpp"

#include <fmt/format.h>

#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>

namespace reusex::core {

std::string SensorIntrinsics::to_json() const {
  std::string lt = "[";
  for (size_t i = 0; i < 16; ++i) {
    if (i > 0)
      lt += ',';
    lt += fmt::format("{}", local_transform[i]);
  }
  lt += ']';

  return fmt::format(
      R"({{"fx":{},"fy":{},"cx":{},"cy":{},"width":{},"height":{},"local_transform":{}}})",
      fx, fy, cx, cy, width, height, lt);
}

// Minimal JSON parser — avoids pulling in a JSON library for six numbers and
// an array.  The format is fixed and emitted by to_json() above.
SensorIntrinsics SensorIntrinsics::from_json(const std::string &json) {
  SensorIntrinsics si;
  if (json.empty())
    return si;

  auto extract_double = [&](const char *key) -> double {
    auto pos = json.find(key);
    if (pos == std::string::npos)
      return 0.0;
    pos = json.find(':', pos);
    if (pos == std::string::npos)
      return 0.0;
    return std::strtod(json.c_str() + pos + 1, nullptr);
  };

  auto extract_int = [&](const char *key) -> int {
    auto pos = json.find(key);
    if (pos == std::string::npos)
      return 0;
    pos = json.find(':', pos);
    if (pos == std::string::npos)
      return 0;
    return static_cast<int>(std::strtol(json.c_str() + pos + 1, nullptr, 10));
  };

  si.fx = extract_double("\"fx\"");
  si.fy = extract_double("\"fy\"");
  si.cx = extract_double("\"cx\"");
  si.cy = extract_double("\"cy\"");
  si.width = extract_int("\"width\"");
  si.height = extract_int("\"height\"");

  // Parse local_transform array
  auto arr_pos = json.find("\"local_transform\"");
  if (arr_pos != std::string::npos) {
    auto bracket = json.find('[', arr_pos);
    if (bracket != std::string::npos) {
      const char *p = json.c_str() + bracket + 1;
      for (int i = 0; i < 16; ++i) {
        char *end = nullptr;
        si.local_transform[i] = std::strtod(p, &end);
        p = end;
        if (*p == ',')
          ++p;
      }
    }
  }

  return si;
}

} // namespace reusex::core
