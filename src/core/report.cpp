#include "report.hh"
#include <spdlog/spdlog.h>

namespace ReUseX {

Report::Report(const std::string &name) : _name(name) {};

void Report::addLine(const std::string &line, float value) {
  _lines.push_back(line);
  _values.push_back(value);
}

void Report::print() const {

  size_t max_line_width = 0;
  for (const auto &line : _lines)
    max_line_width = std::max(max_line_width, line.size());

  size_t max_value_width = 0;
  for (const auto &value : _values)
    max_value_width =
        std::max(max_value_width, fmt::format("{:.3f}s", value).size());

  spdlog::info(
      fmt::format("{:=^{}}=", "", max_line_width + max_value_width + 2));
  spdlog::info("Report: {}", _name);
  for (size_t i = 0; i < _lines.size(); ++i)
    spdlog::info("{:<{}} - {:>{}.3f}s", _lines[i], max_line_width, _values[i],
                 max_value_width - 1);
  spdlog::info(
      fmt::format("{:=^{}}=", "", max_line_width + max_value_width + 2));
}

} // namespace ReUseX
