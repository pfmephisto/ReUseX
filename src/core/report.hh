#pragma once
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace ReUseX {

class Report {
    private:
  std::string _name;
  std::vector<std::string> _lines;
  std::vector<float> _values;

    public:
  Report(const std::string &name = "Profiller");
  void addLine(const std::string &line, float value);
  void print() const;
};

} // namespace ReUseX
