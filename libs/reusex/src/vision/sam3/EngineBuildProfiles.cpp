// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/sam3/EngineBuildProfiles.hpp"
#include "reusex/vision/sam3/engine_build_default.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

namespace reusex::vision::sam3 {

namespace {

std::vector<int> parse_dims(const nlohmann::json &j, const std::string &engine,
                            const std::string &input, const char *which) {
  if (!j.is_array())
    throw std::runtime_error(
        fmt::format("engine-build.json: engines.{}.shapes.{}.{} must be an "
                    "array of integers",
                    engine, input, which));
  std::vector<int> dims;
  dims.reserve(j.size());
  for (const auto &d : j) {
    if (!d.is_number_integer())
      throw std::runtime_error(
          fmt::format("engine-build.json: engines.{}.shapes.{}.{} contains a "
                      "non-integer dimension",
                      engine, input, which));
    dims.push_back(d.get<int>());
  }
  return dims;
}

EngineBuildProfiles parse(const nlohmann::json &root) {
  EngineBuildProfiles out;
  out.schema_version = root.value("schema_version", 1);
  if (out.schema_version != 1)
    throw std::runtime_error(fmt::format(
        "engine-build.json: unsupported schema_version {} (expected 1)",
        out.schema_version));

  auto engines_it = root.find("engines");
  if (engines_it == root.end() || !engines_it->is_object())
    throw std::runtime_error(
        "engine-build.json: missing or malformed 'engines' object");

  for (const auto &[name, spec] : engines_it->items()) {
    EngineProfile prof;
    prof.precision = spec.value("precision", std::string("fp16"));
    if (prof.precision != "fp16" && prof.precision != "fp32")
      throw std::runtime_error(
          fmt::format("engine-build.json: engines.{}.precision must be 'fp16' "
                      "or 'fp32' (got '{}')",
                      name, prof.precision));
    prof.workspace_mb = spec.value("workspace_mb", 8192);

    auto shapes_it = spec.find("shapes");
    if (shapes_it != spec.end() && shapes_it->is_object()) {
      for (const auto &[input, triple] : shapes_it->items()) {
        EngineProfile::ShapeProfile sp;
        sp.min = parse_dims(triple.at("min"), name, input, "min");
        sp.opt = parse_dims(triple.at("opt"), name, input, "opt");
        sp.max = parse_dims(triple.at("max"), name, input, "max");
        prof.shapes.emplace(input, std::move(sp));
      }
    }
    out.engines.emplace(name, std::move(prof));
  }
  return out;
}

} // namespace

EngineBuildProfiles
EngineBuildProfiles::from_file(const std::filesystem::path &path) {
  std::ifstream in(path);
  if (!in.is_open())
    throw std::runtime_error(
        fmt::format("engine-build.json: cannot open {}", path.string()));
  nlohmann::json root;
  try {
    in >> root;
  } catch (const nlohmann::json::exception &e) {
    throw std::runtime_error(fmt::format(
        "engine-build.json: parse error in {}: {}", path.string(), e.what()));
  }
  return parse(root);
}

EngineBuildProfiles EngineBuildProfiles::from_string(const std::string &json) {
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(json);
  } catch (const nlohmann::json::exception &e) {
    throw std::runtime_error(
        fmt::format("engine-build.json: parse error: {}", e.what()));
  }
  return parse(root);
}

const EngineBuildProfiles &EngineBuildProfiles::builtin() {
  static const EngineBuildProfiles profiles =
      from_string(detail::kDefaultEngineBuildJson);
  return profiles;
}

std::optional<EngineProfile>
EngineBuildProfiles::find(const std::string &engine_name) const {
  auto it = engines.find(engine_name);
  if (it == engines.end())
    return std::nullopt;
  return it->second;
}

} // namespace reusex::vision::sam3
