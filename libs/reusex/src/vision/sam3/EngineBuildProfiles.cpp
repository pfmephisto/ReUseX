// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/sam3/EngineBuildProfiles.hpp"
#include "reusex/vision/sam3/engine_build_default.hpp"

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string_view>

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
  out.recipe_version = root.value("recipe_version", 1);
  if (out.recipe_version < 1)
    throw std::runtime_error(
        fmt::format("engine-build.json: recipe_version must be >= 1 (got {})",
                    out.recipe_version));

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

  if (auto fb = root.find("fallback_engines"); fb != root.end()) {
    if (!fb->is_array())
      throw std::runtime_error(
          "engine-build.json: 'fallback_engines' must be an array of names");
    for (const auto &n : *fb) {
      if (!n.is_string())
        throw std::runtime_error(
            "engine-build.json: 'fallback_engines' must be an array of names");
      out.fallback_engines.push_back(n.get<std::string>());
    }
  }
  if (auto fp = root.find("fallback_onnx_sha256"); fp != root.end()) {
    if (!fp->is_object())
      throw std::runtime_error("engine-build.json: 'fallback_onnx_sha256' "
                               "must map engine names to digests");
    for (const auto &[name, digest] : fp->items()) {
      if (!digest.is_string())
        throw std::runtime_error("engine-build.json: 'fallback_onnx_sha256' "
                                 "must map engine names to digests");
      out.fallback_onnx_sha256.emplace(name, digest.get<std::string>());
    }
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

std::string EngineBuildProfiles::to_json() const {
  nlohmann::json engines_json = nlohmann::json::object();
  for (const auto &[name, prof] : engines) {
    nlohmann::json shapes = nlohmann::json::object();
    for (const auto &[input, sp] : prof.shapes)
      shapes[input] = {{"min", sp.min}, {"opt", sp.opt}, {"max", sp.max}};
    engines_json[name] = {{"precision", prof.precision},
                          {"workspace_mb", prof.workspace_mb},
                          {"shapes", shapes}};
  }
  nlohmann::json root{{"schema_version", schema_version},
                      {"recipe_version", recipe_version},
                      {"engines", engines_json}};
  if (!fallback_engines.empty())
    root["fallback_engines"] = fallback_engines;
  if (!fallback_onnx_sha256.empty())
    root["fallback_onnx_sha256"] = fallback_onnx_sha256;
  return root.dump(2);
}

std::optional<EngineProfile> text_only_fallback(const std::string &engine_name,
                                                const EngineProfile &profile) {
  // input name -> pin the geometry axis (1) to its min (true) or max (false)
  std::vector<std::pair<std::string, bool>> pins;
  if (engine_name == "decoder")
    pins = {{"prompt_features", true}, {"prompt_mask", true}};
  else if (engine_name == "geometry-encoder")
    pins = {{"input_boxes", false}, {"input_boxes_labels", false}};
  else
    return std::nullopt;

  EngineProfile out = profile;
  for (const auto &[input, use_min] : pins) {
    auto it = out.shapes.find(input);
    if (it == out.shapes.end() || it->second.min.size() < 2 ||
        it->second.opt.size() < 2 || it->second.max.size() < 2)
      continue;
    auto &sp = it->second;
    const int v = use_min ? sp.min[1] : sp.max[1];
    sp.min[1] = sp.opt[1] = sp.max[1] = v;
  }
  return out;
}

std::optional<std::string>
profile_shape_conflict(const std::vector<long long> &network_dims,
                       const EngineProfile::ShapeProfile &profile) {
  for (const auto *dims : {&profile.min, &profile.opt, &profile.max})
    if (dims->size() != network_dims.size())
      return fmt::format("rank {} in the network, {} in the profile",
                         network_dims.size(), dims->size());
  for (std::size_t i = 0; i < network_dims.size(); ++i) {
    const long long d = network_dims[i];
    if (d < 0)
      continue;
    if (profile.min[i] != d || profile.opt[i] != d || profile.max[i] != d)
      return fmt::format("axis {} is fixed at {} in the network but the "
                         "profile asks for {}..{}",
                         i, d, profile.min[i], profile.max[i]);
  }
  return std::nullopt;
}

bool is_shape_build_error(const std::vector<std::string> &builder_errors) {
  auto lower = [](std::string s) {
    for (auto &c : s)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  };
  auto has_any = [](const std::string &s,
                    std::initializer_list<std::string_view> words) {
    return std::any_of(words.begin(), words.end(), [&](std::string_view w) {
      return s.find(w) != std::string::npos;
    });
  };
  bool shape = false;
  for (const auto &raw : builder_errors) {
    const std::string m = lower(raw);
    if (has_any(m, {"memory", "alloc", "oom", "disk", "no space",
                    "insufficient", "cuda error", "cudaerror"}))
      return false;
    if (has_any(m, {"shape", "dimension", "dims", "profile", "reshape",
                    "volume", "broadcast", "mismatch"}))
      shape = true;
  }
  return shape;
}

std::optional<EngineProfile>
EngineBuildProfiles::find(const std::string &engine_name) const {
  auto it = engines.find(engine_name);
  if (it == engines.end())
    return std::nullopt;
  return it->second;
}

} // namespace reusex::vision::sam3
