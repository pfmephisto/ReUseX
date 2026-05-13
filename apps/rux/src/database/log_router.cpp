// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/log_router.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <charconv>

namespace rux::database {

namespace {

nlohmann::json entry_to_json(const reusex::ProjectDB::PipelineLogEntry &e) {
  nlohmann::json j;
  j["id"] = e.id;
  j["stage"] = e.stage;
  j["started_at"] = e.started_at;
  j["finished_at"] =
      e.finished_at.empty() ? nlohmann::json(nullptr) : nlohmann::json(e.finished_at);
  j["status"] = e.status;
  j["error_msg"] =
      e.error_msg.empty() ? nlohmann::json(nullptr) : nlohmann::json(e.error_msg);
  if (!e.parameters.empty()) {
    try {
      j["parameters"] = nlohmann::json::parse(e.parameters);
    } catch (const nlohmann::json::parse_error &) {
      j["parameters"] = e.parameters;
    }
  } else {
    j["parameters"] = nlohmann::json(nullptr);
  }
  return j;
}

std::optional<int> parse_id(std::string_view s) {
  int value = 0;
  auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc{} || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

} // namespace

std::vector<std::string> LogRouter::list() const {
  auto entries = db_->pipeline_log();
  std::vector<std::string> ids;
  ids.reserve(entries.size());
  for (const auto &e : entries) {
    ids.push_back(std::to_string(e.id));
  }
  return ids; // Newest first (DESC by id from DB query)
}

DataPayload LogRouter::get(const std::vector<PathComponent> &components) {
  auto entries = db_->pipeline_log();

  if (components.empty()) {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto &e : entries) {
      arr.push_back(entry_to_json(e));
    }
    return arr;
  }

  // Resolve target entry
  const reusex::ProjectDB::PipelineLogEntry *match = nullptr;

  if (components[0].is_index()) {
    int idx = *components[0].index;
    if (idx < 0 || idx >= static_cast<int>(entries.size())) {
      throw std::runtime_error("Array index out of range: " +
                               std::to_string(idx));
    }
    match = &entries[idx];
  } else if (components[0].is_item()) {
    auto id_opt = parse_id(components[0].value);
    if (!id_opt) {
      throw std::runtime_error(
          "Log entry must be referenced by numeric id, got: " +
          components[0].value);
    }
    auto it = std::find_if(entries.begin(), entries.end(),
                           [&](const auto &e) { return e.id == *id_opt; });
    if (it == entries.end()) {
      throw std::runtime_error("Log entry not found: id=" +
                               std::to_string(*id_opt));
    }
    match = &*it;
  } else {
    throw std::runtime_error("Expected entry id or index after collection");
  }

  if (components.size() == 1) {
    return entry_to_json(*match);
  }

  const auto &prop = components[1].value;
  if (prop == "stage")
    return match->stage;
  if (prop == "started_at")
    return match->started_at;
  if (prop == "finished_at")
    return match->finished_at;
  if (prop == "status")
    return match->status;
  if (prop == "error_msg")
    return match->error_msg;
  if (prop == "parameters") {
    if (match->parameters.empty()) {
      return nlohmann::json(nullptr);
    }
    try {
      return nlohmann::json::parse(match->parameters);
    } catch (const nlohmann::json::parse_error &) {
      return match->parameters;
    }
  }

  throw std::runtime_error("Unknown property: " + prop +
                           "\nAvailable properties: stage, started_at, "
                           "finished_at, parameters, status, error_msg");
}

void LogRouter::set(const std::vector<PathComponent> & /*components*/,
                    const DataPayload & /*data*/) {
  throw std::runtime_error(
      "Cannot set pipeline_log entries via 'rux set'. "
      "Entries are written automatically by pipeline subcommands.");
}

void LogRouter::del(const std::vector<PathComponent> & /*components*/) {
  throw std::runtime_error(
      "Cannot delete pipeline_log entries — the log is append-only.");
}

} // namespace rux::database
