// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "log.hpp"
#include <reusex/core/ProjectDB.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>

using json = nlohmann::json;

namespace {

// Calculate duration between two ISO timestamps
std::string calculate_duration(const std::string &start, const std::string &end) {
  if (end.empty()) {
    return "running";
  }

  // Simple duration calculation (this could be more sophisticated)
  // Format: YYYY-MM-DD HH:MM:SS
  std::tm start_tm = {}, end_tm = {};
  std::istringstream ss_start(start);
  std::istringstream ss_end(end);

  ss_start >> std::get_time(&start_tm, "%Y-%m-%d %H:%M:%S");
  ss_end >> std::get_time(&end_tm, "%Y-%m-%d %H:%M:%S");

  auto start_time = std::mktime(&start_tm);
  auto end_time = std::mktime(&end_tm);

  if (start_time == -1 || end_time == -1) {
    return "unknown";
  }

  double diff = std::difftime(end_time, start_time);
  int seconds = static_cast<int>(diff);

  if (seconds < 60) {
    return std::to_string(seconds) + "s";
  } else if (seconds < 3600) {
    int mins = seconds / 60;
    int secs = seconds % 60;
    return std::to_string(mins) + "m " + std::to_string(secs) + "s";
  } else {
    int hours = seconds / 3600;
    int mins = (seconds % 3600) / 60;
    return std::to_string(hours) + "h " + std::to_string(mins) + "m";
  }
}

// Format terminal output with execution history
void format_terminal_output(const std::vector<ReUseX::ProjectDB::PipelineLogEntry> &entries) {
  if (entries.empty()) {
    std::cout << "No pipeline executions recorded.\n";
    return;
  }

  std::cout << fmt::format("Pipeline Execution History ({} entries)\n", entries.size());
  std::cout << fmt::format("{}\n\n", std::string(80, '='));

  for (const auto &entry : entries) {
    // Status indicator
    std::string status_icon;
    if (entry.status == "success") {
      status_icon = "✓";
    } else if (entry.status == "failed") {
      status_icon = "✗";
    } else {
      status_icon = "⋯";
    }

    std::cout << fmt::format("{} [{}] {}\n", status_icon, entry.id, entry.stage);
    std::cout << fmt::format("  Started:  {}\n", entry.started_at);

    if (!entry.finished_at.empty()) {
      std::cout << fmt::format("  Finished: {}\n", entry.finished_at);
      std::cout << fmt::format("  Duration: {}\n",
                               calculate_duration(entry.started_at, entry.finished_at));
    } else {
      std::cout << "  Status:   Running...\n";
    }

    std::cout << fmt::format("  Status:   {}\n", entry.status);

    if (!entry.parameters.empty() && entry.parameters != "{}") {
      std::cout << fmt::format("  Parameters: {}\n", entry.parameters);
    }

    if (!entry.error_msg.empty()) {
      std::cout << fmt::format("  Error: {}\n", entry.error_msg);
    }

    std::cout << "\n";
  }
}

// Format JSON output
void format_json_output(const std::vector<ReUseX::ProjectDB::PipelineLogEntry> &entries) {
  json j = json::array();

  for (const auto &entry : entries) {
    json entry_json = {
      {"id", entry.id},
      {"stage", entry.stage},
      {"started_at", entry.started_at},
      {"status", entry.status}
    };

    if (!entry.finished_at.empty()) {
      entry_json["finished_at"] = entry.finished_at;
      entry_json["duration_seconds"] = calculate_duration(entry.started_at, entry.finished_at);
    }

    if (!entry.parameters.empty()) {
      // Try to parse parameters as JSON, fallback to string
      try {
        entry_json["parameters"] = json::parse(entry.parameters);
      } catch (...) {
        entry_json["parameters"] = entry.parameters;
      }
    }

    if (!entry.error_msg.empty()) {
      entry_json["error_msg"] = entry.error_msg;
    }

    j.push_back(entry_json);
  }

  std::cout << j.dump(2) << std::endl;
}

} // anonymous namespace

void setup_subcommand_log(CLI::App &app) {
  auto *sub = app.add_subcommand(
      "log", "Display pipeline execution history from project database");

  static SubcommandLogOptions opt;

  sub->add_option("project", opt.project, "Path to project database file (.rux)")
      ->default_val(GlobalParams::project_db);

  sub->add_flag("-j,--json", opt.json_output, "Output in JSON format")
      ->default_val(false);

  sub->add_option("-n,--limit", opt.limit, "Limit number of entries (0 = all)")
      ->default_val(0)
      ->check(CLI::NonNegativeNumber);

  sub->callback([opt_ptr = &opt]() {
    spdlog::trace("Running log subcommand");
    return run_subcommand_log(*opt_ptr);
  });
}

int run_subcommand_log(SubcommandLogOptions const &opt) {
  try {
    // Open project database in read-only mode
    ReUseX::ProjectDB db(opt.project, /* readOnly */ true);

    // Get pipeline log entries
    auto entries = db.pipeline_log(opt.limit);

    // Format output
    if (opt.json_output) {
      format_json_output(entries);
    } else {
      format_terminal_output(entries);
    }

    return RuxError::SUCCESS;

  } catch (const std::exception &e) {
    spdlog::error("Failed to read pipeline log: {}", e.what());
    return RuxError::IO;
  }
}
