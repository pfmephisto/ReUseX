// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "log.hpp"
#include <nlohmann/json.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <spdlog/spdlog.h>

#include <fmt/color.h>
#include <fmt/format.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace {

// Calculate duration between two ISO timestamps
std::string calculate_duration(const std::string &start,
                               const std::string &end) {
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
void format_terminal_output(
    const std::vector<reusex::ProjectDB::PipelineLogEntry> &entries) {
  if (entries.empty()) {
    fmt::print("No pipeline executions recorded.\n");
    return;
  }

  fmt::print("Pipeline Execution History ({} entries)\n", entries.size());
  fmt::print("{}\n\n", std::string(80, '='));

  for (const auto &entry : entries) {
    // Status indicator with color
    std::string status_icon;
    if (entry.status == "success") {
      status_icon = fmt::format(
          "{}", fmt::styled("✓", fmt::fg(fmt::terminal_color::green)));
    } else if (entry.status == "failed") {
      status_icon = fmt::format(
          "{}", fmt::styled("✗", fmt::fg(fmt::terminal_color::red)));
    } else {
      status_icon = fmt::format(
          "{}", fmt::styled("⋯", fmt::fg(fmt::terminal_color::yellow)));
    }

    // Calculate duration
    std::string duration =
        calculate_duration(entry.started_at, entry.finished_at);

    // Build main line: [ID] stage_name   STATUS duration  start -> end
    std::string main_line;
    if (!entry.finished_at.empty()) {
      // Finished: show full timestamp range and duration
      main_line = fmt::format("[{:>3}]{} {:<20}  {:>6}  {} -> {}", entry.id,
                              status_icon, entry.stage, duration,
                              entry.started_at, entry.finished_at);
    } else {
      // Running: show start time only with arrow
      main_line = fmt::format("[{:>3}] {:<20} {}        {} ->", entry.id,
                              entry.stage, status_icon, entry.started_at);
    }

    fmt::print("{}\n", main_line);

    // Parameters on second line (indented)
    if (!entry.parameters.empty() && entry.parameters != "{}") {
      // std::cout << fmt::styled(
      //     fmt::format("     Parameters: {}\n", entry.parameters),
      //     fmt::fg(fmt::color::light_slate_gray));
      fmt::print("{}", fmt::styled(fmt::format("     Parameters: {}\n",
                                               entry.parameters),
                                   fmt::fg(fmt::terminal_color::bright_black)));
    }

    // Error message on third line (indented, red text)
    if (!entry.error_msg.empty()) {
      fmt::print("{}",
                 fmt::styled(fmt::format("     Error: {}\n", entry.error_msg),
                             fmt::fg(fmt::terminal_color::red)));
    }
  }
}

// Format JSON output
void format_json_output(
    const std::vector<reusex::ProjectDB::PipelineLogEntry> &entries) {
  json j = json::array();

  for (const auto &entry : entries) {
    json entry_json = {{"id", entry.id},
                       {"stage", entry.stage},
                       {"started_at", entry.started_at},
                       {"status", entry.status}};

    if (!entry.finished_at.empty()) {
      entry_json["finished_at"] = entry.finished_at;
      entry_json["duration_seconds"] =
          calculate_duration(entry.started_at, entry.finished_at);
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

  fmt::print("{}\n", j.dump(2));
}

} // anonymous namespace

void setup_subcommand_log(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandLogOptions>();

  auto *sub = app.add_subcommand("log", "Display pipeline execution history");

  sub->footer(R"(
DESCRIPTION:
  Displays pipeline execution history logged in the project database.
  Shows command execution timeline with start times, durations, parameters,
  status, and error messages. Useful for debugging, auditing, and tracking
  processing workflows.

EXAMPLES:
  rux log                              # Terminal formatted table
  rux log --json                       # JSON output for scripting
  rux -p scan.rux log                  # Custom project path
  rux log --json | jq '.[] | select(.status=="failed")'  # Filter

OUTPUT COLUMNS:
  ID          Unique log entry identifier
  Stage       Pipeline stage (import, clouds, planes, mesh, etc.)
  Started     Timestamp when command started
  Duration    Execution time (e.g., "2m 34s", "running")
  Status      success, failed, or running
  Parameters  Command parameters (JSON)

NOTES:
  - Timeline ordered by ID (chronological)
  - Parameters show algorithm settings used for each run
  - Duration auto-calculated from start/end timestamps
  - Running stages show "running" without duration
  - JSON mode includes full error messages for failed stages
  - Read-only operation (does not modify database)
)");

  sub->add_flag("-j,--json", opt->json_output, "Output in JSON format")
      ->default_val(false);

  sub->add_option("-n,--limit", opt->limit, "Limit number of entries (0 = all)")
      ->default_val(0)
      ->check(CLI::NonNegativeNumber);

  sub->callback([opt, global_opt]() {
    spdlog::trace("Running log subcommand");
    return run_subcommand_log(*opt, *global_opt);
  });
}

int run_subcommand_log(SubcommandLogOptions const &opt,
                       const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    // Open project database in read-only mode
    reusex::ProjectDB db(project_path, /* readOnly */ true);

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
