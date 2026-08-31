// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "validate.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/validate.hpp>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fmt/color.h>
#include <fmt/format.h>

#include <iostream>

using json = nlohmann::json;

namespace {

const char *severity_str(reusex::core::ValidationSeverity s) {
  return s == reusex::core::ValidationSeverity::error ? "error" : "warning";
}

void print_terminal(const reusex::core::ValidationReport &report,
                    const std::string &path) {
  fmt::print("Validating project: {}\n\n", path);

  if (report.issues.empty()) {
    fmt::print("{}\n", fmt::styled("No problems found.",
                                   fmt::fg(fmt::terminal_color::green)));
    return;
  }

  for (const auto &issue : report.issues) {
    bool is_err = issue.severity == reusex::core::ValidationSeverity::error;
    auto color =
        is_err ? fmt::terminal_color::red : fmt::terminal_color::yellow;
    fmt::print("{} [{}] {}\n",
               fmt::styled(is_err ? "ERROR  " : "WARNING", fmt::fg(color)),
               issue.check, issue.message);
  }

  fmt::print("\n{} error(s), {} warning(s)\n", report.error_count(),
             report.warning_count());
}

void print_json(const reusex::core::ValidationReport &report,
                const std::string &path) {
  json out;
  out["path"] = path;
  out["ok"] = report.ok();
  out["error_count"] = report.error_count();
  out["warning_count"] = report.warning_count();
  out["issues"] = json::array();
  for (const auto &issue : report.issues) {
    out["issues"].push_back({{"check", issue.check},
                             {"severity", severity_str(issue.severity)},
                             {"message", issue.message}});
  }
  std::cout << out.dump(2) << std::endl;
}

} // namespace

void setup_subcommand_validate(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandValidateOptions>();

  auto *sub = app.add_subcommand(
      "validate", "Check project database referential integrity");

  sub->footer(R"(
DESCRIPTION:
  Runs read-only integrity checks over a ReUseX project database and reports
  problems: orphaned material passports, dangling instance_materials links,
  instances without label definitions, and size-mismatched parallel clouds.
  Exits non-zero when any error-severity problem is found (warnings do not
  affect the exit code).

EXAMPLES:
  rux validate                         # Human-readable report
  rux validate --json                  # JSON report for scripting
  rux -p scan.rux validate             # Custom project path

NOTES:
  - Read-only: never modifies the database
  - Exit code 0 = clean, non-zero = at least one error found
)");

  sub->add_flag("-j,--json", opt->json_output, "Output in JSON format")
      ->default_val(false);

  sub->callback([opt, global_opt]() {
    int exit_code = run_subcommand_validate(*opt, *global_opt);
    if (exit_code != RuxError::SUCCESS)
      throw CLI::RuntimeError(exit_code);
  });
}

int run_subcommand_validate(SubcommandValidateOptions const &opt,
                            const RuxOptions &global_opt) {
  try {
    fs::path project_path = global_opt.project_db;
    reusex::ProjectDB db(project_path, /* readOnly */ true);

    auto report = reusex::core::validate_project(db);

    if (opt.json_output)
      print_json(report, project_path.string());
    else
      print_terminal(report, project_path.string());

    // Non-zero exit when errors are present so CI / scripts can gate on it.
    return report.ok() ? RuxError::SUCCESS : RuxError::GENERIC;

  } catch (const std::exception &e) {
    spdlog::error("Validation failed: {}", e.what());
    return RuxError::IO;
  }
}
