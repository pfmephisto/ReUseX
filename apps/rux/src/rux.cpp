// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assemble.hpp>
#include <create.hpp>
#include <del.hpp>
#include <export.hpp>
#include <get.hpp>
#include <import.hpp>
#include <info.hpp>
#include <log.hpp>
#include <processing_observer.hpp>
#include <set.hpp>
#include <view.hpp>

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cstdint>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace {
constexpr int kMaxVerbosity = 3;
} // namespace

int main(int argc, char **argv) {
  // Initialize async logger thread pool (lock-free queue, background writer
  // thread) Queue size: 8192 messages, 1 background thread
  spdlog::init_thread_pool(8192, 1);

  // Create async logger with lock-free queue (no mutex contention)
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  auto console_logger = std::make_shared<spdlog::async_logger>(
      "rux", console_sink, spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);

  // Replace the default logger with our async logger
  spdlog::set_default_logger(console_logger);

  // Set the global processing observer to enable progress reporting and
  // visualization
  rux::setup_processing_observer();

  spdlog::set_level(spdlog::level::warn); // Default level
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%=7l%$] %v");

  reusex::core::set_log_handler(
      [](reusex::core::LogLevel level, std::string_view message) {
        switch (level) {
        case reusex::core::LogLevel::trace:
          spdlog::trace("{}", message);
          break;
        case reusex::core::LogLevel::debug:
          spdlog::debug("{}", message);
          break;
        case reusex::core::LogLevel::info:
          spdlog::info("{}", message);
          break;
        case reusex::core::LogLevel::warn:
          spdlog::warn("{}", message);
          break;
        case reusex::core::LogLevel::error:
          spdlog::error("{}", message);
          break;
        case reusex::core::LogLevel::critical:
          spdlog::critical("{}", message);
          break;
        case reusex::core::LogLevel::off:
          break;
        }
      });
  reusex::core::set_log_level(reusex::core::LogLevel::warn);

  CLI::App app{"rux: ReUseX a tool for processing "
               "interior lidar scans of buildings."};

  auto opt = std::make_shared<RuxOptions>();

  app.get_formatter()->column_width(40);

  app.add_flag(
         "-v, --verbose",
         [](int count) {
           const int safe_count = std::clamp(count, 0, kMaxVerbosity);
           const auto level = static_cast<spdlog::level::level_enum>(
               kMaxVerbosity - safe_count);
           spdlog::set_level(level);
           reusex::core::set_log_level(
               static_cast<reusex::core::LogLevel>(kMaxVerbosity - safe_count));
           spdlog::info("Verbosity level: {}",
                        spdlog::level::to_string_view(spdlog::get_level()));
         },
         "Increase verbosity, use -vv & -vvv for more details.")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));
  app.add_flag(
      "-V, --version",
      [](bool /*count*/) {
        std::cout << "rux version: " << reusex::core::VERSION << std::endl;
        exit(0);
      },
      "Show version information");
  app.add_flag(
      "-L, --license",
      [](bool /*count*/) {
        std::cout << reusex::core::LICENSE_TEXT << std::endl;
        exit(0);
      },
      "Show license information");
  app.add_flag(
         "-D, --visualize", [](bool) { rux::start_viewer(); },
         "Enable visualization of processing steps")
      ->default_val(false);

  // Global project file flag (applied before subcommands)
  // Note: No file existence check here - commands will validate when needed
  app.add_option("-p,--project", opt->project_db,
                 "Path to .rux project file (applies to all subcommands)")
      ->default_val(fs::current_path() / "project.rux");

  setup_subcommand_assemble(app, opt);
  setup_subcommand_create(app, opt);
  setup_subcommand_import(app, opt);
  setup_subcommand_export(app, opt);

  // Unified path-based database commands (replaces old get/add/set/remove)
  setup_subcommand_get(app, opt);
  setup_subcommand_set(app, opt);
  setup_subcommand_del(app, opt);

  setup_subcommand_info(app, opt);
  setup_subcommand_log(app, opt);
  setup_subcommand_view(app, opt);

  app.require_subcommand(/* min */ 0, /* max */ 2);

  argv = app.ensure_utf8(argv);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    spdlog::shutdown(); // Flush async queue before exit
    return app.exit(e);
  }

  rux::wait_for_viewer();
  reusex::core::reset_visual_observer();
  reusex::core::reset_progress_observer();
  // g_processing_observer.stop();

  // Flush async queue to ensure all logs are written before exit
  spdlog::shutdown();
  return 0;
}
