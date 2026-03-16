// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <annotate.hpp>
#include <assemble.hpp>
#include <export.hpp>
#include <import.hpp>
#include <mesh.hpp>
#include <processing_observer.hpp>
#include <project.hpp>
#include <segment.hpp>
#include <texture.hpp>
#include <view.hpp>

#include <ReUseX/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cstdint>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace {
constexpr int kMaxVerbosity = 3;
}

int main(int argc, char **argv) {
  // Create a new logger with a custom name
  auto console_logger = spdlog::stdout_color_mt("rux");

  // Replace the default logger with our custom logger
  spdlog::set_default_logger(console_logger);

  spdlog::set_level(spdlog::level::warn); // Default level
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%=7l%$] %v");

  ReUseX::core::set_log_handler(
      [](ReUseX::core::LogLevel level, std::string_view message) {
        switch (level) {
        case ReUseX::core::LogLevel::trace:
          spdlog::trace("{}", message);
          break;
        case ReUseX::core::LogLevel::debug:
          spdlog::debug("{}", message);
          break;
        case ReUseX::core::LogLevel::info:
          spdlog::info("{}", message);
          break;
        case ReUseX::core::LogLevel::warn:
          spdlog::warn("{}", message);
          break;
        case ReUseX::core::LogLevel::error:
          spdlog::error("{}", message);
          break;
        case ReUseX::core::LogLevel::critical:
          spdlog::critical("{}", message);
          break;
        case ReUseX::core::LogLevel::off:
          break;
        }
      });
  ReUseX::core::set_log_level(ReUseX::core::LogLevel::warn);

  CLI::App app{"rux: ReUseX a tool for processing "
               "interior lidar scans of buildings."};
  app.get_formatter()->column_width(40);

  app.add_flag(
         "-v, --verbose",
         [](int count) {
           const int safe_count = std::clamp(count, 0, kMaxVerbosity);
           const auto level = static_cast<spdlog::level::level_enum>(
               kMaxVerbosity - safe_count);
           spdlog::set_level(level);
           ReUseX::core::set_log_level(
               static_cast<ReUseX::core::LogLevel>(kMaxVerbosity - safe_count));
           spdlog::info("Verbosity level: {}",
                        spdlog::level::to_string_view(spdlog::get_level()));
         },
         "Increase verbosity, use -vv & -vvv for more details.")
      ->multi_option_policy(CLI::MultiOptionPolicy::Sum)
      ->check(CLI::Range(0, 3));
  app.add_flag(
      "-V, --version",
      [](bool /*count*/) {
        std::cout << "rux version: " << ReUseX::core::VERSION << std::endl;
        exit(0);
      },
      "Show version information");
  app.add_flag(
      "-L, --license",
      [](bool /*count*/) {
        std::cout << ReUseX::core::LICENSE_TEXT << std::endl;
        exit(0);
      },
      "Show license information");
  app.add_flag(
         "--visualize",
         [](std::int64_t count) { rux::enable_processing_observer(count > 0); },
         "Enable top-level processing observer hooks for GUI/visual "
         "integrations.")
      ->default_val(false);

  setup_subcommand_assemble(app);

  setup_subcommand_import(app);
  setup_subcommand_export(app);

  setup_subcommand_annotate(app);

  setup_subcommand_segment(app);

  setup_subcommand_texture(app);
  setup_subcommand_project(app);

  setup_subcommand_view(app);
  setup_subcommand_mesh(app);

  app.require_subcommand(/* min */ 0, /* max */ 2);

  argv = app.ensure_utf8(argv);

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError &e) {
    return app.exit(e);
  }
  return 0;
}
