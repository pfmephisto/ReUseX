// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd::run(): parse the command line (src/cli/cli.cpp, ruxd_cli_lib), then
// run `ruxd admin` or the server (src/server.cpp), or local mode
// (src/local.cpp).

#include <cli.hpp>

#include <local.hpp>
#include <server.hpp>

#include <reusex/core/logging.hpp>
#include <reusex/core/version.hpp>

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <string>
#include <utility>

namespace {

// Bridge the ReUseX library logger into spdlog, mirroring the setup in
// apps/rux/src/rux.cpp so library log output is rendered the same way.
void setup_logging() {
  spdlog::init_thread_pool(8192, 1);
  auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto console_logger = std::make_shared<spdlog::async_logger>(
      "ruxd", console_sink, spdlog::thread_pool(),
      spdlog::async_overflow_policy::block);
  spdlog::set_default_logger(console_logger);

  spdlog::set_level(spdlog::level::warn); // Default level (raise with -v)
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
}

} // namespace

namespace ruxd {

int run(int argc, char **argv) {
  setup_logging();

  CLI::App cli{"ruxd: ReUseX HTTP service worker."};
  Invocation inv;
  configure_cli(cli, inv);
  CLI11_PARSE(cli, argc, argv);
  finish_invocation(cli, inv);

  if (inv.is_admin())
    return run_admin(std::move(inv));
  if (inv.is_local()) {
    for (const char *name : {"--pg-url", "--s3-bucket", "--s3-endpoint"})
      if (cli.get_option(name)->count() > 0)
        spdlog::warn("{} is ignored in local mode (no Postgres, Redis or S3)",
                     name);
    return run_local(std::move(inv.local));
  }

  reusex::core::info("ReUseX library linked, version {}",
                     reusex::core::VERSION);
  return run_server(std::move(inv));
}

} // namespace ruxd
