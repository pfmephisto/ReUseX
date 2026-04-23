// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/logging.hpp>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

TEST_CASE("Custom log handler can be registered", "[core][logging]") {
  std::vector<std::string> messages;

  reusex::core::set_log_handler(
      [&messages](reusex::core::LogLevel level, std::string_view message) {
        messages.emplace_back(std::to_string(static_cast<int>(level)) + ":" +
                              std::string(message));
      });

  reusex::core::set_log_level(reusex::core::LogLevel::trace);
  reusex::core::info("hello {}", "reusex");

  REQUIRE(messages.size() == 1);
  REQUIRE(messages.front().find("hello reusex") != std::string::npos);

  reusex::core::reset_log_handler();
  reusex::core::set_log_level(reusex::core::LogLevel::info);
}

TEST_CASE("Log level filtering works", "[core][logging]") {
  int called = 0;
  reusex::core::set_log_handler(
      [&called](reusex::core::LogLevel, std::string_view) { ++called; });

  reusex::core::set_log_level(reusex::core::LogLevel::warn);
  reusex::core::debug("debug message");
  reusex::core::warn("warn message");

  REQUIRE(called == 1);

  reusex::core::reset_log_handler();
  reusex::core::set_log_level(reusex::core::LogLevel::info);
}
