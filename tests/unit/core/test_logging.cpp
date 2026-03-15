// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/core/logging.hpp>

#include <catch2/catch_test_macros.hpp>

#include <string>
#include <vector>

TEST_CASE("Custom log handler can be registered", "[core][logging]") {
  std::vector<std::string> messages;

  ReUseX::core::set_log_handler(
      [&messages](ReUseX::core::LogLevel level, std::string_view message) {
        messages.emplace_back(std::to_string(static_cast<int>(level)) + ":" +
                              std::string(message));
      });

  ReUseX::core::set_log_level(ReUseX::core::LogLevel::trace);
  ReUseX::core::info("hello {}", "reusex");

  REQUIRE(messages.size() == 1);
  REQUIRE(messages.front().find("hello reusex") != std::string::npos);

  ReUseX::core::reset_log_handler();
  ReUseX::core::set_log_level(ReUseX::core::LogLevel::info);
}

TEST_CASE("Log level filtering works", "[core][logging]") {
  int called = 0;
  ReUseX::core::set_log_handler([&called](ReUseX::core::LogLevel,
                                          std::string_view) { ++called; });

  ReUseX::core::set_log_level(ReUseX::core::LogLevel::warn);
  ReUseX::core::debug("debug message");
  ReUseX::core::warn("warn message");

  REQUIRE(called == 1);

  ReUseX::core::reset_log_handler();
  ReUseX::core::set_log_level(ReUseX::core::LogLevel::info);
}

