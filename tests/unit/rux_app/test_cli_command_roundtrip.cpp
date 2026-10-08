// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// "Kopiér som rux-kommando" (Qt client, Stream Q extra b) must print a
// command that really does what the GUI run did. The key -> flag table lives
// in rux_qt/cli_command.cpp, far from the CLI it describes; this test parses
// every generated command back through the REAL CLI11 setup of each stage
// subcommand (setup_subcommand_create / setup_subcommand_optimize) and checks
// that the stage receives the same parameters — key by key, for every
// parameter pipeline::stage_parameters() lists, and all at once.
//
// rux::set_stage_params_sink() keeps the subcommands from running: their
// callbacks hand the parameter JSON over and return before opening a project.

#include <catch2/catch_test_macros.hpp>

#include <create.hpp>
#include <create/stage_bridge.hpp>
#include <global-params.hpp>
#include <optimize.hpp>

#include <reusex/pipeline/stage_parameters.hpp>
#include <rux_qt/cli_command.hpp>

#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace pl = reusex::pipeline;
using nlohmann::json;
using rux::qt::CliParam;
using rux::qt::CliValueKind;

namespace {

/// Keys the CLI has no flag for: the command runs them at their default and
/// the Qt client says so next to the command.
const std::set<std::string> kNoFlag = {"rooms.propagate_k"};

struct Captured {
  std::string stage;
  json params;
};

/// Parse @p args (the words after `rux`) through the real subcommand setup.
Captured parse_through_cli(const std::vector<std::string> &args) {
  CLI::App app{"rux"};
  auto global = std::make_shared<RuxOptions>();
  app.add_option("-p,--project", global->project_db);
  setup_subcommand_create(app, global);
  setup_subcommand_optimize(app, global);

  Captured got;
  rux::set_stage_params_sink(
      [&](std::string_view stage, const std::string &params) {
        got.stage = std::string(stage);
        got.params = json::parse(params);
      });
  std::vector<std::string> reversed(args.rbegin(), args.rend());
  try {
    app.parse(reversed);
  } catch (const CLI::ParseError &e) {
    rux::set_stage_params_sink({});
    FAIL("CLI rejected the generated command: " << e.what());
  }
  rux::set_stage_params_sink({});
  return got;
}

CliValueKind kind_of(pl::ParameterType t) {
  switch (t) {
  case pl::ParameterType::number:
    return CliValueKind::number;
  case pl::ParameterType::integer:
    return CliValueKind::integer;
  case pl::ParameterType::boolean:
    return CliValueKind::boolean;
  case pl::ParameterType::string:
    return CliValueKind::string;
  case pl::ParameterType::integer_list:
    return CliValueKind::integer_list;
  }
  return CliValueKind::string;
}

/// A value for @p d that differs from its default and passes its bounds.
std::string changed_value(const pl::ParameterDescriptor &d) {
  const double lo = d.minimum.value_or(-1e9);
  const double hi = d.maximum.value_or(1e9);
  switch (d.type) {
  case pl::ParameterType::number: {
    const double def = std::get<double>(d.default_value);
    const double step =
        d.minimum && d.maximum ? (hi - lo) * 0.1 : 0.25 * std::abs(def) + 0.5;
    double v = def + step;
    if (v > hi)
      v = def - step;
    return rux::qt::format_number(static_cast<float>(v));
  }
  case pl::ParameterType::integer: {
    const long long def = std::get<long long>(d.default_value);
    return std::to_string(def + 1 <= hi ? def + 1 : def - 1);
  }
  case pl::ParameterType::boolean:
    return std::get<bool>(d.default_value) ? "false" : "true";
  case pl::ParameterType::string:
    if (d.key == "filter")
      return "planes in [1, 2]";
    if (d.key == "solver")
      return "highs";
    return "x_" + d.key;
  case pl::ParameterType::integer_list:
    return "1,4";
  }
  return {};
}

/// The captured JSON value as the canonical text CliParam uses.
std::string canonical(const json &v, pl::ParameterType t) {
  switch (t) {
  case pl::ParameterType::number:
    return rux::qt::format_number(static_cast<float>(v.get<double>()));
  case pl::ParameterType::integer:
    return std::to_string(v.get<long long>());
  case pl::ParameterType::boolean:
    return v.get<bool>() ? "true" : "false";
  case pl::ParameterType::string:
    return v.get<std::string>();
  case pl::ParameterType::integer_list: {
    std::string s;
    for (const auto &e : v) {
      if (!s.empty())
        s += ',';
      s += std::to_string(e.get<long long>());
    }
    return s;
  }
  }
  return {};
}

std::string default_text(const pl::ParameterDescriptor &d) {
  if (std::holds_alternative<double>(d.default_value))
    return rux::qt::format_number(
        static_cast<float>(std::get<double>(d.default_value)));
  if (std::holds_alternative<long long>(d.default_value))
    return std::to_string(std::get<long long>(d.default_value));
  if (std::holds_alternative<bool>(d.default_value))
    return std::get<bool>(d.default_value) ? "true" : "false";
  if (std::holds_alternative<std::string>(d.default_value))
    return std::get<std::string>(d.default_value);
  return {};
}

} // namespace

TEST_CASE("CliCommandRoundTrip_EveryParameter_ReachesTheStageUnchanged",
          "[rux_app][cli][qt]") {
  for (const std::string &stage : pl::job_stage_names()) {
    const auto parsed = pl::parse_job_stage(stage);
    REQUIRE(parsed);
    for (const auto &d : pl::stage_parameters(*parsed)) {
      const std::string id = stage + "." + d.key;
      INFO(id);
      if (rux::qt::cli_flag_for(stage, d.key).empty()) {
        CHECK(kNoFlag.count(id) == 1); // a new key needs a flag (or a ruling)
        continue;
      }
      const CliParam p{d.key, kind_of(d.type), changed_value(d)};
      const auto cmd =
          rux::qt::build_cli_command(stage, "/tmp/kopi af projekt.rux", {p});
      REQUIRE(cmd.supported);
      REQUIRE(cmd.unmapped.empty());
      INFO(cmd.text);

      const Captured got = parse_through_cli(cmd.args);
      CHECK(got.stage == stage);
      REQUIRE(got.params.contains(d.key));
      CHECK(canonical(got.params.at(d.key), d.type) == p.value);
    }
  }
}

TEST_CASE("CliCommandRoundTrip_AllParametersAtOnce_AndTheShellLine",
          "[rux_app][cli][qt]") {
  for (const std::string &stage : pl::job_stage_names()) {
    INFO(stage);
    std::vector<CliParam> params;
    for (const auto &d : pl::stage_parameters(*pl::parse_job_stage(stage)))
      if (!rux::qt::cli_flag_for(stage, d.key).empty())
        params.push_back({d.key, kind_of(d.type), changed_value(d)});
    const auto cmd = rux::qt::build_cli_command(stage, "/tmp/a b.rux", params);
    // What the user pastes is the text: split it like a shell would.
    auto words = rux::qt::split_shell_words(cmd.text);
    REQUIRE(words.front() == "rux");
    words.erase(words.begin());
    CHECK(words == cmd.args);
    const Captured got = parse_through_cli(words);
    for (const auto &p : params) {
      INFO(p.key);
      const auto &d = *std::find_if(
          pl::stage_parameters(*pl::parse_job_stage(stage)).begin(),
          pl::stage_parameters(*pl::parse_job_stage(stage)).end(),
          [&](const auto &x) { return x.key == p.key; });
      REQUIRE(got.params.contains(p.key));
      CHECK(canonical(got.params.at(p.key), d.type) == p.value);
    }
  }
}

TEST_CASE("CliCommandRoundTrip_NoParameters_MeansTheStageDefaults",
          "[rux_app][cli][qt]") {
  // An untouched GUI form sends no keys, and its command passes no flags; the
  // CLI must then hand the stage the same defaults the form showed — and
  // must not send a presence-sensitive key (#214) that would pin it.
  for (const std::string &stage : pl::job_stage_names()) {
    const auto cmd = rux::qt::build_cli_command(stage, "p.rux", {});
    const Captured got = parse_through_cli(cmd.args);
    for (const auto &d : pl::stage_parameters(*pl::parse_job_stage(stage))) {
      const std::string id = stage + "." + d.key;
      INFO(id);
      if (d.presence_sensitive ||
          std::holds_alternative<std::monostate>(d.default_value)) {
        CHECK_FALSE(got.params.contains(d.key));
        continue;
      }
      if (kNoFlag.count(id))
        continue;
      REQUIRE(got.params.contains(d.key));
      CHECK(canonical(got.params.at(d.key), d.type) == default_text(d));
    }
  }
}
