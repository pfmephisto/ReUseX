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

#include <reusex/pipeline/mesh_parameters.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/slam/optimize_parameters.hpp>
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

TEST_CASE("CliCommandRoundTrip_MeshAndOptimize_ShareTheStageReader",
          "[rux_app][cli][qt]") {
  // `rux create mesh` and `rux optimize` drive their solvers themselves, but
  // read their options through the same functions the in-process stages use
  // (pipeline::mesh_options_from_parameters,
  // geometry::apply_optimize_parameters). So
  // the parameters a command line produces become the same options a GUI run
  // with that JSON gets.
  {
    const auto cmd = rux::qt::build_cli_command(
        "mesh", "p.rux",
        {{"solver", CliValueKind::string, "highs"},
         {"time_limit_seconds", CliValueKind::number, "90"},
         {"sectioned", CliValueKind::boolean, "false"},
         {"output_name", CliValueKind::string, "mesh2"}});
    const Captured got = parse_through_cli(cmd.args);
    const auto parsed = pl::mesh_options_from_parameters(got.params.dump());
    CHECK(parsed.output_name == "mesh2");
    CHECK(parsed.options.time_limit_seconds == 90.0);
    CHECK_FALSE(parsed.options.sectioned);
    CHECK(parsed.options.solver ==
          reusex::geometry::parse_solver_choice("highs"));
  }
  {
    const auto cmd = rux::qt::build_cli_command(
        "optimize", "p.rux",
        {{"min_observations", CliValueKind::integer, "7"},
         {"no_gnc", CliValueKind::boolean, "true"}});
    const Captured got = parse_through_cli(cmd.args);
    auto from_cli = plane_graph_options(SubcommandOptimizeOptions{});
    bool dry = false;
    reusex::geometry::apply_optimize_parameters(from_cli, dry,
                                                got.params.dump());
    CHECK(from_cli.min_landmark_observations == 7);
    CHECK_FALSE(from_cli.use_gnc);
    CHECK_FALSE(dry);
  }
}

TEST_CASE("CliCommandRoundTrip_OptimizeFlagDefaults_MatchTheStageBase",
          "[rux_app][cli][qt]") {
  // The in-process optimize stage (pipeline/optimize_stage.hpp: the Qt client
  // and ruxd's web GUI) starts from PlaneGraphOptions{}; `rux optimize`
  // starts from its flag defaults. Both then apply the same stage
  // parameters, so they solve the same problem only while every flag default
  // mirrors the library default (STANDARDS §4).
  const reusex::geometry::PlaneGraphOptions lib;
  const auto cli = plane_graph_options(SubcommandOptimizeOptions{});
  CHECK(cli.max_planes_per_frame == lib.max_planes_per_frame);
  CHECK(cli.min_plane_inliers == lib.min_plane_inliers);
  CHECK(cli.ransac_distance == lib.ransac_distance);
  CHECK(cli.ransac_normal_angle == lib.ransac_normal_angle);
  CHECK(cli.ransac_iterations == lib.ransac_iterations);
  CHECK(cli.assoc_normal_angle == lib.assoc_normal_angle);
  CHECK(cli.assoc_distance == lib.assoc_distance);
  CHECK(cli.min_landmark_observations == lib.min_landmark_observations);
  CHECK(cli.assoc_overlap_margin == lib.assoc_overlap_margin);
  CHECK(cli.min_landmark_spread_ratio == lib.min_landmark_spread_ratio);
  CHECK(cli.assoc_rounds == lib.assoc_rounds);
  CHECK(cli.assoc_round_tol == lib.assoc_round_tol);
  CHECK(cli.odometry_sigma_rot == lib.odometry_sigma_rot);
  CHECK(cli.odometry_sigma_trans == lib.odometry_sigma_trans);
  CHECK(cli.underconstrained_odom_scale == lib.underconstrained_odom_scale);
  CHECK(cli.plane_sigma_normal == lib.plane_sigma_normal);
  CHECK(cli.plane_sigma_distance == lib.plane_sigma_distance);
  CHECK(cli.plane_noise_model == lib.plane_noise_model);
  CHECK(cli.odometry_noise_model == lib.odometry_noise_model);
  CHECK(cli.odometry_weight_min == lib.odometry_weight_min);
  CHECK(cli.odometry_weight_max == lib.odometry_weight_max);
  CHECK(cli.odometry_robust == lib.odometry_robust);
  CHECK(cli.odometry_gnc_inlier_cost == lib.odometry_gnc_inlier_cost);
  CHECK(cli.plane_weight_min == lib.plane_weight_min);
  CHECK(cli.plane_weight_max == lib.plane_weight_max);
  CHECK(cli.plane_sigma_scale == lib.plane_sigma_scale);
  CHECK(cli.use_plane_factors == lib.use_plane_factors);
  CHECK(cli.prior_sigma_rot == lib.prior_sigma_rot);
  CHECK(cli.prior_sigma_trans == lib.prior_sigma_trans);
  CHECK(cli.use_gnc == lib.use_gnc);
  CHECK(cli.gnc_inlier_cost == lib.gnc_inlier_cost);
  CHECK(cli.max_iterations == lib.max_iterations);
  CHECK(cli.seed == lib.seed);
  CHECK(cli.surfel.min_distance == lib.surfel.min_distance);
  CHECK(cli.surfel.max_distance == lib.surfel.max_distance);
  CHECK(cli.surfel.sampling_factor == lib.surfel.sampling_factor);
  CHECK(cli.surfel.confidence_threshold == lib.surfel.confidence_threshold);
  CHECK(cli.surfel.voxel_size == lib.surfel.voxel_size);
  CHECK(cli.loop_closure.enable == lib.loop_closure.enable);
  CHECK(cli.loop_closure.proposal == lib.loop_closure.proposal);
  CHECK(cli.loop_closure.min_frame_gap == lib.loop_closure.min_frame_gap);
  CHECK(cli.loop_closure.max_candidate_distance ==
        lib.loop_closure.max_candidate_distance);
  CHECK(cli.loop_closure.max_view_angle == lib.loop_closure.max_view_angle);
  CHECK(cli.loop_closure.max_candidates_per_frame ==
        lib.loop_closure.max_candidates_per_frame);
  CHECK(cli.loop_closure.min_match_inliers ==
        lib.loop_closure.min_match_inliers);
  CHECK(cli.loop_closure.max_features == lib.loop_closure.max_features);
  CHECK(cli.loop_closure.ratio_test == lib.loop_closure.ratio_test);
  CHECK(cli.loop_closure.ransac_inlier_dist ==
        lib.loop_closure.ransac_inlier_dist);
  CHECK(cli.loop_closure.max_seed_disagreement ==
        lib.loop_closure.max_seed_disagreement);
  CHECK(cli.loop_closure.min_seed_disagreement ==
        lib.loop_closure.min_seed_disagreement);
  CHECK(cli.loop_closure.min_seed_disagreement_fraction ==
        lib.loop_closure.min_seed_disagreement_fraction);
  CHECK(cli.loop_closure.pcm == lib.loop_closure.pcm);
  CHECK(cli.loop_edges_trusted == lib.loop_edges_trusted);
  CHECK(cli.loop_trust_inlier_cost == lib.loop_trust_inlier_cost);
  CHECK(cli.loop_edges_file == lib.loop_edges_file);
  CHECK(cli.loop_edges_min_seed_disagreement ==
        lib.loop_edges_min_seed_disagreement);
  CHECK(cli.loop_edges_min_seed_disagreement_fraction ==
        lib.loop_edges_min_seed_disagreement_fraction);
  CHECK(cli.panorama_loops.enable == lib.panorama_loops.enable);
  CHECK(cli.panorama_loops.max_frames == lib.panorama_loops.max_frames);
  CHECK(cli.panorama_loops.min_frame_inliers ==
        lib.panorama_loops.min_frame_inliers);
  CHECK(cli.panorama_loops.max_edges_per_panorama ==
        lib.panorama_loops.max_edges_per_panorama);
  CHECK(cli.panorama_loops.n_yaw == lib.panorama_loops.n_yaw);
  CHECK(cli.panorama_loops.max_features == lib.panorama_loops.max_features);
  CHECK(cli.panorama_loops.max_pano_distance ==
        lib.panorama_loops.max_pano_distance);
}
