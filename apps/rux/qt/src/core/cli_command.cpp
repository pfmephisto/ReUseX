// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/cli_command.hpp>

#include <array>
#include <charconv>
#include <cmath>

namespace rux::qt {
namespace {

/// How one key reaches the CLI. For a boolean, `on`/`off` are the flags for
/// true/false; an empty one means "that value is the CLI default: emit
/// nothing".
struct FlagSpec {
  std::string_view key;
  std::string_view flag; ///< value-taking option (non-boolean)
  std::string_view on;
  std::string_view off;
};

struct StageSpec {
  std::string_view stage;
  std::array<std::string_view, 2> subcommand; ///< {"create","planes"}
  std::vector<FlagSpec> flags;
};

/// Mirrors apps/rux/src/create/*.cpp and optimize.cpp. Keys absent here
/// have no flag (rooms.propagate_k) and are reported as unmapped.
const std::vector<StageSpec> &stage_specs() {
  static const std::vector<StageSpec> specs = {
      {"clouds",
       {"create", "clouds"},
       {{"resolution", "--grid", {}, {}},
        {"min_distance", "--min-distance", {}, {}},
        {"max_distance", "--max-distance", {}, {}},
        {"sampling_factor", "--sampling-factor", {}, {}},
        {"confidence_threshold", "--confidence", {}, {}},
        {"glass_filter", {}, "--glass-filter", {}},
        {"glass_threshold", "--glass-threshold", {}, {}}}},
      {"planes",
       {"create", "planes"},
       {{"angle_threshold", "--angle-threshold", {}, {}},
        {"plane_dist_threshold", "--plane-dist-threshold", {}, {}},
        {"min_inliers", "--min-cluster-size", {}, {}},
        {"radius", "--radius", {}, {}},
        {"interval_0", "--interval-0", {}, {}},
        {"interval_factor", "--interval-factor", {}, {}},
        {"adaptive", {}, {}, "--no-adaptive"},
        {"noise_seed", "--noise-seed", {}, {}},
        {"filter", "--filter", {}, {}}}},
      {"rooms",
       {"create", "rooms"},
       {{"grid_size", "--grid-size", {}, {}},
        {"resolution", "--resolution", {}, {}},
        {"beta", "--beta", {}, {}},
        {"max_iter", "--max-iter", {}, {}},
        {"propagate_max_radius", "--propagate-radius", {}, {}},
        {"filter", "--filter", {}, {}}}},
      {"instances",
       {"create", "instances"},
       {{"semantic_cloud", "--semantic", {}, {}},
        {"output_cloud", "--output", {}, {}},
        {"cluster_tolerance", "--tolerance", {}, {}},
        {"min_cluster_size", "--min-size", {}, {}},
        {"max_cluster_size", "--max-size", {}, {}},
        {"labels", "--labels", {}, {}}}},
      {"mesh",
       {"create", "mesh"},
       {{"solver", "--solver", {}, {}},
        {"time_limit_seconds", "--time-limit", {}, {}},
        {"output_name", "--output-name", {}, {}},
        {"search_threshold", "--threshold", {}, {}},
        {"new_plane_offset", "--offset", {}, {}},
        {"alpha", "--alpha", {}, {}},
        {"max_cells", "--max-cells", {}, {}},
        {"sectioned", {}, {}, "--no-sectioned"},
        {"sectioned_threshold", "--sectioned-threshold", {}, {}},
        {"filter", "--filter", {}, {}}}},
      {"optimize",
       {"optimize", {}},
       {{"min_observations", "--min-observations", {}, {}},
        {"assoc_rounds", "--assoc-rounds", {}, {}},
        {"no_gnc", {}, "--no-gnc", {}},
        {"dry_run", {}, "--dry-run", {}}}},
  };
  return specs;
}

const StageSpec *find_stage(std::string_view stage) {
  for (const auto &s : stage_specs())
    if (s.stage == stage)
      return &s;
  return nullptr;
}

const FlagSpec *find_flag(const StageSpec &s, std::string_view key) {
  for (const auto &f : s.flags)
    if (f.key == key)
      return &f;
  return nullptr;
}

bool safe_char(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
         (c >= '0' && c <= '9') ||
         std::string_view("_-./:=,+@%").find(c) != std::string_view::npos;
}

} // namespace

std::string format_number(double value) {
  if (!std::isfinite(value))
    return "0";
  std::array<char, 64> buf{};
  const auto r = std::to_chars(buf.data(), buf.data() + buf.size(), value);
  return std::string(buf.data(), r.ptr);
}

std::string shell_quote(std::string_view word) {
  if (word.empty())
    return "''";
  bool safe = true;
  for (char c : word)
    safe = safe && safe_char(c);
  if (safe)
    return std::string(word);
  std::string out = "'";
  for (char c : word) {
    if (c == '\'')
      out += "'\\''";
    else
      out += c;
  }
  out += '\'';
  return out;
}

std::vector<std::string> split_shell_words(std::string_view line) {
  std::vector<std::string> out;
  std::string cur;
  bool in_word = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (c == '\'') {
      in_word = true;
      for (++i; i < line.size() && line[i] != '\''; ++i)
        cur += line[i];
    } else if (c == '"') {
      in_word = true;
      for (++i; i < line.size() && line[i] != '"'; ++i) {
        if (line[i] == '\\' && i + 1 < line.size() &&
            std::string_view("\"\\$`").find(line[i + 1]) !=
                std::string_view::npos)
          ++i;
        cur += line[i];
      }
    } else if (c == '\\' && i + 1 < line.size()) {
      in_word = true;
      cur += line[++i];
    } else if (c == ' ' || c == '\t' || c == '\n') {
      if (in_word)
        out.push_back(std::move(cur));
      cur.clear();
      in_word = false;
    } else {
      in_word = true;
      cur += c;
    }
  }
  if (in_word)
    out.push_back(std::move(cur));
  return out;
}

std::string cli_flag_for(std::string_view stage, std::string_view key) {
  const StageSpec *s = find_stage(stage);
  if (!s)
    return {};
  const FlagSpec *f = find_flag(*s, key);
  if (!f)
    return {};
  if (!f->flag.empty())
    return std::string(f->flag);
  return std::string(!f->off.empty() ? f->off : f->on);
}

CliCommand build_cli_command(std::string_view stage, std::string_view project,
                             const std::vector<CliParam> &params) {
  CliCommand cmd;
  const StageSpec *s = find_stage(stage);
  if (!s)
    return cmd;
  cmd.supported = true;
  if (!project.empty()) {
    cmd.args.emplace_back("-p");
    cmd.args.emplace_back(project);
  }
  for (const auto &word : s->subcommand)
    if (!word.empty())
      cmd.args.emplace_back(word);

  for (const CliParam &p : params) {
    if (p.key == "job_id")
      continue; // the runner's bookkeeping, not a parameter
    const FlagSpec *f = find_flag(*s, p.key);
    if (!f) {
      cmd.unmapped.push_back(p.key);
      continue;
    }
    if (p.kind == CliValueKind::boolean || f->flag.empty()) {
      const bool on = p.value == "true" || p.value == "1";
      const std::string_view flag = on ? f->on : f->off;
      if (!flag.empty())
        cmd.args.emplace_back(flag);
      continue;
    }
    // An empty filter / list is "not set" on the CLI too.
    if (p.value.empty() &&
        (p.kind == CliValueKind::integer_list || p.key == "filter"))
      continue;
    cmd.args.emplace_back(f->flag);
    cmd.args.push_back(p.value);
  }

  cmd.text = "rux";
  for (const auto &a : cmd.args) {
    cmd.text += ' ';
    cmd.text += shell_quote(a);
  }
  return cmd;
}

} // namespace rux::qt
