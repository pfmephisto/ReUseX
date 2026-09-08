// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Shared glue between the `rux create <stage>` subcommands and the library's
// pipeline::run_stage (#284).
//
// The stage bodies themselves live in libs/reusex/src/pipeline/stages.cpp and
// exist exactly once. What stays here is the part that is genuinely the CLI's
// job: turning parsed flags into a parameter blob, and turning the library's
// outcome back into a process exit code.

#include "../global-params.hpp"

#include <reusex/pipeline/stages.hpp>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <string>
#include <type_traits>

namespace rux {

/// Map a library stage outcome onto a `rux` exit code.
///
/// `invalid_input` is what keeps `rux create planes -f 'bogus'` reporting
/// INVALID_ARGUMENT rather than a generic error, as it did when the subcommand
/// evaluated the filter itself.
inline int exit_code_for(const reusex::pipeline::StageResult &result) {
  if (result.ok)
    return RuxError::SUCCESS;
  if (result.invalid_input)
    return RuxError::INVALID_ARGUMENT;
  return RuxError::GENERIC;
}

/// Accumulates stage parameters as a JSON object.
///
/// Floats are routed through fmt's shortest round-trip for their *own* type
/// before being stored, so a `0.05f` flag value stays `0.05` in
/// `pipeline_log.parameters` instead of widening to `0.05000000074505806` the
/// way a plain float-to-double conversion would render it. That column is read
/// by humans (`rux log`), and it is what the CLI has always written.
class StageParams {
  nlohmann::json object_ = nlohmann::json::object();

    public:
  template <typename T> StageParams &set(const char *key, const T &value) {
    if constexpr (std::is_floating_point_v<T>)
      object_[key] = nlohmann::json::parse(fmt::format("{}", value));
    else
      object_[key] = value;
    return *this;
  }

  /// Set `key` only when `condition` holds. Used for the parameters whose mere
  /// presence carries meaning — an explicitly passed `-d`/`-m` pins that
  /// threshold and bypasses adaptive derivation for it (#214).
  template <typename T>
  StageParams &set_if(bool condition, const char *key, const T &value) {
    return condition ? set(key, value) : *this;
  }

  std::string dump() const { return object_.dump(); }
};

} // namespace rux
