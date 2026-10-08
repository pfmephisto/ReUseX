// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// What the Pipeline and Log workspaces share about stages: Danish names and
// field labels, and "Kopiér som rux-kommando" for a parameter JSON object —
// a form's values or a pipeline_log row's parameters.

#include <rux_qt/cli_command.hpp>

#include <reusex/core/stage_contract.hpp>
#include <reusex/pipeline/stage_parameters.hpp>

#include <QJsonValue>
#include <QString>

#include <optional>
#include <string_view>

namespace rux::qt {

/// "Planer", "Punktskyer", … for a stage.
QString stage_name_da(reusex::pipeline::JobStage stage);
/// One line of what the stage does (Danish).
QString stage_blurb_da(reusex::pipeline::JobStage stage);
/// A Danish field label for @p key of @p stage (falls back to the
/// descriptor's own label).
QString parameter_label_da(reusex::pipeline::JobStage stage,
                           const reusex::pipeline::ParameterDescriptor &d);
/// The stage contract entry a job stage is checked against.
reusex::core::PipelineStage contract_stage(reusex::pipeline::JobStage stage);

/// The job stage that writes @p log_name into pipeline_log.stage
/// ("segment_planes" -> planes), if any.
std::optional<reusex::pipeline::JobStage>
stage_from_log_name(std::string_view log_name);

/// The CLI parameter for one descriptor's JSON value, in canonical text.
/// Returns nullopt when @p json_value does not fit the descriptor's type.
std::optional<CliParam>
cli_param(const reusex::pipeline::ParameterDescriptor &d,
          const QJsonValue &value);
/// Whether @p value equals the descriptor's default.
bool is_default(const reusex::pipeline::ParameterDescriptor &d,
                const QJsonValue &value);

/// "Kopiér som rux-kommando" for a parameters JSON object: every key whose
/// value differs from its default (presence-sensitive keys whenever present),
/// plus keys the descriptors do not know but the CLI does (glass_filter).
CliCommand cli_for_parameters(reusex::pipeline::JobStage stage,
                              const QString &parameters_json,
                              const QString &project);

} // namespace rux::qt
