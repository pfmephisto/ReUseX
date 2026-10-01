// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <reusex/core/survey_service.hpp>

#include <CLI/CLI.hpp>
#include <memory>

/// CLI options for `rux create survey` — fill the Ressourcekortlægning
/// (survey_types / survey_parts) from the instances table.
struct SubcommandCreateSurveyOptions {
  reusex::core::SurveySyncOptions sync;
};

void setup_subcommand_create_survey(CLI::App &parent,
                                    std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_create_survey(SubcommandCreateSurveyOptions const &opt,
                                 const RuxOptions &global_opt);
