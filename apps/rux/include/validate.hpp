// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

namespace fs = std::filesystem;

/// Options for the validate subcommand
struct SubcommandValidateOptions {
  bool json_output = false;
};

// Function declarations
void setup_subcommand_validate(CLI::App &app,
                               std::shared_ptr<RuxOptions> global_opt);
int run_subcommand_validate(SubcommandValidateOptions const &opt,
                            const RuxOptions &global_opt);
