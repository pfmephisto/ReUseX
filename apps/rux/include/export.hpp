// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

// Function declaration for parent export command.
void setup_subcommand_export(CLI::App &app, std::shared_ptr<RuxOptions> global_opt);
