// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
#include <memory>

void setup_subcommand_align(CLI::App &app,
                            std::shared_ptr<RuxOptions> global_opt);
