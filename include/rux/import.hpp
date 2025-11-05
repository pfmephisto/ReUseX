// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "global-params.hpp"

#include <CLI/CLI.hpp>
namespace fs = std::filesystem;

/// Collection of all options of Subcommand A.
struct SubcommandImportOptions {
  // fs::path path_in;
  fs::path cloud_path_out = GlobalParams::cloud;
  fs::path normals_path_out = GlobalParams::normals;
  fs::path labels_path_out = GlobalParams::labels;

  bool ascii{false};
};

struct ImportContext {
  CloudPtr cloud;
  CloudNPtr normals;
  CloudLPtr labels;

  bool ascii{false};
};

// We could manually make a few variables and use shared pointers for each; this
// is just done this way to be nicely organized

// Function declarations.
void setup_subcommand_import(CLI::App &app);
int run_subcommand_import(SubcommandImportOptions const &opt,
                          ImportContext &ctx);
