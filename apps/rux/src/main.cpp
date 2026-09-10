// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The `rux` entry point, and deliberately nothing else (#249).
//
// Everything the CLI actually does lives in `rux_lib` (src/rux.cpp holds
// `rux::run`), so the app logic is linkable from `reusex_unit_tests_vision`.
// A translation unit that defines `main` cannot go into a test binary, which
// is the whole reason this file is one line long.

#include <rux_app.hpp>

int main(int argc, char **argv) { return rux::run(argc, argv); }
