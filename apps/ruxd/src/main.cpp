// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd: HTTP service worker for ReUseX. Everything lives in ruxd_lib
// (src/cli.cpp) so it can be tested; see STANDARDS §1.1.

#include <cli.hpp>

int main(int argc, char **argv) { return ruxd::run(argc, argv); }
