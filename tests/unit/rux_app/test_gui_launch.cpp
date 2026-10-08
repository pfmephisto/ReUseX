// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Plain `rux` (no subcommand) hands the desktop app a GuiLaunch through the
// launcher main() injects (rux_app.hpp). The seam keeps Qt out of rux_lib —
// and out of this binary — and lets the hand-off be tested with a fake.
//
// rux::run mutates process-global state (spdlog, signal handlers), so each
// TEST_CASE calls it once; ctest runs every case in its own process.

#include <catch2/catch_test_macros.hpp>

#include <rux_app.hpp>

#include "../../support/temp_path.hpp"

#include <cstdlib>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace {

struct Call {
  bool called = false;
  rux::GuiLaunch request;
};

int run_with(std::vector<std::string> args, Call &call,
             bool with_launcher = true) {
  // A display the launch decision accepts without a real X/Wayland session.
  ::setenv("QT_QPA_PLATFORM", "offscreen", 1);
  std::vector<char *> argv;
  for (auto &a : args)
    argv.push_back(a.data());
  argv.push_back(nullptr);
  rux::GuiLauncher fake;
  if (with_launcher)
    fake = [&](int, char **, const rux::GuiLaunch &r) {
      call.called = true;
      call.request = r;
      return 7;
    };
  return rux::run(static_cast<int>(args.size()), argv.data(), fake);
}

} // namespace

TEST_CASE("GuiLaunch_ExplicitProject_IsHandedToTheLauncher",
          "[rux_app][gui_launch]") {
  reusex::test_support::TempPath tmp("test_gui_launch");
  Call call;
  const int rc =
      run_with({"rux", "-p", tmp.path.string(), "--quit-after-ms", "5"}, call);
  CHECK(rc == 7); // the launcher's exit code is rux's
  REQUIRE(call.called);
  REQUIRE(call.request.project.has_value());
  CHECK(*call.request.project == tmp.path);
  CHECK(call.request.quit_after_ms == 5);
  CHECK_FALSE(std::filesystem::exists(tmp.path)); // rux itself never opens it
}

TEST_CASE("GuiLaunch_NoProjectFlag_NeverPassesTheDefault",
          "[rux_app][gui_launch]") {
  Call call;
  run_with({"rux"}, call);
  REQUIRE(call.called);
  // The ./project.rux default is not a request: opening it would create it.
  CHECK_FALSE(call.request.project.has_value());
  CHECK(call.request.quit_after_ms == -1);
}

TEST_CASE("GuiLaunch_NoLauncher_PrintsHelpAndSucceeds",
          "[rux_app][gui_launch]") {
  Call call;
  CHECK(run_with({"rux"}, call, /*with_launcher=*/false) == 0);
  CHECK_FALSE(call.called);
}
