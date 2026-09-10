// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The offscreen-GL probe (#313).
//
// The probe decides whether render_view() refuses to run, so the failure that
// matters is a false negative: a probe that reports "unusable" on a machine
// that renders perfectly well turns every render into an error. That cannot be
// asserted directly here — a test cannot know what hardware it is on — but it
// can be pinned from both ends:
//
//   - the verdict must be stable and self-describing, whatever it is, and
//   - a "usable" verdict must be consistent with what the machine can do,
//     which the integration tests then confirm by actually rendering on it.
//
// The genuinely device-less case (a CI runner, a nix sandbox) is covered by
// construction rather than by hardware: REUSEX_SKIP_EGL_PROBE exercises the
// path that returns without a verdict.

#include <reusex/visualize/offscreen_gl.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <string>

namespace viz = reusex::visualize;

namespace {

/// Set or clear an environment variable for the duration of a scope.
class ScopedEnv {
    public:
  ScopedEnv(const char *name, const char *value) : name_(name) {
    if (const char *previous = std::getenv(name)) {
      had_previous_ = true;
      previous_ = previous;
    }
    if (value != nullptr) {
      setenv(name, value, 1);
    } else {
      unsetenv(name);
    }
  }
  ~ScopedEnv() {
    if (had_previous_) {
      setenv(name_.c_str(), previous_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

  ScopedEnv(const ScopedEnv &) = delete;
  ScopedEnv &operator=(const ScopedEnv &) = delete;
  ScopedEnv(ScopedEnv &&) = delete;
  ScopedEnv &operator=(ScopedEnv &&) = delete;

    private:
  std::string name_;
  std::string previous_;
  bool had_previous_ = false;
};

} // namespace

TEST_CASE("ProbeOffscreenGl_OnThisMachine_ReturnsAStableExplainedVerdict",
          "[visualize][offscreen_gl]") {
  const viz::OffscreenGlProbe probe = viz::probe_offscreen_gl();

  // Whatever the answer, it explains itself: the detail string is what ends up
  // in the error message a user has to act on (STANDARDS §5).
  INFO("probe detail: " << probe.detail);
  CHECK_FALSE(probe.detail.empty());

  // Repeating the probe must not change its mind — render_view() caches the
  // first answer for the life of the process and would otherwise be
  // inconsistent with a second caller.
  const viz::OffscreenGlProbe again = viz::probe_offscreen_gl();
  CHECK(again.status == probe.status);

  if (probe.status == viz::OffscreenGlStatus::usable) {
    // A usable verdict came from an actual eglInitialize, so it names the EGL
    // version it got.
    CHECK(probe.detail.find("EGL") != std::string::npos);
  }
  if (probe.status == viz::OffscreenGlStatus::unusable) {
    // The refusal path is the one that must be diagnosable. It reports what it
    // enumerated rather than just saying no.
    CHECK(probe.devices >= 0);
    CHECK(probe.detail.find("device") != std::string::npos);
  }
}

TEST_CASE("ProbeOffscreenGl_WithTheEscapeHatchSet_DeclinesToJudge",
          "[visualize][offscreen_gl]") {
  // The escape hatch exists so a machine the probe judges wrongly can still
  // render. It must yield `unknown` — the one verdict render_view() treats as
  // "carry on" — and never `unusable`, which would be a refusal.
  const ScopedEnv skip("REUSEX_SKIP_EGL_PROBE", "1");

  const viz::OffscreenGlProbe probe = viz::probe_offscreen_gl();
  CHECK(probe.status == viz::OffscreenGlStatus::unknown);
  CHECK(probe.detail.find("REUSEX_SKIP_EGL_PROBE") != std::string::npos);
}

TEST_CASE("DisplayConfigured_FollowsTheEnvironment",
          "[visualize][offscreen_gl]") {
  // display_configured() is what keeps an unusable EGL from being fatal on a
  // machine with a working X server, so it must read the environment and
  // nothing else.
  {
    const ScopedEnv display("DISPLAY", nullptr);
    const ScopedEnv wayland("WAYLAND_DISPLAY", nullptr);
    CHECK_FALSE(viz::display_configured());
  }
  {
    const ScopedEnv display("DISPLAY", ":0");
    const ScopedEnv wayland("WAYLAND_DISPLAY", nullptr);
    CHECK(viz::display_configured());
  }
  {
    // An empty value is how a shell spells "no display", not a display named
    // "".
    const ScopedEnv display("DISPLAY", "");
    const ScopedEnv wayland("WAYLAND_DISPLAY", nullptr);
    CHECK_FALSE(viz::display_configured());
  }
  {
    const ScopedEnv display("DISPLAY", nullptr);
    const ScopedEnv wayland("WAYLAND_DISPLAY", "wayland-0");
    CHECK(viz::display_configured());
  }
}
