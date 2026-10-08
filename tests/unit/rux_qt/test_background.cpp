// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The process-wide counter of the Qt client's detached work
// (rux_qt/background.hpp), which decides whether rux may tear spdlog down.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/background.hpp>

#include <chrono>
#include <future>
#include <thread>

using namespace rux::qt;

TEST_CASE("BackgroundWork_CountsLiveMarkers", "[rux_qt][background]") {
  CHECK(background_work_in_flight() == 0);
  {
    BackgroundWork a;
    CHECK(background_work_in_flight() == 1);
    {
      BackgroundWork b;
      CHECK(background_work_in_flight() == 2);
    }
    CHECK(background_work_in_flight() == 1);
    CHECK_FALSE(wait_for_background_work(10)); // times out
  }
  CHECK(background_work_in_flight() == 0);
  CHECK(wait_for_background_work(0));
}

TEST_CASE("BackgroundWork_WaitReturnsWhenTheThreadEnds",
          "[rux_qt][background]") {
  std::promise<void> go;
  std::promise<void> started;
  std::thread t([&] {
    BackgroundWork w;
    started.set_value();
    go.get_future().wait();
  });
  started.get_future().wait();
  CHECK(background_work_in_flight() == 1);
  go.set_value();
  CHECK(wait_for_background_work(5000));
  t.join();
}
