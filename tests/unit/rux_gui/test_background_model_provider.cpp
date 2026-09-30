// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for rux::gui::BackgroundModelProvider — the provisioning state
// machine behind GET /api/v1/models/sam3/status and the managed-model path of
// the segment endpoints. The prepare/probe functions are fakes, so nothing is
// downloaded or built.

#include <catch2/catch_test_macros.hpp>

#include <gui/BackgroundModelProvider.hpp>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using rux::gui::BackgroundModelProvider;
using rux::gui::ModelPrepStatus;

namespace {

ModelPrepStatus probe_absent(bool) {
  return {"absent", 0.0f, "nothing on disk", ""};
}

/// Poll @p provider until its status leaves the in-flight states.
ModelPrepStatus wait_settled(BackgroundModelProvider &provider, bool cuda) {
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  ModelPrepStatus st = provider.status(cuda);
  while ((st.state == "downloading" || st.state == "building") &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
    st = provider.status(cuda);
  }
  return st;
}

} // namespace

TEST_CASE("BackgroundModelProvider_ExplicitModel_IsReadyWithoutPreparing",
          "[gui][models]") {
  std::atomic<int> calls{0};
  BackgroundModelProvider provider(
      [&](bool, const auto &, const auto &) {
        ++calls;
        return std::string("/never");
      },
      probe_absent, "/explicit/sam3");

  const auto st = provider.ensure(true);
  CHECK(st.state == "ready");
  CHECK(st.model_path == "/explicit/sam3");
  CHECK(provider.status(false).model_path == "/explicit/sam3");
  CHECK(calls == 0);
}

TEST_CASE("BackgroundModelProvider_StatusBeforeEnsure_IsPureProbe",
          "[gui][models]") {
  std::atomic<int> calls{0};
  BackgroundModelProvider provider(
      [&](bool, const auto &, const auto &) {
        ++calls;
        return std::string("/m");
      },
      [](bool) { return ModelPrepStatus{"not_built", 0.0f, "onnx only", ""}; });

  CHECK(provider.status(true).state == "not_built");
  CHECK(calls == 0); // status() never starts preparation
}

TEST_CASE("BackgroundModelProvider_Ensure_PreparesInBackgroundToReady",
          "[gui][models]") {
  BackgroundModelProvider provider(
      [](bool use_cuda, const BackgroundModelProvider::ProgressFn &progress,
         const std::atomic<bool> &) {
        progress({"building", 0.5f, "building decoder", ""});
        return std::string(use_cuda ? "/engines" : "/onnx");
      },
      probe_absent);

  const auto first = provider.ensure(true);
  CHECK(first.state != "ready"); // non-blocking: returns before prepare ends
  const auto st = wait_settled(provider, true);
  CHECK(st.state == "ready");
  CHECK(st.model_path == "/engines");

  // Slots are independent: the CPU slot has not been touched.
  CHECK(provider.status(false).state == "absent");
}

TEST_CASE("BackgroundModelProvider_FailureThenEnsure_RetriesAfterBackoff",
          "[gui][models]") {
  std::atomic<int> calls{0};
  BackgroundModelProvider provider(
      [&](bool, const auto &, const auto &) -> std::string {
        if (++calls == 1)
          throw std::runtime_error("network unreachable");
        return "/recovered";
      },
      probe_absent, "", /*retry_backoff=*/0ms);

  provider.ensure(false);
  const auto failed = wait_settled(provider, false);
  REQUIRE(failed.state == "error");
  CHECK(failed.message.find("network unreachable") != std::string::npos);
  CHECK(failed.message.find("retry") != std::string::npos);

  // Not sticky: the next ensure() starts a fresh attempt.
  provider.ensure(false);
  const auto st = wait_settled(provider, false);
  CHECK(st.state == "ready");
  CHECK(st.model_path == "/recovered");
  CHECK(calls == 2);
}

TEST_CASE("BackgroundModelProvider_FailureWithinBackoff_IsNotRetried",
          "[gui][models]") {
  std::atomic<int> calls{0};
  BackgroundModelProvider provider(
      [&](bool, const auto &, const auto &) -> std::string {
        ++calls;
        throw std::runtime_error("boom");
      },
      probe_absent, "", /*retry_backoff=*/1h);

  provider.ensure(true);
  REQUIRE(wait_settled(provider, true).state == "error");
  CHECK(provider.ensure(true).state == "error");
  CHECK(calls == 1); // backoff not elapsed: no hammering on every request
}

TEST_CASE("BackgroundModelProvider_Destructor_CancelsInFlightPreparation",
          "[gui][models]") {
  std::atomic<bool> saw_stop{false};
  const auto t0 = std::chrono::steady_clock::now();
  {
    BackgroundModelProvider provider(
        [&](bool, const auto &, const std::atomic<bool> &stop) -> std::string {
          // Stands in for a long download: polls the stop flag like the curl
          // progress callback does.
          const auto give_up = std::chrono::steady_clock::now() + 30s;
          while (!stop.load() && std::chrono::steady_clock::now() < give_up)
            std::this_thread::sleep_for(1ms);
          saw_stop = stop.load();
          throw std::runtime_error("cancelled");
        },
        probe_absent);
    provider.ensure(true);
    std::this_thread::sleep_for(20ms);
  } // ~BackgroundModelProvider must not wait out the 30 s "download"
  CHECK(saw_stop);
  CHECK(std::chrono::steady_clock::now() - t0 < 10s);
}
