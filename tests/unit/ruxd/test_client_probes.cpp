// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests for the ruxd readiness probes' failure behaviour (#282).
//
// The contract every backend probe shares: ping() reports failure as *data*
// (PingResult{false, detail}) and never throws, so /readyz can answer 503 with
// a reason instead of letting an exception escape into the Crow handler as a
// 500. These tests pin that for the paths reachable without a live server.
//
// RedisClient::ping() additionally lazily creates its shared sw::redis::Redis
// under init_mutex_. It used to read the member back *outside* the lock, so
// concurrent /readyz requests raced the shared_ptr. The concurrency case below
// is the regression guard; it is a smoke test by nature (a data race is not
// reliably observable without TSan), so its value is in being the thing a
// TSan-instrumented run has to execute — build with LIN_ENABLE_TSAN=ON to get
// the strong version of this assertion.
//
// UNTESTED HERE, and why:
//   * A *successful* Redis or S3 ping — needs a live server; that belongs in
//     an integration test with a container, not a unit test.
//   * S3Client::ping()'s catch clause against a real AWS SDK throw. Reaching
//     it needs a constructed Aws::S3::S3Client, which needs a process-wide
//     AwsApiGuard (Aws::InitAPI/ShutdownAPI) and then an endpoint that makes
//     the SDK throw rather than return a failed outcome — the SDK's default
//     retry/backoff against an unreachable host would also make the test slow
//     and timing-dependent. The unconfigured path is asserted instead, and the
//     catch clause itself is a two-line mirror of the Postgres/Redis probes.

#include <clients.hpp>
#include <config.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

namespace {

// A port nothing listens on, so the connection is refused immediately rather
// than hanging on a timeout.
constexpr const char *kUnreachableRedis = "tcp://127.0.0.1:1";

} // namespace

TEST_CASE("RedisClientPing_UnreachableServer_ReturnsFailureResult",
          "[ruxd][clients][redis]") {
  ruxd::RedisClient client(kUnreachableRedis);

  // Must not throw — the probe converts the failure into a PingResult.
  const ruxd::PingResult result = client.ping();

  REQUIRE_FALSE(result.ok);
  REQUIRE_FALSE(result.detail.empty());
}

TEST_CASE("RedisClientPing_ConcurrentCalls_NoThrowOrRace",
          "[ruxd][clients][redis]") {
  // Regression guard for #282: ping() created redis_ under init_mutex_ but
  // read it back outside the lock, so concurrent probes raced the shared_ptr.
  // Every thread here goes through the lazy-init path at once.
  ruxd::RedisClient client(kUnreachableRedis);

  constexpr int kThreads = 8;
  std::atomic<int> failures{0};
  std::atomic<int> threw{0};

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int i = 0; i < kThreads; ++i) {
    threads.emplace_back([&client, &failures, &threw] {
      try {
        const ruxd::PingResult result = client.ping();
        if (!result.ok) {
          ++failures;
        }
      } catch (...) {
        ++threw;
      }
    });
  }
  for (auto &t : threads) {
    t.join();
  }

  // Nothing escaped, and every probe answered (as a failure, since the server
  // is unreachable) rather than crashing on a torn shared_ptr read.
  REQUIRE(threw.load() == 0);
  REQUIRE(failures.load() == kThreads);
}

TEST_CASE(
    "S3ClientPing_UnconfiguredBackend_ReturnsFailureResultWithoutThrowing",
    "[ruxd][clients][s3]") {
  ruxd::Config cfg; // no s3_endpoint, no credentials
  ruxd::S3Client client(cfg);

  REQUIRE_FALSE(client.is_configured());

  const ruxd::PingResult result = client.ping();
  REQUIRE_FALSE(result.ok);
  REQUIRE(result.detail == "not configured");
}
