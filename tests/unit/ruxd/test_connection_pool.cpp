// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Tests for ruxd's fixed-size connection pool (#201).
//
// The pool takes a connection factory, so everything here runs against a fake
// connection type — no PostgreSQL server, no sockets. Covered: lazy creation
// and reuse, cycling through every slot, exhaustion (blocking, then a
// ConnectionPoolTimeout with a useful message), a blocked waiter being woken
// by a release, discard + lazy replacement of broken/unhealthy connections,
// slot accounting when the factory fails, and a bounded concurrent stress run.

#include <connection_pool.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using Catch::Matchers::ContainsSubstring;
using namespace std::chrono_literals;

namespace {

// Stand-in for pqxx::connection. Counts live instances so the tests can prove
// a discarded connection is really destroyed, and flags concurrent use so the
// stress test can prove the pool never hands one out twice.
struct FakeConnection {
  explicit FakeConnection(int identifier) : id(identifier) { ++live_count; }
  ~FakeConnection() { --live_count; }

  FakeConnection(const FakeConnection &) = delete;
  FakeConnection &operator=(const FakeConnection &) = delete;

  int id = 0;
  bool healthy = true;
  std::atomic<bool> leased{false};

  static inline std::atomic<int> live_count{0};
};

using Pool = ruxd::ConnectionPool<FakeConnection>;

// A factory handing out connections with increasing ids, counting calls.
struct CountingFactory {
  std::shared_ptr<std::atomic<int>> calls =
      std::make_shared<std::atomic<int>>(0);

  std::unique_ptr<FakeConnection> operator()() const {
    return std::make_unique<FakeConnection>(++*calls);
  }
};

ruxd::ConnectionPoolOptions options(std::size_t capacity,
                                    std::chrono::milliseconds timeout = 1s) {
  return ruxd::ConnectionPoolOptions{capacity, timeout, "test"};
}

bool is_open(FakeConnection &conn) { return conn.healthy; }

} // namespace

TEST_CASE("ConnectionPool creates connections lazily and reuses them",
          "[ruxd][pool]") {
  const int live_before = FakeConnection::live_count.load();
  CountingFactory factory;
  Pool pool(factory, options(2));

  // Nothing is opened until a connection is actually needed.
  REQUIRE(pool.capacity() == 2);
  REQUIRE(pool.created_count() == 0);
  REQUIRE(pool.free_slot_count() == 2);
  REQUIRE(pool.idle_count() == 0);
  REQUIRE(FakeConnection::live_count.load() == live_before);

  int first_id = 0;
  {
    auto lease = pool.acquire();
    REQUIRE(static_cast<bool>(lease));
    first_id = lease->id;
    REQUIRE(pool.created_count() == 1);
    REQUIRE(pool.in_use_count() == 1);
    REQUIRE(pool.free_slot_count() == 1);
  }

  // Released, parked in the pool, not destroyed.
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.idle_count() == 1);
  REQUIRE(FakeConnection::live_count.load() == live_before + 1);

  {
    auto lease = pool.acquire();
    REQUIRE(lease->id == first_id); // same connection, no new one opened
  }
  REQUIRE(pool.created_count() == 1);
  REQUIRE(factory.calls->load() == 1);
}

TEST_CASE("ConnectionPool cycles through every slot and returns them all",
          "[ruxd][pool]") {
  Pool pool(CountingFactory{}, options(3));

  std::vector<Pool::Lease> leases;
  for (int i = 0; i < 3; ++i) {
    leases.push_back(pool.acquire());
  }
  REQUIRE(pool.in_use_count() == 3);
  REQUIRE(pool.free_slot_count() == 0);
  REQUIRE(pool.idle_count() == 0);
  REQUIRE(pool.created_count() == 3);

  // Distinct connections, never the same one twice.
  REQUIRE(leases[0]->id != leases[1]->id);
  REQUIRE(leases[1]->id != leases[2]->id);
  REQUIRE(leases[0]->id != leases[2]->id);

  // Early release returns the connection before the lease goes out of scope.
  leases[1].release();
  REQUIRE_FALSE(static_cast<bool>(leases[1]));
  REQUIRE(pool.idle_count() == 1);
  REQUIRE(pool.in_use_count() == 2);

  leases.clear();
  REQUIRE(pool.idle_count() == 3);
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.created_count() == 3);
}

TEST_CASE("ConnectionPool leases are movable", "[ruxd][pool]") {
  Pool pool(CountingFactory{}, options(1));

  {
    auto lease = pool.acquire();
    const int id = lease->id;
    auto moved = std::move(lease);
    REQUIRE(moved->id == id);
    REQUIRE_FALSE(static_cast<bool>(lease)); // NOLINT(bugprone-use-after-move)
    REQUIRE(pool.in_use_count() == 1);       // moved-from must not release
  }
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.idle_count() == 1);
}

TEST_CASE("ConnectionPool blocks then times out when exhausted",
          "[ruxd][pool]") {
  Pool pool(CountingFactory{}, options(1, 80ms));

  auto held = pool.acquire();
  REQUIRE(pool.free_slot_count() == 0);

  const auto start = std::chrono::steady_clock::now();
  REQUIRE_THROWS_AS(pool.acquire(), ruxd::ConnectionPoolTimeout);
  const auto waited = std::chrono::steady_clock::now() - start;

  // It really waited for the timeout rather than failing immediately.
  REQUIRE(waited >= 70ms);

  // The failed attempt must not have leaked accounting.
  REQUIRE(pool.in_use_count() == 1);
  REQUIRE(pool.created_count() == 1);

  // The error names the pool, its capacity and the timeout.
  REQUIRE_THROWS_WITH(pool.acquire(), ContainsSubstring("test") &&
                                          ContainsSubstring("1") &&
                                          ContainsSubstring("80ms"));

  held.release();
  REQUIRE_NOTHROW(pool.acquire().release());
}

TEST_CASE("ConnectionPool wakes a waiter when a connection is returned",
          "[ruxd][pool]") {
  Pool pool(CountingFactory{}, options(1, 5s));

  auto held = pool.acquire();

  std::atomic<bool> waiting{false};
  std::atomic<bool> acquired{false};
  std::atomic<int> acquired_id{0};

  std::thread waiter([&] {
    waiting = true;
    auto lease = pool.acquire(); // blocks until `held` is released
    acquired_id = lease->id;
    acquired = true;
  });

  while (!waiting) {
    std::this_thread::yield();
  }
  std::this_thread::sleep_for(50ms);
  REQUIRE_FALSE(acquired.load()); // still blocked while we hold the only slot

  const int held_id = held->id;
  held.release();
  waiter.join();

  REQUIRE(acquired.load());
  REQUIRE(acquired_id.load() == held_id); // the very connection we returned
  REQUIRE(pool.created_count() == 1);     // no second connection was opened
}

TEST_CASE("ConnectionPool discards a broken connection and replaces it lazily",
          "[ruxd][pool]") {
  const int live_before = FakeConnection::live_count.load();
  Pool pool(CountingFactory{}, options(1), is_open);

  int broken_id = 0;
  {
    auto lease = pool.acquire();
    broken_id = lease->id;
    lease.mark_broken();
    REQUIRE(lease.is_broken());
  }

  // Destroyed, not recycled — and the slot is empty again.
  REQUIRE(FakeConnection::live_count.load() == live_before);
  REQUIRE(pool.idle_count() == 0);
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.free_slot_count() == 1);
  REQUIRE(pool.discarded_count() == 1);

  // Replacement is opened on the next acquire, not before.
  REQUIRE(pool.created_count() == 1);
  {
    auto lease = pool.acquire();
    REQUIRE(lease->id != broken_id);
    REQUIRE(pool.created_count() == 2);
  }
  REQUIRE(pool.idle_count() == 1);
  REQUIRE(pool.discarded_count() == 1);
}

TEST_CASE("ConnectionPool discards a connection the health check rejects",
          "[ruxd][pool]") {
  const int live_before = FakeConnection::live_count.load();
  Pool pool(CountingFactory{}, options(2), is_open);

  {
    auto lease = pool.acquire();
    lease->healthy = false; // e.g. the server closed the socket underneath us
  }

  REQUIRE(pool.discarded_count() == 1);
  REQUIRE(pool.idle_count() == 0);
  REQUIRE(pool.free_slot_count() == 2);
  REQUIRE(FakeConnection::live_count.load() == live_before);

  // A healthy one is still recycled normally.
  {
    auto lease = pool.acquire();
  }
  REQUIRE(pool.idle_count() == 1);
  REQUIRE(pool.discarded_count() == 1);
}

TEST_CASE("ConnectionPool releases the slot when the factory fails",
          "[ruxd][pool]") {
  std::atomic<bool> fail{true};
  std::atomic<int> next_id{100};

  Pool pool(
      [&]() -> std::unique_ptr<FakeConnection> {
        if (fail) {
          throw std::runtime_error("connect refused");
        }
        return std::make_unique<FakeConnection>(++next_id);
      },
      options(2, 100ms));

  REQUIRE_THROWS_WITH(pool.acquire(), ContainsSubstring("connect refused"));

  // The failed attempt must not shrink the pool.
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.free_slot_count() == 2);
  REQUIRE(pool.created_count() == 0);

  fail = false;
  auto lease = pool.acquire();
  REQUIRE(static_cast<bool>(lease));
  REQUIRE(pool.created_count() == 1);
}

TEST_CASE("ConnectionPool rejects a factory that returns nothing",
          "[ruxd][pool]") {
  Pool pool([]() -> std::unique_ptr<FakeConnection> { return nullptr; },
            options(1, 100ms));

  REQUIRE_THROWS_AS(pool.acquire(), ruxd::ConnectionPoolError);
  REQUIRE(pool.free_slot_count() == 1);
  REQUIRE(pool.in_use_count() == 0);
}

TEST_CASE("ConnectionPool requires a factory and clamps capacity 0",
          "[ruxd][pool]") {
  REQUIRE_THROWS_AS(Pool(Pool::Factory{}, options(1)),
                    ruxd::ConnectionPoolError);

  Pool pool(CountingFactory{}, options(0));
  REQUIRE(pool.capacity() == 1);
}

TEST_CASE("ConnectionPool holds its invariants under concurrent use",
          "[ruxd][pool]") {
  constexpr std::size_t kCapacity = 4;
  constexpr int kThreads = 8;
  constexpr int kIterations = 200;

  Pool pool(CountingFactory{}, options(kCapacity, 5s), is_open);

  std::atomic<int> live_leases{0};
  std::atomic<int> max_live_leases{0};
  std::atomic<int> completed{0};
  std::atomic<int> double_leased{0};
  std::atomic<int> failures{0};

  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&] {
      for (int i = 0; i < kIterations; ++i) {
        try {
          auto lease = pool.acquire();

          // Nobody else may hold this connection at the same time.
          if (lease->leased.exchange(true)) {
            ++double_leased;
          }

          const int now = ++live_leases;
          int previous = max_live_leases.load();
          while (now > previous &&
                 !max_live_leases.compare_exchange_weak(previous, now)) {
          }

          std::this_thread::yield();
          --live_leases;
          lease->leased = false;
          ++completed;
        } catch (const std::exception &) {
          ++failures;
        }
      }
    });
  }
  for (auto &worker : workers) {
    worker.join();
  }

  REQUIRE(failures.load() == 0);
  REQUIRE(completed.load() == kThreads * kIterations);
  REQUIRE(double_leased.load() == 0);

  // Deterministic bounds: never more concurrent leases than slots, and never
  // more connections opened than the capacity (nothing was broken here).
  REQUIRE(max_live_leases.load() <= static_cast<int>(kCapacity));
  REQUIRE(pool.created_count() <= kCapacity);
  REQUIRE(pool.discarded_count() == 0);

  // Everything came back.
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.idle_count() == pool.created_count());
  REQUIRE(pool.idle_count() + pool.free_slot_count() == kCapacity);
}

TEST_CASE("ConnectionPool replaces broken connections under concurrent use",
          "[ruxd][pool]") {
  constexpr std::size_t kCapacity = 3;
  constexpr int kThreads = 6;
  constexpr int kIterations = 60;

  Pool pool(CountingFactory{}, options(kCapacity, 5s), is_open);

  std::atomic<int> broken{0};
  std::atomic<int> failures{0};

  std::vector<std::thread> workers;
  workers.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    workers.emplace_back([&, t] {
      for (int i = 0; i < kIterations; ++i) {
        try {
          auto lease = pool.acquire();
          // Every third iteration of every other thread tears its connection.
          if (t % 2 == 0 && i % 3 == 0) {
            lease.mark_broken();
            ++broken;
          }
        } catch (const std::exception &) {
          ++failures;
        }
      }
    });
  }
  for (auto &worker : workers) {
    worker.join();
  }

  REQUIRE(failures.load() == 0);
  REQUIRE(broken.load() > 0);
  const auto broken_count = static_cast<std::size_t>(broken.load());
  REQUIRE(pool.discarded_count() == broken_count);
  // Each discard is refilled lazily, so opens = initial fills + replacements,
  // and the pool never grew past its capacity.
  REQUIRE(pool.created_count() <= kCapacity + broken_count);
  REQUIRE(pool.in_use_count() == 0);
  REQUIRE(pool.idle_count() + pool.free_slot_count() == kCapacity);
}
