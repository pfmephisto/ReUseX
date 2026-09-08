// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// A small fixed-size connection pool for ruxd's backend clients.
//
// The pool is deliberately generic: it knows nothing about PostgreSQL and is
// constructed with a *factory* that produces connections, so the concurrency
// logic can be unit tested against a fake connection type without a live
// server (see tests/unit/ruxd/test_connection_pool.cpp).
//
// Shape:
//   * Fixed capacity, set once at construction. Connections are created
//     lazily — a fresh pool holds `capacity` empty slots and only materialises
//     a real connection when a slot is first handed out, so an idle ruxd costs
//     the backend nothing.
//   * acquire() blocks on a condition variable while the pool is exhausted and
//     throws ConnectionPoolTimeout once the configured timeout elapses. There
//     is no unbounded wait and no silent failure.
//   * acquire() returns a move-only RAII Lease that returns the connection to
//     the pool on destruction.
//   * A connection can be flagged with Lease::mark_broken() (handlers do this
//     when a driver exception escapes) and is additionally validated by an
//     optional health predicate on return. Either check failing destroys the
//     connection and frees its slot, so the next acquire() re-creates it.

#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ruxd {

// Thrown by ConnectionPool::acquire() when no connection becomes available
// within the configured timeout. Callers should map this to HTTP 503.
class ConnectionPoolTimeout : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

// Thrown when the factory returns a null connection without throwing itself.
class ConnectionPoolError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

// Tunables for a ConnectionPool. `name` only ever appears in log lines and
// error messages.
struct ConnectionPoolOptions {
  std::size_t capacity = 1;
  std::chrono::milliseconds acquire_timeout{5000};
  std::string name = "pool";
};

template <class Connection> class ConnectionPool {
    public:
  using ConnectionPtr = std::unique_ptr<Connection>;

  // Creates one new connection. May throw; the slot is released and the
  // exception propagates to the acquire() caller.
  using Factory = std::function<ConnectionPtr()>;

  // Returns false if the connection must not be recycled. Must not throw —
  // an escaping exception is treated as "unhealthy".
  using HealthCheck = std::function<bool(Connection &)>;

  // RAII lease of a pooled connection. Move-only; returns the connection on
  // destruction.
  class Lease {
      public:
    Lease() = default;
    ~Lease() { give_back(); }

    Lease(const Lease &) = delete;
    Lease &operator=(const Lease &) = delete;

    Lease(Lease &&other) noexcept
        : pool_(std::exchange(other.pool_, nullptr)),
          conn_(std::move(other.conn_)),
          broken_(std::exchange(other.broken_, false)) {}

    Lease &operator=(Lease &&other) noexcept {
      if (this != &other) {
        give_back();
        pool_ = std::exchange(other.pool_, nullptr);
        conn_ = std::move(other.conn_);
        broken_ = std::exchange(other.broken_, false);
      }
      return *this;
    }

    [[nodiscard]] Connection *get() const noexcept { return conn_.get(); }
    Connection &operator*() const noexcept { return *conn_; }
    Connection *operator->() const noexcept { return conn_.get(); }
    explicit operator bool() const noexcept { return conn_ != nullptr; }

    // Marks the connection unusable: it is destroyed instead of recycled when
    // the lease is released. Call this after catching a driver exception.
    void mark_broken() noexcept { broken_ = true; }
    [[nodiscard]] bool is_broken() const noexcept { return broken_; }

    // Returns the connection early, before the lease goes out of scope.
    void release() noexcept { give_back(); }

      private:
    friend class ConnectionPool;

    Lease(ConnectionPool *pool, ConnectionPtr conn)
        : pool_(pool), conn_(std::move(conn)) {}

    void give_back() noexcept {
      if (pool_ != nullptr && conn_ != nullptr) {
        pool_->release(std::move(conn_), broken_);
      }
      pool_ = nullptr;
      conn_.reset();
      broken_ = false;
    }

    ConnectionPool *pool_ = nullptr;
    ConnectionPtr conn_;
    bool broken_ = false;
  };

  ConnectionPool(Factory factory, ConnectionPoolOptions options,
                 HealthCheck health_check = {})
      : factory_(std::move(factory)), health_check_(std::move(health_check)),
        options_(std::move(options)),
        free_slots_(options_.capacity == 0 ? 1 : options_.capacity) {
    if (!factory_) {
      throw ConnectionPoolError("connection pool '" + options_.name +
                                "': no connection factory supplied");
    }
    if (options_.capacity == 0) {
      spdlog::warn("connection pool '{}': capacity 0 requested, using 1",
                   options_.name);
      options_.capacity = 1;
    }
    // Reserved up front so release() can never fail on an allocation while
    // handing a connection back.
    idle_.reserve(options_.capacity);
  }

  ~ConnectionPool() = default;

  ConnectionPool(const ConnectionPool &) = delete;
  ConnectionPool &operator=(const ConnectionPool &) = delete;
  ConnectionPool(ConnectionPool &&) = delete;
  ConnectionPool &operator=(ConnectionPool &&) = delete;

  // Blocks until a connection is available, then hands out a lease. Throws
  // ConnectionPoolTimeout if the pool stays exhausted for acquire_timeout, or
  // whatever the factory throws when a new connection cannot be opened.
  [[nodiscard]] Lease acquire() {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!slot_available_.wait_for(lock, options_.acquire_timeout, [this] {
          return !idle_.empty() || free_slots_ > 0;
        })) {
      spdlog::error(
          "connection pool '{}': exhausted, all {} connections busy after {}ms",
          options_.name, options_.capacity, options_.acquire_timeout.count());
      throw ConnectionPoolTimeout(
          "connection pool '" + options_.name + "': all " +
          std::to_string(options_.capacity) +
          " connections busy, timed out after " +
          std::to_string(options_.acquire_timeout.count()) + "ms");
    }

    if (!idle_.empty()) {
      ConnectionPtr conn = std::move(idle_.back());
      idle_.pop_back();
      ++in_use_;
      return Lease(this, std::move(conn));
    }

    // Take the empty slot, then build the connection with the lock released —
    // opening a socket must not block every other worker.
    --free_slots_;
    ++in_use_;
    lock.unlock();

    ConnectionPtr conn;
    try {
      conn = factory_();
      if (!conn) {
        throw ConnectionPoolError("connection pool '" + options_.name +
                                  "': factory returned no connection");
      }
    } catch (...) {
      lock.lock();
      --in_use_;
      ++free_slots_;
      lock.unlock();
      slot_available_.notify_one();
      throw;
    }

    lock.lock();
    ++created_;
    const std::size_t created = created_;
    lock.unlock();
    spdlog::debug("connection pool '{}': opened connection {}/{}",
                  options_.name, created, options_.capacity);

    return Lease(this, std::move(conn));
  }

  [[nodiscard]] std::size_t capacity() const noexcept {
    return options_.capacity;
  }

  // Connections currently parked in the pool, ready to be handed out.
  [[nodiscard]] std::size_t idle_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return idle_.size();
  }

  // Leases currently outstanding.
  [[nodiscard]] std::size_t in_use_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return in_use_;
  }

  // Slots that hold no connection yet (never opened, or discarded as broken).
  [[nodiscard]] std::size_t free_slot_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return free_slots_;
  }

  // Total connections the factory has produced over the pool's lifetime.
  // Grows again whenever a broken connection is replaced.
  [[nodiscard]] std::size_t created_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return created_;
  }

  // Connections destroyed instead of recycled (marked broken or unhealthy).
  [[nodiscard]] std::size_t discarded_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return discarded_;
  }

    private:
  // Returns a leased connection. Never throws: a lease is released from a
  // destructor.
  void release(ConnectionPtr conn, bool broken) noexcept {
    bool healthy = true;
    if (health_check_) {
      try {
        healthy = health_check_(*conn);
      } catch (...) {
        healthy = false;
      }
    }
    const bool discard = broken || !healthy;

    if (discard) {
      spdlog::warn("connection pool '{}': discarding {} connection, slot will "
                   "be refilled on next acquire",
                   options_.name, broken ? "broken" : "unhealthy");
      // Destroy outside the lock — closing a socket can block.
      conn.reset();
    }

    {
      const std::lock_guard<std::mutex> lock(mutex_);
      --in_use_;
      if (discard) {
        ++free_slots_;
        ++discarded_;
      } else {
        idle_.push_back(std::move(conn));
      }
    }
    slot_available_.notify_one();
  }

  Factory factory_;
  HealthCheck health_check_;
  ConnectionPoolOptions options_;

  mutable std::mutex mutex_;
  std::condition_variable slot_available_;

  // Invariant: idle_.size() + free_slots_ + in_use_ == options_.capacity
  std::vector<ConnectionPtr> idle_;
  std::size_t free_slots_ = 0;
  std::size_t in_use_ = 0;

  std::size_t created_ = 0;
  std::size_t discarded_ = 0;
};

} // namespace ruxd
