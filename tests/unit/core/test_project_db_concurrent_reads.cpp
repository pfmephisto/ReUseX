// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Concurrent read-only ProjectDB connections (GUI Phase 5, R1). A reader must
// wait out a briefly held lock instead of failing at once, and many readers
// opening and closing together — what `ruxd --local` does on a full page load —
// must never see SQLITE_BUSY.

#include <catch2/catch_test_macros.hpp>

#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using reusex::ProjectDB;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB()
      : reusex::test_support::TempPath("test_projectdb_concurrent", ".rux") {}
};
} // namespace

TEST_CASE("ProjectDbReadOnly_WaitsOutABrieflyHeldLock",
          "[projectdb][concurrency]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path); // creates, migrates and switches to WAL
  }

  // A second connection takes the file's exclusive lock and holds it for
  // 300 ms. EXCLUSIVE locking mode on a WAL database bypasses the shared WAL
  // index and keeps that lock until close, so no other connection can read
  // in the meantime. Assertions stay on the main thread (Catch2 is not
  // thread-safe); the holder only records return codes.
  using Clock = std::chrono::steady_clock;
  int open_rc = -1;
  int lock_rc = -1;
  // The instant just before the holder lets go. A read that waited the lock
  // out finishes after it; one that failed fast finishes before it. No
  // wall-clock bound on the wait itself.
  std::atomic<Clock::rep> released_at{0};
  std::promise<void> locked;
  std::thread holder([&] {
    sqlite3 *raw = nullptr;
    open_rc = sqlite3_open(tmp.path.string().c_str(), &raw);
    lock_rc = sqlite3_exec(raw,
                           "PRAGMA locking_mode=EXCLUSIVE;"
                           "BEGIN EXCLUSIVE;"
                           "CREATE TABLE IF NOT EXISTS lock_probe (x INTEGER);"
                           "COMMIT;",
                           nullptr, nullptr, nullptr);
    locked.set_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    released_at = Clock::now().time_since_epoch().count();
    sqlite3_close(raw);
  });
  locked.get_future().wait();

  int version = -1;
  std::string error;
  try {
    ProjectDB db(tmp.path, /*readOnly=*/true);
    version = db.schema_version();
    (void)db.survey_types();
  } catch (const std::exception &e) {
    error = e.what();
  }
  const Clock::rep finished_at = Clock::now().time_since_epoch().count();
  holder.join();

  REQUIRE(open_rc == SQLITE_OK);
  REQUIRE(lock_rc == SQLITE_OK);
  INFO("read failed with: " << error);
  CHECK(error.empty());
  // Without a busy timeout the open's schema probe sees SQLITE_BUSY and
  // silently reads the version as -1.
  CHECK(version == ProjectDB::latest_schema_version());
  // The read finished only after the holder released the lock: it waited.
  CHECK(finished_at > released_at.load());
}

TEST_CASE("ProjectDbReadOnly_ManyConcurrentOpens_NeverBusy",
          "[projectdb][concurrency]") {
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }

  constexpr int kThreads = 8;
  constexpr int kRounds = 40;
  std::atomic<int> failures{0};
  std::mutex first_mutex;
  std::string first_error;
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back([&] {
      for (int i = 0; i < kRounds; ++i) {
        try {
          ProjectDB db(tmp.path, /*readOnly=*/true);
          if (db.schema_version() != ProjectDB::latest_schema_version())
            throw std::runtime_error("schema_version read as " +
                                     std::to_string(db.schema_version()));
          (void)db.survey_types();
          (void)db.list_report_pdfs();
        } catch (const std::exception &e) {
          if (failures.fetch_add(1) == 0) {
            std::lock_guard lock(first_mutex);
            first_error = e.what();
          }
        }
      }
    });
  for (auto &thread : threads)
    thread.join();

  INFO("first failure: " << first_error);
  CHECK(failures.load() == 0);
}

TEST_CASE("ProjectDbReadOnly_LockHeldPastTheTimeout_ThrowsInsteadOfMinusOne",
          "[projectdb][concurrency]") {
  // When a lock outlasts the busy timeout, the schema probe must say so. It
  // used to read a busy step as "no schema_version table" and report -1, so a
  // locked project looked like a pre-versioning one (STANDARDS §5). `ruxd
  // --local` turns the "locked" in the message into its 503.
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }
  // Rollback-journal mode, so an idle reader holds no file lock and another
  // connection can take the exclusive lock while the reader stays open. (A
  // WAL reader keeps a shared lock for as long as it is open.)
  {
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const int rc = sqlite3_exec(raw, "PRAGMA journal_mode=DELETE;", nullptr,
                                nullptr, nullptr);
    sqlite3_close(raw);
    REQUIRE(rc == SQLITE_OK);
  }

  ProjectDB reader(tmp.path, /*readOnly=*/true);
  REQUIRE(reader.schema_version() == ProjectDB::latest_schema_version());

  // Hold the exclusive lock comfortably past the 5 s busy timeout.
  int open_rc = -1;
  int lock_rc = -1;
  std::promise<void> locked;
  std::promise<void> release;
  std::thread holder([&] {
    sqlite3 *raw = nullptr;
    open_rc = sqlite3_open(tmp.path.string().c_str(), &raw);
    lock_rc = sqlite3_exec(raw, "BEGIN EXCLUSIVE;", nullptr, nullptr, nullptr);
    locked.set_value();
    release.get_future().wait_for(std::chrono::seconds(8));
    sqlite3_exec(raw, "ROLLBACK;", nullptr, nullptr, nullptr);
    sqlite3_close(raw);
  });
  locked.get_future().wait();

  int version = 0;
  std::string error;
  const auto start = std::chrono::steady_clock::now();
  try {
    version = reader.schema_version();
  } catch (const std::exception &e) {
    error = e.what();
  }
  const auto waited = std::chrono::steady_clock::now() - start;
  release.set_value();
  holder.join();

  REQUIRE(open_rc == SQLITE_OK);
  REQUIRE(lock_rc == SQLITE_OK);
  INFO("schema_version returned " << version << ", error: " << error);
  CHECK(error.find("locked") != std::string::npos);
  CHECK(waited >= std::chrono::milliseconds(4500));
}

namespace {
/// File descriptors this process holds on @p file (Linux /proc).
int open_descriptors_on(const std::filesystem::path &file) {
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path target = fs::canonical(file, ec);
  int count = 0;
  for (const auto &entry : fs::directory_iterator("/proc/self/fd", ec)) {
    std::error_code link_ec;
    const fs::path link = fs::read_symlink(entry.path(), link_ec);
    if (!link_ec && link == target)
      ++count;
  }
  return count;
}
} // namespace

TEST_CASE("ProjectDbReadOnly_OpenFailsOnALock_ClosesItsHandle",
          "[projectdb][concurrency]") {
  // An open that throws after sqlite3_open_v2 (here: the schema probe finds
  // the file locked past the timeout) must still close the handle; ~Impl
  // never runs for a constructor that throws. In `ruxd --local` that is one
  // leaked connection, file descriptor and WAL read lock per 503.
  TempDB tmp;
  {
    ProjectDB db(tmp.path);
  }
  {
    // Rollback-journal mode, as in the test above, so the holder can take an
    // exclusive lock that a fresh open's probe cannot get past.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const int rc = sqlite3_exec(raw, "PRAGMA journal_mode=DELETE;", nullptr,
                                nullptr, nullptr);
    sqlite3_close(raw);
    REQUIRE(rc == SQLITE_OK);
  }
  REQUIRE(open_descriptors_on(tmp.path) == 0);

  int open_rc = -1;
  int lock_rc = -1;
  std::promise<void> locked;
  std::promise<void> release;
  std::thread holder([&] {
    sqlite3 *raw = nullptr;
    open_rc = sqlite3_open(tmp.path.string().c_str(), &raw);
    lock_rc = sqlite3_exec(raw, "BEGIN EXCLUSIVE;", nullptr, nullptr, nullptr);
    locked.set_value();
    release.get_future().wait_for(std::chrono::seconds(8));
    sqlite3_exec(raw, "ROLLBACK;", nullptr, nullptr, nullptr);
    sqlite3_close(raw);
  });
  locked.get_future().wait();

  std::string error;
  try {
    ProjectDB db(tmp.path, /*readOnly=*/true);
  } catch (const std::exception &e) {
    error = e.what();
  }
  release.set_value();
  holder.join();

  REQUIRE(open_rc == SQLITE_OK);
  REQUIRE(lock_rc == SQLITE_OK);
  INFO("open error: " << error);
  REQUIRE(error.find("locked") != std::string::npos);
  // Every connection is gone, so a leaked handle is the only thing that could
  // still hold the file open.
  CHECK(open_descriptors_on(tmp.path) == 0);
}
