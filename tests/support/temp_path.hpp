// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Unique, self-cleaning temporary paths for tests (#262).
//
// `catch_discover_tests` registers one ctest test per TEST_CASE and each one
// runs in its own process, so a temp name has to be unique across *processes*,
// not just within one. The helpers these replaced derived their names from
// `reinterpret_cast<uintptr_t>(this)`; under ASLR two concurrent ctest workers
// routinely land the same stack/heap address, pick the same file, and then
// fight over the same sqlite database ("database is locked" / "disk I/O
// error"). The name is therefore built from:
//
//   <prefix>_<pid>_<per-process random salt>_<atomic counter><suffix>
//
// pid separates concurrent processes, the counter separates instances inside
// one process, and the salt keeps a recycled pid from colliding with files a
// previous crashed run left behind.
//
// Include as "../../support/temp_path.hpp" from tests/unit/<module>/.

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <string_view>
#include <system_error>

#include <unistd.h>

namespace reusex::test_support {

namespace detail {

/// Monotonic per-process counter, distinguishing instances within a process.
inline std::uint64_t next_temp_id() {
  static std::atomic<std::uint64_t> counter{0};
  return counter.fetch_add(1, std::memory_order_relaxed);
}

/// Random salt drawn once per process; guards against pid reuse picking up
/// leftovers from an earlier run that died before cleaning up.
inline std::uint32_t process_salt() {
  static const std::uint32_t salt = [] {
    std::random_device rd;
    return static_cast<std::uint32_t>(rd());
  }();
  return salt;
}

inline std::filesystem::path unique_temp_path(std::string_view prefix,
                                              std::string_view suffix) {
  return std::filesystem::temp_directory_path() /
         (std::string(prefix) + "_" +
          std::to_string(static_cast<long long>(::getpid())) + "_" +
          std::to_string(process_salt()) + "_" +
          std::to_string(next_temp_id()) + std::string(suffix));
}

} // namespace detail

/// A unique temp *file* path that is not created up front — hand it to
/// whatever is under test (e.g. `ProjectDB::create`) and let that make the
/// file. The destructor removes the file plus the sqlite `-wal` / `-shm`
/// sidecars, which the per-file helpers this replaced used to leak.
class TempPath {
    public:
  /// @param prefix  basename prefix, ideally naming the test file
  /// @param suffix  extension including the dot (default: `.rux`)
  explicit TempPath(std::string_view prefix = "reusex_test",
                    std::string_view suffix = ".rux")
      : path(detail::unique_temp_path(prefix, suffix)) {}

  TempPath(const TempPath &) = delete;
  TempPath &operator=(const TempPath &) = delete;

  ~TempPath() noexcept {
    std::error_code ec;
    std::filesystem::remove(path, ec);
    std::filesystem::remove(path.string() + "-wal", ec);
    std::filesystem::remove(path.string() + "-shm", ec);
  }

  /// Public by design: every call site this replaced already spelled it
  /// `tmp.path`.
  const std::filesystem::path path;
};

/// A unique temp *directory*, created on construction and removed
/// recursively on destruction.
class TempDir {
    public:
  explicit TempDir(std::string_view prefix = "reusex_test")
      : path(detail::unique_temp_path(prefix, "")) {
    std::filesystem::create_directories(path);
  }

  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;

  ~TempDir() noexcept {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }

  const std::filesystem::path path;
};

} // namespace reusex::test_support
