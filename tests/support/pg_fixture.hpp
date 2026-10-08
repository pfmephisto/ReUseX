// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// An ephemeral PostgreSQL server for ruxd's [postgres] integration tests
// (spec 2026-10-08, phase S3).
//
// `initdb` a fresh cluster into a TempDir, start it with `pg_ctl` listening on
// a Unix socket only (no TCP port, so parallel ctest processes never clash),
// and stop it (`-m immediate`) and delete it on destruction. Each test case
// is its own process under catch_discover_tests, so each gets its own
// cluster; initdb takes about a second.
//
// When `initdb` or `pg_ctl` is not on PATH — the nix build sandbox, a machine
// without PostgreSQL — the tests SKIP rather than fail:
//
//   TEST_CASE("…", "[postgres]") {
//     REUSEX_REQUIRE_POSTGRES();
//     reusex::test_support::EphemeralPostgres pg;
//     pqxx::connection conn(pg.dsn());
//
// The devshell (shell.nix) carries `postgresql`, so they run locally.
//
// Include as "../../support/pg_fixture.hpp" from tests/unit/<module>/.

#include "temp_path.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace reusex::test_support {

/// The first executable named @p name on PATH, or nullopt.
inline std::optional<std::filesystem::path>
find_on_path(const std::string &name) {
  const char *path = std::getenv("PATH");
  if (path == nullptr)
    return std::nullopt;
  std::stringstream dirs(path);
  std::string dir;
  while (std::getline(dirs, dir, ':')) {
    if (dir.empty())
      continue;
    const auto candidate = std::filesystem::path(dir) / name;
    if (::access(candidate.c_str(), X_OK) == 0)
      return candidate;
  }
  return std::nullopt;
}

class EphemeralPostgres {
    public:
  /// initdb and pg_ctl are both on PATH.
  static bool available() {
    return find_on_path("initdb") && find_on_path("pg_ctl");
  }

  /// @throws std::runtime_error (with the server's log) when it cannot start.
  EphemeralPostgres() {
    // The socket path must fit sockaddr_un (~107 bytes): keep it short and
    // under /tmp whatever TMPDIR is.
    socket_dir_ = std::filesystem::path("/tmp") /
                  ("rxpg_" + std::to_string(::getpid()) + "_" +
                   std::to_string(detail::next_temp_id()));
    std::filesystem::create_directories(socket_dir_);
    const auto data = dir_.path / "data";
    const auto log = dir_.path / "server.log";
    run({find_on_path("initdb")->string(), "-D", data.string(), "-U", kUser,
         "-A", "trust", "--no-sync", "-E", "UTF8", "--locale=C"},
        log);
    const std::string options = "-c listen_addresses='' -c fsync=off "
                                "-c unix_socket_directories=" +
                                socket_dir_.string();
    run({find_on_path("pg_ctl")->string(), "-D", data.string(), "-l",
         log.string(), "-o", options, "-w", "-t", "30", "start"},
        log);
    started_ = true;
  }

  ~EphemeralPostgres() {
    if (started_)
      try {
        run({find_on_path("pg_ctl")->string(), "-D",
             (dir_.path / "data").string(), "-m", "immediate", "-w", "stop"},
            dir_.path / "stop.log");
      } catch (...) {
      }
    std::error_code ec;
    std::filesystem::remove_all(socket_dir_, ec);
  }

  EphemeralPostgres(const EphemeralPostgres &) = delete;
  EphemeralPostgres &operator=(const EphemeralPostgres &) = delete;

  /// A libpq connection string for database @p db (default: postgres).
  std::string dsn(const std::string &db = "postgres") const {
    return "host=" + socket_dir_.string() + " user=" + kUser + " dbname=" + db;
  }

    private:
  static constexpr const char *kUser = "ruxd";

  /// Run @p argv to completion with output appended to @p log.
  /// @throws std::runtime_error with the log when it fails.
  static void run(const std::vector<std::string> &argv,
                  const std::filesystem::path &log) {
    const pid_t pid = ::fork();
    if (pid < 0)
      throw std::runtime_error("fork failed");
    if (pid == 0) {
      const int fd = ::open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0600);
      if (fd >= 0) {
        ::dup2(fd, STDOUT_FILENO);
        ::dup2(fd, STDERR_FILENO);
      }
      std::vector<char *> args;
      for (const auto &a : argv)
        args.push_back(const_cast<char *>(a.c_str()));
      args.push_back(nullptr);
      ::execv(args[0], args.data());
      ::_exit(127);
    }
    int status = 0;
    ::waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      std::ifstream in(log);
      std::stringstream text;
      text << in.rdbuf();
      throw std::runtime_error(argv[0] + " failed:\n" + text.str());
    }
  }

  TempDir dir_{"ruxd_pg"};
  std::filesystem::path socket_dir_;
  bool started_ = false;
};

} // namespace reusex::test_support

/// SKIP the current test when no PostgreSQL server binaries are on PATH.
#define REUSEX_REQUIRE_POSTGRES()                                              \
  do {                                                                         \
    if (!::reusex::test_support::EphemeralPostgres::available())               \
      SKIP("initdb/pg_ctl not on PATH (add postgresql to the shell)");         \
  } while (false)
