// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// `ruxd admin …`: account and case administration against the server's
// Postgres database, from a shell on the server (spec 2026-10-08, phase S3).
// The first administrator is made this way:
//
//   ruxd admin create-user --email anna@example.dk --name "Anna" --admin
//
// Passwords are never taken from argv (it shows in `ps` and in shell
// history): they are prompted for, without echo, on a terminal, or read as
// one line from stdin otherwise (`… < secret.txt`, or a pipe).

#include <api/AuthService.hpp>

#include <filesystem>
#include <functional>
#include <iosfwd>
#include <string>

namespace ruxd::pg {

class PgCaseStore;

struct AdminCommand {
  enum class Kind {
    none,
    create_user,
    set_password,
    list_users,
    disable_user,
    create_token,
    register_case,
  };
  Kind kind = Kind::none;
  std::string email;
  std::string display_name;
  bool is_admin = false;
  /// disable-user --enable: re-enable instead.
  bool enable = false;
  /// create-token --name.
  std::string token_name;
  /// create-token --case: limit the token to this case id.
  std::string case_id;
  /// register-case: the existing project file, the case name and owner.
  std::filesystem::path path;
  std::string case_name;
};

/// Asks for a password; @p prompt names what for. Returns it without the
/// line ending. @throws std::runtime_error when none can be read.
using PasswordReader = std::function<std::string(const std::string &prompt)>;

/// The terminal-or-stdin reader described above: on a terminal it prompts on
/// stderr with echo off and asks twice; otherwise it reads one line.
PasswordReader stdin_password_reader();

/// Run @p command. Output for a person on @p out, errors on @p err.
/// @return a process exit code (0 = done).
int run_admin(const AdminCommand &command, api::AuthService &auth,
              PgCaseStore *cases, const PasswordReader &read_password,
              std::ostream &out, std::ostream &err);

} // namespace ruxd::pg
