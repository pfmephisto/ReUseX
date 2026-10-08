// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pg/admin.hpp"

#include "pg/stores.hpp"

#include <api/api.hpp>

#include <fmt/format.h>

#include <array>
#include <chrono>
#include <csignal>
#include <ctime>
#include <iostream>
#include <stdexcept>

#include <termios.h>
#include <unistd.h>

namespace ruxd::pg {

namespace {

std::string strip_line_end(std::string line) {
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
    line.pop_back();
  return line;
}

/// The terminal settings to put back if Ctrl-C lands mid-prompt.
termios g_saved_tty{};
volatile std::sig_atomic_t g_tty_saved = 0;

extern "C" void restore_tty_and_die(int sig) {
  if (g_tty_saved)
    ::tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tty);
  ::signal(sig, SIG_DFL);
  ::raise(sig);
}

/// Echo off on the terminal for as long as this lives — and back on if the
/// prompt is interrupted (SIGINT/SIGTERM/SIGHUP), not just on return.
class NoEcho {
    public:
  NoEcho() {
    if (::tcgetattr(STDIN_FILENO, &saved_) == 0) {
      g_saved_tty = saved_;
      g_tty_saved = 1;
      struct sigaction handler{};
      handler.sa_handler = restore_tty_and_die;
      ::sigemptyset(&handler.sa_mask);
      for (std::size_t i = 0; i < kSignals.size(); ++i)
        ::sigaction(kSignals[i], &handler, &previous_[i]);
      termios quiet = saved_;
      quiet.c_lflag &= ~static_cast<tcflag_t>(ECHO);
      active_ = ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &quiet) == 0;
    }
  }
  ~NoEcho() {
    if (active_)
      ::tcsetattr(STDIN_FILENO, TCSAFLUSH, &saved_);
    if (g_tty_saved) {
      for (std::size_t i = 0; i < kSignals.size(); ++i)
        ::sigaction(kSignals[i], &previous_[i], nullptr);
      g_tty_saved = 0;
    }
  }
  NoEcho(const NoEcho &) = delete;
  NoEcho &operator=(const NoEcho &) = delete;

    private:
  static constexpr std::array<int, 3> kSignals{SIGINT, SIGTERM, SIGHUP};
  termios saved_{};
  std::array<struct sigaction, 3> previous_{};
  bool active_ = false;
};

std::string read_line_quietly(const std::string &prompt) {
  std::cerr << prompt << std::flush;
  std::string line;
  {
    NoEcho quiet;
    if (!std::getline(std::cin, line))
      throw std::runtime_error("no password given");
  }
  std::cerr << '\n';
  return strip_line_end(line);
}

api::User require_user(api::AuthService &auth, const std::string &email) {
  if (email.empty())
    throw api::HttpError(400, "--email is required");
  auto user = auth.stores().users->find_by_email(email);
  if (!user)
    throw api::HttpError(404, "no user has the email '" +
                                  api::normalize_email(email) + "'");
  return *user;
}

} // namespace

PasswordReader stdin_password_reader() {
  return [](const std::string &prompt) -> std::string {
    if (::isatty(STDIN_FILENO) == 0) {
      std::string line;
      if (!std::getline(std::cin, line))
        throw std::runtime_error("no password on stdin");
      return strip_line_end(line);
    }
    const auto first = read_line_quietly(prompt + ": ");
    const auto again = read_line_quietly("Repeat it: ");
    if (first != again)
      throw std::runtime_error("the two passwords differ");
    return first;
  };
}

int run_admin(const AdminCommand &command, api::AuthService &auth,
              PgCaseStore *cases, const PasswordReader &read_password,
              std::ostream &out, std::ostream &err) {
  using Kind = AdminCommand::Kind;
  try {
    switch (command.kind) {
    case Kind::create_user: {
      if (command.email.empty())
        throw api::HttpError(400, "--email is required");
      const auto password =
          read_password("Password for " + api::normalize_email(command.email));
      const auto user = auth.create_user(command.email, command.display_name,
                                         password, command.is_admin);
      auth.audit(api::superuser_principal(), std::nullopt, "admin.create_user",
                 user.email);
      out << fmt::format("Created {} (id {}{})\n", user.email, user.id,
                         user.is_admin ? ", administrator" : "");
      return 0;
    }
    case Kind::set_password: {
      const auto user = require_user(auth, command.email);
      auth.set_password(user.id,
                        read_password("New password for " + user.email));
      auth.audit(api::superuser_principal(), std::nullopt, "admin.set_password",
                 user.email);
      out << fmt::format("Password changed for {}; their sessions ended\n",
                         user.email);
      return 0;
    }
    case Kind::list_users: {
      const auto users = auth.stores().users->list();
      out << fmt::format("{:>4}  {:<32} {:<24} {}\n", "id", "email", "name",
                         "flags");
      for (const auto &u : users)
        out << fmt::format("{:>4}  {:<32} {:<24} {}{}\n", u.id, u.email,
                           u.display_name, u.is_admin ? "admin " : "",
                           u.disabled ? "disabled" : "");
      if (users.empty())
        out << "(no users; create the first with `ruxd admin create-user "
               "--email … --admin`)\n";
      return 0;
    }
    case Kind::disable_user: {
      const auto user = require_user(auth, command.email);
      auth.set_disabled(user.id, !command.enable);
      auth.audit(api::superuser_principal(), std::nullopt,
                 command.enable ? "admin.enable_user" : "admin.disable_user",
                 user.email);
      out << fmt::format("{} {}\n", user.email,
                         command.enable ? "enabled"
                                        : "disabled; their sessions ended");
      return 0;
    }
    case Kind::create_token: {
      const auto user = require_user(auth, command.email);
      std::optional<std::string> scope;
      if (!command.case_id.empty())
        scope = command.case_id;
      std::optional<std::chrono::seconds> life;
      if (command.expires_days >= 0)
        life = std::chrono::hours(24) * command.expires_days;
      const auto token =
          auth.create_api_token(user.id, command.token_name, scope, life);
      auth.audit(api::superuser_principal(), scope, "admin.create_token",
                 user.email + " " + command.token_name);
      // The token goes to stdout alone, so `token=$(ruxd admin …)` works;
      // the explanation goes to stderr.
      err << fmt::format(
          "API token '{}' for {}{} — shown once, store it now:\n",
          command.token_name, user.email,
          scope ? " (case " + *scope + " only)" : "");
      out << token << '\n';
      return 0;
    }
    case Kind::list_tokens: {
      std::optional<std::int64_t> owner;
      if (!command.email.empty())
        owner = require_user(auth, command.email).id;
      const auto tokens = auth.stores().tokens->list(owner);
      auto day = [](api::SystemClock::time_point tp) {
        const std::time_t t = api::SystemClock::to_time_t(tp);
        std::tm utc{};
        ::gmtime_r(&t, &utc);
        char buf[16];
        std::strftime(buf, sizeof(buf), "%Y-%m-%d", &utc);
        return std::string(buf);
      };
      out << fmt::format("{:>4}  {:<6} {:<20} {:<16} {:<10} {:<10} {}\n", "id",
                         "user", "name", "case", "expires", "last used",
                         "created");
      for (const auto &t : tokens)
        out << fmt::format("{:>4}  {:<6} {:<20} {:<16} {:<10} {:<10} {}\n",
                           t.id, t.user_id, t.name, t.case_id.value_or("(all)"),
                           t.expires_at ? day(*t.expires_at) : "never",
                           t.last_used_at ? day(*t.last_used_at) : "-",
                           day(t.created_at));
      if (tokens.empty())
        out << "(no API tokens)\n";
      return 0;
    }
    case Kind::revoke_token: {
      if (command.token_id <= 0)
        throw api::HttpError(400, "--id is required");
      if (!auth.stores().tokens->revoke(command.token_id, std::nullopt))
        throw api::HttpError(404, "no API token has id " +
                                      std::to_string(command.token_id));
      auth.audit(api::superuser_principal(), std::nullopt, "admin.revoke_token",
                 std::to_string(command.token_id));
      out << fmt::format("Revoked API token {}\n", command.token_id);
      return 0;
    }
    case Kind::register_case: {
      if (!cases)
        throw std::runtime_error("no case store");
      std::optional<std::int64_t> owner;
      if (!command.email.empty())
        owner = require_user(auth, command.email).id;
      const auto info =
          cases->register_path(command.case_name, command.path, owner);
      if (owner)
        auth.stores().members->set_role(info.id, *owner, api::Role::owner);
      auth.audit(api::superuser_principal(), info.id, "admin.register_case",
                 info.path.string());
      out << fmt::format("Registered case '{}' -> {}\n", info.id,
                         info.path.string());
      return 0;
    }
    case Kind::none:
      break;
    }
    err << "ruxd admin: name a command (see ruxd admin --help)\n";
    return 2;
  } catch (const api::HttpError &e) {
    err << "ruxd admin: " << e.what() << '\n';
    return e.status() == 404 ? 3 : 1;
  } catch (const std::exception &e) {
    err << "ruxd admin: " << e.what() << '\n';
    return 1;
  }
}

} // namespace ruxd::pg
