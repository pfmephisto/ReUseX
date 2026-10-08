// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/access.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <utility>

namespace ruxd::api {
namespace {

bool starts_with(std::string_view text, std::string_view prefix) {
  return text.substr(0, prefix.size()) == prefix;
}

/// @p path with @p prefix removed when it is exactly the prefix or the
/// prefix followed by '/', else nullopt. "/api/v1/users" and "/api/v1/users/7"
/// are under "/api/v1/users"; "/api/v1/usersx" is not.
std::optional<std::string_view> under(std::string_view path,
                                      std::string_view prefix) {
  if (!starts_with(path, prefix))
    return std::nullopt;
  const auto rest = path.substr(prefix.size());
  if (rest.empty() || rest.front() == '/')
    return rest;
  return std::nullopt;
}

} // namespace

std::string_view to_string(Role role) {
  switch (role) {
  case Role::viewer:
    return "viewer";
  case Role::editor:
    return "editor";
  case Role::owner:
    return "owner";
  }
  return "viewer";
}

std::optional<Role> parse_role(std::string_view name) {
  if (name == "viewer")
    return Role::viewer;
  if (name == "editor")
    return Role::editor;
  if (name == "owner")
    return Role::owner;
  return std::nullopt;
}

bool role_allows(Role role, CaseOp op) {
  switch (op) {
  case CaseOp::read:
    return true;
  case CaseOp::write:
    return role == Role::editor || role == Role::owner;
  case CaseOp::delete_case:
  case CaseOp::manage_members:
    return role == Role::owner;
  }
  return false;
}

bool is_safe_method(std::string_view method) {
  return method == "GET" || method == "HEAD" || method == "OPTIONS";
}

RouteAccess classify_route(std::string_view method, std::string_view path) {
  using Kind = RouteAccess::Kind;
  if (!under(path, "/api"))
    return {Kind::static_file, {}, CaseOp::read};

  static constexpr std::array<std::string_view, 5> kPublic{{
      "/api/v1/health",
      "/api/v1/readyz",
      "/api/v1/livez",
      "/api/v1/endpoints",
      "/api/v1/auth/login",
  }};
  if (std::find(kPublic.begin(), kPublic.end(), path) != kPublic.end())
    return {Kind::public_route, {}, CaseOp::read};

  if (under(path, "/api/v1/users"))
    return {Kind::admin, {}, CaseOp::read};
  if (under(path, "/api/v1/uploads"))
    return {Kind::create_case, {}, CaseOp::write};

  if (const auto rest = under(path, "/api/v1/cases")) {
    if (rest->empty() || *rest == "/")
      return {method == "POST" ? Kind::create_case : Kind::authenticated,
              {},
              CaseOp::read};
    // rest = "/<cid>[/...]"
    const auto tail = rest->substr(1);
    const auto slash = tail.find('/');
    RouteAccess out{Kind::case_route, std::string(tail.substr(0, slash)),
                    CaseOp::read};
    const std::string_view sub = slash == std::string_view::npos
                                     ? std::string_view{}
                                     : tail.substr(slash);
    if (is_safe_method(method))
      out.op = CaseOp::read;
    else if (sub.empty() || sub == "/")
      out.op = method == "DELETE" ? CaseOp::delete_case : CaseOp::write;
    else if (under(sub, "/members"))
      out.op = CaseOp::manage_members;
    else
      out.op = CaseOp::write;
    return out;
  }
  return {Kind::authenticated, {}, CaseOp::read};
}

std::string_view to_string(PrincipalKind kind) {
  switch (kind) {
  case PrincipalKind::anonymous:
    return "anonymous";
  case PrincipalKind::local:
    return "local";
  case PrincipalKind::superuser:
    return "superuser";
  case PrincipalKind::session:
    return "session";
  case PrincipalKind::api_token:
    return "token";
  }
  return "anonymous";
}

Principal local_principal() {
  Principal p;
  p.kind = PrincipalKind::local;
  p.user.id = 0;
  p.user.email = "local";
  p.user.display_name = "local";
  p.user.is_admin = true;
  return p;
}

Principal superuser_principal() {
  Principal p;
  p.kind = PrincipalKind::superuser;
  p.user.id = 0;
  p.user.email = "superuser";
  p.user.display_name = "superuser";
  p.user.is_admin = true;
  return p;
}

AccessDecision decide_access(const Principal &who, const RouteAccess &route,
                             std::optional<Role> role) {
  using Kind = RouteAccess::Kind;
  switch (route.kind) {
  case Kind::public_route:
  case Kind::static_file:
    return {};
  default:
    break;
  }
  if (!who.authenticated())
    return {401, "sign in first (no valid session or token)"};

  switch (route.kind) {
  case Kind::authenticated:
    return {};
  case Kind::create_case:
    if (who.case_scope)
      return {403, "a token limited to one case cannot create cases"};
    return {};
  case Kind::admin:
    if (who.case_scope || !who.is_admin())
      return {403, "only an administrator may manage users"};
    return {};
  case Kind::case_route:
    break;
  default:
    return {};
  }

  // A case route. A token scoped to another case sees nothing here, exactly
  // as a non-member does.
  if (who.case_scope && *who.case_scope != route.cid)
    return {404, "no such case '" + route.cid + "'"};
  if (who.is_admin())
    return {};
  if (!role)
    return {404, "no such case '" + route.cid + "'"};
  if (!role_allows(*role, route.op)) {
    const std::string_view what =
        route.op == CaseOp::delete_case      ? "delete this case"
        : route.op == CaseOp::manage_members ? "manage this case's members"
                                             : "change this case";
    return {403, "your role in this case (" + std::string(to_string(*role)) +
                     ") does not allow you to " + std::string(what)};
  }
  return {};
}

// --- LoginRateLimiter
// ----------------------------------------------------------

LoginRateLimiter::LoginRateLimiter(LoginRateLimitOptions options)
    : options_(std::move(options)) {}

std::chrono::seconds
LoginRateLimiter::wait_for(std::deque<Clock::time_point> &window,
                           std::size_t limit, Clock::time_point now) {
  while (!window.empty() && window.front() <= now - options_.window)
    window.pop_front();
  if (window.size() < limit)
    return std::chrono::seconds(0);
  const auto free_at = window.front() + options_.window;
  const auto wait = std::chrono::ceil<std::chrono::seconds>(free_at - now);
  return std::max(wait, std::chrono::seconds(1));
}

void LoginRateLimiter::prune(Clock::time_point now) {
  for (auto it = windows_.begin(); it != windows_.end();) {
    auto &window = it->second;
    while (!window.empty() && window.front() <= now - options_.window)
      window.pop_front();
    it = window.empty() ? windows_.erase(it) : std::next(it);
  }
  // Still over: drop the keys whose newest attempt is oldest.
  while (windows_.size() > options_.max_keys) {
    auto oldest = windows_.begin();
    for (auto it = windows_.begin(); it != windows_.end(); ++it)
      if (it->second.back() < oldest->second.back())
        oldest = it;
    windows_.erase(oldest);
  }
}

std::chrono::seconds LoginRateLimiter::attempt(std::string_view ip,
                                               std::string_view email,
                                               Clock::time_point now) {
  std::string lowered(email);
  std::transform(
      lowered.begin(), lowered.end(), lowered.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const std::string ip_key = "ip\n" + std::string(ip);
  const std::string pair_key = "pair\n" + std::string(ip) + "\n" + lowered;

  std::lock_guard<std::mutex> lock(mutex_);
  if (windows_.size() > options_.max_keys)
    prune(now);
  auto &by_ip = windows_[ip_key];
  auto &by_pair = windows_[pair_key];
  const auto wait = std::max(wait_for(by_pair, options_.per_ip_and_email, now),
                             wait_for(by_ip, options_.per_ip, now));
  if (wait.count() > 0)
    return wait;
  by_ip.push_back(now);
  by_pair.push_back(now);
  return std::chrono::seconds(0);
}

} // namespace ruxd::api
