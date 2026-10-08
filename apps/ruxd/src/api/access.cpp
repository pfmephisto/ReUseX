// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/access.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <sys/socket.h>

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

  static constexpr std::array<std::string_view, 4> kPublic{{
      "/api/v1/health",
      "/api/v1/readyz",
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

// --- client addresses
// ------------------------------------------------------------

namespace {

struct Ip {
  int family = 0; // AF_INET / AF_INET6
  std::array<unsigned char, 16> bytes{};
};

std::optional<Ip> parse_ip(std::string_view text) {
  std::string s(text);
  // Trim and drop brackets ("[::1]").
  const auto first = s.find_first_not_of(" \t");
  const auto last = s.find_last_not_of(" \t");
  if (first == std::string::npos)
    return std::nullopt;
  s = s.substr(first, last - first + 1);
  if (s.size() >= 2 && s.front() == '[' && s.back() == ']')
    s = s.substr(1, s.size() - 2);
  Ip ip;
  if (::inet_pton(AF_INET, s.c_str(), ip.bytes.data()) == 1) {
    ip.family = AF_INET;
    return ip;
  }
  if (::inet_pton(AF_INET6, s.c_str(), ip.bytes.data()) == 1) {
    ip.family = AF_INET6;
    // An IPv4-mapped IPv6 address (::ffff:a.b.c.d) is the IPv4 address.
    static constexpr std::array<unsigned char, 12> kMapped{
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff};
    if (std::equal(kMapped.begin(), kMapped.end(), ip.bytes.begin())) {
      std::array<unsigned char, 16> v4{};
      std::copy(ip.bytes.begin() + 12, ip.bytes.end(), v4.begin());
      ip.bytes = v4;
      ip.family = AF_INET;
    }
    return ip;
  }
  return std::nullopt;
}

struct Cidr {
  Ip net;
  int prefix = 0;
};

std::optional<Cidr> parse_cidr(std::string_view text) {
  const auto slash = text.find('/');
  const auto ip = parse_ip(text.substr(0, slash));
  if (!ip)
    return std::nullopt;
  const int max = ip->family == AF_INET ? 32 : 128;
  int prefix = max;
  if (slash != std::string_view::npos) {
    const auto digits = text.substr(slash + 1);
    if (digits.empty() || digits.size() > 3 ||
        !std::all_of(digits.begin(), digits.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; }))
      return std::nullopt;
    prefix = std::stoi(std::string(digits));
    if (prefix > max)
      return std::nullopt;
  }
  return Cidr{*ip, prefix};
}

bool in_network(const Cidr &cidr, const Ip &ip) {
  if (cidr.net.family != ip.family)
    return false;
  int bits = cidr.prefix;
  for (std::size_t i = 0; bits > 0; ++i, bits -= 8) {
    const unsigned char mask =
        bits >= 8 ? 0xff : static_cast<unsigned char>(0xff << (8 - bits));
    if ((cidr.net.bytes[i] & mask) != (ip.bytes[i] & mask))
      return false;
  }
  return true;
}

bool trusted(std::string_view ip, const std::vector<std::string> &proxies) {
  return std::any_of(proxies.begin(), proxies.end(), [&](const std::string &c) {
    return cidr_contains(c, ip);
  });
}

} // namespace

bool cidr_contains(std::string_view cidr, std::string_view ip) {
  const auto net = parse_cidr(cidr);
  const auto addr = parse_ip(ip);
  return net && addr && in_network(*net, *addr);
}

bool is_valid_cidr(std::string_view cidr) {
  return parse_cidr(cidr).has_value();
}

std::string client_address(std::string_view peer,
                           std::string_view forwarded_for,
                           const std::vector<std::string> &trusted_proxies) {
  if (forwarded_for.empty() || !trusted(peer, trusted_proxies))
    return std::string(peer);
  // Right to left: the entries a trusted proxy appended are trustworthy up
  // to the first one that is not itself one of our proxies.
  std::vector<std::string> hops;
  std::size_t start = 0;
  for (;;) {
    const auto comma = forwarded_for.find(',', start);
    auto hop = forwarded_for.substr(start, comma - start);
    const auto a = hop.find_first_not_of(" \t");
    const auto b = hop.find_last_not_of(" \t");
    if (a != std::string_view::npos)
      hops.emplace_back(hop.substr(a, b - a + 1));
    if (comma == std::string_view::npos)
      break;
    start = comma + 1;
  }
  for (auto it = hops.rbegin(); it != hops.rend(); ++it) {
    if (!parse_ip(*it))
      return std::string(peer); // Garbage: fall back to the proxy itself.
    if (!trusted(*it, trusted_proxies))
      return *it;
  }
  return hops.empty() ? std::string(peer) : hops.front();
}

std::string rate_limit_key(std::string_view text) {
  const auto ip = parse_ip(text);
  if (!ip)
    return "raw:" + std::string(text);
  char buffer[INET6_ADDRSTRLEN] = {};
  if (ip->family == AF_INET) {
    ::inet_ntop(AF_INET, ip->bytes.data(), buffer, sizeof(buffer));
    return std::string("v4:") + buffer;
  }
  std::array<unsigned char, 16> net{};
  std::copy(ip->bytes.begin(), ip->bytes.begin() + 8, net.begin());
  ::inet_ntop(AF_INET6, net.data(), buffer, sizeof(buffer));
  return std::string("v6:") + buffer + "/64";
}

// --- LoginRateLimiter
// ----------------------------------------------------------

LoginRateLimiter::LoginRateLimiter(LoginRateLimitOptions options)
    : options_(std::move(options)) {}

namespace {
std::string account_key(std::string_view email) {
  std::string out = "acct:";
  for (const char c : email)
    out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}
std::string address_key_of(std::string_view key) {
  return "addr:" + std::string(key);
}
} // namespace

std::chrono::seconds LoginRateLimiter::wait_locked(const std::string &key,
                                                   std::size_t free,
                                                   Clock::time_point now) {
  auto it = failures_.find(key);
  if (it == failures_.end())
    return std::chrono::seconds(0);
  if (now - it->second.last >= options_.forget_after) {
    failures_.erase(it);
    return std::chrono::seconds(0);
  }
  if (it->second.count < free)
    return std::chrono::seconds(0);
  const std::size_t over = it->second.count - free; // 0 for the first lock
  auto delay = options_.base_delay;
  for (std::size_t i = 0; i < over && delay < options_.max_delay; ++i)
    delay *= 2;
  delay = std::min(delay, options_.max_delay);
  const auto until = it->second.last + delay;
  if (until <= now)
    return std::chrono::seconds(0);
  return std::max(std::chrono::ceil<std::chrono::seconds>(until - now),
                  std::chrono::seconds(1));
}

void LoginRateLimiter::fail_locked(const std::string &key,
                                   Clock::time_point now) {
  auto &f = failures_[key];
  if (f.count > 0 && now - f.last >= options_.forget_after)
    f.count = 0;
  ++f.count;
  f.last = now;
}

void LoginRateLimiter::prune(Clock::time_point now) {
  std::erase_if(failures_, [&](const auto &kv) {
    return now - kv.second.last >= options_.forget_after;
  });
  while (failures_.size() > options_.max_keys) {
    auto stalest = failures_.begin();
    for (auto it = failures_.begin(); it != failures_.end(); ++it)
      if (it->second.last < stalest->second.last)
        stalest = it;
    failures_.erase(stalest);
  }
}

std::chrono::seconds LoginRateLimiter::wait(std::string_view address,
                                            std::string_view email,
                                            Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  return std::max(
      wait_locked(account_key(email), options_.free_failures_per_account, now),
      wait_locked(address_key_of(address), options_.free_failures_per_address,
                  now));
}

void LoginRateLimiter::failure(std::string_view address, std::string_view email,
                               Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  fail_locked(account_key(email), now);
  fail_locked(address_key_of(address), now);
  if (failures_.size() > options_.max_keys)
    prune(now);
}

void LoginRateLimiter::success(std::string_view email) {
  std::lock_guard<std::mutex> lock(mutex_);
  failures_.erase(account_key(email));
}

std::chrono::seconds LoginRateLimiter::wait_address(std::string_view address,
                                                    Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  return wait_locked(address_key_of(address),
                     options_.free_failures_per_address, now);
}

void LoginRateLimiter::failure_address(std::string_view address,
                                       Clock::time_point now) {
  std::lock_guard<std::mutex> lock(mutex_);
  fail_locked(address_key_of(address), now);
  if (failures_.size() > options_.max_keys)
    prune(now);
}

} // namespace ruxd::api
