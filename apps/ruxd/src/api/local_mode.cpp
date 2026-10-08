// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/local_mode.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace ruxd::api {

namespace fs = std::filesystem;

namespace {

std::string_view trim(std::string_view value) {
  const auto first = value.find_first_not_of(" \t");
  if (first == std::string_view::npos)
    return {};
  const auto last = value.find_last_not_of(" \t");
  return value.substr(first, last - first + 1);
}

bool iequals(std::string_view a, std::string_view b) {
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) ==
                  std::tolower(static_cast<unsigned char>(y));
         });
}

/// A dotted-quad whose first octet is 127, each octet 0..255.
bool is_ipv4_loopback(std::string_view host) {
  int octets = 0;
  std::size_t pos = 0;
  int first = -1;
  while (pos <= host.size()) {
    const auto dot = std::min(host.find('.', pos), host.size());
    const auto part = host.substr(pos, dot - pos);
    if (part.empty() || part.size() > 3 ||
        !std::all_of(part.begin(), part.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; }))
      return false;
    const int value = std::stoi(std::string(part));
    if (value > 255)
      return false;
    if (octets == 0)
      first = value;
    ++octets;
    pos = dot + 1;
    if (dot == host.size())
      break;
  }
  return octets == 4 && first == 127;
}

} // namespace

bool is_loopback_bind(std::string_view host) {
  if (iequals(host, "localhost"))
    return true;
  if (host == "::1" || host == "[::1]")
    return true;
  return is_ipv4_loopback(host);
}

fs::path resolve_local_project(const fs::path &target) {
  if (target.empty())
    throw std::runtime_error("--local needs a .rux file or a directory");

  std::error_code ec;
  if (fs::is_directory(target, ec)) {
    std::vector<fs::path> found;
    for (const auto &entry : fs::directory_iterator(target, ec))
      if (entry.is_regular_file() && entry.path().extension() == ".rux")
        found.push_back(entry.path());
    if (ec)
      throw std::runtime_error("cannot list '" + target.string() +
                               "': " + ec.message());
    std::sort(found.begin(), found.end());
    if (found.size() == 1)
      return found.front();
    if (found.empty())
      throw std::runtime_error("no .rux project in directory '" +
                               target.string() + "'");
    std::string names;
    for (const auto &p : found)
      names += (names.empty() ? "" : ", ") + p.filename().string();
    throw std::runtime_error(
        "directory '" + target.string() + "' holds " +
        std::to_string(found.size()) + " .rux projects (" + names +
        "); serving several at once is not supported yet — pass one file");
  }

  if (fs::exists(target, ec) && !fs::is_regular_file(target, ec))
    throw std::runtime_error("'" + target.string() +
                             "' is neither a .rux file nor a directory");
  if (target.extension() != ".rux")
    throw std::runtime_error("'" + target.string() +
                             "' is not a .rux project file");
  return target;
}

std::string token_cookie_name(std::uint16_t port) {
  return "ruxd_token_" + std::to_string(port);
}

TokenCheck check_token(std::string_view authorization_header,
                       std::string_view cookie_header,
                       std::string_view query_token,
                       std::string_view cookie_name,
                       std::string_view expected) {
  TokenCheck result;
  if (expected.empty())
    return result;

  const auto auth = trim(authorization_header);
  constexpr std::string_view kBearer = "Bearer ";
  if (auth.size() > kBearer.size() &&
      iequals(auth.substr(0, kBearer.size()), kBearer) &&
      token_matches(trim(auth.substr(kBearer.size())), expected))
    result.ok = true;

  // Cookie: a=b; ruxd_token_8420=...; c=d — every same-named entry counts.
  std::size_t pos = 0;
  while (!result.ok && pos < cookie_header.size()) {
    const auto end =
        std::min(cookie_header.find(';', pos), cookie_header.size());
    const auto pair = trim(cookie_header.substr(pos, end - pos));
    const auto eq = pair.find('=');
    if (eq != std::string_view::npos &&
        trim(pair.substr(0, eq)) == cookie_name &&
        token_matches(trim(pair.substr(eq + 1)), expected))
      result.ok = true;
    pos = end + 1;
  }

  if (!query_token.empty() && token_matches(query_token, expected)) {
    result.ok = true;
    result.via_query = true;
  }
  return result;
}

std::string strip_query_param(std::string_view raw_url, std::string_view key) {
  const auto q = raw_url.find('?');
  if (q == std::string_view::npos)
    return std::string(raw_url);
  std::string out(raw_url.substr(0, q));
  std::string kept;
  std::string_view query = raw_url.substr(q + 1);
  std::size_t pos = 0;
  while (pos <= query.size()) {
    const auto end = std::min(query.find('&', pos), query.size());
    const auto part = query.substr(pos, end - pos);
    const auto name = part.substr(0, part.find('='));
    if (!part.empty() && name != key) {
      if (!kept.empty())
        kept += '&';
      kept += part;
    }
    if (end == query.size())
      break;
    pos = end + 1;
  }
  if (!kept.empty())
    out += "?" + kept;
  return out;
}

std::string redact_query_strings(std::string_view text) {
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] != '?') {
      out += text[i];
      continue;
    }
    out += "?<redacted>";
    while (i + 1 < text.size() &&
           !std::isspace(static_cast<unsigned char>(text[i + 1])))
      ++i;
  }
  return out;
}

bool origin_matches_host(std::string_view origin, std::string_view host) {
  if (host.empty())
    return false;
  for (std::string_view scheme : {"http://", "https://"}) {
    if (origin.size() == scheme.size() + host.size() &&
        iequals(origin.substr(0, scheme.size()), scheme) &&
        iequals(origin.substr(scheme.size()), host))
      return true;
  }
  return false;
}

namespace {

/// The hostname part of a Host header or URL authority, brackets kept for
/// IPv6: "a:8420" -> "a", "[::1]:8420" -> "[::1]", "::1" -> "::1".
std::string_view host_name_of(std::string_view authority) {
  if (!authority.empty() && authority.front() == '[') {
    const auto close = authority.find(']');
    return close == std::string_view::npos ? authority
                                           : authority.substr(0, close + 1);
  }
  const auto colon = authority.find(':');
  if (colon != std::string_view::npos &&
      authority.find(':', colon + 1) == std::string_view::npos)
    return authority.substr(0, colon);
  return authority;
}

/// "::1" and "[::1]" compare equal; so do "0.0.0.0" and itself.
std::string_view unbracket(std::string_view host) {
  if (host.size() >= 2 && host.front() == '[' && host.back() == ']')
    return host.substr(1, host.size() - 2);
  return host;
}

} // namespace

bool host_allowed(std::string_view host_header, std::string_view bind_address,
                  const std::vector<std::string> &allowed_origins) {
  const auto host = unbracket(host_name_of(trim(host_header)));
  if (host.empty())
    return false;

  if (is_loopback_bind(bind_address))
    return iequals(host, "localhost") || host == "127.0.0.1" || host == "::1";

  const auto bind = unbracket(bind_address);
  if (bind == "0.0.0.0" || bind == "::")
    return true;

  if (iequals(host, bind))
    return true;
  for (const auto &origin : allowed_origins) {
    const auto scheme_end = origin.find("://");
    const std::string_view authority =
        scheme_end == std::string::npos
            ? std::string_view(origin)
            : std::string_view(origin).substr(scheme_end + 3);
    if (iequals(host, unbracket(host_name_of(authority))))
      return true;
  }
  return false;
}

bool token_matches(std::string_view presented, std::string_view expected) {
  if (expected.empty())
    return false;
  // Fold over the full expected length whatever the presented one is, so
  // neither the position of the first difference nor a length mismatch shows
  // up as an early return.
  unsigned char diff = presented.size() == expected.size() ? 0 : 1;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    const unsigned char p =
        i < presented.size() ? static_cast<unsigned char>(presented[i]) : 0;
    diff |=
        static_cast<unsigned char>(p ^ static_cast<unsigned char>(expected[i]));
  }
  return diff == 0;
}

} // namespace ruxd::api
