// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/tokens.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <regex>
#include <set>

namespace rux::qt {
namespace {

/// Replace every /* … */ comment with spaces so offsets stay meaningful.
std::string blank_comments(std::string_view in) {
  std::string out(in);
  std::size_t pos = 0;
  while ((pos = out.find("/*", pos)) != std::string::npos) {
    std::size_t end = out.find("*/", pos + 2);
    end = end == std::string::npos ? out.size() : end + 2;
    for (std::size_t i = pos; i < end; ++i)
      if (out[i] != '\n')
        out[i] = ' ';
    pos = end;
  }
  return out;
}

std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
    s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
    s.remove_suffix(1);
  return s;
}

/// Bodies of the top-level blocks whose selector is exactly @p selector.
/// Nested blocks (the `:root` inside `@media`) are skipped because only
/// depth-0 selectors are compared.
std::vector<std::string> top_level_blocks(const std::string &css,
                                          std::string_view selector) {
  std::vector<std::string> bodies;
  int depth = 0;
  std::size_t sel_start = 0;
  std::size_t body_start = 0;
  bool capture = false;
  for (std::size_t i = 0; i < css.size(); ++i) {
    const char c = css[i];
    if (c == '{') {
      if (depth == 0) {
        capture = trim(std::string_view(css).substr(sel_start,
                                                    i - sel_start)) == selector;
        body_start = i + 1;
      }
      ++depth;
    } else if (c == '}') {
      --depth;
      if (depth == 0) {
        if (capture)
          bodies.push_back(css.substr(body_start, i - body_start));
        capture = false;
        sel_start = i + 1;
      }
    } else if (c == ';' && depth == 0) {
      sel_start = i + 1; // e.g. an @import line
    }
  }
  return bodies;
}

void parse_declarations(const std::string &body, TokenMap &out) {
  // Declarations are `--name: value;` — split on ';' (token values never
  // contain one).
  std::size_t start = 0;
  while (start < body.size()) {
    std::size_t end = body.find(';', start);
    if (end == std::string::npos)
      end = body.size();
    std::string_view decl =
        trim(std::string_view(body).substr(start, end - start));
    start = end + 1;
    if (decl.substr(0, 2) != "--")
      continue;
    const std::size_t colon = decl.find(':');
    if (colon == std::string_view::npos)
      continue;
    const std::string name(trim(decl.substr(0, colon)));
    out[name] = normalise_value(decl.substr(colon + 1));
  }
}

const std::regex &var_re() {
  static const std::regex re(R"(var\(\s*(--[A-Za-z0-9_-]+)\s*\))");
  return re;
}

std::string format_px(double px) {
  const long r = std::lround(px);
  return r == 0 ? std::string("0") : std::to_string(r) + "px";
}

/// Locale-independent: std::stod honours LC_NUMERIC, and a QApplication
/// sets the user's locale — under da_DK "0.75" parses as 0.
std::optional<double> to_double(std::string_view s) {
  double v = 0;
  const auto *b = s.data();
  const auto *e = s.data() + s.size();
  auto [p, ec] = std::from_chars(b, e, v);
  if (ec != std::errc() || p != e)
    return std::nullopt;
  return v;
}

std::optional<double> number_with_unit(std::string_view value,
                                       std::string_view unit) {
  value = trim(value);
  if (value.size() <= unit.size() ||
      value.substr(value.size() - unit.size()) != unit)
    return std::nullopt;
  return to_double(value.substr(0, value.size() - unit.size()));
}

int hex_digit(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return -1;
}

std::uint8_t clamp_byte(double v) {
  return static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L));
}

} // namespace

std::string normalise_value(std::string_view value) {
  // Collapse runs of whitespace (multi-line font stacks).
  std::string v;
  bool space = false;
  for (char c : trim(value)) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      space = true;
      continue;
    }
    if (space && !v.empty())
      v += ' ';
    space = false;
    v += c;
  }

  // rem -> whole px.
  static const std::regex rem(R"((-?\d*\.?\d+)rem\b)");
  std::string out;
  auto it = std::sregex_iterator(v.begin(), v.end(), rem);
  std::size_t last = 0;
  for (; it != std::sregex_iterator(); ++it) {
    out.append(v, last, it->position() - last);
    out += format_px(to_double((*it)[1].str()).value_or(0.0) * kRootFontPx);
    last = it->position() + it->length();
  }
  out.append(v, last);

  // rgb(r g b / a) -> rgba(r, g, b, a). The comma form passes through.
  static const std::regex css4(
      R"(rgba?\(\s*([\d.]+%?)\s+([\d.]+%?)\s+([\d.]+%?)\s*(?:/\s*([\d.]+%?)\s*)?\))");
  std::string result;
  last = 0;
  for (auto m = std::sregex_iterator(out.begin(), out.end(), css4);
       m != std::sregex_iterator(); ++m) {
    result.append(out, last, m->position() - last);
    const std::string alpha = (*m)[4].matched ? (*m)[4].str() : "255";
    result += "rgba(" + (*m)[1].str() + ", " + (*m)[2].str() + ", " +
              (*m)[3].str() + ", " + alpha + ")";
    last = m->position() + m->length();
  }
  result.append(out, last);
  return result;
}

TokenMap parse_tokens_css(std::string_view css_in, ThemeMode mode) {
  const std::string css = blank_comments(css_in);
  TokenMap tokens;
  for (const auto &body : top_level_blocks(css, ":root"))
    parse_declarations(body, tokens);
  if (mode == ThemeMode::dark) {
    for (const auto &body : top_level_blocks(css, "[data-theme='dark']"))
      parse_declarations(body, tokens);
    for (const auto &body : top_level_blocks(css, "[data-theme=\"dark\"]"))
      parse_declarations(body, tokens);
  }
  // Tokens defined in terms of other tokens. A few passes cover any sane
  // chain; a cycle just stops resolving.
  for (int pass = 0; pass < 4; ++pass) {
    bool changed = false;
    for (auto &[name, value] : tokens) {
      if (value.find("var(") == std::string::npos)
        continue;
      auto r = resolve_vars(value, tokens);
      if (r.text != value && r.missing.empty()) {
        value = r.text;
        changed = true;
      }
    }
    if (!changed)
      break;
  }
  return tokens;
}

ResolveResult resolve_vars(std::string_view tmpl, const TokenMap &tokens) {
  ResolveResult res;
  std::set<std::string, std::less<>> seen;
  const std::string text(tmpl);
  std::size_t pos = 0;
  while (pos < text.size()) {
    // Copy comments through untouched.
    const std::size_t comment = text.find("/*", pos);
    const std::size_t seg_end =
        comment == std::string::npos ? text.size() : comment;
    const std::string seg = text.substr(pos, seg_end - pos);
    std::size_t last = 0;
    for (auto m = std::sregex_iterator(seg.begin(), seg.end(), var_re());
         m != std::sregex_iterator(); ++m) {
      res.text.append(seg, last, m->position() - last);
      const std::string name = (*m)[1].str();
      if (auto it = tokens.find(name); it != tokens.end()) {
        res.text += it->second;
      } else {
        res.text += kMissingColour;
        if (seen.insert(name).second)
          res.missing.push_back(name);
      }
      last = m->position() + m->length();
    }
    res.text.append(seg, last);
    if (comment == std::string::npos)
      break;
    std::size_t close = text.find("*/", comment + 2);
    close = close == std::string::npos ? text.size() : close + 2;
    res.text.append(text, comment, close - comment);
    pos = close;
  }
  return res;
}

std::optional<double> length_px(std::string_view value) {
  value = trim(value);
  if (value == "0")
    return 0.0;
  if (auto px = number_with_unit(value, "px"))
    return px;
  if (auto rem = number_with_unit(value, "rem"))
    return *rem * kRootFontPx;
  return std::nullopt;
}

std::optional<double> length_em(std::string_view value) {
  value = trim(value);
  if (number_with_unit(value, "rem"))
    return std::nullopt;
  return number_with_unit(value, "em");
}

std::optional<Rgba> parse_color(std::string_view value) {
  value = trim(value);
  if (!value.empty() && value.front() == '#') {
    const std::string_view h = value.substr(1);
    for (char c : h)
      if (hex_digit(c) < 0)
        return std::nullopt;
    Rgba c;
    if (h.size() == 3 || h.size() == 4) {
      c.r = static_cast<std::uint8_t>(hex_digit(h[0]) * 17);
      c.g = static_cast<std::uint8_t>(hex_digit(h[1]) * 17);
      c.b = static_cast<std::uint8_t>(hex_digit(h[2]) * 17);
      if (h.size() == 4)
        c.a = static_cast<std::uint8_t>(hex_digit(h[3]) * 17);
      return c;
    }
    if (h.size() == 6 || h.size() == 8) {
      auto byte = [&](std::size_t i) {
        return static_cast<std::uint8_t>(hex_digit(h[i]) * 16 +
                                         hex_digit(h[i + 1]));
      };
      c.r = byte(0);
      c.g = byte(2);
      c.b = byte(4);
      if (h.size() == 8)
        c.a = byte(6);
      return c;
    }
    return std::nullopt;
  }

  static const std::regex fn(
      R"(rgba?\(\s*([\d.]+)\s*[,\s]\s*([\d.]+)\s*[,\s]\s*([\d.]+)\s*(?:[,/]\s*([\d.]+)(%?)\s*)?\))");
  std::cmatch m;
  const std::string s(value);
  if (!std::regex_match(s.c_str(), m, fn))
    return std::nullopt;
  Rgba c;
  c.r = clamp_byte(to_double(m[1].str()).value_or(0.0));
  c.g = clamp_byte(to_double(m[2].str()).value_or(0.0));
  c.b = clamp_byte(to_double(m[3].str()).value_or(0.0));
  if (m[4].matched) {
    const double a = to_double(m[4].str()).value_or(255.0);
    c.a = clamp_byte(m[5].length() ? a / 100.0 * 255.0
                                   : (a <= 1.0 ? a * 255.0 : a));
  }
  return c;
}

} // namespace rux::qt
