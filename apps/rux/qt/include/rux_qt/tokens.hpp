// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Design tokens for the native Qt client — the Qt-free half.
//
// tokens.css is the single source of every design value for both surfaces:
// the web GUI (rux-frontend repo) reads it through CSS, this Qt client
// vendors a copy at apps/rux/qt/theme/tokens.css (scripts/sync-tokens.sh) and
// parses it at run time with these functions, substituting the values into a
// QSS template written with the same `var(--x)` syntax as the CSS Modules.
// The repo owns the token *names*; the Claude Design project owns the values.
//
// Nothing here includes Qt, so the parsing rules are unit-tested in the light
// test binary (tests/unit/rux_qt/).

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace rux::qt {

/// Which tokens.css block applies: `:root` alone (light) or `:root`
/// overlaid with `[data-theme='dark']`.
enum class ThemeMode { light, dark };

/// Token name (with the leading `--`) to its normalised value.
using TokenMap = std::map<std::string, std::string, std::less<>>;

/// Root font size used to turn `rem` into pixels, as in the browser.
inline constexpr double kRootFontPx = 16.0;

/// Read the custom properties of @p css for @p mode.
///
/// Only the top-level `:root { }` and `[data-theme='dark'] { }` blocks count;
/// `@media` blocks (reduced motion) and comments are ignored. Every value goes
/// through normalise_value(), and a value that is itself `var(--y)` is
/// replaced by --y's value.
TokenMap parse_tokens_css(std::string_view css, ThemeMode mode);

/// Make one CSS value QSS-compatible: collapse whitespace, `rem` to whole
/// pixels (QSS has no rem), and the CSS4 `rgb(r g b / a%)` colour form to
/// `rgba(r, g, b, a%)`. Anything else passes through.
std::string normalise_value(std::string_view value);

struct ResolveResult {
  std::string text;
  /// Unknown token names, each once, in first-seen order.
  std::vector<std::string> missing;
};

/// The colour substituted for an unknown token: loud on purpose, so a typo or
/// a design-side rename is impossible to miss in a screenshot.
inline constexpr std::string_view kMissingColour = "magenta";

/// Substitute every `var(--x)` in @p qss_template with its value from
/// @p tokens. An unknown name becomes kMissingColour and is reported.
/// `/* … */` comments are copied through unresolved.
ResolveResult resolve_vars(std::string_view qss_template,
                           const TokenMap &tokens);

/// `12px`, `0.75rem` or `0` as pixels; std::nullopt otherwise.
std::optional<double> length_px(std::string_view value);

/// `0.08em` as a multiple of the font size; std::nullopt otherwise.
std::optional<double> length_em(std::string_view value);

struct Rgba {
  std::uint8_t r = 0, g = 0, b = 0, a = 255;
};

/// `#rgb`, `#rrggbb`, `#rrggbbaa`, `rgb()/rgba()` (comma or CSS4 slash form,
/// alpha as 0–1 or a percentage). std::nullopt for anything else.
std::optional<Rgba> parse_color(std::string_view value);

} // namespace rux::qt
