// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The client's bundled typefaces: Archivo (text), Oswald (display) and
// JetBrains Mono (figures), all SIL OFL-1.1 (LICENSES/OFL-1.1.txt).
//
// Bundled as static TTFs in the Qt resource system rather than taken from the
// host: an offscreen screenshot uses whatever fontconfig finds, and a nix
// build sandbox has no fontconfig at all, so only bundled faces make the
// screenshots deterministic. Static TTFs, not fontsource's woff2 — one of
// those registers under the family "Archivo SemiBold", and QSS
// `font-family: 'Archivo'` then silently falls back.

#include <QStringList>

namespace rux::qt {

/// The families tokens.css names that the bundle must provide.
QStringList required_font_families();

/// Register the bundled fonts with QFontDatabase (once per process) and check
/// that every required family is now known. Each family that is not is
/// logged as an error and returned.
QStringList ensure_bundled_fonts();

} // namespace rux::qt
