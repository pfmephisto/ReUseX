// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The GUI's two typefaces, bundled rather than fetched: `rux gui` is
 * local-first and must render the same on an offline laptop on site.
 *
 * Oswald (display) carries headings and large figures; Archivo (text) is the
 * body face. Both are SIL OFL-1.1. Only the weights the design uses are
 * imported, so the bundle does not carry unused faces. `tokens.css` names the
 * families in `--font-display` and `--font-sans`.
 */
import '@fontsource/oswald/500.css';
import '@fontsource/oswald/600.css';
import '@fontsource/oswald/700.css';
import '@fontsource/archivo/400.css';
import '@fontsource/archivo/400-italic.css';
import '@fontsource/archivo/500.css';
import '@fontsource/archivo/600.css';
import '@fontsource/archivo/700.css';
