# The native Qt client — design loop reference

The second surface of the one design system. Status: **in progress** (Stream
Q of `docs/superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md`).
Q0 — this loop, the theme loader, the gallery and two demo pages — is done;
the app shell (Q1), the Database workspace (Q2) and 3D / pose graph /
Pipeline (Q3) build on it.

## 1. Where things live

```
apps/rux/qt/
├── CMakeLists.txt            three targets (below)
├── include/rux_qt/
│   ├── tokens.hpp            Qt-free: parse tokens.css, resolve var(--x)
│   ├── gallery_args.hpp      Qt-free: the gallery's command line
│   ├── Theme.hpp             tokens -> QSS + QPalette + fonts, hot reload
│   ├── fonts.hpp             bundled Archivo / Oswald / JetBrains Mono
│   ├── widgets.hpp           CapsLabel, Pill, Panel, StatCard, NavRail,
│   │                         NavItem, LabelLegend, Swatch, PropertyList
│   ├── ViewportView.hpp      3D pane: EGL snapshot or QVTKOpenGLNativeWidget
│   └── pages.hpp             page registry (name -> factory(PageContext))
├── src/  src/core/           implementations (core/ = the Qt-free half)
├── styles/app.qss            THE stylesheet template — var(--token) only
├── resources/rux_qt.qrc      release snapshot: tokens.css, app.qss, fonts
├── resources/fonts/*.ttf     OFL-1.1 (LICENSES/OFL-1.1.txt, REUSE.toml)
└── gallery/                  rux-qt-gallery: main.cpp + demo pages
```

| Target | Links | Why separate |
|---|---|---|
| `rux_qt_core` | std only | tokens.css parsing + gallery args, tested in the LIGHT unit binary (`tests/unit/rux_qt/`) |
| `rux_qt_lib` | Qt6 Widgets/OpenGLWidgets, VTK GUISupportQt, `reusex_core`, `reusex_visualize` (private) | the Qt layer; never the `reusex` umbrella, so no libtorch/TensorRT — the gallery links in seconds |
| `rux-qt-gallery` | `rux_qt_lib` | renders any page to PNG headless |

AUTOMOC/AUTORCC are set **per target** (they are off globally). Public headers
with `Q_OBJECT` are listed as sources, because AUTOMOC only looks for headers
next to a `.cpp`. Sources are globbed (`src/*.cpp`, `gallery/*.cpp`).

## 2. The loop

```bash
# Every page x both themes, 1440x900 at 2x, into shots/qt/
bash <skill-dir>/scripts/qt_shot.sh --all
# One page, one theme
bash <skill-dir>/scripts/qt_shot.sh --page components --theme dark
# The real interactive VTK widget, under Xvfb
bash <skill-dir>/scripts/qt_shot.sh --page viewport --gl
```

Then **Read every PNG** and critique (checklist §Qt). Measured on this box
(2026-10-08):

| Edit | What happens | Time |
|---|---|---|
| `app.qss` or `tokens.css` | no build (the gallery reads both live with `--dev`); one page x one theme | **~1.3 s** |
| same, all pages x both themes | 4 shots | ~6.7 s |
| a `.cpp` in apps/rux/qt, from inside `nix develop` | incremental build (one TU + link) + shot | **~10 s** |
| same, from outside the dev shell | + `nix develop` start-up | ~22 s |

`qt_shot.sh` only builds when an `apps/rux/qt` C++/CMake/qrc/ttf file is newer
than the binary, so a pure style iteration never pays for the dev shell. For
live tweaking on a display: `build/apps/rux/qt/rux-qt-gallery --dev --page
components` — `QFileSystemWatcher` re-applies the stylesheet on save (editors
that replace the file are re-watched) and the gallery rebuilds the page, so
values read in code refresh too.

**Fixture:** by default `qt_shot.sh` prepares a copy of
`tests/fixtures/scans/office_corridor.rux` once (`rux create clouds -g 0.02
--sampling-factor 2` + `create planes`; the tracked fixture has no cloud) in
`~/.cache/reusex/qt-shot/`, keyed on the fixture's hash, and copies it again
for every shot. It never opens the tracked file. `--project` takes your own
copy. Needs `build/apps/rux/rux` for the one-time preparation; without it the
3D page shows its empty state.

**Failure is loud:** an unknown token renders **magenta**, is logged as
`MISSING token --x`, makes the gallery exit 3 and `qt_shot.sh` fail. Other
exit codes: 2 bad flags, 4 project/page not found, 5 PNG not written.
`RUX_QT_DUMP_QSS=<file>` writes the resolved stylesheet for debugging.

## 3. Token mapping

`rux::qt::Theme` parses `:root` (light) and overlays `[data-theme='dark']`
(dark); `@media` blocks are ignored. Values are normalised for QSS:

| CSS | Qt |
|---|---|
| `0.75rem` | `12px` (16 px root, rounded to whole px — QSS font sizes are ints) |
| `rgb(29 45 61 / 10%)` | `rgba(29, 45, 61, 10%)` |
| multi-line font stack | one line; QSS `font-family` lists fall back correctly |
| `var(--x)` inside a token | resolved |
| `var(--qt-icon-check)` / `--qt-icon-chevron` | **generated at run time**: glyph PNGs painted in `--color-on-accent` / `--color-text-muted` (no QtSvg in the shell to tint an SVG) |

In C++ never write a value; ask the theme: `theme().color("--color-canvas")`,
`theme().px("--space-4")`, `theme().em("--tracking-caps")`,
`theme().font(FontRole::display, "--font-size-3xl", "--font-weight-bold")`.
Every lookup of an unknown name is recorded as missing, exactly like the QSS.

**QPalette** (for what Fusion draws and QSS does not reach — menus, tooltips,
focus, selection): Window=`surface`, Base/Button/Light=`surface-raised`,
AlternateBase=`surface-sunken`, Text/WindowText/ButtonText=`text`,
PlaceholderText + all Disabled text=`text-faint`, Highlight=`accent-muted`
(+ HighlightedText=`text`, as the web's selected row), Accent=`accent`,
Link=`accent-deep`, Midlight=`border`, Mid/Dark=`border-strong`,
Shadow=`scrim`, ToolTipBase=`surface-overlay`. Style is always Fusion so no
platform theme leaks into a shot.

## 4. What QSS cannot do — and the substitute

| CSS feature | Substitute |
|---|---|
| `text-transform: uppercase`, `letter-spacing` | `CapsLabel` paints with `QFont::AllUppercase` + absolute spacing = `--tracking-*` em x pixel size. Give it a QSS `role` for colour/size. |
| `box-shadow` (`--shadow-*`) | not used: elevation is a 1 px `--color-border` + a raised surface. `QGraphicsDropShadowEffect` exists but is slow and blurs children — avoid. |
| transitions, easing (`--duration-*`) | `QPropertyAnimation` with the token duration (none yet). |
| `calc()`, `color-mix()`, custom properties | compute in code from tokens. |
| per-instance colour (a legend swatch) | `Swatch` paints `theme().color(token)`; a per-widget `setStyleSheet` is forbidden. |
| descendant selectors on a parent's state (`:checked QLabel`) | set a dynamic property on the child (`active`) and call `repolish(child)`. |
| line-height | layout spacing (`--space-*`) instead. |
| `:focus-visible` | `:focus` — QSS has no keyboard-only focus; keep the focus border subtle. |

Also: QSS `font-size` 0 or a missing family silently gives you a default. If a
shot shows microscopic or wrong-face text, dump the QSS (`RUX_QT_DUMP_QSS`).

## 5. Gotchas (recorded the hard way)

- **`&` is a mnemonic** in `QPushButton`/`QAction`/`QCheckBox` text: "Miljø &
  prøver" renders "Miljø _prøver". Pass button labels through
  `NavItem::escape_mnemonic()` (`&` -> `&&`). A plain `QLabel` is safe.
- **Locale**: `QApplication` calls `setlocale(LC_ALL, "")`; under `da_DK`
  `std::stod("0.75")` is 0. Parse numbers with `std::from_chars` (the token
  parser does; a unit test pins it). Found as "every font size is 0".
- **QVTKOpenGLNativeWidget is blank under `QT_QPA_PLATFORM=offscreen`** (no GL
  context) and `minimalegl` core-dumps. Screenshot mode therefore renders 3D
  through `reusex::visualize::render_view()` — VTK's EGL offscreen window —
  into the pane; `--gl` runs the real widget under `xvfb-run` (whose exit code
  is 1 on success: judge by the PNG). The two modes frame the scene
  differently until Q3 extracts `populate_scene()`.
- **Static library resources**: `Q_INIT_RESOURCE(rux_qt)` must run (it does, in
  `ensure_bundled_fonts()`), or the linker drops the fonts and the snapshot.
- **Fonts**: bundle static TTFs. fontsource's woff2 registers Archivo 400 as
  the family "Archivo SemiBold", and nix sandboxes have no fontconfig at all.
  `ensure_bundled_fonts()` checks Archivo, Oswald and JetBrains Mono are
  registered and logs an error per missing family.
- **qrc is XML**: no `--` inside a comment.
- **Numbers** are formatted Danish (`format_count` -> `39.723`), mono
  (`JetBrains Mono`), right-aligned — same as the web's `da-DK` figures.
- Installed binaries need Qt's platform plugins: `default.nix` uses
  `dontWrapQtApps = true`; the launched app (Q1) must be wrapped.

## 6. Building a page

1. A factory `QWidget *make_x(const PageContext &)`, registered with
   `register_page({"x", "one line", make_x})` — explicit, not a static
   registrar (the static lib would drop it).
2. Compose the shared widgets; give anything QSS must style an `objectName`
   or a `kind`/`tone`/`role` property; put its rules in `styles/app.qss`.
3. Real data from `ctx.db` (a project copy) and a designed empty state when it
   is `nullptr` or the data is missing ("Ingen punktsky endnu — kør rux create
   clouds"). Danish copy.
4. `python <skill-dir>/scripts/token_lint.py apps/rux/qt --qt` must pass.
5. `qt_shot.sh --page x` in both themes; Read the PNGs; iterate.
