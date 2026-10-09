#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Guard the rux GUI token rule: no literal colour, radius, spacing or type size
in component styles — only var(--…).

Why this exists: tokens.css (owned by rux-frontend, where the Claude Design
project writes it; vendored here at apps/rux/qt/theme/tokens.css) is the
single source of truth for every visual value. Components must reference
tokens by name so a design change (or the Qt theme generated from the same
names) is a value swap, not a refactor. A hardcoded colour or size is
invisible to that system and drifts the two surfaces apart.

This script is duplicated verbatim into both the rux-frontend and ReUseX
repos — see find_tokens_css() below for the two layouts it resolves.

What it flags, per the documented rule (README "Design tokens"):
  - colour  : any hex / rgb() / rgba() / hsl() / named colour in any property
  - radius  : a literal length/percent on border-radius*
  - spacing : a literal non-zero length on margin*/padding*/gap/inset/top/…
  - type    : a literal length on font-size
Values that are entirely var(...) / calc(var(...)) / benign keywords are fine.
tokens.css is exempt (it defines the values). base.css is exempt by default (it
legitimately holds a couple of chrome primitives like the scrollbar width).

The native Qt client (apps/rux/qt) follows the same rule:
  .qss      the same property checks as CSS, plus every var(--x) must name a
            token in tokens.css (or a run-time `--qt-*` token Theme paints)
  .cpp/.hpp hex colours in strings, QColor(r, g, b) / QColor("…"),
            Qt::<colour> globals, qRgb(), literal setPixelSize/setPointSize,
            setStyleSheet("…") with a literal (styles belong in app.qss), and
            "--token" names that tokens.css does not define
  A line ending in `// token-lint: allow` is exempt (e.g. the 1px hairline
  divider, or a documented fallback).

Usage:
  python token_lint.py apps/rux/qt --qt               # QSS + Qt C++ under a tree
  python token_lint.py apps/rux/qt/styles/app.qss     # one stylesheet
  python token_lint.py path/to/Foo.module.css         # lint one CSS file
  python token_lint.py src --tsx                      # (rux-frontend) also scan JSX style={{…}}

Exit status is non-zero if any violation is found, so it works in a pre-commit
hook or CI step.
"""
import argparse
import re
import sys
from pathlib import Path

# --- what counts as a literal --------------------------------------------------

HEX = re.compile(r"#[0-9a-fA-F]{3,8}\b")
FUNC_COLOUR = re.compile(r"\b(?:rgba?|hsla?)\s*\(", re.I)
LENGTH = re.compile(r"(?<![\w.-])-?\d*\.?\d+(?:px|rem|em|vh|vw|vmin|vmax|%|ch)\b", re.I)

# Named CSS colours worth catching (whole-word). Not exhaustive — the frequent
# offenders. Keywords that are *not* colours (transparent/currentColor/…) are
# handled by ALLOWED_KEYWORDS below.
NAMED_COLOURS = {
    "white", "black", "red", "green", "blue", "yellow", "orange", "purple",
    "gray", "grey", "silver", "gold", "pink", "cyan", "magenta", "lime",
    "navy", "teal", "maroon", "olive", "aqua", "fuchsia", "coral", "salmon",
    "crimson", "indigo", "violet", "khaki", "beige", "ivory", "azure",
    "darkgray", "darkgrey", "lightgray", "lightgrey", "dimgray", "dimgrey",
    "whitesmoke", "gainsboro", "slategray", "slategrey",
}
ALLOWED_KEYWORDS = {
    "transparent", "currentcolor", "inherit", "initial", "unset", "none",
    "auto", "revert", "revert-layer",
}
# Literal lengths that carry no design meaning and are allowed even in the
# guarded properties (a hairline, a full-bleed, a circle).
ALLOWED_LENGTHS = {"0", "0px", "0rem", "50%", "100%", "1px"}

RADIUS_PROPS = re.compile(r"^border(-[a-z]+)?-radius$|^border-radius$")
# Spacing = the 8-step scale: margins, paddings, gaps. Deliberately NOT
# top/right/bottom/left/inset — those are layout positioning (often negative
# nudges), not values drawn from --space-*.
SPACING_PROPS = re.compile(
    r"^(margin|padding)(-(top|right|bottom|left|inline|block)(-(start|end))?)?$"
    r"|^(row-|column-)?gap$"
)
TYPE_PROPS = re.compile(r"^font-size$")

COMMENT = re.compile(r"/\*.*?\*/", re.S)


def is_only_var_or_keyword(value: str) -> bool:
    """A value made purely of var()/calc()/whitespace/benign keywords is clean."""
    stripped = re.sub(r"\bvar\s*\([^()]*(?:\([^()]*\)[^()]*)*\)", " ", value)
    stripped = re.sub(r"\bcalc\s*\([^()]*(?:\([^()]*\)[^()]*)*\)", " ", stripped)
    tokens = [t for t in re.split(r"[\s,/]+", stripped) if t]
    return all(t.lower() in ALLOWED_KEYWORDS for t in tokens)


def literal_colours(value: str):
    for m in HEX.finditer(value):
        yield m.group(0)
    for m in FUNC_COLOUR.finditer(value):
        yield value[m.start():].split(")", 1)[0] + ")"
    for word in re.findall(r"[a-zA-Z][a-zA-Z-]*", value):
        if word.lower() in NAMED_COLOURS:
            yield word


def literal_lengths(value: str):
    for m in LENGTH.finditer(value):
        if m.group(0).lower() not in ALLOWED_LENGTHS:
            yield m.group(0)


def lint_css(path: Path):
    """Yield (line, prop, snippet, reason) for each violation in a CSS file."""
    text = path.read_text(encoding="utf-8", errors="replace")
    # Blank out comments but keep newline count so line numbers stay right.
    text = COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"), text)

    for m in re.finditer(r"([\w-]+)\s*:\s*([^;{}]+)", text):
        prop, value = m.group(1).lower(), m.group(2).strip()
        if is_only_var_or_keyword(value):
            continue
        line = text.count("\n", 0, m.start()) + 1
        snippet = f"{prop}: {value}"

        for c in literal_colours(value):
            yield line, prop, snippet, f"literal colour {c!r} — use a --color-*/--label-* token"
            break  # one report per declaration is enough
        else:
            if RADIUS_PROPS.match(prop):
                for lit in literal_lengths(value):
                    yield line, prop, snippet, f"literal radius {lit!r} — use a --radius-* token"
                    break
            elif SPACING_PROPS.match(prop):
                for lit in literal_lengths(value):
                    yield line, prop, snippet, f"literal spacing {lit!r} — use a --space-* token"
                    break
            elif TYPE_PROPS.match(prop):
                for lit in literal_lengths(value):
                    yield line, prop, snippet, f"literal font-size {lit!r} — use a --font-size-* token"
                    break


JSX_STYLE = re.compile(r"style\s*=\s*\{\{(.*?)\}\}", re.S)


def lint_tsx(path: Path):
    """Flag hardcoded colours inside JSX inline style={{…}} blocks."""
    text = path.read_text(encoding="utf-8", errors="replace")
    for m in JSX_STYLE.finditer(text):
        block = m.group(1)
        # A `${…}` interpolation is a value computed at runtime (e.g. a per-datum
        # label colour) — it cannot be a static token, so don't flag it.
        if "${" in block:
            continue
        for c in literal_colours(block):
            line = text.count("\n", 0, m.start()) + 1
            yield line, "style", block.strip()[:60], f"literal colour {c!r} in inline style — use a token via CSS Module or var()"
            break


# --- Qt ---------------------------------------------------------------------

def find_tokens_css(start: Path) -> Path | None:
    """tokens.css of the repo that contains @p start (or the cwd). Tried at
    both this repo's vendored layout (apps/rux/qt/theme/tokens.css) and the
    rux-frontend repo layout (src/tokens.css) this script is also duplicated
    verbatim into, since this file is shared between the two repos."""
    for base in [start.resolve(), Path.cwd().resolve()]:
        for d in [base, *base.parents]:
            for rel in ("apps/rux/qt/theme/tokens.css", "src/tokens.css"):
                cand = d / rel
                if cand.is_file():
                    return cand
    return None


def known_tokens(tokens_css: Path | None) -> set[str] | None:
    if tokens_css is None:
        return None
    text = COMMENT.sub("", tokens_css.read_text(encoding="utf-8"))
    return set(re.findall(r"(--[\w-]+)\s*:", text))


def token_families(known: set[str] | None) -> set[str]:
    """`--color`, `--space`, … — what a token name starts with. In C++ a
    string like "--page" is a CLI flag, not a token; only strings in a token
    family are checked."""
    return {"--" + t[2:].split("-", 1)[0] for t in known or ()}


def unknown_token(name: str, known: set[str] | None, cpp: bool = False) -> bool:
    if known is None or name in known or name.startswith("--qt-"):
        return False
    if cpp and "--" + name[2:].split("-", 1)[0] not in token_families(known):
        return False
    return True


VAR = re.compile(r"var\(\s*(--[\w-]+)\s*\)")


def lint_qss(path: Path, known):
    yield from lint_css(path)
    text = COMMENT.sub(lambda m: "\n" * m.group(0).count("\n"),
                       path.read_text(encoding="utf-8", errors="replace"))
    for m in VAR.finditer(text):
        if unknown_token(m.group(1), known):
            line = text.count("\n", 0, m.start()) + 1
            yield line, "var", m.group(0), f"unknown token {m.group(1)!r} — not in tokens.css (it would render magenta)"


QT_COLOUR_NAMES = ("white|black|red|green|blue|yellow|cyan|magenta|gray|"
                   "darkGray|lightGray|darkRed|darkGreen|darkBlue|darkCyan|"
                   "darkMagenta|darkYellow")
CPP_RULES = [
    (re.compile(r'"#[0-9a-fA-F]{3,8}\b'), "hex colour in a string — use theme().color(\"--color-…\")"),
    (re.compile(r"\bQColor\s*\(\s*(?:\d|\")"), "literal QColor — use theme().color(\"--color-…\")"),
    (re.compile(r"\bQt::(?:" + QT_COLOUR_NAMES + r")\b"), "Qt global colour — use a --color-* token"),
    (re.compile(r"\bqRgba?\s*\("), "qRgb literal — use a --color-* token"),
    (re.compile(r"\bset(?:PixelSize|PointSizeF?)\s*\(\s*\d"), "literal font size — use theme().font(…, \"--font-size-…\")"),
    (re.compile(r'\bsetStyleSheet\s*\(\s*(?:QStringLiteral\s*\(\s*)?"'), "inline stylesheet literal — style by objectName/property in app.qss"),
]
CPP_STRING_TOKEN = re.compile(r'"(--[a-z0-9][\w-]*)"')
CPP_COMMENT_LINE = re.compile(r"^\s*(//|\*|/\*)")


def lint_cpp(path: Path, known):
    for n, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        if "token-lint: allow" in line or CPP_COMMENT_LINE.match(line):
            continue
        code = line.split("//", 1)[0] if '"' not in line else line
        for rx, reason in CPP_RULES:
            if rx.search(code):
                yield n, "c++", line.strip()[:80], reason
                break
        for m in CPP_STRING_TOKEN.finditer(code):
            if unknown_token(m.group(1), known, cpp=True):
                yield n, "c++", line.strip()[:80], f"unknown token {m.group(1)!r} — not in tokens.css"


def should_skip(path: Path) -> bool:
    name = path.name.lower()
    return "tokens.css" in name or name == "base.css"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("paths", nargs="+", help="Files or directories to lint")
    p.add_argument("--tsx", action="store_true", help="Also scan .tsx JSX style={{…}} for hardcoded colours")
    p.add_argument("--include-base", action="store_true", help="Do not exempt base.css / tokens.css")
    p.add_argument("--qt", action="store_true", help="In directories, also scan Qt .cpp/.hpp (QSS is always scanned)")
    args = p.parse_args()

    css_files, tsx_files, qss_files, cpp_files = [], [], [], []
    for raw in args.paths:
        root = Path(raw)
        if root.is_dir():
            css_files += sorted(root.rglob("*.css"))
            qss_files += sorted(root.rglob("*.qss"))
            if args.tsx:
                tsx_files += sorted(root.rglob("*.tsx"))
            if args.qt:
                cpp_files += sorted(f for ext in ("*.cpp", "*.hpp") for f in root.rglob(ext))
        elif root.suffix == ".css":
            css_files.append(root)
        elif root.suffix == ".qss":
            qss_files.append(root)
        elif root.suffix in (".cpp", ".hpp"):
            cpp_files.append(root)
        elif root.suffix == ".tsx" and args.tsx:
            tsx_files.append(root)
    known = known_tokens(find_tokens_css(Path(args.paths[0]))) if (qss_files or cpp_files) else None

    if not args.include_base:
        css_files = [f for f in css_files if not should_skip(f)]

    total = 0
    for f in css_files:
        for line, prop, snippet, reason in lint_css(f):
            print(f"{f}:{line}: {reason}\n    {snippet}")
            total += 1
    for f in tsx_files:
        for line, prop, snippet, reason in lint_tsx(f):
            print(f"{f}:{line}: {reason}\n    {snippet}")
            total += 1
    for f in qss_files:
        for line, prop, snippet, reason in lint_qss(f, known):
            print(f"{f}:{line}: {reason}\n    {snippet}")
            total += 1
    for f in cpp_files:
        for line, prop, snippet, reason in lint_cpp(f, known):
            print(f"{f}:{line}: {reason}\n    {snippet}")
            total += 1

    files_n = len(css_files) + len(tsx_files) + len(qss_files) + len(cpp_files)
    if total:
        print(f"\n{total} token violation(s) in {files_n} file(s). "
              f"Replace each literal with the matching var(--…); never edit tokens.css.")
        sys.exit(1)
    print(f"OK — {files_n} file(s) clean, no literal design values.")


if __name__ == "__main__":
    main()
