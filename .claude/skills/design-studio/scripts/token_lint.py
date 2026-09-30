#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""Guard the rux GUI token rule: no literal colour, radius, spacing or type size
in component styles — only var(--…).

Why this exists: `apps/rux/frontend/src/tokens.css` is owned by the Claude Design
project and is the single source of truth for every visual value. Components must
reference tokens by name so a design change (or the future Qt theme generated from
the same names) is a value swap, not a refactor. A hardcoded colour or size is
invisible to that system and drifts the two surfaces apart.

What it flags, per the documented rule (README "Design tokens"):
  - colour  : any hex / rgb() / rgba() / hsl() / named colour in any property
  - radius  : a literal length/percent on border-radius*
  - spacing : a literal non-zero length on margin*/padding*/gap/inset/top/…
  - type    : a literal length on font-size
Values that are entirely var(...) / calc(var(...)) / benign keywords are fine.
tokens.css is exempt (it defines the values). base.css is exempt by default (it
legitimately holds a couple of chrome primitives like the scrollbar width).

Usage:
  python token_lint.py apps/rux/frontend/src          # lint a tree
  python token_lint.py path/to/Foo.module.css         # lint one file
  python token_lint.py apps/rux/frontend/src --tsx    # also scan JSX style={{…}}

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


def should_skip(path: Path) -> bool:
    name = path.name.lower()
    return "tokens.css" in name or name == "base.css"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("paths", nargs="+", help="Files or directories to lint")
    p.add_argument("--tsx", action="store_true", help="Also scan .tsx JSX style={{…}} for hardcoded colours")
    p.add_argument("--include-base", action="store_true", help="Do not exempt base.css / tokens.css")
    args = p.parse_args()

    css_files, tsx_files = [], []
    for raw in args.paths:
        root = Path(raw)
        if root.is_dir():
            css_files += sorted(root.rglob("*.css"))
            if args.tsx:
                tsx_files += sorted(root.rglob("*.tsx"))
        elif root.suffix == ".css":
            css_files.append(root)
        elif root.suffix == ".tsx" and args.tsx:
            tsx_files.append(root)

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

    files_n = len(css_files) + len(tsx_files)
    if total:
        print(f"\n{total} token violation(s) in {files_n} file(s). "
              f"Replace each literal with the matching var(--…); never edit tokens.css.")
        sys.exit(1)
    print(f"OK — {files_n} file(s) clean, no literal design values.")


if __name__ == "__main__":
    main()
