#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Turn a gcovr `--json` report into a per-module coverage summary table.

Companion to scripts/coverage.sh - not meant to be run standalone against
arbitrary input, though it will happily read any gcovr JSON report file.

"Module" means the first path component under libs/reusex/src/ (e.g.
libs/reusex/src/geometry/segment_planes.cpp -> module "geometry"). Header-only
coverage (inline functions, templates) attributes to whichever path the
compiler actually included, which is one of two other layouts for the same
module - the source tree (libs/reusex/include/<module>/Foo.hpp) or the
build tree's mirrored include prefix (<build-dir>/include/reusex/<module>/Foo.hpp,
used so `#include <reusex/...>` resolves) - both are folded into the same
module bucket as their .cpp siblings. Files outside libs/reusex entirely
(apps/, tests/, bindings/) are grouped under their own top-level directory
instead, so nothing silently disappears from the table.

Usage:
    scripts/coverage_report.py gcovr.json
"""

from __future__ import annotations

import json
import sys
from collections import defaultdict
from pathlib import Path

# Anchors that all resolve to "libs/reusex/<module>/...", checked in order.
# The build-tree mirror (.../include/reusex/<module>/...) is deliberately a
# bare "include/reusex/" search (not anchored to a specific build dir name)
# since COVERAGE_BUILD_DIR is configurable.
REUSEX_MODULE_ANCHORS = (
    "libs/reusex/src/",
    "libs/reusex/include/",
    "include/reusex/",
)


def module_for(file_path: str) -> str:
    """Map a gcovr-relative file path to a module label for grouping."""
    posix = file_path.replace("\\", "/")
    for anchor in REUSEX_MODULE_ANCHORS:
        idx = posix.find(anchor)
        if idx == -1:
            continue
        rest = posix[idx + len(anchor):]
        parts = rest.split("/", 1)
        return parts[0] if len(parts) > 1 else "(root)"
    # Anything outside libs/reusex (apps/rux, apps/ruxd, bindings/...) groups
    # by its own top-level directory so it's still visible.
    return posix.split("/", 1)[0] if "/" in posix else posix


def pct(covered: int, total: int) -> float:
    return 100.0 * covered / total if total else 100.0


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"usage: {argv[0]} <gcovr.json>", file=sys.stderr)
        return 2

    data = json.loads(Path(argv[1]).read_text())

    # module -> {lines_covered, lines_total, branches_covered, branches_total, files}
    modules: dict[str, dict[str, int]] = defaultdict(
        lambda: {
            "lines_covered": 0,
            "lines_total": 0,
            "branches_covered": 0,
            "branches_total": 0,
            "files": 0,
        }
    )

    # Defense-in-depth on top of scripts/coverage.sh's gcovr --exclude flags:
    # vendored headers (libs/reusex/extern/) have occasionally slipped through
    # gcovr's own filtering (observed with gcovr 8.4 - the --exclude 'extern/'
    # passed to gcovr did not catch libs/reusex/extern/include/**), so skip
    # them here too rather than let ~5 vendored files skew a module's numbers.
    skip_substrings = ("/extern/", "/tests/", "/bindings/")

    for file_entry in data.get("files", []):
        path = file_entry["file"]
        posix_path = path.replace("\\", "/")
        if any(s in f"/{posix_path}" for s in skip_substrings):
            continue
        mod = module_for(path)
        lines = file_entry.get("lines", [])
        branches = file_entry.get("branches", [])
        lines_total = len(lines)
        # gcovr line entries: {"line_number":.., "count":N, "branches":[...]}
        lines_covered = sum(1 for ln in lines if ln.get("count", 0) > 0)
        branch_total = 0
        branch_covered = 0
        for ln in lines:
            for br in ln.get("branches", []):
                branch_total += 1
                if br.get("count", 0) > 0:
                    branch_covered += 1
        branch_total += len(branches)
        branch_covered += sum(1 for b in branches if b.get("count", 0) > 0)

        m = modules[mod]
        m["lines_covered"] += lines_covered
        m["lines_total"] += lines_total
        m["branches_covered"] += branch_covered
        m["branches_total"] += branch_total
        m["files"] += 1

    if not modules:
        print("no coverage data found in report")
        return 0

    rows = []
    total = {"lines_covered": 0, "lines_total": 0, "branches_covered": 0, "branches_total": 0, "files": 0}
    for mod, m in sorted(modules.items()):
        rows.append(m)
        for k in total:
            total[k] += m[k]

    name_w = max(len(mod) for mod in modules) if modules else len("module")
    name_w = max(name_w, len("MODULE"))

    header = f"{'MODULE':<{name_w}}  {'FILES':>5}  {'LINES':>13}  {'LINE %':>7}  {'BRANCHES':>15}  {'BRANCH %':>8}"
    sep = "-" * len(header)
    print(header)
    print(sep)
    for mod in sorted(modules):
        m = modules[mod]
        line_pct = pct(m["lines_covered"], m["lines_total"])
        branch_pct = pct(m["branches_covered"], m["branches_total"])
        print(
            f"{mod:<{name_w}}  {m['files']:>5}  "
            f"{m['lines_covered']:>5}/{m['lines_total']:<7}  {line_pct:>6.1f}%  "
            f"{m['branches_covered']:>6}/{m['branches_total']:<8}  {branch_pct:>7.1f}%"
        )
    print(sep)
    total_line_pct = pct(total["lines_covered"], total["lines_total"])
    total_branch_pct = pct(total["branches_covered"], total["branches_total"])
    print(
        f"{'TOTAL':<{name_w}}  {total['files']:>5}  "
        f"{total['lines_covered']:>5}/{total['lines_total']:<7}  {total_line_pct:>6.1f}%  "
        f"{total['branches_covered']:>6}/{total['branches_total']:<8}  {total_branch_pct:>7.1f}%"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
