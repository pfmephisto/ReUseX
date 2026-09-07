#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Compare two Catch2 v3 benchmark XML reports (baseline vs candidate) and
report the per-benchmark mean/stddev delta (STANDARDS.md §8: a change may
not regress a benchmark mean by more than 5% without justification).

Usage:
    scripts/bench-compare.py baseline.xml candidate.xml
    scripts/bench-compare.py --threshold 10 baseline.xml candidate.xml

Exit status:
    0  no benchmark regressed beyond --threshold percent
    1  at least one benchmark regressed beyond --threshold percent,
       or an input file could not be parsed

Python 3 stdlib only (xml.etree.ElementTree, argparse) - no third-party
dependencies, so this runs anywhere the dev shell's python3 is available.
"""

from __future__ import annotations

import argparse
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass
class Benchmark:
    """A single named benchmark result (mean/stddev in nanoseconds)."""

    key: str
    mean: float
    stddev: float


def load_benchmarks(path: str) -> "dict[str, Benchmark]":
    """Parse a Catch2 v3 XML report and return benchmarks keyed by
    "<test-case name>/<benchmark name>".

    Raises SystemExit with a clear message on malformed/missing XML so the
    caller can surface a clean CLI error rather than a raw traceback.
    """
    try:
        tree = ET.parse(path)
    except FileNotFoundError:
        raise SystemExit(f"error: file not found: {path}")
    except ET.ParseError as exc:
        raise SystemExit(f"error: malformed XML in {path}: {exc}")

    root = tree.getroot()
    if root.tag != "Catch2TestRun":
        raise SystemExit(
            f"error: {path} does not look like a Catch2 XML report "
            f"(root element is <{root.tag}>, expected <Catch2TestRun>)"
        )

    benchmarks: "dict[str, Benchmark]" = {}
    for test_case in root.iter("TestCase"):
        test_name = test_case.get("name", "<unnamed test case>")
        for bench_result in test_case.iter("BenchmarkResults"):
            bench_name = bench_result.get("name", "<unnamed benchmark>")
            mean_el = bench_result.find("mean")
            stddev_el = bench_result.find("standardDeviation")
            if mean_el is None or stddev_el is None:
                raise SystemExit(
                    f"error: {path}: BenchmarkResults '{bench_name}' is "
                    "missing a <mean> or <standardDeviation> child element"
                )
            try:
                mean = float(mean_el.get("value", "nan"))
                stddev = float(stddev_el.get("value", "nan"))
            except ValueError as exc:
                raise SystemExit(
                    f"error: {path}: non-numeric mean/stddev for "
                    f"'{bench_name}': {exc}"
                )

            key = f"{test_name}/{bench_name}"
            benchmarks[key] = Benchmark(key=key, mean=mean, stddev=stddev)

    return benchmarks


def format_ns(value: float) -> str:
    """Format a nanosecond duration with an auto-scaled unit."""
    units = [(1e9, "s"), (1e6, "ms"), (1e3, "us")]
    for scale, unit in units:
        if abs(value) >= scale:
            return f"{value / scale:.3f}{unit}"
    return f"{value:.1f}ns"


def compare(
    baseline: "dict[str, Benchmark]",
    candidate: "dict[str, Benchmark]",
    threshold: float,
) -> "tuple[list[str], bool]":
    """Build the printable table rows and determine whether any benchmark
    regressed beyond `threshold` percent. Returns (rows, has_regression).
    """
    all_keys = sorted(set(baseline) | set(candidate))
    rows: list[str] = []
    has_regression = False

    header = (
        f"{'benchmark':<55} {'baseline mean':>14} {'candidate mean':>14} "
        f"{'delta':>9} {'base stddev':>12} {'cand stddev':>12}  flag"
    )
    rows.append(header)
    rows.append("-" * len(header))

    for key in all_keys:
        base = baseline.get(key)
        cand = candidate.get(key)

        if base is None:
            rows.append(f"{key:<55} {'-':>14} {format_ns(cand.mean):>14} "
                         f"{'-':>9} {'-':>12} {format_ns(cand.stddev):>12}  ADDED")
            continue
        if cand is None:
            rows.append(f"{key:<55} {format_ns(base.mean):>14} {'-':>14} "
                         f"{'-':>9} {format_ns(base.stddev):>12} {'-':>12}  REMOVED")
            continue

        if base.mean == 0:
            delta_str = "n/a"
            flag = "WARN (zero baseline)"
        else:
            delta_pct = (cand.mean - base.mean) / base.mean * 100.0
            delta_str = f"{delta_pct:+.2f}%"
            if delta_pct > threshold:
                flag = "REGRESSION"
                has_regression = True
            elif delta_pct < -threshold:
                flag = "improved"
            else:
                flag = ""

        rows.append(
            f"{key:<55} {format_ns(base.mean):>14} {format_ns(cand.mean):>14} "
            f"{delta_str:>9} {format_ns(base.stddev):>12} {format_ns(cand.stddev):>12}  {flag}"
        )

    return rows, has_regression


def main(argv: "list[str] | None" = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare two Catch2 v3 benchmark XML reports and flag regressions.",
    )
    parser.add_argument("baseline", help="baseline benchmark XML report")
    parser.add_argument("candidate", help="candidate benchmark XML report")
    parser.add_argument(
        "--threshold",
        type=float,
        default=5.0,
        help="regression threshold in percent (default: 5)",
    )
    args = parser.parse_args(argv)

    baseline = load_benchmarks(args.baseline)
    candidate = load_benchmarks(args.candidate)

    if not baseline and not candidate:
        print("warning: no <BenchmarkResults> found in either file", file=sys.stderr)

    rows, has_regression = compare(baseline, candidate, args.threshold)
    print("\n".join(rows))

    if has_regression:
        print(
            f"\nFAIL: one or more benchmarks regressed by more than "
            f"{args.threshold}% (STANDARDS.md §8)",
            file=sys.stderr,
        )
        return 1

    print(f"\nOK: no benchmark regressed by more than {args.threshold}%")
    return 0


if __name__ == "__main__":
    sys.exit(main())
