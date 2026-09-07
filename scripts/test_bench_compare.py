#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Self-tests for scripts/bench-compare.py, runnable with plain `python3`
(no pytest / third-party dependency required):

    python3 scripts/test_bench_compare.py

Exercises bench-compare.py both as a library (import + call compare()) and
as a subprocess (to check exit codes / stdout / stderr the way a CI job
would invoke it), against the synthetic fixtures in scripts/testdata/.
"""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
TESTDATA = SCRIPT_DIR / "testdata"
BENCH_COMPARE = SCRIPT_DIR / "bench-compare.py"

_failures: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    status = "ok" if condition else "FAIL"
    print(f"[{status}] {name}" + (f" - {detail}" if detail and not condition else ""))
    if not condition:
        _failures.append(name)


def load_bench_compare_module():
    spec = importlib.util.spec_from_file_location("bench_compare", BENCH_COMPARE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    # dataclasses needs the module registered in sys.modules while it is
    # being executed (it looks up sys.modules[cls.__module__] internally).
    sys.modules["bench_compare"] = module
    spec.loader.exec_module(module)
    return module


def run_cli(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(BENCH_COMPARE), *args],
        capture_output=True,
        text=True,
    )


def test_regression_exits_nonzero() -> None:
    result = run_cli(
        str(TESTDATA / "baseline.xml"),
        str(TESTDATA / "candidate_regression.xml"),
    )
    check("regression -> exit code 1", result.returncode == 1, f"got {result.returncode}")
    check("regression -> REGRESSION flag printed", "REGRESSION" in result.stdout)
    check(
        "regression -> FAIL message on stderr",
        "FAIL" in result.stderr and "5" in result.stderr,
    )
    check("regression -> ADDED flag for new_op", "ADDED" in result.stdout)
    check("regression -> REMOVED flag for legacy_op", "REMOVED" in result.stdout)
    check("regression -> improved flag for voxel benchmark", "improved" in result.stdout)


def test_improvement_exits_zero() -> None:
    result = run_cli(
        str(TESTDATA / "baseline.xml"),
        str(TESTDATA / "candidate_improved.xml"),
    )
    check("improvement -> exit code 0", result.returncode == 0, f"got {result.returncode}")
    check("improvement -> no REGRESSION flag", "REGRESSION" not in result.stdout)
    check("improvement -> OK message printed", "OK" in result.stdout)


def test_missing_benchmark_handled() -> None:
    # baseline.xml has legacy_op, candidate_improved.xml also has legacy_op with
    # unchanged mean, so use candidate_regression.xml which drops legacy_op and
    # adds new_op -- neither side should crash, both should be reported.
    result = run_cli(
        str(TESTDATA / "baseline.xml"),
        str(TESTDATA / "candidate_regression.xml"),
    )
    check("missing benchmark -> process does not crash", result.returncode in (0, 1))
    check("missing benchmark -> ADDED reported, not an exception", "Traceback" not in result.stderr)
    check("missing benchmark -> REMOVED reported, not an exception", "REMOVED" in result.stdout)


def test_zero_baseline_mean_handled() -> None:
    result = run_cli(
        str(TESTDATA / "baseline_zero.xml"),
        str(TESTDATA / "candidate_zero.xml"),
    )
    check("zero baseline -> does not crash", result.returncode in (0, 1))
    check("zero baseline -> no traceback", "Traceback" not in result.stderr)
    check("zero baseline -> n/a delta reported", "n/a" in result.stdout)


def test_malformed_xml_reports_clear_error() -> None:
    result = run_cli(
        str(TESTDATA / "malformed.xml"),
        str(TESTDATA / "candidate_improved.xml"),
    )
    check("malformed XML -> non-zero exit", result.returncode != 0)
    check("malformed XML -> no Python traceback leaked", "Traceback" not in result.stderr)
    check(
        "malformed XML -> clear 'malformed XML' error message",
        "malformed XML" in result.stderr,
        result.stderr,
    )


def test_missing_file_reports_clear_error() -> None:
    result = run_cli(str(TESTDATA / "does_not_exist.xml"), str(TESTDATA / "baseline.xml"))
    check("missing file -> non-zero exit", result.returncode != 0)
    check("missing file -> clear 'file not found' error message", "file not found" in result.stderr)


def test_custom_threshold_flag() -> None:
    # segment_rooms regresses by ~22% in candidate_regression.xml; with a
    # generous 50% threshold it should no longer count as a regression.
    result = run_cli(
        "--threshold",
        "50",
        str(TESTDATA / "baseline.xml"),
        str(TESTDATA / "candidate_regression.xml"),
    )
    check("high threshold -> exit code 0", result.returncode == 0, f"got {result.returncode}")
    check("high threshold -> no REGRESSION flag", "REGRESSION" not in result.stdout)


def test_library_api_matches_cli() -> None:
    module = load_bench_compare_module()
    baseline = module.load_benchmarks(str(TESTDATA / "baseline.xml"))
    candidate = module.load_benchmarks(str(TESTDATA / "candidate_regression.xml"))
    check("library -> baseline has 4 benchmarks", len(baseline) == 4, f"got {len(baseline)}")
    check("library -> candidate has 4 benchmarks", len(candidate) == 4, f"got {len(candidate)}")
    _, has_regression = module.compare(baseline, candidate, threshold=5.0)
    check("library -> compare() flags regression", has_regression is True)


def main() -> int:
    test_regression_exits_nonzero()
    test_improvement_exits_zero()
    test_missing_benchmark_handled()
    test_zero_baseline_mean_handled()
    test_malformed_xml_reports_clear_error()
    test_missing_file_reports_clear_error()
    test_custom_threshold_flag()
    test_library_api_matches_cli()

    print()
    if _failures:
        print(f"{len(_failures)} check(s) FAILED: {', '.join(_failures)}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
