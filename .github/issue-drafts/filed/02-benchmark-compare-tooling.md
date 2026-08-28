title: Benchmark comparison tooling (baseline vs candidate delta)
labels: infrastructure, phase-0

## Problem
`scripts/bench.sh` runs the Catch2 benchmark suite and stores XML reports in
`bench-results/`, but comparing two reports is manual. Agents (and humans)
need a one-command answer to "did my change regress performance?"
(docs/STANDARDS.md §8: >5% regression requires justification).

## Tasks
- [ ] `scripts/bench-compare.py <baseline.xml> <candidate.xml>`: per-benchmark
      mean/stddev delta table, exit non-zero if any mean regresses >5%
- [ ] Document the baseline/candidate workflow in `docs/STANDARDS.md`
- [ ] Optional: store a committed baseline for main and check in CI

## Acceptance
`scripts/bench.sh && scripts/bench-compare.py old.xml new.xml` prints a delta
table and fails on regression.

category=CLI estimate=4h
