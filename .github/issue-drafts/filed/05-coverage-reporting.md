title: Wire up code coverage reporting
labels: infrastructure, phase-0

## Problem
`ENABLE_COVERAGE` exists in CMakeLists.txt but nothing consumes it. There is
no visibility into what the 300+ tests actually cover, so "add a test" work
cannot be targeted.

## Tasks
- [ ] Coverage build preset (`-DENABLE_COVERAGE=ON` + gcovr/llvm-cov report)
- [ ] `scripts/coverage.sh` producing an HTML + summary report
- [ ] Report coverage in CI once hosted CI is enabled

## Acceptance
One command produces a per-module coverage table.

category=CLI estimate=4h
