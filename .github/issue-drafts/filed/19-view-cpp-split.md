title: Split view.cpp monolith into focused components
labels: refactor, phase-3

## Problem
`apps/rux/src/view.cpp` (~58 KB) mixes viewer setup, label overlay rendering,
panorama state, and keyboard/mouse handling; `processing_observer.cpp`
(~30 KB) mixes viewport math with rendering callbacks. Too large for safe
concurrent agent edits and impossible to unit test.

## Tasks
- [ ] Extract ViewerState, LabelRenderer, PanoramaHandler, input dispatch
      into separate files under `apps/rux/src/view/`
- [ ] Move reusable viewport-partitioning math into the visualization library
      target with unit tests
- [ ] No behavior change — pure decomposition

## Acceptance
No file in apps/rux exceeds ~15 KB; viewport math has unit tests.

category=Visualization estimate=2d
