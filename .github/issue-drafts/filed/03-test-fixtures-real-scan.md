title: Add small real-scan test fixture (git-lfs)
labels: testing, phase-0

## Problem
`tests/fixtures/` contains only a README. Unit and integration tests generate
synthetic data; nothing exercises the pipeline against real sensor noise,
real RTABMap poses, or real segmentation output. Bugs that only manifest on
real data (label projection artifacts, depth filter behavior) are invisible.

## Tasks
- [ ] Trim a real scan to a minimal fixture: a few sensor frames
      (color/depth/confidence/pose/intrinsics) + expected outputs
- [ ] Store via git-lfs (hooks already configured in flake.nix)
- [ ] Add an integration test: fixture frames -> reconstruct -> plane count /
      cloud size within expected bounds

## Acceptance
`ctest -R fixture` runs against real data in < 60 s.

category=I/O estimate=1d
