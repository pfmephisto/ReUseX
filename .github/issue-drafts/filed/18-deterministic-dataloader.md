title: Seedable Dataloader (deterministic shuffling)
labels: testing, phase-3

## Problem
`vision/Dataloader.cpp:109` seeds shuffling from `std::random_device`
unconditionally — identical runs produce different batch orders, so
annotation outputs cannot be compared before/after a change
(docs/STANDARDS.md §6).

## Tasks
- [ ] Add `seed` to the Dataloader options (default fixed, e.g. 42);
      entropy opt-in
- [ ] Thread the option through `rux annotate`
- [ ] Unit test: same seed -> same order; different seed -> different order

## Acceptance
Two `rux annotate` runs with default options produce identical
segmentation_images.

category=Vision estimate=2h
