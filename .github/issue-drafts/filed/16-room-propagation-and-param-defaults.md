title: Room-label propagation fallback + single source of parameter defaults
labels: robustness, phase-2

## Problem
Two smaller robustness holes:
1. `segment_rooms.cpp:173-181` propagates room labels via unchecked 1-NN — a
   single misclassified seed point smears across whole regions, and a failed
   `nearestKSearch` silently leaves the sentinel label.
2. CLI defaults diverge from library defaults for the same parameters (e.g.
   angle 15° in `create/mesh.cpp` vs 25° in the library), so tuning results
   are misleading.

## Tasks
- [ ] Distance-bounded, k>1 majority propagation with an explicit "unknown"
      outcome; log the count of unpropagated points
- [ ] Leiden `max_iter` default made finite; expose in CLI
- [ ] CLI options take defaults *from* the library options structs
      (`->default_val(opt.field)` initialized from the struct), never
      re-declared literals — per docs/STANDARDS.md §4
- [ ] Audit all `rux create`/`segment` flags for divergence

## Acceptance
Library and CLI report identical defaults in `--help`; propagation failures
are visible in logs.

category=Geometry estimate=1d
