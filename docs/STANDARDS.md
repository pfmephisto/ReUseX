<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Engineering Standards

This document defines the objective bar every change must meet — whether written
by a human or an AI agent. It complements `CLAUDE.md` (naming conventions, build
instructions, TODO format) and is referenced by CI checks and code review.

Rules marked **[target]** describe the intended state; existing code may not yet
comply. New code must comply; touched code should be migrated opportunistically.

---

## 1. Module boundaries

The library (`libs/reusex/`) is layered. A module may only depend on modules
in lower layers:

```
Layer 4:  visualize                      (optional, PCL/Qt/VTK)
Layer 3:  geometry   io   vision         (peers — MUST NOT include each other) [target]
Layer 2:  core                           (ProjectDB, logging, materials, stages)
Layer 1:  utils, types.hpp               (no internal dependencies)
External: apps/rux                       (may use everything; keeps logic thin)
```

**Rules:**

- `core/` MUST NOT include PCL visualization, CGAL, RTABMap, or ML headers.
  **[target]** — `processing_observer.hpp` currently pulls PCL into core.
- `geometry/`, `io/`, and `vision/` are peers. Cross-peer data exchange goes
  through types defined in `core/` or `types.hpp`, not through peer headers.
  **[target]** — `io/export_scene.hpp` currently includes
  `geometry/BuildingComponent.hpp`.
- `apps/rux/` subcommands are thin wrappers: parse arguments, validate, call
  one library entry point, report. Business logic lives in the library.
- New heavyweight dependencies (anything that adds > 30 s to a clean build)
  require an issue discussing alternatives before being added.

## 2. Header hygiene

- Public headers (`libs/reusex/include/**`) include the *minimum* needed.
  Prefer forward declarations; use the Pimpl idiom to keep sqlite3, RTABMap,
  TensorRT, and CGAL out of public headers (as `ProjectDB` and `Solidifier`
  already do).
- Do not add new includes of PCL, CGAL, Boost.Graph, or OpenCV to a public
  header if a forward declaration or a `.cpp`-local include suffices.
- Public headers include siblings via the `reusex/` prefix
  (e.g. `#include "reusex/core/logging.hpp"`).
- `#pragma once` everywhere; SPDX header on every file.

## 3. Label & identity contract

This is the single source of truth for label semantics. Any code that reads or
writes labels MUST follow it; any deviation is a bug.

### 3.1 Encodings per layer

| Layer                                   | Type       | Unlabeled/background | Valid labels |
|-----------------------------------------|------------|----------------------|--------------|
| `segmentation_images` storage (PNG)     | `CV_16U`   | `0` (stored as label+1) | `1..65535` (i.e. label ≤ 65534) |
| `ProjectDB` segmentation API            | `CV_32S`   | `-1`                 | `0..N`       |
| In-memory point labels (`pcl::Label`)   | `uint32_t` | `0` **[target]**     | `1..N`       |

**Rules:**

- `0` means "unlabeled" in every `CloudL` point cloud (labels, planes, rooms,
  instances). Valid labels start at `1`. **[target]** — room segmentation
  currently initializes to `-1` (wraps to `0xFFFFFFFF`).
- Never index a vector with `label - 1` without first checking `label >= 1`.
- Conversions between the layers above happen ONLY through dedicated helpers
  **[target]** (planned: `core/label_semantics.hpp`), never inline casts.
- Writing a label ≥ 65535 to segmentation storage MUST throw, not wrap.

### 3.2 Identity & referential integrity

- **Stable identity**: entities that outlive a single pipeline run (material
  passports, building components, instances) are identified by GUID, never by
  sequential index alone. **[target]** — instance IDs are currently
  regenerated sequentially on every `segment_instances` run.
- **No silent orphaning**: an operation that invalidates an association
  (regenerating instances, downsampling a cloud with derived label clouds)
  MUST either migrate the association or fail/warn loudly. Silent breakage of
  `instance_materials` links is a bug.
- **Parallel clouds move together**: "cloud", "normals", "labels", "planes",
  "rooms", "instances" for the same scan are index-aligned. Any filter,
  downsample, or reorder MUST be applied to all of them (or invalidate the
  derived ones explicitly).

## 4. Parameters

- Every tunable parameter lives in exactly one place: an options struct in the
  library header (e.g. `SegmentPlanesOptions`). CLI flags mirror the library
  defaults — never define a second default in `apps/rux`. **[target]**
- No new magic numbers in algorithm code. Thresholds, epsilons, and limits go
  into the options struct or a named `constexpr`.
- Geometric epsilon policy **[target]**: use the named constants in
  `utils/` (planned) rather than ad-hoc `1e-6`/`1e-9`/`1e-15` literals.
  Absolute tolerances must state their unit (meters) in the name or comment.

## 5. Error handling & diagnostics

- **No silent failure.** A pipeline stage that produces empty/degenerate output
  (0 planes, 0 cells, infeasible MIP, empty mesh) MUST log at `warn` or above
  *with the reason and the relevant quantities* (e.g. "MIP infeasible:
  412 cells, 87 planes — consider stronger plane merging").
- Long-running external calls (MIP solve, Leiden clustering) MUST have a
  configurable timeout/iteration limit. Unbounded loops are bugs.
- Validate inputs at library entry points: sizes match, clouds non-empty,
  required property maps present. Fail fast with `std::runtime_error` and a
  message that names the missing/mismatched thing.
- `assert()` is for programmer invariants only — never the sole guard against
  bad data (it vanishes in Release builds).

## 6. Determinism & reproducibility

- Algorithms with randomness (RANSAC, Leiden, shuffling) MUST accept a seed in
  their options struct. Default seed is fixed (`42`); pass entropy explicitly
  when non-determinism is desired. **[target]** — `Dataloader` currently uses
  `std::random_device` unconditionally.
- Given the same inputs, seeds, and thread count, a pipeline stage must
  produce identical output. Parallel reductions that break this need a
  deterministic mode.
- This is what makes before/after comparison — and therefore agent-driven
  optimization — possible. Treat regressions in determinism like functional
  regressions.

## 7. Testing

- **Unit tests** (Catch2, `tests/unit/<module>/`): every new public function
  in `geometry/`, `io/`, `core/` gets at least one test with a real (possibly
  synthetic) input, including one degenerate case (empty cloud, single point,
  all-collinear, etc.).
- **Integration tests** (`tests/integration/`): changes to pipeline stages
  must keep the end-to-end pipeline test green.
- Tests use fixed seeds and `WithinAbs`/`WithinRel` matchers for floats.
- Bug fixes come with a regression test that fails before the fix.

## 8. Performance

- Changes to hot paths (reconstruction, segmentation, cell complex, MIP,
  inference) must run the benchmark suite before and after; report the delta
  in the PR. **[target]** — benchmark harness in progress.
- A change may not regress a benchmark by more than 5 % without an explicit
  justification in the PR description.
- Prefer measuring to guessing: use `reusex::core::stopwatch` for coarse
  timing; profile before optimizing.

## 9. Definition of Done

A change is done when ALL of the following hold:

1. Builds cleanly (`cmake --build build`) with no new warnings.
2. `ctest` passes, including integration tests.
3. New/changed behavior is covered by tests (§7).
4. Benchmarks are neutral or improved, or the regression is justified (§8).
5. `clang-format` clean; naming follows `CLAUDE.md`.
6. No new violations of the module boundaries (§1) or label contract (§3).
7. SPDX header present on new files; TODO comments follow the tdg format
   (see `CLAUDE.md`).
8. Pipeline-stage failures introduced or touched by the change are loud (§5).

## 10. Process notes for AI agents

- Work from a GitHub issue; keep the change scoped to it. Split unrelated
  fixes into separate PRs.
- If a rule here conflicts with what existing code does, the rule wins for new
  code — but do not mass-migrate old code in the same PR unless the issue asks
  for it. File a follow-up issue instead.
- When a rule marked **[target]** blocks you (e.g. the label helpers don't
  exist yet), implement the smallest slice of the target that unblocks you,
  or flag it in the PR.
