<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Pipeline Stage Contracts

This document describes what each pipeline stage **consumes** and **produces**
in a ReUseX project database (`*.rux`). It complements the module-boundary rules
in [`STANDARDS.md` §1](STANDARDS.md#1-module-boundaries): STANDARDS §1 governs
*code* dependencies between library modules; this document governs the *data*
dependencies between pipeline stages.

> **This file is a prose mirror, not the source of truth.** The machine-readable
> contract lives in
> [`libs/reusex/include/core/stage_contract.hpp`](../libs/reusex/include/core/stage_contract.hpp)
> (table in `src/core/stage_contract.cpp`), and everything that enforces the
> contract reads *that*: `core::check_stage_inputs()`, every `rux create`
> subcommand, and the refusal path of `pipeline::run_stage()`. The **Consumes**
> and **Produces** rows below are asserted against the table by
> `tests/unit/core/test_stage_contract.cpp`, so this page cannot silently drift
> — but when the two disagree, the table wins and this page is the bug (#246).

Each stage is a `rux` subcommand that reads named point clouds / tables from the
project, does its work, and writes named point clouds / tables back. The names
are stable strings (`"cloud"`, `"planes"`, …) used as keys in `ProjectDB`.

`rux validate --stage <name>` asserts that a stage's **inputs** are present and
mutually consistent before it runs. That is the *same call* every `rux create`
subcommand makes, so a stage that validates cannot then be refused when run.

## Conventions

- **Named clouds** are `ProjectDB` point clouds addressed by name. The canonical
  per-scan clouds — `cloud`, `normals`, `planes`, `rooms`, `instances`,
  `labels` — are **index-aligned**: point *i* is the same physical sample in all
  of them (STANDARDS §3.2). Any stage that filters or reorders one MUST apply
  the same operation to its siblings.
- **Per-plane clouds** `plane_centroids` (`PointXYZ`) and `plane_normals`
  (`Normal`) are indexed by plane label, not by point, so they are shorter than
  the per-point clouds and aligned only with each other.
- Alignment is recorded in the table as a property of the **artifact**
  (`Alignment::per_point` / `per_plane` / `none`), not of the stage, so no stage
  can declare a per-plane cloud index-aligned with a per-point one.
- **Tables** are relational rows keyed by cloud name and id — `sensor_frames`,
  `segmentation_images`, `instances`, `instance_materials`,
  `building_components`, `meshes`.
- **Options struct** is the single library-side parameter object for the stage;
  CLI flags mirror its defaults (STANDARDS §4).
- Several commands can be pointed at a differently-named cloud (`rux create
  instances --semantic-cloud foo`). Those flags are passed to the checker as
  *overrides* of the contract's declared name; the contract itself does not
  change.

## Stages

### `import` (`rux import rtabmap`)

| | |
|---|---|
| Consumes | An external RTABMap scan file (not project data) |
| Produces | `sensor_frames` |
| Options  | import-specific CLI options (depth filters) |
| Checks   | nothing in-project (reads an external scan) |

### `optimize` / `register` (`rux optimize`, `rux register`)

Refine the stored per-frame sensor poses in place (plane-graph optimization or
joint pairwise registration).

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `sensor_frames` (updated poses) |
| Options  | `PlaneGraphOptions` / `JprParams` (`libs/reusex/include/slam/`) |
| Checks   | ≥2 stored frames, ≥2 of them carrying depth (refinement is depth-driven) |

### `clouds` (`rux create clouds`)

Back-project depth frames into a fused point cloud.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `cloud` (`PointXYZRGB`), `normals` (`Normal`) |
| Options  | `ReconstructionParams` (`libs/reusex/include/segmentation/reconstruct.hpp`) |
| Checks   | ≥1 stored sensor frame |

### `annotate` (`rux create annotate`)

Run a semantic segmentation model over the stored frames.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `segmentation_images` |
| Options  | `rux create annotate` CLI options (`-n/--net`, batching, CUDA) |
| Checks   | ≥1 stored sensor frame |

### `project` (`rux create project`)

Project the 2D semantic labels onto the 3D cloud.

| | |
|---|---|
| Consumes | `cloud`, `segmentation_images` |
| Produces | `labels` (`Label`) |
| Options  | `rux create project` CLI options |
| Checks   | `cloud` present; at least one frame carries a segmentation image |

### `planes` (`rux create planes`)

Detect planar surfaces (noise-adaptive region growing).

| | |
|---|---|
| Consumes | `cloud`, `normals` |
| Produces | `planes` (`Label`), `plane_centroids` (`PointXYZ`), `plane_normals` (`Normal`) |
| Options  | `SegmentPlanesOptions` (`libs/reusex/include/segmentation/segment_planes.hpp`) |
| Checks   | `cloud`, `normals` present and index-aligned |

### `rooms` (`rux create rooms`)

Partition the plane graph into rooms (Leiden clustering).

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `rooms` (`Label`) |
| Options  | `SegmentRoomsOptions` (`libs/reusex/include/segmentation/segment_rooms.hpp`) |
| Checks   | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

### `instances` (`rux create instances`)

Separate semantic labels into spatial instances (connected-component clustering).

| | |
|---|---|
| Consumes | `cloud`, and a semantic label cloud — `labels`, or `planes` as a fallback |
| Produces | `instances` (label cloud + `instances` table) |
| Options  | `SegmentInstancesRequest` (`libs/reusex/include/segmentation/segment_instances.hpp`) |
| Checks   | `cloud` present; a semantic label cloud present; the two index-aligned |

`--semantic-cloud <name>` overrides `labels`. An explicit override is an
instruction, not a preference: the fallback is dropped and the named cloud must
exist.

### `mesh` (`rux create mesh`)

Solidify the cell complex into a room-partitioned mesh.

| | |
|---|---|
| Consumes | `cloud`, `normals`, `rooms`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `mesh` (`meshes` table) |
| Options  | `SolidifierOptions` → `Solidifier` / `mesh()` (`libs/reusex/include/reconstruction/`) |
| Checks   | `cloud`/`normals`/`rooms`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

### `texture` (`rux create texture`)

Texture-map the reconstructed mesh from the cloud's colors.

| | |
|---|---|
| Consumes | `mesh`, `cloud`, `sensor_frames` |
| Produces | `textured_mesh` |
| Options  | `TextureQualityParams` (`libs/reusex/include/reconstruction/texture_mesh.hpp`) |
| Checks   | named mesh + named cloud present; ≥1 stored sensor frame |

`--mesh-name` / `--cloud-name` override `mesh` / `cloud`.

### `windows` (`rux create windows`)

Derive window building components from the instances and the mesh.

| | |
|---|---|
| Consumes | `cloud`, `labels`, `instances`, `mesh` |
| Produces | `building_components` |
| Options  | `rux create windows` CLI options (`--mode`, `--offset`, `--alpha`) |
| Checks   | all four present; `cloud`/`labels`/`instances` index-aligned |

`--semantic`, `--instances` and `--mesh` override `labels`, `instances` and
`mesh` respectively.

## Typical pipeline order

```
import → (optimize|register) → clouds → planes → rooms → mesh → texture
                                  ↘ annotate → project → instances → windows
```

Stage order in the table is load-bearing and asserted by a test: an input may
only be produced by a **strictly earlier** stage. That invariant is what lets a
missing prerequisite be turned into a "run these commands in order" hint
automatically, instead of the hand-written hint strings the CLI used to carry.
