<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Pipeline Stage Contracts

This document is the single source of truth for what each pipeline stage
**consumes** and **produces** in a ReUseX project database (`*.rux`). It
complements the module-boundary rules in [`STANDARDS.md` §1](STANDARDS.md#1-module-boundaries):
STANDARDS §1 governs *code* dependencies between library modules; this document
governs the *data* dependencies between pipeline stages.

Each stage is a `rux` subcommand that reads named point clouds / tables from the
project, does its work, and writes named point clouds / tables back. The names
are stable strings (`"cloud"`, `"planes"`, …) used as keys in `ProjectDB`.

`rux validate --stage <name>` asserts that a stage's **inputs** are present and
mutually consistent before it runs (implemented in
`libs/reusex/src/core/validate.cpp`, `check_stage_inputs`). This catches a
missing or size-mismatched prerequisite loudly instead of crashing deep inside
an algorithm.

## Conventions

- **Named clouds** are `ProjectDB` point clouds addressed by name. The canonical
  per-scan clouds — `cloud`, `normals`, `planes`, `rooms`, `instances`,
  `labels` — are **index-aligned**: point *i* is the same physical sample in all
  of them (STANDARDS §3.2). Any stage that filters or reorders one MUST apply
  the same operation to its siblings.
- **Per-plane clouds** `plane_centroids` (`PointXYZ`) and `plane_normals`
  (`Normal`) are indexed by plane label, not by point, so they are shorter than
  the per-point clouds and aligned only with each other.
- **Tables** are relational rows keyed by cloud name and id — `sensor_frames`,
  `instances`, `instance_materials`, `building_components`, `meshes`.
- **Options struct** is the single library-side parameter object for the stage;
  CLI flags mirror its defaults (STANDARDS §4).

## Stages

### `import` (`rux import rtabmap`)

| | |
|---|---|
| Consumes | An external RTABMap `.db` (not project data) |
| Produces | `sensor_frames` table (color / depth / confidence / pose / intrinsics) |
| Options  | import-specific CLI options (depth filters) |
| `--stage import` checks | nothing in-project (reads an external scan) |

### `optimize` / `register` (`rux optimize`, `rux register`)

Refine the stored per-frame sensor poses in place (plane-graph optimization or
joint pairwise registration).

| | |
|---|---|
| Consumes | `sensor_frames` (poses + depth for surfel extraction) |
| Produces | updated poses in `sensor_frames` |
| Options  | `PlaneGraphOptions` / `JprParams` (`libs/reusex/include/slam/`) |
| `--stage optimize` checks | ≥1 stored sensor frame |

### `clouds` (`rux create clouds`)

Back-project depth frames into a fused point cloud.

| | |
|---|---|
| Consumes | `sensor_frames` |
| Produces | `cloud` (`PointXYZRGB`), `normals` (`Normal`) |
| Options  | `ReconstructParams` (`libs/reusex/include/segmentation/reconstruct.hpp`) |
| `--stage clouds` checks | ≥1 stored sensor frame |

### `planes` (`rux create planes`)

Detect planar surfaces (noise-adaptive region growing).

| | |
|---|---|
| Consumes | `cloud`, `normals` |
| Produces | `planes` (`Label`), `plane_centroids` (`PointXYZ`), `plane_normals` (`Normal`) |
| Options  | `SegmentPlanesOptions` (`libs/reusex/include/segmentation/segment_planes.hpp`) |
| `--stage planes` checks | `cloud`, `normals` present and index-aligned |

### `rooms` (`rux create rooms`)

Partition the plane graph into rooms (Leiden clustering).

| | |
|---|---|
| Consumes | `cloud`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `rooms` (`Label`) |
| Options  | `SegmentRoomsOptions` (`libs/reusex/include/segmentation/segment_rooms.hpp`) |
| `--stage rooms` checks | `cloud`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

### `instances` (`rux create instances`)

Separate semantic labels into spatial instances (connected-component clustering).

| | |
|---|---|
| Consumes | `cloud`, a semantic label cloud (default `labels`) |
| Produces | instance-label cloud (default `instances`) + `instances` table |
| Options  | `SegmentInstancesOptions` (`libs/reusex/include/segmentation/segment_instances.hpp`) |
| `--stage instances` checks | `cloud` present; a `labels` (or `planes`) label cloud present; `cloud`/`labels` aligned |

### `mesh` (`rux create mesh`)

Solidify the cell complex into a room-partitioned mesh.

| | |
|---|---|
| Consumes | `cloud`, `normals`, `rooms`, `planes`, `plane_centroids`, `plane_normals` |
| Produces | `mesh` (`meshes` table) |
| Options  | mesh CLI options → `Solidifier` / `mesh()` (`libs/reusex/include/reconstruction/`) |
| `--stage mesh` checks | `cloud`/`normals`/`rooms`/`planes` present + aligned; `plane_centroids`/`plane_normals` present + aligned |

## Typical pipeline order

```
import → (optimize|register) → clouds → planes → rooms → instances → mesh
                                                        ↘ (annotate feeds labels)
```

`annotate` (ML semantic labelling) populates `segmentation_images` and, via
label projection, the `labels` cloud consumed by `instances`; it is orthogonal
to the geometric plane/room pipeline and is not gated by `--stage`.
