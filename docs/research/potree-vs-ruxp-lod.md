<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Potree-Next vs RUXP for LOD point-cloud rendering — evaluation

**Issue #321.** Three questions to answer: (1) feasibility of exporting `.rux`
clouds to the Potree octree format, (2) serving-model fit with `ruxd`, (3)
recommendation vs the #320 approach (voxel `max_points`).

Research date: 2026-09-21. PotreeConverter 2.1.3, potree/potree 1.8.2,
tentone/potree-core 2.0.15, pnext/three-loader 0.2.5.

> **Terminology note.** "Potree-Next" in the issue title conflates two distinct
> projects. This doc evaluates both and clarifies the distinction (§2.3).

---

## 1. Current state

**What we have — `rux gui` + RUXP (#283 + #320, both closed/merged)**

The `rux gui` subcommand runs a Crow HTTP server that serves:

- `/api/v1/clouds/{name}/points` — the RUXP v1 binary endpoint, with:
  - `offset`/`limit` for sequential paging (default page: 100 000 pts)
  - `max_points=N` — a **voxel LOD** page of the whole cloud (#320, merged):
    2-pass dyadic voxel grid, streaming pass over SQLite blobs, O(N) time per
    request, O(budget) memory. Sets the `LOD` flag bit in the RUXP header.
  - `lod_source=name` — for label clouds to use the geometry cloud's voxel
    selection, keeping the two index-aligned.

Client logic (`useCloudStream.ts`):

1. Fire `max_points=200 000` first — puts the whole scan on screen in one round
   trip (the "overview" layer).
2. Then page the cloud sequentially at 100 000 pts/page into a full-resolution
   layer.
3. Drop the overview layer when paging catches up.

Wire format: planar binary (SoA), zero-copy into `THREE.BufferAttribute`. ~15 MB
per 1 M points (positions + RGB as u8).

**What is NOT yet there:**

- Multi-level LOD (more than one overview + full).
- Frustum-culled tile fetching (only fetch what the camera sees).
- Morton / space-filling-curve storage order (needed for prefix = spatial LOD).
- Progressive refinement as the user zooms in.

---

## 2. Potree 2.0 format

### 2.1 Wire format — three files total

Potree 2.0 (PotreeConverter 2.x) produces exactly three files:

| File | Role |
|---|---|
| `metadata.json` | Bounding box, scale, spacing, attribute schema, hierarchy parameters |
| `octree.bin` | All point data concatenated; nodes fetched via HTTP `Range:` requests |
| `hierarchy.bin` | Node tree: byte offset + byte size per node in `octree.bin` |

This replaced the v1.x format's thousands-to-millions of per-node `.bin` files.
The browser fetches only the nodes that intersect the frustum at the current LOD
level, using `Range: bytes=start-end` HTTP requests.

Example `metadata.json` structure (from `indexer.cpp`):
```json
{
  "version": "2.0",
  "points": 12345678,
  "projection": "",
  "hierarchy": { "firstChunkSize": N, "stepSize": N, "depth": N },
  "offset": [x, y, z],
  "scale": [x, y, z],
  "spacing": N,
  "boundingBox": { "min": [x,y,z], "max": [x,y,z] },
  "encoding": "BROTLI",
  "attributes": [{ "name": "position", "size": 12, "numElements": 3, ... }]
}
```

### 2.2 License

- **potree/potree**: BSD 2-Clause. ✅ Compatible with GPL-3.0-or-later.
- **PotreeConverter**: BSD 2-Clause. ✅ Compatible.
- **tentone/potree-core**: MIT (confirmed via npm registry). ✅ Compatible.

### 2.3 "Potree-Next" disambiguation and maintenance state

There are **two distinct Potree-Next codebases** — the issue title does not
distinguish them:

| Component | License | Notes |
|---|---|---|
| **m-schuetz/Potree-Next** | **AGPL-3.0** | WebGPU rewrite from scratch, **not a three.js integration**. AGPL-3.0 closes the SaaS loophole: serving it over HTTP requires delivering source to every user. Incompatible with a proprietary server component. Not evaluable as a drop-in. |
| **potree/potree** (the original) | BSD-2-Clause | WebGL1 viewer, 1.8.2 (2023-12-12), last code commit Apr 2024. Development stalled (822 open issues, no maintainer-driven changes for 16+ months). Coupling to its own viewer makes three.js integration awkward. |

| Component | Latest | Date | Notes |
|---|---|---|---|
| potree/PotreeConverter | 2.1.3 | 2026-06-22 | BSD-2-Clause. Linux binary missing in 2.1.3 (Windows only); 2.1.2 had a Linux binary. Must build from source for latest on Linux. |
| @pnext/three-loader | 0.2.5 | 2021-08-26 | Maintenance mode — dependabot-only commits. Open issue #79 (2020, unresolved) confirms **no Potree 2.0 format support** in the released package. Not recommended. |
| **tentone/potree-core** | **2.0.15** | **2026-04-08** | **MIT.** Last commit **2026-09-14** — actively maintained. Stripped-down ES module (no viewer, no jQuery/TWEEN), three.js as peer dep. Supports Potree 2.0 (`octree.bin`/`hierarchy.bin`/`metadata.json`). npm: `potree-core`. ✅ Best choice for three.js integration. |

---

## 3. Feasibility analysis — `.rux` → Potree octree

### 3.1 Conversion pipeline required

PotreeConverter 2.x reads **LAS and LAZ only** — no PLY, no E57, no custom
formats. Verified experimentally: running the nixpkgs binary
(`unstable-2023-02-27`) against a binary PLY exported from the NewOffice scan
(`rux export ply`, 1 212 572 pts, 18 MB) produces `#points: 0` and crashes
(`nlohmann::detail::type_error` on a null bounding box). The source confirms
a single `LasLoader` with `.las`/`.laz` extension guards — no PLY code path
exists. We have no LAS exporter in the codebase (exports are PLY, E57, Rhino,
COLMAP, Speckle, CSV, semantic images).

An alternative format with the same static-tile model is **COPC (Cloud-Optimized
Point Cloud)**: a LAZ file with an octree index header, servable via HTTP Range
from any static host. The same conversion barrier applies (LAS/LAZ input needed)
and a C++ COPC library (`copc-lib`, Apache-2.0) exists, but adds a new
dependency.

To feed PotreeConverter from a `.rux` project, two paths exist:

**Path A — subprocess pipeline:**
1. `rux export las` (new subcommand, new `liblas`/`PDAL`/`laszip` dependency)
   → temp `.las` file
2. Call `PotreeConverter` subprocess → `octree.bin` + `hierarchy.bin` + `metadata.json`
3. Serve the 3 output files as static assets from `rux gui`

**Path B — native C++ Potree 2.0 writer:**
Write the Potree 2.0 octree directly from `ProjectDB`:
1. Streaming spatial sort into an in-memory octree (O(N log N) or O(N) via
   LOD-bucket approach)
2. Serialize to `octree.bin` (contiguous, one node per chunk), `hierarchy.bin`,
   `metadata.json`

This would be the cleanest integration but requires implementing the octree
chunker from scratch (roughly equivalent to PotreeConverter's `indexer.cpp`,
~1 500 lines of C++).

### 3.2 Conversion cost and storage overhead

No official throughput benchmarks from Potree/PotreeConverter exist. The README
states version 2.0 is "10–50× faster than v1.7 on SSDs" — implying v1.7 rates
of roughly 2–5 M pts/sec scale to 20–250 M pts/sec for v2.x on modern NVMe.
Community reports (GitHub issues) for 100 M-point LAS files on SSD: 60–120 s
(~1–1.7 M pts/sec), suggesting moderate throughput on large files.

**Measured** (this evaluation, NewOffice 1.2 M-point PointXYZRGB):
- `rux export ply`: 3.9 s, 18 MB output
- PotreeConverter: aborted (PLY not supported — see §3.1)
- To estimate: at 1 M pts/sec, a 1.2 M-point cloud would convert in ~1.2 s;
  the NewOffice scan at 100 M pts would take ~100 s, assuming linear scaling.

**Storage overhead:** the Potree 2.0 binary representation stores 3 f32 values
per point for XYZ (12 bytes) plus attributes. Our RUXP storage is 16 bytes per
`PointXYZRGB` (12 xyz f32 + 4 rgba u32) in SQLite blobs. Potree applies a
quantisation scale that can reduce XYZ to `u16` per axis with a per-chunk bbox,
yielding ~6 bytes/point for positions. Color adds 3 bytes. Net: roughly **9–12
bytes/point** vs our **16 bytes/point in `.rux`** — a slight size advantage for
Potree, but less important than build/serving complexity.

### 3.3 Feature integration gaps

The RUXP endpoint carries information Potree tiles cannot:

| Feature | RUXP (#320) | Potree tiles |
|---|---|---|
| Per-point label (room/plane/instance) | ✅ `lod_source` zips label cloud | ❌ Needs a parallel Potree convert per attribute |
| Label color mode toggle | ✅ retained per page | ❌ Reload tiles, no server-side mapping |
| Dynamic filtering (`rux export ply -f 'room_labels in [1,2]'`) | ✅ server-side, lazy | ❌ Needs re-conversion |
| Freshness | ✅ always current | ❌ stale until re-converted |
| HTTP Range serving | not needed | ✅ standard; CDN-friendly |
| Per-request server compute | O(N) for LOD | None (pre-baked) |

---

## 4. Serving model

**Potree tiles** are three static files. `rux gui`'s Crow server can serve them
with a `GET /clouds/{name}/potree/*` static-file handler — no new server
infrastructure needed for serving. The conversion step (Path A or B above) is
what requires work, not the serving.

CDN compatibility is a theoretical advantage. In practice `rux gui` is a
single-user localhost server; CDN matters only for a multi-user deployment, which
is `ruxd`'s territory (Phase 6 of the GUI plan, #265).

**Dynamic RUXP** requires server compute (O(N) voxel pass per LOD request) but
is already implemented. The SQLite read is sequential-blob–friendly; the
NewOffice 1.2 M-point cloud takes under 2 s to LOD-select 200 000 points
(measured during #320 implementation).

---

## 5. Decision matrix

| Criterion | Potree-Next (A+B) | Extend RUXP |
|---|---|---|
| **LOD quality** at 1–10 M pts | ✅ Frustum-aware, hierarchical | ⚠️ Single overview level; no frustum cull yet |
| **LOD quality** at >100 M pts | ✅ Purpose-built for this scale | ❌ O(N) per request becomes expensive |
| Label / instance overlay | ❌ Needs parallel conversion pipeline | ✅ Already works |
| Feature freshness after pipeline run | ❌ Re-convert required | ✅ Always live |
| New dependencies | LAS exporter OR custom octree writer | None |
| Frontend code change | Replace paging with three-loader | Extend existing useCloudStream |
| Linux converter binary | Must build from source (2.1.3) | N/A |
| License risk | Verify tentone/potree-core license | No change |
| Implementation effort | 4–6 weeks | 2–3 weeks |
| Blocks label-mode GPU work | No | No |

---

## 6. Recommendation

**Extend RUXP with Morton ordering and frustum-aware LOD (close #321 as "won't
do — not now"; #320 approach is sufficient for current scan sizes).**

### Rationale

Our target clouds are 1–15 M points after `create clouds`. Potree's hierarchical
streaming was designed for >100 M-point LiDAR datasets served by a static file
host to many concurrent browser users. That is not our deployment:

- **Scale mismatch.** A 10 M-point scan at RUXP 15 MB/M-pt is 150 MB total.
  Even with no LOD, that streams from localhost in under 2 s at loopback
  speeds. Potree's main advantage — not loading the parts you don't see — is
  less valuable when the whole dataset fits in one HTTP response.
- **Integration cost is asymmetric.** Potree requires either a new LAS exporter
  + PotreeConverter subprocess orchestration, or a custom C++ Potree 2.0 writer
  (~1 500 lines). The existing RUXP stack needs Morton ordering + a small
  frustum-culling tile index — incremental work on proven code.
- **Label streaming is a first-class feature.** `lod_source` lets geometry and
  label clouds share one voxel selection. Pre-baked Potree tiles cannot express
  this; label mode would require a separate parallel convert pipeline.
- **The remaining RUXP gap is solvable.** The overview pass gives a coarse whole-
  cloud view immediately; the only missing pieces are (a) multiple LOD levels as
  the user zooms in and (b) fetching only tiles in the camera frustum. Both are
  addressed by:

  - **Morton ordering** stored at `create clouds` time: the cloud is written to
    `ProjectDB` in Morton / Hilbert curve order so any prefix is a uniform spatial
    sample. Prefix paging then doubles as LOD without extra server compute.
  - **Tile index**: a simple `[offset, limit]` spatial index computed once
    enables per-frustum fetching; the client requests only pages whose bounding
    boxes intersect the camera view volume.

  This is issue #320's stated design goal ("if points are stored in a progressive
  (Morton / octree) order, LOD and paging collapse into the same mechanism").

### When Potree would win

Potree becomes the right answer if:
1. **Scan sizes exceed ~50 M points** after `create clouds` — the O(N) LOD
   pass and full-cloud binary become expensive at that scale.
2. **Multi-user serving via `ruxd`** becomes a requirement — pre-baked static
   tiles trivially CDN/edge-cache; RUXP requires per-request server-side
   compute.
3. **Label streaming is no longer needed** in the 3D viewport (e.g. if the
   label view moves to a separate 2D panel).

### Concrete next step

File one issue scoped to Morton ordering + RUXP tile index (see §7). Do not open
a Potree integration issue until scan sizes consistently exceed 50 M points.

---

## 7. Follow-up issues

If this recommendation is accepted:

- **Issue A — Morton ordering at `create clouds` time.** Store the fused cloud
  in Morton curve order in `ProjectDB` blobs. Update `point_lod.cpp` to skip the
  voxel pass when storage order is already spatial (a single flag in the cloud
  metadata). Effort: ~1 week.
- **Issue B — RUXP tile index + frustum-culled fetching.** Add a compact spatial
  tile index (node bounding boxes, ~100 bytes/tile) returned alongside the first
  page or from a `/clouds/{name}/tiles` endpoint. Client sends a camera frustum
  (6 planes) and only paginates the pages whose bboxes intersect it. Effort: ~1
  week (server index) + ~1 week (client frustum query).
- **Issue C — Multi-level LOD.** With Morton order, a coarse level is `skip=8`,
  a medium level is `skip=4`, full is `skip=1`. Server generates the skip-sampled
  view with a fast stride read; client transitions between levels as the camera
  moves. Effort: ~1 week.

If the recommendation is overridden and Potree is chosen:

- **Potree-A — `rux export las`.** Add LAS export to `io/` (likely liblas or
  PDAL). Effort: ~1 week.
- **Potree-B — PotreeConverter integration.** Add `rux create potree` that runs
  the converter subprocess and places output in a `{project}.potree/` dir. Serve
  via `GET /clouds/{name}/potree/*`. Effort: ~1 week.
- **Potree-C — Frontend loader.** Replace `useCloudStream.ts` paging with
  `tentone/potree-core` (`npm: potree-core 2.0.15`). Verify license. Wire the
  three.js `PointCloudOctree` into `PointCloudScene.ts`. Effort: ~2 weeks.
  **Label overlay becomes a separate issue with no clear solution.**

---

## 8. Sources

- m-schuetz/Potree-Next: <https://github.com/m-schuetz/Potree-Next> (AGPL-3.0, WebGPU rewrite — **not three.js**)
- potree/potree: <https://github.com/potree/potree> (BSD-2-Clause, rel. 1.8.2, 2023-12-12; stalled)
- potree/PotreeConverter: <https://github.com/potree/PotreeConverter> (BSD-2-Clause, rel. 2.1.3, 2026-06-22; LAS/LAZ only)
- tentone/potree-core: <https://github.com/tentone/potree-core> — npm: `potree-core` (MIT, v2.0.15, 2026-04-08; last commit 2026-09-14)
- pnext/three-loader: <https://github.com/pnext/three-loader> (v0.2.5, 2021-08-26; 42 open issues; no Potree 2.0 in releases)
- COPC spec: <https://copc.io/> (Apache-2.0 `copc-lib`)
- RUXP v1 spec: `docs/gui/binary-points.md`
- Voxel LOD implementation: `apps/rux/src/gui/point_lod.cpp`
- GUI server: `apps/rux/src/gui/api.cpp`, `Server.cpp`
- GUI plan: issue #265
- Related: #283 (RUXP transport), #320 (voxel LOD, closed/merged), #265 (GUI plan)
