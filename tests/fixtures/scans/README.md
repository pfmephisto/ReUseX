<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Real-scan test fixtures

Small, git-lfs-tracked slices of real captures. They exist so the test suite
has at least one input that carries genuine sensor noise, genuine SLAM poses
and genuine depth dropouts — the synthetic scenes in `tests/support/` cannot
reproduce those, and the bugs that only appear on real data are otherwise
invisible (#204).

| Fixture | Size | Frames | Test |
|---|---|---|---|
| `office_corridor.rux` | 2,138,112 B (2.04 MiB) | 10 | `tests/integration/test_real_scan_fixture.cpp` |

---

## `office_corridor.rux`

### Provenance

Ten consecutive sensor frames (`node_id` 1995–2004) trimmed from the
**NewOffice** capture — the maintainer's own scan of the Link Arkitektur A/S
office, Nordre Fasanvej 108B, 2000 Frederiksberg, Denmark.

The source capture is a handheld iOS LiDAR scan, converted to an RTABMap SLAM
database and imported with `rux import rtabmap` on 2026-04-23. It is the same
scan the 360-integration and loop-closure figures in `docs/research/` are
drawn from. It is **not** redistributed in full — only the ten frames below.

The selected range is ~5.2 s of walking (timestamps 1773126992.2 → 1773126997.3)
covering ~1.2 m of camera travel along an empty corridor.

### Licence

`GPL-3.0-or-later`, © 2026 Povl Filip Sonne-Frederiksen — the repository
licence. The data was captured by the maintainer, so there is no third-party
redistribution question.

This is why the fixture is **not** cut from a public benchmark. Both datasets
already used for development were checked and rejected as fixture sources:

- **ARKitScenes** — CC BY-NC-ND 4.0 (see the `LICENSE` file in Apple's
  ARKitScenes repository). Non-commercial *and* no-derivatives: incompatible
  with GPL-3.0-or-later, and a ten-frame trim is exactly the derivative the
  ND clause forbids.
- **MuSHRoom** — research-use terms with no explicit redistribution grant.
  Absent a clear grant, the answer is no.

Both remain fine to *use* locally for benchmarking (`scripts/bench-arkitscenes.sh`,
`scripts/bench-mushroom.sh`); neither may be vendored into this repository.

### Privacy

Every candidate frame was opened and visually inspected before selection. The
corridor run shows an empty circulation space: no people, no screens, no
readable documents, no whiteboards. Frames elsewhere in the same capture were
rejected for showing vehicle number plates through a street-facing window; the
selected range has no such exterior view.

### What is inside

A `.rux` project database (sqlite3) at **schema version 11**, containing the
full ProjectDB schema with exactly one populated data table:

- `sensor_frames` — 10 rows, each with `color` (JPEG, 720×960), `depth`,
  `confidence`, `transform` (4×4 pose, 128-byte blob), `width`, `height`,
  `camera_model` (pinhole intrinsics + local transform, JSON) and `timestamp`
- `projects` — one row carrying the provenance note above

No point clouds, meshes, labels or pipeline log: the test generates those, so
storing them would only be a second, staler copy of the expected output.

Because the fixture is already at the current schema version, opening it does
not trigger a migration. If `LATEST_SCHEMA_VERSION` moves, the test still
passes — `ProjectDB` migrates a copy in a temp directory — but the fixture
should be regenerated so it stays self-describing.

### How it was made

From the repo root, with `rux` built and the source project at
`~/repos/NewOffice/project.rux`:

```bash
SRC=~/repos/NewOffice/project.rux
OUT=tests/fixtures/scans/office_corridor.rux

# 1. Create an empty project at the CURRENT schema version. `rux set` opens
#    the database read-write, which is what creates the file and runs the
#    migrations; a read-only command such as `rux info` would not.
rux -p "$OUT" set projects.default.name "office corridor fixture"

# 2. Copy the ten frames verbatim -- no re-encoding, no re-scaling.
sqlite3 "$OUT" "ATTACH DATABASE '$SRC' AS src;
INSERT INTO sensor_frames (node_id, color, depth, confidence, transform,
                           width, height, camera_model, timestamp)
  SELECT node_id, color, depth, confidence, transform,
         width, height, camera_model, timestamp
  FROM src.sensor_frames WHERE node_id BETWEEN 1995 AND 2004;
DETACH DATABASE src;"

# 3. Record provenance inside the fixture itself, so a stray copy is still
#    traceable without this README.
sqlite3 "$OUT" "UPDATE projects SET
  name = 'ReUseX test fixture: NewOffice corridor',
  building_address = 'Link Arkitektur A/S, Nordre Fasanvej 108B, 2000 Frederiksberg, Denmark',
  survey_organisation = 'Povl Filip Sonne-Frederiksen',
  notes = 'Ten consecutive sensor frames (node_id 1995-2004) trimmed from the NewOffice iOS-LiDAR capture. GPL-3.0-or-later. See tests/fixtures/scans/README.md.'
  WHERE id = 'default';"

# 4. Compact: fold the WAL back in and drop the free pages left by step 1.
sqlite3 "$OUT" "PRAGMA wal_checkpoint(TRUNCATE); VACUUM;"
rm -f "$OUT-wal" "$OUT-shm"
```

Original `node_id`s are kept rather than renumbered to 0–9, so a frame in the
fixture can always be traced back to the same frame in the source capture.

### Storage

Tracked in git-lfs via `tests/fixtures/**/*.rux` in `.gitattributes`. A clone
without `git lfs pull` gets a ~130-byte pointer file instead of the database;
the test detects that and skips with an actionable message rather than failing
on a corrupt sqlite header.

```bash
git lfs pull            # fetch the fixture
git lfs ls-files        # confirm it is LFS-tracked, not committed inline
```

### What the test asserts

`tests/integration/test_real_scan_fixture.cpp`, one Catch2 case tagged
`[integration][fixture]`, so `ctest -R fixture` selects it. It copies the
fixture to a unique temp directory (never mutating the source tree, and safe
under `ctest --parallel`), then runs

```
sensor frames -> reconstruct_point_clouds -> segment_planes
```

with every parameter pinned in the test rather than defaulted, so a change to
a pipeline default cannot silently retune the bounds. It uses a 2 cm voxel and
no pixel subsampling; at the 5 cm / factor-4 pipeline defaults ten frames yield
only ~6.3 k points and three merged planes, too coarse to notice a regression.

| Property | Reference | Bound | Why |
|---|---|---|---|
| Sensor frames | 10 | `== 10` | Fixture integrity |
| Colour / depth / confidence per frame | non-empty | non-empty | All five components survive the trim |
| Intrinsics | 720×960, fx,fy > 0 | exact / positive | Camera model round-trips |
| Pose bottom row | `[0 0 0 1]` | exact | Real rigid transform, not a placeholder |
| Reconstructed points | 68 761 | 65 300 – 72 200 (±5 %) | Depth/confidence filter, voxel grid |
| Bbox extent Z | 2.94 m | 2.6 – 3.3 m | Floor-to-ceiling height; catches depth scaling |
| Bbox extent X | 3.64 m | 3.0 – 4.6 m | Footprint at a 4 m depth clip |
| Bbox extent Y | 3.97 m | 3.2 – 4.8 m | Footprint at a 4 m depth clip |
| Planes detected | 5 | 4 – 6 | Below 4 = collapse, above 6 = fragmentation |
| Labeled fraction | 71.6 % | > 60 % | Planar surfaces are still being recovered |
| Every plane normal | \|n·z\| > 0.95 or < 0.20 | all | No slanted planes — surfaces are not smeared |
| Horizontal planes | 2 | `== 2` | Floor + ceiling |
| Vertical planes | 3 | `>= 2` | Walls |
| Floor↔ceiling centroid gap | 2.94 m | 2.5 – 3.3 m | Room height, independent of bbox outliers |

The orientation assertions are the sharpest of these. An axis-flipped pose
convention (the ARKitScenes class of bug, #224) or gross pose drift fans the
walls out and destroys the horizontal/vertical split while leaving the point
count and the plane count essentially untouched.

**On tolerances.** Running the reference configuration three times gave
bit-identical output, so the pipeline is deterministic on a fixed toolchain and
none of the slack above is for run-to-run jitter — it is headroom for a PCL or
compiler bump in the flake. Every regression this test exists to catch moves
the numbers by tens of percent, so narrow bands cost no sensitivity. If a
legitimate change moves them, re-measure and update this table and the header
comment in the test together.

Reference run: Release, GCC 15.2, PCL from the flake, 2026-09-08. Runtime
**4.7 s** via `ctest -R fixture`.
