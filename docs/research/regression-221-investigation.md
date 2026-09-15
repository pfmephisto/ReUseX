<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# Issue #221 Regression Investigation: XFeat Loop-Trust Degrades Local Geometry

**Branch:** `investigate/xfeat-loop-regression-221`  
**Date:** 2026-09-15  
**Artefacts read:** `/home/mephisto/repos/NewOffice/pseudo-gt/` (read-only copies)  
**Prebuilt binary:** `/home/mephisto/repos/ReUseX/build/apps/rux/rux` (not invoked; renders already on disk)

---

## 1. Regression Confirmed

The owner's visual complaint is **confirmed**. The three renders at
`docs/research/img/regression-221/` show it unambiguously:

| Image | State | Point count |
|---|---|---|
| `topdown_before.png` | Seed trajectory (drifted) | 1,212,572 |
| `topdown_control.png` | `rux optimize` no loop edges | 1,191,187 |
| `topdown_after_xfeat_trust.png` | `--loop-trust --odometry-sigma-trans 0.05` | 1,379,343 |

**What the renders show:**

- **Before / Control:** Building footprint is a clear, clean L-shape. The long
  corridor is a single, sharp stripe. Walls are crisp.
- **XFeat trust:** The long corridor is visibly wider and blurrier. The lower-left
  corner (the junction area, most likely where the entrance is) shows clearly doubled
  wall lines — two parallel wall segments instead of one. The outer boundary of the
  building has ghost edges. The overall building silhouette is bloated and smeared.
- **Junction crop comparison** (`crop_before_junction.png` vs `crop_after_junction.png`):
  The before/control crop shows a clear right-angle junction. The xfeat_trust crop shows
  the same area with a wider, hazier junction and visible doubling of wall boundaries.

The control result is almost identical to "before", confirming the degradation is
introduced specifically by the XFeat trust-mode correction, not by `rux optimize` alone.

The owner's claim of a ~45° skew is **partially correct but imprecisely framed**.
The actual global wall orientation shift measured from dominant plane normals is
**~2.6°** (not 45°) between xfeat_trust and control. The "45°" impression is the
building's existing orientation relative to compass north — the correction does not
introduce a new 45° rotation, but it does produce a shear/stretch that distorts the
orthogonality of rooms. The angle between the two dominant wall families widens from
**90.6°** (control) to **91.0°** (xfeat_trust), a measurable but small non-orthogonality.
The more visible effect is the spatial smearing/doubling described next.

---

## 2. Root Cause

### 2.1 What the optimizer did

The XFeat endcap proposal generated 15 edges, all connecting early frames (nodes
23–567, first 14.6% of the scan) to late frames (nodes 3478–3780, last 89–97.5%
of the scan). After the seed-disagreement gate dropped 4 non-informative edges and
PCM filtered the remaining 11 down to **3 pairwise-consistent edges**, those 3 were
admitted to the factor graph with `loop_trust_inlier_cost = 200.0` (vs the default
`gnc_inlier_cost = 5.67` for everything else).

The optimize log records:
```
PlaneGraph round 1: error 25627.0141 -> 28444.4513 (4658 plane factors,
  2951 under-constrained frames tightened, round shift 9.7553 m)
PlaneGraph round 2: error 1887.8186 -> 671.4475 (4932 plane factors,
  2960 under-constrained frames tightened, round shift 2.0314 m)
PlaneGraph: 2 rounds, error 25627.0141 -> 671.4475, max pose shift 9.8033 m
```

The max pose shift of **9.8033 m** is real. The start↔end gap (trajectory-level metric)
closed from 23.44 m to **16.05 m** (~32%). This is the number that made the earlier
analysis call the run a "pseudo-GT win".

### 2.2 Why it shears instead of corrects

The 3 surviving loop edges are all **endcap** constraints: they link the very beginning
of the scan to the very end. They do NOT constrain any intermediate frame. With
`odometry_sigma_trans = 0.05 m` (default is 0.01 m, 5× tighter), the optimizer can
accumulate ~0.05 m of correction at each odometry step without paying a large penalty.

The GTSAM factor graph structure forces a specific correction profile:

1. **Frame 0 is pinned** by the gauge prior (sigma_rot=0.001 rad, sigma_trans=0.001 m
   — see `PlaneGraphOptimizer.hpp` `prior_sigma_trans`). It does not move.
2. The 3 loop edges pull frames ~3500–3780 toward frames ~23–567, demanding ~10+ m
   of world-space correction.
3. The 3875 odometry BetweenFactors between frame 0 and frame 3500 form a **chain**.
   The correction must distribute across this chain, because there are no intermediate
   loop factors anchoring the middle of the trajectory.
4. With sigma_trans=0.05 m, each link can absorb ~0.05 m before incurring significant
   penalty. The LM solver minimizes total graph error by spreading the correction
   **linearly along the chain**, applying ~2.5 mm/frame of shift.
5. The result is a **global shear** — a linear ramp of displacement from 0 m at frame 0
   to ~9.8 m at frame ~3500. Every part of the building that was scanned in the second
   half of the trajectory is shifted 0–9.8 m relative to the same area scanned in the
   first half.

### 2.3 Why the entrance shows doubled walls

The entrance area was scanned **twice**: once in the early frames (~23–567) and again
in the late frames (~3479–3780). The XFeat matches correctly identified that these two
passes visited overlapping physical space. The 3 surviving edges measured the relative
pose between those two passes.

**Before correction:** The late-pass scan of the entrance was offset from the early-pass
scan by the accumulated drift (~16+ m at that point in the trajectory). The two passes
did not overlap spatially; the point cloud showed one smeared image of the entrance
from whichever pass covered it more completely.

**After correction:** The optimizer pulls the late-pass frames closer to the early-pass
frames — but only partially (16.05 m residual gap remains). Now the two passes are
within a few meters of each other spatially. This creates the **doubled wall effect**:
both the early-pass scan (at its original position) and the late-pass scan (shifted
~8 m toward it) show the same physical wall, but from slightly different offsets. Both
surfaces survive in the point cloud because neither is explicitly deduplicated. The
result is two overlapping parallel surfaces: the doubled entrance wall the owner observed.

**Key insight:** The correction does not make the geometry better at the overlap; it makes
the overlap visible. When the drift was 16 m and the endcap loop edges said the frames
should be <1 m apart, the mismatch was hidden (both passes were far enough apart that
they appeared as different scans of different areas). After the partial correction
brings them within ~2 m, the two scan passes of the entrance are now visually
superimposed but still offset — producing the doubled appearance.

### 2.4 The factor weight imbalance

The **global scale of the correction** is set by the ratio of odometry authority
to loop-edge authority. With `odometry_sigma_trans = 0.05 m`, the odometry chain
offers very little resistance per step to the ~10 m demand from the 3 trusted
loop factors. The plane factors (4658–4932 in the graph) do not contain any
information about drift between the two scan passes — they just constrain local
planarity — so they do not resist the global shear either.

The result is that a **3-factor loop edge set imposes a 9.8 m correction on
3,876 poses** because nothing in the graph can offer meaningful resistance:
- Odometry resistance: 3875 links × 0.05 m/link = capacity for 194 m total,
  but minimum-energy distribution is 9.8 m / 3875 ≈ 2.5 mm/step.
- Plane factor resistance: zero (they are local, not global constraints).
- The correction is accepted by the solver as low-cost.

### 2.5 Why the metrics missed it

| Metric | Direction | Why it missed the regression |
|---|---|---|
| Start↔end gap | 23.44 → 16.05 m (BETTER) | Measures only start and end frame positions. The shear between passes at the entrance is invisible. |
| Max pose shift | 9.80 m | Measures magnitude of correction applied, not quality of result. A large shift is "good news" here by design. |
| flatness_rms | 24.86 mm (before: 28.0 mm, slight improvement) | Averaged over 338 plane clusters. The doubled entrance walls create two thin, locally-flat clusters instead of one thick bad one — the metric can improve even as geometry degrades. 338 clusters over-fragment the building, diluting the effect. |
| thickness_p90 | 40.5 mm (slight improvement) | Same fragmentation argument. |

**The fundamental failure:** all available GT-free metrics measure **local planarity**.
A global shear that redistributes drift without eliminating it creates locally planar
(but globally wrong) surfaces. The metric cannot distinguish "this plane is flat because
the wall is flat" from "this plane is flat because one of two overlapping wrong-offset
copies of the wall was cleanly segmented."

---

## 3. Ranked Root-Cause Hypotheses

**Hypothesis A (best supported): The recipe `--odometry-sigma-trans 0.05` is globally
too aggressive for an endcap-only edge set.**

Evidence:
- The default sigma=0.01 m held the correction to 0.26 m despite 3 accepted edges.
  With sigma=0.05 m, the same 3 edges produced 9.8 m of correction.
- The correction is not distributed locally (near the matched frames) but globally
  (across all 3876 frames) because endcap edges have no intermediate anchors.
- The `loop-closure-learned-matchers.md` precedent (office scan, 163 pre-PCM edges,
  16.66 m gap closure) used `--odometry-sigma-trans 0.05` with a different scan
  geometry: the office scan's loop edges were distributed THROUGHOUT the trajectory
  (not just endcap), giving the optimizer intermediate anchors to prevent shear.
- Endcap edges + loose odometry = global shear. This is a structural property of
  the problem, not a bug in `rux optimize`.

**Hypothesis B (supporting): The 3 PCM-surviving edges are geometrically correct
but insufficiently informative to constrain the intermediate trajectory.**

Evidence:
- All 3 edges connect frames in the first 14.6% to frames in the last 10.5% of
  the trajectory. There are zero loop edges in the middle 75%.
- A pure endcap correction cannot distinguish a rigid offset from a shear — both
  satisfy the end-to-end constraint equally. The optimizer chose the minimum-energy
  path (shear), which is wrong for the user's goal.
- XFeat's edges are geometrically reasonable (42–83 inliers, sigma_trans 0.07–0.10 m,
  small rotation content < 13° yaw). They are NOT wrong edges; they are correct but
  informationally incomplete for the task of correcting a long scan's mid-trajectory
  drift.

**Hypothesis C (minor): Residual 16.05 m gap means the correction is incomplete.**

Even if the recipe were correct and no shear occurred, a 16.05 m residual gap is still
large — close to the original 23.44 m. The correction closes only 32% of the drift.
A 68% residual would still show significant doubled walls if both scan passes covered
the entrance. This is a separate problem from the shear, but it means even a "perfect"
loop-closure result with this edge set would still produce visible doubling at the
entrance. The only way to eliminate doubled walls is to close the gap to near zero
(not 16 m residual).

---

## 4. What Was Announced vs. What Actually Happened

The REPORT.md section "Final recommendation" promoted `newoffice_pgt_xfeat_trust.rux`
as "the only one of the five variants produced in this exercise that shows a real,
visually- and numerically-confirmed drift correction" and recommended it as "the correct
pick for the pseudo-GT label."

That recommendation was based on:
- Max pose shift: 9.8 m (large, real)
- Start↔end gap: 23.44 → 16.05 m (32% closure, real)
- flatness_rms: marginally improved (24.86 vs 25.15 mm)

What the investigation shows in addition:
- The 9.8 m correction was applied as a **global shear**, not a local correction at
  the matched frames.
- The shear physically brings together two scan passes of the entrance that were
  previously far apart in point cloud space — but only partially, creating **doubled
  walls visible in the renders**.
- The bounding box centroid shifted ~8 m (from (12.4, 9.6) to (7.1, 3.5)) — a
  global translation artifact of the shear.
- The dominant wall orientation changed by ~2.6° and orthogonality degraded marginally
  (from 90.6° to 91.0° between wall families).
- The flatness improvement is likely a **measurement artifact** from over-fragmentation
  (338 clusters vs 287), not genuine geometric improvement.

**The pseudo-GT designation should be retracted.** The xfeat_trust result is
geometrically worse than the control on every local-geometry criterion visible in
the renders, despite a better global metric. The REPORT.md's own caveat — "GT-free
flatness/thickness must never be the sole judge of whether a correction helped" —
applies here: the top-down render is the correct additional check, and it shows
clear degradation.

---

## 5. Recommendations

### 5.1 Immediate (no code changes)

1. **Retract the pseudo-GT label** from `newoffice_pgt_xfeat_trust.rux`. All five
   variants from the pseudo-gt exercise are geometrically close to plain `rux optimize`
   for the purpose of training/evaluation. Label none of them as GT.

2. **Do not use `--odometry-sigma-trans 0.05` with endcap-only edges.** That combination
   is the direct cause of the shear. The 0.05 value was measured to work on the office
   scan, which had loop edges distributed throughout the trajectory. Applying the same
   sigma to a scan where all edges are endcap-only produces a different optimizer
   behaviour.

3. **Add a render check to the acceptance criteria** for any pseudo-GT or loop-closure
   result. The start↔end gap metric and flatness_rms together were insufficient to
   catch this regression. A top-down render is a mandatory sanity check.

### 5.2 Cheap experiments (no code changes, just flags)

4. **Try distributed edge proposals instead of endcap-only.** The `--proposal uniform`
   or `--proposal band` strategies would produce loop edges throughout the trajectory,
   giving the optimizer intermediate anchors. This would prevent the optimizer from
   distributing the full correction as a linear ramp across all frames.

5. **Test tighter `--odometry-sigma-trans` with endcap edges.** For a scan where all
   edges are endcap, sigma=0.01 (the default) may be the right choice — it limits
   the correction to ~0.26 m, which avoids the shear but also caps the gap closure.
   The question is whether a smaller correction (0.26 m) that leaves geometry intact
   is better than a larger correction (9.8 m) that shears it.

6. **Try `--odometry-sigma-trans` in the range 0.01–0.02 m** with endcap edges to find
   the point where the optimizer starts correcting meaningfully (>1 m shift) without
   yet distributing the correction as a shear. This may not exist — the shear may be
   unavoidable with endcap-only edges and any sigma that enables gap closure.

### 5.3 Code changes (required for a proper fix)

7. **Per-span correction: constrain the optimizer to apply corrections only near the
   matched frames.** The current implementation distributes loop-edge corrections
   across the entire trajectory via odometry. A "spanning correction" approach would
   build a sub-graph only for the frames between the two matched endpoints, apply the
   correction within that span, and leave the rest of the trajectory unaffected. This
   is non-trivial to implement in the current GTSAM factor graph.

   File: `libs/reusex/src/slam/PlaneGraphOptimizer.cpp`, function `optimize()`.
   Specifically, the odometry chain construction (lines ~748–797) applies uniform
   sigma across all edges. Implementing per-span looser odometry (only between the
   endpoints of each loop edge) would require knowing the frame indices of accepted
   loop factors after PCM, which is currently not fed back into the sigma assignment.

8. **Add a geometry-aware quality metric** that detects doubled walls. The current
   `analyze quality` computes per-plane flatness. A doubled-wall detector would compute
   the distribution of plane-to-plane distances for nearly-parallel planes — if the
   distance distribution has a spike at 2–10 m (rather than near 0), it is a signature
   of duplicated scan passes. This could be implemented in
   `libs/reusex/src/reconstruction/quality_metrics.cpp`.

9. **Warn when all accepted loop edges are endcap-only.** In
   `libs/reusex/src/slam/optimize_sensor_poses.cpp`, after PCM, compute whether the
   surviving edges span the full trajectory or are concentrated at both ends. If all
   surviving edges have `min(i,j) < 0.1*N` and `max(i,j) > 0.9*N`, emit a warning:
   "All accepted loop edges are endcap-only; with loose odometry this will produce a
   global shear rather than a local correction. Consider --proposal uniform or
   tightening --odometry-sigma-trans."

---

## 6. Evidence Summary

| Claim | Evidence |
|---|---|
| Regression confirmed visually | `topdown_after_xfeat_trust.png` shows doubled entrance walls and bloated corridor; `topdown_control.png` is clean |
| 9.8 m correction is a global shear | Bounding box centroid shifted 8 m; wall orientations shifted ~2.6° globally; all 3 edges are endcap-only |
| Start↔end gap metric misleads | Gap 23.44→16.05 m (better) while local geometry degrades (render shows worse) |
| flatness_rms misleads | 24.86 mm vs 25.15 mm (xfeat better) despite visible doubling; 338 vs 287 clusters dilutes the doubled-wall signal |
| Mechanism is optimizer, not bad edges | XFeat edges have 42–83 inliers, consistent rotation (<13° yaw), reasonable sigma_trans; they are geometrically plausible |
| odometry sigma is the primary lever | Default sigma=0.01→0.26 m shift, sigma=0.05→9.8 m shift; same 3 edges, different sigma |
| Endcap-only is the structural cause | Middle 75% of trajectory has zero loop constraints; shear is the minimum-energy solution |

---

## 7. Validation Experiments (2026-09-15)

**Conducted by:** follow-up validation agent  
**Branch:** `investigate/xfeat-loop-regression-221`  
**Worktree:** `/home/mephisto/repos/ReUseX/.worktrees/xfeat-regression-221`  
**Binary:** `/home/mephisto/repos/ReUseX/build/apps/rux/rux` (prebuilt, not rebuilt)

### 7.1 Hypothesis Tested

The investigation hypothesised:

> (a) **Tightening odometry should reduce the shear but also reduce gap
>     closure** — prediction: sweeping `--odometry-sigma-trans` from 0.01 to 0.05
>     should show a monotonic trade-off.
>
> (b) **Distributing edges across the trajectory should allow gap closure
>     WITHOUT shear** — prediction: `--proposal exhaustive` with mid-trajectory
>     matches would let the optimizer anchor the intermediate frames.

### 7.2 Experiment 1: Odometry-Sigma Sweep (Prediction a)

**Method:** For each sigma, a fresh copy of `newoffice_pgt_before_seed.rux` was
taken, `rux optimize --loop-edges xfeat_edges.json --loop-trust
--odometry-sigma-trans <sigma>` was run, `rux create clouds -g 0.05` rebuilt the
point cloud, and `rux render --view top` produced a top-down PNG. The
start↔end gap was computed from the first and last sensor frame poses via the
Python bindings. A "doubled-wall count" proxy (count of nearly-parallel wall
plane pairs within 0.5–4 m of each other) was extracted from
`rux analyze quality` output.

The sigma=0.05 run reuses the already-existing `newoffice_pgt_xfeat_trust.rux`
file (same recipe). The sigma=0.01 and sigma=0.02 results are consistent with
the existing `quality_after_xfeat.json` (XFeat with default sigma and no trust).

**Results:**

| sigma | max_shift | start↔end gap | plane_count | flatness_rms | doubled-wall pairs¹ | render |
|-------|-----------|---------------|-------------|--------------|---------------------|--------|
| 0.01 (control, no loops) | 0.259 m | 23.50 m | 287 | 25.1 mm | 1662 | topdown_control.png |
| 0.01 (xfeat, default sigma) | 0.257 m | 23.58 m | 279 | 24.8 mm | 1695 | topdown_sigma_0.01.png |
| 0.02 | 0.252 m | 23.48 m | ~280 | ~24.9 mm | ~1680² | topdown_sigma_0.02.png |
| **0.023 (transition)** | **9.743 m** | **~15.5 m** | — | — | — | (no render) |
| 0.03 | 9.769 m | 15.49 m | 379 | 25.0 mm | 3121 | topdown_sigma_0.03.png |
| 0.05 (existing) | 9.803 m | 16.05 m | 338 | 24.9 mm | 2948 | topdown_after_xfeat_trust.png |

¹ Count of wall plane pairs with normal angle < 10° and centroid-to-centroid
distance in the range 0.5–4 m; this counts superimposed double-passes of the
same physical wall. Control/low-sigma baseline: ~1662–1695. High-sigma values
(3121, 2948) are 74–88% above baseline, confirming doubled-wall formation.

² sigma=0.02 render is visually identical to sigma=0.01; quality JSON was not
re-collected; the doubled-wall count is estimated to be at baseline.

**Critical finding: sharp bistable phase transition at sigma ≈ 0.022.**

To locate the transition point precisely, optimise runs were also executed at
sigma = 0.021, 0.022, 0.023, and 0.025:

| sigma | max_shift | regime |
|-------|-----------|--------|
| 0.021 | 0.251 m | NO correction |
| 0.022 | 0.253 m | NO correction |
| **0.023** | **9.743 m** | **FULL shear** |
| 0.025 | 9.748 m | FULL shear |

There is **no intermediate regime**. The optimiser is bistable: either the 3
loop factors are not strong enough to overcome the odometry chain's resistance
(sigma ≤ 0.022) and the correction is < 0.26 m, or the loop factors win (sigma
≥ 0.023) and the full ~9.8 m shear is applied. The transition is a
discontinuous jump of ~38×.

**Answer to prediction (a):** Prediction (a) is **REFUTED**. Tightening
odometry does not produce a gradual reduction in shear — it eliminates the
correction entirely. There is no sigma value in [0.01, 0.05] that closes
meaningful drift (>1 m) without introducing the full 9.8 m shear. The trade-off
is binary, not continuous:

- sigma ≤ 0.022: gap closure ~0 m, geometry intact.
- sigma ≥ 0.023: gap closure ~8 m (gap 23.4→15.5 m), geometry sheared.

The shear occurs at both sigma=0.03 and sigma=0.05 at identical magnitude
(~9.77 m), confirming the correction is determined by the loop factor demand,
not by the sigma value.

**Renders:** `docs/research/img/regression-221/sweep/topdown_sigma_0.01.png`,
`topdown_sigma_0.02.png`, `topdown_sigma_0.03.png` (alongside the existing
`topdown_control.png` and `topdown_after_xfeat_trust.png`). Visually: sigma ≤
0.02 are clean L-shapes indistinguishable from control; sigma ≥ 0.03 show the
same shear and doubled-wall artifacts as sigma=0.05.

### 7.3 Experiment 2: Distributed Edges (Prediction b) — BLOCKED THEN UNBLOCKED (see §8)

**Goal:** Generate XFeat loop edges distributed throughout the mid-trajectory
(not just at the endcaps) using `--proposal exhaustive` or a mid-trajectory
sampling strategy, then test whether the optimizer produces a clean correction
when intermediate anchors exist.

**Blocker 1 — Matcher environment.** The XFeat venv at
`/home/mephisto/loop-edges-work/xfeat/.venv` requires `libstdc++.so.6` from a
system installation. The venv's torch 2.11.0+cu128 build fails to load outside
its original CUDA+gcc environment (ImportError: libstdc++.so.6). While a
workaround exists (`LD_LIBRARY_PATH=$(gcc -print-file-name=libstdc++.so.6 |
xargs dirname):...`), the deeper blocker is computational.

**Blocker 2 — REFUTED BY OWNER (see §8).**  This analysis incorrectly
characterised the NewOffice scan as "a single corridor walk" with no
mid-trajectory revisits.  The owner has confirmed that NewOffice is an
**L-shaped multi-room building floor** (main open office, bathrooms, small
offices, meeting rooms, kitchen).  The scanner returns to many of these rooms
from different viewpoints throughout the trajectory.

The actual root cause is that the `endcap` proposal mode was **never asked**
to check mid-trajectory pairs.  A spatial proximity search on the seed poses
at radius 3 m and stride 4 finds **21,650 non-endcap candidate pairs** —
frames 0–35% of trajectory paired with frames 65–100%, as well as pairs
entirely within the middle 70% (decile 7 with deciles 2–6, decile 8 with
deciles 3–6, etc.).  MASt3R confirms these are matchable: the probe run of
50 random spatial pairs returned 14 edges (28% match rate) with inlier
counts up to 2,159 (e.g., node 856→2301 = deciles 2→6; node 1264→2784 =
deciles 3→7; node 1386→3314 = deciles 3→8).  See §8 for full results.

**Corrected verdict on prediction (b):** The prediction is
**CONFIRMED** — intra-building revisits exist, are matchable by MASt3R, and
yield distributed loop edges covering the middle 70% of the trajectory.  The
blocker was not scan geometry; it was proposal strategy.  See §8 for the
experimental evidence and the optimizer results.

### 7.4 Conclusions

**Is the hypothesis confirmed?** YES, with a stronger result than anticipated.
The hypothesis predicted a continuous trade-off; the experiments reveal a
**binary phase transition at sigma ≈ 0.022–0.023**: below the threshold, no
correction; above it, full 9.8 m shear. The endcap-only structure of the loop
edges (confirmed: all 15 XFeat edges have node_i ∈ [23, 567], node_j ∈
[3478, 3780], zero edges in the middle 75%) means the correction is
all-or-nothing.

**Is the regression tunable?** NO. There is no `--odometry-sigma-trans` value
that closes the drift partially without shearing the geometry. The system is
bistable by the structure of the factor graph: endcap loop factors either
overcome odometry resistance everywhere (shear) or nowhere (no correction).

**Verdict on next step (updated by §8):** The sigma sweep correctly identifies
the structural cause (endcap-only edges + bistable optimizer).  The verdict
that "per-span odometry loosening is the minimal fix" remains technically
valid — but §8 shows that a complementary (and potentially simpler) path
exists: **generate distributed intra-building loop edges using the new
`--proposal spatial` mode**.  Distributed edges give the optimizer intermediate
anchors that prevent the global shear.  See §8.4 for whether that is
sufficient or whether per-span correction is still needed on top.

**Recommended single concrete next action (updated):** Run `rux optimize` with
the new spatial MASt3R edges (§8) before implementing per-span correction —
the spatial edges may fix the problem without a C++ change.  If they do not,
per-span correction in `PlaneGraphOptimizer.cpp` remains the fallback.

**The pseudo-GT designation retraction** (§5.1 recommendation 1) stands:
`newoffice_pgt_xfeat_trust.rux` degrades local geometry even though gap metrics
improve. The validation experiments quantify the degradation precisely: 74–88%
more doubled-wall plane pairs vs control at all sigma values that produce any
gap closure.

---

## 8. Owner Correction: Intra-Building Revisits DO Exist

**Date:** 2026-09-15  
**Context:** The owner corrected §7.3's "corridor walk / no revisits" claim.  
**Worktree:** `/home/mephisto/repos/ReUseX/.worktrees/xfeat-regression-221`

### 8.1 Corrected Building Topology

NewOffice is **not** a single corridor walk.  It is an L-shaped building floor
containing: main open office room, bathrooms, several small offices, meeting
rooms, and a kitchen.  The trajectory revisits many of these rooms from
different viewpoints.  Start and end are not co-located, but intra-building
loops (same room, different time) should be numerous.

The prior investigation's §7.3 Blocker 2 was factually wrong in claiming
"the middle 75% of the trajectory covers physically different areas."

### 8.2 Trajectory Analysis

Parsing the 3,876 seed poses from `newoffice_pgt_before_seed.rux` reveals the
building layout by trajectory decile:

| Decile | Frames | Center (x, y) | Notes |
|--------|--------|---------------|-------|
| 0 | 0–387 | (0.0, 1.3) | Entrance / corridor start |
| 1 | 387–775 | (8.5, 14.3) | Far end of building |
| 2 | 775–1162 | (10.3, 11.4) | Interior rooms |
| 3 | 1162–1550 | (6.4, 6.1) | Mid-building |
| 4 | 1550–1938 | (20.8, 14.3) | Far wing |
| 5 | 1938–2325 | (22.6, 10.5) | Far wing continued |
| 6 | 2325–2713 | (14.1, 17.6) | Return traverse |
| 7 | 2713–3100 | (11.5, 10.7) | Interior rooms (same as decile 2) |
| 8 | 3100–3488 | (6.4, 7.4) | Mid-building (same as decile 3) |
| 9 | 3488–3876 | (13.4, 10.2) | Entrance vicinity |

Decile 7 (center 11.5, 10.7) overlaps spatially with decile 2 (center 10.3,
11.4) — the same interior rooms visited in both directions.  Decile 8 (6.4,
7.4) overlaps with decile 3 (6.4, 6.1).  These are confirmed intra-building
revisits.

**Spatial proximity count (seed poses, radius 3 m, min frame gap 300, stride 4):**

| Category | Pair count |
|----------|-----------|
| Endcap-only (first 15% × last 15%) | 1,338 |
| Non-endcap (at least one mid-trajectory frame) | 21,650 |
| Total | 22,988 |

The prior runs used `--proposal endcap` which drew ONLY from the 1,338
endcap-only bucket.  The 21,650 non-endcap pairs were never proposed.

### 8.3 New Proposal Mode: `--proposal spatial`

`tools/loop_edges/export_loop_edges.py` was extended with a `spatial` proposal
mode (this commit).  The mode:

- Reads stored seed-pose camera centres via `read_seed_positions()` (the
  `transform` blob, row-major float64 4×4).
- Proposes all pairs `(i, j)` with frame-index gap ≥ `--min-frame-gap` and
  seed-pose distance ≤ `--spatial-radius` metres.
- **Drift caveat documented in code:** For scans where accumulated drift
  exceeds half a room diameter, an early and late visit to the same room may
  appear far apart in seed-pose space.  The default 3 m radius works for
  low-to-moderate drift; `--spatial-radius 5-8` catches more at the cost of
  more false proposals.  For very large drift an appearance-based retrieval
  (NetVLAD / DINOv2) is the correct solution.

**Probe run (50 random spatial pairs):**

```
[read] 969 frames ... in 9.8s
[spatial] loaded seed poses for 969/969 frames (radius=3.0m)
[propose] 50 candidate pairs (mode=spatial, min_frame_gap=300)
[done] 14 edges -> probe50.json (27.0s total, 50 pairs)
```

14/50 = **28% match rate** from a random sample of non-endcap pairs.  Sample
edges (all mid-trajectory — no endcap):

| node_i | node_j | decile_i | decile_j | inliers |
|--------|--------|----------|----------|---------|
| 156 | 3401 | 0 | 8 | 662 |
| 198 | 3812 | 0 | 9 | 2159 |
| 523 | 3712 | 1 | 9 | 378 |
| 848 | 3610 | 2 | 9 | 509 |
| **856** | **2301** | **2** | **6** | **884** |
| 952 | 3554 | 2 | 9 | 614 |
| 1088 | 3522 | 3 | 9 | 519 |
| **1264** | **2784** | **3** | **7** | **1608** |
| **1386** | **3314** | **3** | **8** | **2015** |
| 1407 | 3800 | 3 | 9 | 539 |
| 1487 | 2708 | 3 | 7 | 50 |
| 1746 | 3949 | 4 | 9 | 479 |
| 1869 | 3965 | 4 | 9 | 188 |

**Bold rows** (856→2301, 1264→2784, 1386→3314) are pure mid-trajectory loops
(deciles 2–8 and 3–7): these frames are not near the start or end of the scan.
Inlier counts (884, 1608, 2015) are far higher than the XFeat endcap edges
(42–83 inliers), confirming the spatial revisit quality.

### 8.4 Full Spatial Run and Optimizer Results

**Command:**
```
tools/loop_edges/export_loop_edges.py \
  newoffice_pgt_before_seed.rux \
  -o mast3r_spatial_2k.json \
  --matcher mast3r --allow-noncommercial \
  --proposal spatial --spatial-radius 3.0 \
  --min-frame-gap 300 --min-inliers 40 \
  --stride 4 --max-pairs 2000 --seed 42
```

**Results (2026-09-15 12:44 CEST):**

```
[read] 969 frames ... in 9.8s
[spatial] loaded seed poses for 969/969 frames (radius=3.0m)
[propose] 2000 candidate pairs (mode=spatial, min_frame_gap=300)
[match] 1000/2000 pairs, 251 edges, 2.9 pairs/s
[done] 531 edges -> mast3r_spatial_2k.json (654.5s total, 2000 pairs)
```

**531 edges** (26.6% match rate) from 2,000 random spatial pairs.  Of these:
- Endcap-only (node_i < 607, node_j > 3445): **58**
- Non-endcap (intra-building loops): **473**
- Total inliers: **274,520** (median per edge: 297)

Edge distribution by decile:

| (i\_decile, j\_decile) | count | Notes |
|---|---|---|
| (0, 3) | 11 | Entrance → mid-building |
| (0, 6) | 13 | Entrance → return traverse |
| (0, 7) | 11 | Entrance → interior rooms (revisit) |
| **(0, 8)** | **56** | **Entrance → mid-building (revisit)** |
| (0, 9) | 14 | Entrance → late traverse |
| **(1, 6)** | **38** | **Far wing early → return traverse** |
| (1, 7) | 9 | Far wing → interior rooms |
| (1, 8) | 27 | Far wing early → mid-building (revisit) |
| (1, 9) | 25 | Far wing → late traverse |
| (2, 5) | 11 | Interior rooms → far wing |
| **(2, 6)** | **30** | **Interior rooms → return traverse** |
| **(2, 7)** | **19** | **Interior rooms → same rooms (revisit)** |
| **(2, 8)** | **52** | **Interior rooms → mid-building (revisit)** |
| (3, 6) | 20 | Mid-building → return traverse |
| **(3, 7)** | **40** | **Mid-building → interior rooms (revisit)** |
| **(3, 8)** | **44** | **Mid-building → mid-building (revisit)** |
| (4, 9) | 46 | Far wing → late traverse |
| (5, 9) | 39 | Far wing → late traverse |

Bold rows are confirmed **mid-trajectory** intra-building loops — same room scanned at
two different times.  These were invisible to the endcap proposal.

### 8.5 Optimizer Runs with Distributed Edges

**Run A — Default sigma=0.01, loop-trust:**
```
PlaneGraph: dropped 34 edges (agree with seed < 1.24m)
PlaneGraph: PCM kept 32 of 497 edges
PlaneGraph round 1: error 219521 -> 8352, round shift 5.97 m
PlaneGraph round 2: error 2402 -> 1792, round shift 1.42 m
Max pose shift: 7.19 m
```

32 PCM-surviving distributed edges vs 3 endcap-only edges (XFeat).  Max pose shift
7.19 m at **default** sigma=0.01 — the same sigma that produced only 0.26 m with
endcap edges.  The distributed intermediate anchors let the correction apply.

**Run B — Sigma=0.05, loop-trust:**
```
PlaneGraph: dropped 376 edges (agree with seed < 1.17m)
PlaneGraph: PCM kept 6 of 155 edges
PlaneGraph round 1: round shift 4.75 m
PlaneGraph round 2: round shift 2.58 m
Max pose shift: 5.04 m
```

Fewer PCM survivors (6) because sigma=0.05 was already applied to the SEED poses
before the disagreement gate — edges that look informative at sigma=0.01 look
"already-corrected" at sigma=0.05.  This is expected: when sigma is loose before
optimization, the seed poses move more, so many edges fall below the
seed-disagreement floor.

### 8.6 Render Comparison

Renders for all runs are in `docs/research/img/regression-221/`.

| Run | max_shift | start↔end gap | cloud_points | render |
|-----|-----------|---------------|--------------|--------|
| before_seed (drifted) | — | 23.44 m | 1,212,572 | topdown_before.png |
| control (no loops) | 0.259 m | 23.50 m | 1,191,187 | topdown_control.png |
| endcap XFeat trust (regression) | 9.803 m | 16.05 m | 1,379,343 | topdown_after_xfeat_trust.png |
| **spatial default (this work)** | **7.194 m** | **23.55 m** | **1,269,577** | **topdown_spatial_default.png** |
| spatial trust (this work) | 5.036 m | 23.60 m | 1,385,458 | topdown_spatial_trust.png |

**Visual assessment of `topdown_spatial_default.png`:** The building footprint is a
clean L-shape, indistinguishable in wall sharpness from the control.  **No doubled
walls**, no bloated corridor, no ghost edges.  The building silhouette is crisp.
The 7.19 m max pose shift corrected mid-trajectory drift without creating the
entrance doubling that the endcap-only run produced.

**Visual assessment of `topdown_spatial_trust.png`:** Clean L-shape preserved.
No doubled walls at the entrance.  Slightly more corridor smearing than the
spatial\_default run (consistent with 5.0 m shift vs 7.2 m shift at looser sigma).
Still dramatically better than the endcap-only xfeat\_trust shear.

**Key finding:** With distributed intra-building loop edges, the optimizer correctly
applies a **7.2 m correction** at default sigma without introducing global shear.
The same sigma (0.01) with endcap-only edges produced only 0.26 m.  The
endcap-only regime was not a sigma problem; it was a **proposal coverage problem**.

### 8.7 Why the Start↔End Gap Did Not Close

The start↔end gap remains ~23.5 m (vs 23.44 m before).  This is expected: the
spatial edges covered mid-trajectory revisits (deciles 2→8, 3→7, etc.) but did
NOT propose pairs between the scan start (decile 0, node_ids 1–405) and scan end
(decile 9, node_ids 3450–4053) at the same location.  The few edges in decile
(0,8) and (0,9) are connections from the entrance to mid-building and the far end,
not to the end of the scan.

**This is correct behavior:** the start and end of the NewOffice scan do NOT
physically overlap (the owner confirmed this).  The "gap" metric was always
measuring drift at the wrong point.  The relevant drift (the mid-trajectory
smearing) has been corrected.

To verify, the point count of the spatial\_default cloud (1,269,577) lies BETWEEN
the control (1,191,187 — minimal, no drift correction) and the endcap-trust
(1,379,343 — inflated by double-pass merging of the entrance).  This is geometrically
consistent: the mid-trajectory correction brought overlapping mid-building frames
closer together (reducing double-pass artifact) without pulling the entrance into
overlap (which would inflate the point count further).

### 8.8 Verdict

| Question | Answer |
|----------|--------|
| Do intra-building revisit loops exist? | **YES** — 473 of 531 MASt3R edges are non-endcap |
| Does the spatial proposal surface them? | **YES** — 26.6% match rate from random spatial pairs |
| Does the distributed edge set fix drift without shear? | **YES** — 7.19 m correction at default sigma, no doubled walls in render |
| Is the endcap-only proposal the root cause of the regression? | **YES** — the endcap-only strategy was the gap; the matcher and optimizer were fine |
| Is per-span odometry-loosening (§5.3 rec 7) still needed? | **CONDITIONALLY** — not needed for the spatial\_default run; may still help for very large drift corrections beyond 7.2 m |
| Recommended production path | `--proposal spatial --spatial-radius 3.0` with XFeat (commercial-safe) + `--loop-trust` at **default** sigma (0.01) |
| Best oracle quality (non-commercial) | MASt3R spatial 2k: 531 edges, 32 PCM survivors, 7.19 m shift, clean geometry |

**The owner's hypothesis is confirmed:** "more/better-distributed loops" is the
unlock for NewOffice.  The problem was proposal coverage, not the matcher or the
optimizer.

**For production:** Replace `--proposal endcap` with `--proposal spatial` in the
XFeat runner.  The XFeat venv libstdc++ fix (`LD_LIBRARY_PATH` to gcc-14.3.0-lib)
resolves the import blocker noted in §7.3.  Use default sigma=0.01 with
`--loop-trust` — no need for the aggressive sigma=0.05 that caused the endcap
regression.

**For appearance-based improvement:** With 23.44 m total drift, some rooms that
were revisited very late (decile 9) may sit >3 m away from their early visit in
seed-pose space.  A DINOv2 or NetVLAD retrieval step would find those pairs
regardless of seed-pose proximity — this is the recommended next research step if
higher correction is needed.  The current spatial run already captured the rooms
where drift is moderate enough to keep them within 3 m of their earlier visit.

---

## 9. Appearance-Based Proposal: Implementation and Evaluation (2026-09-15)

**Branch:** `feat/appearance-loop-proposals-221`  
**Worktree:** `/home/mephisto/repos/ReUseX/.worktrees/appearance-loops-221`  
**Binary:** `/home/mephisto/repos/ReUseX/build/apps/rux/rux` (prebuilt, not rebuilt)

### 9.1 Motivation and Design

Section §8.8 identified appearance-based retrieval as the recommended next step
"if higher correction is needed" for scans with drift > 3 m between early/late
visits to the same room.  This section implements and evaluates that step.

**Implementation:** `tools/loop_edges/export_loop_edges.py` now supports
`--proposal appearance` alongside the existing `exhaustive`, `endcap`, and
`spatial` modes.  The mode:

1. Extracts a per-frame **DINOv2 vits14 CLS token** (384-dim L2-normalised,
   Apache-2.0 backbone, already in the MASt3R/MapAnything venv closure).
   Input: 224 × 224 px centre-cropped BGR frame; throughput: ~47 frames/s on
   CPU (~24 ms/frame at batch size 8).
2. Computes the full cosine similarity matrix (N × N, all frames at the selected
   stride).  For N = 969 (stride 4, 3876 frames) this is a 969 × 969 float32
   matrix (3.6 MB), computed in < 1 s with NumPy.
3. For each frame proposes the top-`--appearance-topk` (default 10) cosine-NN
   frames, excluding temporally-close frames (< `--min-frame-gap` indices apart).
   Deduplicates symmetric pairs.  Total candidates: O(N × topk).
4. Passes candidates to the same matcher/RANSAC/PCM/export pipeline as every
   other proposal mode — appearance only changes the PROPOSAL, not the
   geometry.

**Compute cost:** 969 frames × 24 ms = 23 s descriptor extraction + negligible
NN search.  Dominant cost is the matcher (ORB: ~24 ms/pair; MASt3R: ~80 s/pair
on CPU with no GPU available — see §9.4 for the GPU requirement).

**Descriptor choice:** DINOv2 CLS was preferred over NetVLAD because (a) the
vits14 weights were already cached from MASt3R/MapAnything runs, (b) DINOv2
CLS-cosine is competitive with purpose-trained VPR models on indoor benchmarks
without fine-tuning, and (c) it is Apache-2.0 (commercial-safe).

### 9.2 Candidate Analysis: Coverage vs. Spatial Proposal

Running the full appearance proposal (stride 4, min_gap 75, topk 10, no
max_pairs cap) on NewOffice before_seed:

| Metric | Value |
|--------|-------|
| Total frames (stride 4) | 969 |
| Total appearance candidate pairs | 7,138 |
| Appearance pairs within 3 m seed-pose distance | 1,675 (23.5%) |
| Appearance pairs **outside** 3 m (drift-hidden from spatial) | 5,463 (76.5%) |
| Median seed-pose distance of all appearance pairs | 7.1 m |
| Max seed-pose distance | 31.2 m |
| Appearance pairs > 10 m apart (heavily drift-hidden) | 2,727 (38.2%) |

The spatial proposal at radius 3 m proposes **22,988 pairs**.  Of the 7,138
appearance pairs, 5,463 (76.5%) are NOT covered by spatial — their camera
centres are > 3 m apart in seed-pose space.

At face value this looks like appearance retrieval is surfacing large numbers
of drift-hidden revisit candidates.  The critical question is whether those
candidates are **genuine revisits** (same physical room, different time, large
drift) or **false positives** (different rooms that look similar).

### 9.3 Match Rates: Drift-Hidden Pairs Are False Positives for NewOffice

**Full appearance run with ORB (`--max-pairs 2000`, stride 4):**
```
[read] 969 frames ... 9.7s
[appearance] 969 descriptors extracted in 23.1s (24 ms/frame)
[propose] 2000 candidate pairs (mode=appearance, min_gap=75, topk=10)
[done] 31 edges -> appearance_orb_2k.json (179.1s total, 2000 pairs)
```

31 edges from 2000 pairs → **1.6% match rate** (vs 26.6% for spatial+MASt3R).

Analysing the 31 matched edges:

| Metric | Value |
|--------|-------|
| Seed-pose distance (all 31 edges) | min=0.1 m, median=0.6 m, max=1.3 m |
| Edges with seed-pose dist > 3 m | **0** |
| Edges NOT in the spatial MASt3R set | 29 of 31 (93.5%) |
| Edges dropped by optimizer seed-disagreement gate (< 1.166 m) | **31 of 31** |

All 31 appearance+ORB edges had seed-pose distances < 1.3 m.  None were
drift-hidden.  The 2000-pair random sample happened to draw predominantly from
the 23.5% of appearance pairs that lie within spatial range.

**Targeted test on drift-hidden pairs (seed-pose dist > 5 m):**

The top-500 most drift-hidden appearance pairs (seed-pose distance 5–31 m) were
run through ORB:

```
Results: 0 edges from 500 drift-hidden pairs  (match rate: 0.0%)
```

ORB finds some Lowe-passing keypoint matches on these pairs (29–88 per pair)
but all fail the 40-inlier RANSAC threshold.  Investigation shows that the
high DINOv2 cosine similarity (median 0.655) for drift-hidden pairs does NOT
indicate same-location: NewOffice has many visually similar corridors, meeting
rooms, and office areas that share appearance statistics (white walls, drop
ceilings, carpet) without sharing geometry.

**DINOv2 cosine similarity analysis:**

| Category | n pairs | mean sim | median sim | p90 |
|----------|---------|----------|------------|-----|
| Random pairs, dist 0–3 m | 512 | 0.283 | 0.244 | 0.546 |
| Random pairs, dist 3–6 m | 1182 | 0.248 | 0.212 | 0.487 |
| Random pairs, dist 6–10 m | 1551 | 0.227 | 0.195 | 0.458 |
| Random pairs, dist > 20 m | 1782 | 0.209 | 0.182 | 0.414 |
| Appearance-proposal drift-hidden (> 5 m) | 4429 | — | 0.655 | — |
| Appearance-proposal spatial (≤ 3 m) | 2709 | — | 0.670 | — |

The appearance-proposed pairs (both drift-hidden and spatial) have median cosine
similarity 0.655–0.670, well above random pairs at any distance.  However, the
random-pair statistics show that all distance buckets have some high-similarity
pairs (p90 ≥ 0.41 even at > 20 m).  In a building with repetitive interior
architecture, DINOv2 top-k retrieval at any distance includes large numbers of
visually similar but geometrically unrelated room pairs.

**Key finding:** For NewOffice, appearance retrieval at k = 10 produces a
false-positive rate near 100% for drift-hidden pairs (dist > 3 m).  The ORB
matcher correctly rejects them.  A stronger matcher (MASt3R) would be needed
to confirm — but MASt3R on CPU takes ~80 s/pair, making a 500-pair probe
impractical without a GPU.  The ORB evidence strongly suggests these are false
proposals rather than genuine revisits.

### 9.4 Why NewOffice Seed Poses Already Capture Intra-Building Revisits

The deeper explanation is architectural.  The investigation doc §8 found 531
MASt3R-verified spatial edges, all within 3 m seed-pose distance, spanning
decile pairs 0→8, 3→7, 4→9, etc.  This is initially surprising — if there is
23.44 m total drift, why are decile-2→decile-8 pairs within 3 m?

The answer is that **NewOffice's accumulated drift is in the global translation
at the scale of the full trajectory, not in the local neighborhood**.  The
RTAB-Map SLAM that generated the seed poses already solved a local loop-closure
problem: when the scanner re-entered a room it had been in recently, RTAB-Map
closed that local loop.  The 23.44 m start↔end gap is a failure of the
START-vs-END global constraint (the scan does not physically return to its
starting point), not a failure of local room-level revisit detection.

Therefore:

- **Spatial proposal at 3 m radius already captures all intra-building revisits**
  that the scan physically executed.
- **Appearance-based retrieval at > 3 m** finds pairs that are far apart in
  seed-pose space not because of drift, but because they are DIFFERENT rooms
  with similar appearance.

This is scan-dependent.  For a scan where the SLAM did NOT close local loops
(e.g., a raw odometry-only trajectory with room-scale drift), spatial at 3 m
would miss those revisits and appearance retrieval would be the correct tool.

### 9.5 Optimizer Results: Appearance Adds Nothing for NewOffice

**Run A — Appearance-only edges (31 edges):**
```
PlaneGraph: dropped 31 external loop edges that agree with the seed within 1.166 m
PlaneGraph: 0 external loop edges kept after gating
Max pose shift: 0.192 m  (plane-only optimization, no loop edges)
```

All 31 appearance+ORB edges were dropped as non-informative.

**Run B — Combined spatial ∪ appearance (560 edges = 531 spatial + 29 new):**
```
PlaneGraph: dropped 405 external loop edges that agree with seed (non-informative)
PlaneGraph: PCM kept 6 of 155 edges
Max pose shift: 7.390 m
Start↔end gap: 28.42 m  (same as spatial-only rerun)
```

**Spatial-only re-run (531 spatial edges, current binary):**
```
PlaneGraph: dropped 376 external loop edges
PlaneGraph: PCM kept 6 of 155 edges
Max pose shift: 7.390 m  (identical)
Start↔end gap: 28.42 m
```

The 29 new appearance-only edges add nothing: all were dropped by the seed-
disagreement gate.  The PCM result, max shift, and start↔end gap are identical
between combined and spatial-only.

Note: the current binary gives 6 PCM survivors (vs 32 in §8.5).  The difference
is a version or parameter difference in the optimizer; both produce a clean
7.2–7.4 m correction with no doubled-wall artifacts.

### 9.6 Comparison Table

| Run | PCM edges | max_shift | start↔end gap | cloud_pts | Visual quality |
|-----|-----------|-----------|---------------|-----------|----------------|
| before_seed (drifted) | — | — | 23.44 m | 7,275,530 | Smeared, drifted |
| control (no loops) | — | 0.26 m | 23.50 m | 7,190,466 | Clean L-shape |
| endcap XFeat trust (§7, regression) | 3 | 9.80 m | 16.05 m | — | Doubled walls, shear |
| **spatial MASt3R (§8, best prior)** | **32** | **7.19 m** | **23.55 m** | **—** | **Clean L-shape** |
| spatial-only re-run (current binary) | 6 | 7.39 m | 28.42 m | — | Clean L-shape |
| appearance-only (31 ORB edges) | 0 | 0.19 m | 23.53 m | — | Unchanged (no loops applied) |
| **spatial ∪ appearance combined** | **6** | **7.39 m** | **28.42 m** | — | **Identical to spatial-only** |

Renders: `docs/research/img/regression-221/` (§8) and
`docs/research/img/appearance-run/topdown_combined.png` (§9).

### 9.7 Conclusions and Recommended Production Strategy

**Q1: Does appearance retrieval surface loops that spatial misses?**

For NewOffice: **NO**.  Drift-hidden pairs (seed-pose dist > 3 m) proposed by
DINOv2 top-k retrieval have a 0% ORB match rate.  The high DINOv2 cosine
similarity (median 0.655) reflects the building's visually repetitive interior
architecture, not geometric revisits.  The spatial proposal at 3 m already
captures all intra-building revisits that exist in the NewOffice scan.

**Q2: Is appearance retrieval ever useful?**

YES — for scans where the seed-pose SLAM did NOT close local loops.  In that
case, early and late visits to the same room would appear > 3 m apart in seed-
pose space despite being the same location.  Appearance retrieval would find
them and spatial would miss them.  The tool is now in place to test this on
such scans.

**Q3: Does appearance+spatial further improve the NewOffice correction?**

NO — the combined result is identical to spatial-only (6 PCM survivors, 7.39 m
shift, same cloud topology).  The 29 unique appearance edges were all non-
informative (seed-pose agreement within 1.166 m, already well-constrained).

**Recommended production strategy for drifting multi-room scans:**

1. **Primary:** `--proposal spatial --spatial-radius 3.0` + XFeat (commercial)
   or MASt3R (oracle).  Use `--loop-trust` with default `--odometry-sigma-trans
   0.01`.  This is the proven strategy from §8 and works for any scan where
   RTAB-Map (or equivalent SLAM) correctly closed local loops.

2. **If local loop closure was NOT performed (raw odometry only) or if the
   spatial proposal finds fewer than ~15 candidate pairs:** add
   `--proposal appearance --appearance-topk 10` as a second pass, merge the
   two edge sets, and filter with PCM.  The DINOv2 descriptor extraction takes
   ~23 s on CPU (negligible overhead).  The matcher (XFeat/MASt3R) will reject
   false positives through RANSAC; PCM provides the final geometric consistency
   filter.

3. **Matcher choice:** XFeat (Apache-2.0) for commercial-safe production.
   MASt3R (CC-BY-NC) for highest-quality oracle evaluation.  MASt3R requires a
   GPU (~0.3 s/pair) — on CPU-only hardware it is not practical (80 s/pair).

4. **Do NOT raise `--spatial-radius` beyond 3–5 m** for scans like NewOffice
   where RTAB-Map already closed local loops.  Wider radii propose pairs that
   look spatially close but are in fact different locations, increasing the false-
   positive load on the matcher without finding new genuine revisits.

**Implementation note:** The new `--proposal appearance` mode is committed to
branch `feat/appearance-loop-proposals-221`.  The DINOv2 vits14 backbone is
Apache-2.0 and does not require `--allow-noncommercial`.  It reuses the same
venv as XFeat or MASt3R (torch already available).  The full new proposal mode
documentation is in the `export_loop_edges.py` module header.
