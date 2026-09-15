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
