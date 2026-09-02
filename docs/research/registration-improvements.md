<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Improving Pose Registration in ReUseX: ML Loop Closure & Wide-Baseline Alignment

Decision-ready research report for issue
[#221](https://github.com/pfmephisto/ReUseX/issues/221).
Scope: **offline** refinement of per-frame poses from iPad LiDAR captures of
building interiors (238-1600 frames/scan, ARKit/RTABMap seed poses). Not
real-time.

Grounded in the measured failures from the #221 sweep campaign:

- **Basin problem.** Naive spatial pairing of distant frames failed: loop-closure
  pairs start *outside* the ICP correspondence basin and inject noise. Residual
  gating did not help — it removed exactly the informative (drifted) constraints.
- **Metric saturation.** Frame-to-frame point-to-plane JPR converges to ~11 mm
  internal RMS in *every* config, while downstream plane `flatness_rms` varies
  18-23 mm. Pairwise point-to-plane energy does **not** determine global
  consistency; the remaining ~8 mm of headroom (vs ~10 mm sensor floor) will not
  come from tuning this objective.
- **Best current result:** `rux register --prior-weight 0.1 --neighbor-window 10
  --iterations 50` → flatness 17.7 mm, thickness_p90 27.4 mm (-18% vs no
  refinement).

Every recommendation below names the measured problem it addresses.

---

## 1. Executive summary (5 bullets)

- **The two measured failures are different problems needing different tools.**
  The *basin problem* is a **front-end** failure (we can't find good wide-baseline
  correspondences), and *metric saturation* is a **back-end/objective** failure
  (frame-to-frame point-to-plane can't express global consistency). Fixing one
  without the other will not move the GT/flatness numbers.

- **The highest-leverage, lowest-risk win is the back-end already planned in
  #221: a GTSAM pose-graph with plane-landmark factors + GNC.** GTSAM 4.2.1 is
  *already packaged* (MIT), ships `OrientedPlane3Factor` and built-in Graduated
  Non-Convexity (`GncOptimizer`). This directly attacks metric saturation (it
  optimizes global plane consistency, which is what `analyze quality` measures)
  **and** makes bad loop closures survivable by construction (GNC down-weights
  outlier edges), neutralizing the basin problem's blast radius.

- **For loop-closure *detection*, do not over-engineer.** For single-building
  indoor scans of 238-1600 frames, an all-pairs / retrieval-shortlisted image
  descriptor is cheap and sufficient. Use a **commercial-safe** global descriptor
  (SALAD or CosPlace/EigenPlaces, both permissive) or the MASt3R-SfM-style
  "foundation model as retriever" trick. Point-cloud place recognition
  (ScanContext, LoGG3D, BEVPlace2) is **LiDAR-scan-scale tech and a poor fit** for
  small indoor RGB-D frames — skip it.

- **For wide-baseline *relative pose* (the actual basin fix), the decisive
  question is licensing, not accuracy.** MASt3R/DUSt3R/MASt3R-SfM give the best
  metric-scale relative poses from raw pairs but are **CC-BY-NC (non-commercial)
  — GPL/commercial-unsafe**. The commercial-safe path is **EfficientLoFTR
  (Apache-2.0) or LightGlue (Apache-2.0) → matches → PnP/Kabsch with our existing
  depth → metric relative pose**. Note: **our bundled `superpoint.pt` is under a
  non-commercial license** and should be treated as tainted; prefer DISK/ALIKED or
  the Apache LightGlue+detector stack. **VGGT** now has a **commercial checkpoint
  (July 2025, application-gated)** and is the one foundation-model option worth a
  commercial pilot.

- **Recommended phasing tied to #221:** (P1) build the GTSAM plane-factor +
  GNC back-end reusing JPR's surfel/plane extraction — this is the plateau-breaker
  the sweep pointed to and needs no ML; (P2) add commercial-safe image retrieval
  to *propose* loop candidates and EfficientLoFTR+depth+PnP to *turn them into
  metric edges*, fed into the same GNC graph so bad edges are harmless; (P3)
  optional VGGT/MASt3R evaluation as an accuracy ceiling probe on the MuSHRoom GT.

---

## 2. Findings per research question

### Q1 — Loop-closure detection (which frame pairs to constrain)

**Scale reality check.** Our scans are 238-1600 frames of one building interior.
Brute-force all-pairs descriptor comparison is O(N²) but N is small and
descriptors are ~cheap; even N=1600 is 1.3M dot-products of 512-4096-d vectors —
milliseconds. So *place recognition here is a shortlist/retrieval convenience, not
a scaling necessity.* This means we should optimize for **precision on indoor RGB**
and **commercial-safe licensing**, not for LiDAR-scale throughput.

**Image global descriptors (recommended family).**
- **SALAD** (CVPR 2024, "Optimal Transport Aggregation for VPR",
  [code](https://github.com/serizba/salad)) — current SOTA-tier, DINOv2 backbone,
  permissive code. Strong generalization.
- **CosPlace / EigenPlaces** — classification-trained global descriptors;
  EigenPlaces reports R@1 92.5% Pitts30k, 92.4% Tokyo24/7. Permissive.
- **MixVPR** (WACV 2023, [paper](https://arxiv.org/abs/2303.02190)) — R@1 94.6%
  Pitts250k; feature-mixing MLP, easy to export.
- **AnyLoc** — foundation-model (DINOv2) descriptors; **explicitly reported as
  SOTA on indoor benchmarks** (Baidu Mall, 17 Places, Gardens Point),
  outperforming NetVLAD/CosPlace/MixVPR indoors. Best zero-shot indoor option.
- **Indoor caveat (measured in the literature):** VPR models trained on outdoor
  street data suffer a **domain gap** on indoor RGB-D (improper cluster centers),
  which is exactly why the DINOv2-based zero-shot methods (AnyLoc, SALAD) tend to
  win indoors. See NYC-Indoor-VPR
  ([arXiv 2404.00504](https://arxiv.org/pdf/2404.00504)) and the VPR-for-3D-vision
  evaluation ([arXiv 2603.13917](https://arxiv.org/pdf/2603.13917)).
- **Runtime:** a single global descriptor forward pass is a few ms/frame on GPU;
  building the shortlist for a 1600-frame scan is seconds total.
- **ONNX-exportability:** all of these are ResNet/DINOv2 CNN+MLP graphs (no exotic
  ops) → ONNX/TensorRT export is routine, fitting ReUseX's existing
  ONNX/TensorRT backends.

**"Foundation model as free retriever" (MASt3R-SfM trick).** MASt3R-SfM uses its
own encoder features for retrieval "without any overhead," cutting the pairing
from quadratic to linear. Elegant if we adopt MASt3R anyway, but **NC-licensed**
(see Q2). Not recommended as the retrieval mechanism for a commercial product.

**Point-cloud place recognition (NOT recommended for us).**
- **ScanContext** (handcrafted polar BEV descriptor) — rotation-invariant, cheap,
  no training. But it is designed for **360° LiDAR sweeps**; a single iPad depth
  frame is a narrow frustum, not a panoramic scan, so the polar descriptor is
  ill-posed per frame. Could apply to *accumulated* per-room submaps, not frames.
- **LoGG3D-Net** (ICRA, sparse-conv global descriptor) and **BEVPlace2**
  (T-RO 2025, [code](https://github.com/zjuluolun/BEVPlace2)) — both strong, both
  **built and trained for outdoor LiDAR (KITTI-scale)**. Domain mismatch with
  indoor iPad depth is severe; retraining cost is not justified when image
  retrieval already solves detection cheaply.
- **Verdict:** point-cloud PR buys us nothing our RGB frames don't already give us
  more cheaply. Skip.

**Q1 recommendation:** cheap image-descriptor retrieval to build a **loop-candidate
shortlist** (top-k per frame beyond the temporal window). Start with **AnyLoc or
SALAD** for best indoor precision; both commercial-safe, both ONNX-exportable.
This *proposes* pairs; it does **not** by itself fix the basin problem — that is
Q2's job.

### Q2 — Wide-baseline relative pose (the actual basin fix)

This is the direct antidote to the **basin problem**: instead of hoping ICP finds
correspondences across a large baseline, compute a metric relative pose from a
learned matcher/regressor and inject it as a pose-graph edge, so ICP/JPR is never
asked to cross the basin.

**Two-view matcher + depth → PnP/Kabsch (recommended, commercial-safe).**
- **EfficientLoFTR** (CVPR 2024, [code](https://github.com/zju3dv/EfficientLoFTR),
  **Apache-2.0**, now in HF Transformers) — detector-free semi-dense matches,
  ~2.5× faster than LoFTR. Outputs **2D matches only**; we lift to metric relative
  pose using **our stored per-frame depth** (back-project matched pixels in both
  frames → 3D-3D Kabsch/Umeyama, or 2D-3D PnP + RANSAC). Because we have RGB-D,
  scale is metric and unambiguous — no scale-from-SfM headache.
- **LightGlue + detector** (Apache-2.0 code & weights,
  [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX), TensorRT/OpenVINO
  export proven) — sparse, extremely fast, ONNX/TensorRT-ready. **License trap:**
  the classic **SuperPoint detector is non-commercial** — and *our bundled
  `superpoint.pt` inherits that restriction*. For a commercial-safe stack pair
  LightGlue with **DISK or ALIKED** (permissive) detectors, which LightGlue
  officially supports.
- Both are RGB-only matchers; **metric scale comes from our depth**, which is the
  key advantage of being an RGB-D pipeline. This is the pragmatic, license-clean
  basin fix.

**Pointmap-regression foundation models (best accuracy, licensing-gated).**
- **DUSt3R / MASt3R** (NAVER, [MASt3R](https://github.com/naver/mast3r)) — regress
  metric pointmaps from an image pair → **direct metric relative pose**, RGB-only,
  robust to very wide baselines and low overlap (exactly our failure mode).
  **License: CC-BY-NC-SA 4.0 (non-commercial), and the metric checkpoint carries
  extra dataset restrictions (mapfree).** GPL/commercial-unsafe. Runtime ~198
  ms/pair on A40 (original), ~91 ms/pair with **Speedy MASt3R**
  ([arXiv 2503.10017](https://arxiv.org/abs/2503.10017)).
- **MASt3R-SfM** (3DV 2025 Best Student Paper,
  [arXiv 2409.19152](https://arxiv.org/pdf/2409.19152)) — full pipeline:
  retrieval → per-edge local reconstruction → global 3D alignment → global BA,
  exports **COLMAP-style poses**. Architecturally this is *exactly* the pipeline
  #221 wants (retrieval + wide-baseline edges + global optimization). Same **NC
  license** ceiling.
- **VGGT** (CVPR 2025 Best Paper, [code](https://github.com/facebookresearch/vggt))
  — feed-forward transformer, one→hundreds of views → extrinsics, intrinsics,
  depth, pointmaps in <1 s. **Critically: a commercial checkpoint
  `VGGT-1B-Commercial` exists (July 2025, application-gated, excludes military).**
  This is the *only* foundation-model option with a plausible commercial path.
  Worth a pilot as an accuracy ceiling and as a candidate wide-baseline edge
  generator; GPU memory is the main cost (recent May-2026 optimizations give
  2-3× more frames per memory budget).

**Learned point-cloud registration (alternative to image matching).**
- **GeoTransformer** (CVPR 2022 / TPAMI 2023,
  [code](https://github.com/qinzheng93/GeoTransformer), **MIT**) — SOTA indoor
  registration on 3DMatch (2.5 cm voxel, exactly indoor RGB-D scale). Correspondence-
  free coarse-to-fine; robust at low overlap. Commercial-safe. No ONNX out of the
  box (custom sparse ops), so it'd run as a libTorch module — heavier integration.
- **PREDATOR** — designed for **low-overlap** pairs; conceptually the right tool
  for distant loop pairs, but GeoTransformer supersedes it on the same benchmark.
- **FCGF features + TEASER++**
  ([TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus), **MIT**) —
  **certifiably robust** estimator, tolerates 99% outlier correspondences, C++
  (Eigen+Boost), ~0.8 s for ~1900 correspondences with 1700 outliers, **stable
  C++ API**. TEASER++ needs *correspondences as input* (from FPFH — classical,
  no-license-issue — or FCGF — check weight license). This is a strong
  license-clean, no-Python fallback that slots directly into a C++ pipeline and is
  robust to the bad correspondences the basin problem produces.

**Q2 recommendation:** commercial-safe primary = **EfficientLoFTR (or
LightGlue+ALIKED) matches + our depth → RANSAC-PnP/Kabsch → metric relative-pose
edge.** Robust classical fallback with no Python and permissive license =
**FPFH/FCGF + TEASER++ (MIT, C++).** Reserve **VGGT-Commercial** and **MASt3R**
(NC, research-only) for an accuracy-ceiling evaluation on GT, not for the shipping
pipeline.

### Q3 — Robust back-end (this is where metric saturation is actually solved)

The sweep proved the objective is the problem: frame-to-frame point-to-plane RMS
is decoupled from global plane consistency. The fix is to **change what is
optimized** — from pairwise NN residuals to **shared global plane landmarks** —
and to make the back-end **outlier-robust** so wide-baseline loop edges (which will
sometimes be wrong) cannot corrupt the solution.

**GTSAM is already in the flake (MIT, 4.2.1) and has exactly the needed pieces:**
- **`OrientedPlane3Factor`** — pose-to-plane landmark factor. A plane is an
  `OrientedPlane3` (normal + distance); each frame that observes a persistent
  plane contributes a factor. This **directly optimizes plane consistency = the
  `analyze quality` metric**, attacking metric saturation at the objective level.
  (A single plane measurement doesn't fully constrain a pose, so combine with
  odometry/prior factors — well documented,
  [reference test](https://github.com/rising-turtle/graph_slam/blob/master/gtsam/test/testOrientedPlane3Factor.cpp).)
- **`GncOptimizer` (Graduated Non-Convexity)** — built-in outlier-robust
  optimization. Loop-closure edges from Q2 that are wrong get automatically
  down-weighted. **This is the principled replacement for the residual-gating that
  failed in the sweep**: GNC keeps informative-but-large residuals early
  (non-convex → convex schedule) instead of hard-cutting them. Also relevant:
  Efficient/Adaptive GNC variants ([arXiv 2310.06765](https://arxiv.org/abs/2310.06765),
  [arXiv 2308.11444](https://arxiv.org/pdf/2308.11444)) if GNC iteration count
  becomes a bottleneck.
- Alternatives to GNC that GTSAM also supports conceptually: **switchable
  constraints** and **dynamic covariance scaling** — but GNC is built-in and is
  the current best-practice default.

**How modern RGB-D pipelines structure this (reference architectures):**
- **Open3D multiway registration** — the canonical template: pose graph with
  **odometry edges** (temporal neighbors, ICP) + **loop-closure edges**
  (non-neighbors, global registration, less reliable), then a two-pass
  `global_optimization` with a **line-process** that prunes false loop edges.
  This is essentially GNC-by-another-name and validates the whole architecture.
- **HLoc** — retrieval (image descriptors) → local feature matching → PnP; the
  standard "retrieval + matcher + geometric verify" recipe we mirror in Q1+Q2.
- **BundleFusion** — global pose optimization over RGB-D with sparse+dense terms;
  historical proof that global (not pairwise) optimization is what yields metric
  consistency.

**Q3 recommendation:** build a GTSAM factor graph = seed-pose priors + temporal
odometry factors (JPR-quality) + **`OrientedPlane3Factor` plane landmarks** +
wide-baseline loop factors (Q2), solved with **`GncOptimizer`**. This is the
plateau-breaker #221 already identified, now with concrete GTSAM primitives.

### Q4 — Pragmatic minimal high-impact pipeline for ReUseX

Given assets already integrated (TensorRT/libTorch/ONNX backends, GTSAM packaged,
PCL/CGAL, existing JPR surfel+plane extraction, stored per-frame depth + poses):

**Minimal pipeline (each stage maps to a measured problem):**

1. **Plane extraction & association** *(metric saturation)* — reuse JPR's surfel
   extraction to detect persistent planes; associate observations across frames
   into shared `OrientedPlane3` landmarks. Effort ~**3-5 d** (association is the
   fiddly part; can bootstrap from existing CGAL plane segmentation).
2. **GTSAM back-end with plane factors + GNC** *(metric saturation + makes bad
   loops safe)* — assemble the graph, solve with GNC, write refined poses back.
   Effort ~**4-6 d** (GTSAM already packaged; wiring + gauge/prior handling +
   write-back mirrors existing `refine_sensor_poses`). **This P1 alone should move
   flatness/thickness and is ML-free.**
3. **Loop-candidate retrieval** *(basin problem, step 1 of 2)* — AnyLoc or SALAD
   global descriptor, top-k shortlist beyond temporal window. Export to
   ONNX/TensorRT. Effort ~**2-3 d** (fits existing backend; mostly plumbing).
   License: commercial-safe.
4. **Wide-baseline metric edges** *(basin problem, step 2 of 2)* —
   EfficientLoFTR/LightGlue matches on shortlisted pairs + stored depth →
   RANSAC-PnP/Kabsch → metric relative-pose factor into the GNC graph. Effort
   ~**4-6 d** (matcher export + depth back-projection + robust PnP + edge
   covariance calibration). License: commercial-safe (with ALIKED/DISK detector,
   NOT the bundled SuperPoint).
5. **Local polish** *(final mm)* — existing JPR as a post-GNC refinement on the
   now-globally-consistent poses. Effort ~**1 d** (already exists).

**Licensing summary (commercial safety):**
- Commercial-safe: GTSAM (MIT), TEASER++ (MIT), GeoTransformer (MIT),
  EfficientLoFTR (Apache-2.0), LightGlue code+weights (Apache-2.0), SALAD/CosPlace/
  EigenPlaces/MixVPR (permissive), VGGT-1B-**Commercial** (application-gated).
- **NOT commercial-safe (research-only):** MASt3R/DUSt3R/MASt3R-SfM
  (CC-BY-NC-SA), classic **SuperPoint** detector+weights (non-commercial) — **this
  taints our bundled `superpoint.pt`; do not ship it in a commercial pose
  pipeline.**
- **GPL note:** ReUseX is GPL-3.0-or-later. MIT/Apache/BSD deps are compatible
  (they can be combined into GPL). NC-licensed models are *usage*-restricted
  regardless of GPL and must be excluded from any commercial deliverable.

**GPU/runtime cost per scan (order-of-magnitude, 1600 frames):**
- Retrieval descriptors: ~few ms/frame → ~seconds/scan.
- Shortlist matching (say top-8 loops/frame beyond window): ~12.8k pairs ×
  ~10-50 ms (EfficientLoFTR/LightGlue) → **minutes**, GPU-bound but offline-fine.
- GTSAM GNC solve: seconds-to-low-minutes for a graph of 1600 poses + a few
  hundred plane landmarks + loop edges (sparse, CPU).
- (If VGGT/MASt3R pilot: ~0.1-0.2 s/pair, GPU-memory-bound — feasible offline.)

---

## 3. Ranked recommendation table

Impact = expected effect on `flatness_rms`/`thickness_p90` and GT accuracy.
Effort = integration days. License = commercial safety.

| Rank | Approach | Measured problem addressed | Expected impact | Effort | Risk | License |
|------|----------|----------------------------|-----------------|--------|------|---------|
| 1 | **GTSAM pose-graph: `OrientedPlane3Factor` plane landmarks + `GncOptimizer`** (reuse JPR plane extraction) | Metric saturation (changes the objective to global plane consistency) + makes bad loops safe | **High** — directly optimizes the metric `analyze quality` measures; #221's identified plateau-breaker | 7-11 d | Low (GTSAM packaged, primitives exist; plane association is the unknown) | MIT ✅ |
| 2 | **Loop-candidate retrieval (AnyLoc / SALAD) → shortlist** | Basin problem (proposes which pairs to constrain) | Medium (enabler for #3; no direct metric gain alone) | 2-3 d | Low | Permissive ✅ |
| 3 | **EfficientLoFTR / LightGlue+ALIKED + depth → RANSAC-PnP/Kabsch → metric loop edges** | Basin problem (crosses the baseline without ICP) | **High** — supplies the wide-baseline constraints ICP/JPR provably cannot | 4-6 d | Medium (edge covariance calibration; GNC absorbs bad edges) | Apache-2.0 ✅ (avoid SuperPoint) |
| 4 | **FPFH/FCGF + TEASER++** (C++, robust fallback for #3) | Basin problem (certifiably-robust wide-baseline reg.) | Medium-High | 3-5 d | Low-Med (C++/MIT, no Python; FCGF weight license TBD) | MIT ✅ |
| 5 | **VGGT-1B-Commercial** (feed-forward multi-view poses/pointmaps) | Basin problem + accuracy ceiling | Medium-High (pilot/ceiling probe) | 5-8 d | Medium (application-gated weights, GPU memory) | Commercial ckpt ⚠️ (gated) |
| 6 | **GeoTransformer** (learned indoor PC registration) | Basin problem (alt. to image matching) | Medium | 5-8 d | Medium (libTorch module, no ONNX, custom ops) | MIT ✅ |
| 7 | **MASt3R / MASt3R-SfM** (metric pointmap → direct pose) | Basin problem (best raw accuracy) | High accuracy — **research/eval only** | 4-6 d | Med + **license blocker** | CC-BY-NC ❌ |
| 8 | Point-cloud place recognition (ScanContext / LoGG3D / BEVPlace2) | Loop detection | **Low for us** (LiDAR-scale, indoor RGB-D domain mismatch) | 5-10 d | High (domain gap, retraining) | Mixed |

---

## 4. Phased implementation proposal (tied to #221)

**Phase P1 — Plane-landmark GTSAM back-end (ML-free plateau-breaker).**
*Addresses metric saturation.* Reuse JPR surfel/plane extraction → associate
persistent planes into `OrientedPlane3` landmarks → GTSAM graph (seed priors +
temporal odometry + plane factors) solved with `GncOptimizer` → write poses back
via the existing `refine_sensor_poses` path. Gate on `rux analyze quality`
(flatness/thickness) and MuSHRoom GT. **This is the single most important step and
requires no ML.** Effort ~7-11 d. Success criterion: beat the current 17.7 mm /
27.4 mm best.

**Phase P2 — Wide-baseline loop edges (the basin fix), fed into P1's GNC graph.**
*Addresses the basin problem.* (a) AnyLoc/SALAD retrieval shortlist (ONNX/TensorRT);
(b) EfficientLoFTR/LightGlue+ALIKED matches on shortlisted pairs + stored depth →
RANSAC-PnP/Kabsch → metric relative-pose factors added to the *same* GNC graph so
wrong edges are down-weighted, not gated. Effort ~6-9 d. Success criterion: GT
accuracy improves on loopy scans without regressing flatness (GNC guards against
the noise injection the sweep observed).

**Phase P3 — Accuracy-ceiling probe (evaluation only, not shipped).**
Run **VGGT-1B-Commercial** (and, offline/non-commercially, **MASt3R-SfM**) on a few
scans against MuSHRoom GT to measure how much headroom remains above P1+P2. If VGGT
edges materially beat the LoFTR+depth edges, promote VGGT-Commercial to a shipping
edge generator (its commercial checkpoint makes this legally viable, unlike MASt3R).
Effort ~5-8 d, informational.

**Guardrails carried from the sweep:** keep determinism (fixed seeds) so
before/after deltas stay real; every phase reports `flatness_rms`/`thickness_p90`
+ GT; never hard-gate loop residuals — let GNC do outlier handling.

---

## 5. Key sources

- MASt3R (CC-BY-NC): https://github.com/naver/mast3r ·
  paper https://arxiv.org/pdf/2406.09756
- MASt3R-SfM (3DV 2025): https://arxiv.org/pdf/2409.19152
- Speedy MASt3R (91 ms/pair): https://arxiv.org/abs/2503.10017
- VGGT (CVPR 2025 Best Paper; commercial ckpt Jul 2025):
  https://github.com/facebookresearch/vggt · https://vgg-t.github.io/
- EfficientLoFTR (Apache-2.0): https://github.com/zju3dv/EfficientLoFTR
- LightGlue-ONNX (Apache-2.0; SuperPoint NC caveat):
  https://github.com/fabio-sim/LightGlue-ONNX
- SALAD (CVPR 2024): https://github.com/serizba/salad
- MixVPR (WACV 2023): https://arxiv.org/abs/2303.02190
- Indoor VPR / domain gap: https://arxiv.org/pdf/2404.00504 ·
  https://arxiv.org/pdf/2603.13917
- GeoTransformer (MIT): https://github.com/qinzheng93/GeoTransformer ·
  https://arxiv.org/abs/2308.03768
- TEASER++ (MIT, certifiable): https://github.com/MIT-SPARK/TEASER-plusplus
- BEVPlace2 (T-RO 2025): https://github.com/zjuluolun/BEVPlace2
- Open3D multiway registration:
  https://www.open3d.org/docs/latest/tutorial/pipelines/multiway_registration.html
- GTSAM OrientedPlane3Factor test:
  https://github.com/rising-turtle/graph_slam/blob/master/gtsam/test/testOrientedPlane3Factor.cpp
- Efficient/Adaptive GNC for PGO: https://arxiv.org/abs/2310.06765 ·
  https://arxiv.org/pdf/2308.11444

---

## 6. Measured results (2026-09-02, current pipeline)

Everything above was written against the #221 sweep numbers. Re-running the
whole matrix on the **current** pipeline (after the #218–#222 reconstruction /
segmentation work) changes the baselines materially, so the numbers below
supersede the "17.7 mm" targets in the executive summary and in issue #225.

All configs measured identically (fresh copy → pose stage → `create clouds` →
`create planes` → `analyze quality` / `analyze accuracy`), deterministic seeds.

### Office scan `afb3234950` (238 frames) — GT-free flatness

| pose stage | flatness_rms | thickness_p90 |
|---|---|---|
| none (baseline) | 12.50 mm | 20.42 mm |
| `register` (JPR: `--prior-weight 0.1 --neighbor-window 10 --iterations 50`) | **8.68 mm** | 14.42 mm |
| `optimize` (old defaults, unweighted) | 12.60 mm | 20.47 mm |
| `optimize` (new defaults, inlier-weighted) | 11.72 mm | 19.12 mm |
| `optimize` (old) → `register` | 9.24 mm | 15.16 mm |
| `optimize` (dense) → `register` | 10.00 mm | 16.17 mm |

### MuSHRoom honka (1596 frames) — laser GT F-score @ 50 mm + flatness

| pose stage | GT F-score | flatness_rms |
|---|---|---|
| none (baseline) | 0.7950 | 27.98 mm |
| `register` (JPR) | 0.7556 | (flatness improves, **GT worsens**) |
| `optimize` (old defaults, unweighted) | 0.7925 | 27.82 mm |
| `optimize` dense, **un**weighted | 0.7591 | 29.44 mm |
| `optimize` dense, inlier-**weighted** | 0.7922 | 27.09 mm |
| `optimize` (new defaults, weighted) | **0.7958** | **24.91 mm** |
| `optimize --loop-closure` (aggressive) | 0.7701 | 29.61 mm |
| `optimize --loop-closure` (conservative) | 0.7889 | 27.36 mm |

### Findings

1. **The 17.7 mm target is obsolete.** The pipeline improvements moved JPR from
   17.7 mm → 8.68 mm on the office scan. JPR (local point-to-plane) now
   dominates *GT-free flatness* — the "pairwise energy provably saturates"
   premise no longer holds at these magnitudes.

2. **JPR trades laser GT accuracy for flatness.** On honka (which has good input
   poses and a Faro reference) JPR *lowers* the GT F-score 0.795 → 0.756 while
   improving flatness. Optimising the pairwise objective warps global geometry.

3. **"Denser plane extraction" (the #225 next-lever) is counterproductive on its
   own** — measured, not assumed: naive dense extraction dropped honka GT
   0.795 → 0.759. Root cause: every `OrientedPlane3Factor` carried equal
   authority, so the many small/weak planes dense extraction adds warped the
   trajectory. The bottleneck was landmark *reliability*, not count.

4. **Fix — per-observation inlier weighting (shipped, B).** Scaling each plane
   factor's sigma by `sqrt(ref/inliers)` (self-calibrating on the median inlier
   count) turns dense extraction from harmful into helpful: honka dense goes
   0.759 → 0.792, and the new mid-density weighted defaults reach 0.7958 GT
   (> baseline 0.7950, > JPR 0.756) **and** 24.91 mm flatness (< baseline
   27.98). `optimize` is now the only pose stage that improves GT-free flatness
   *without* degrading laser GT.

5. **`optimize` → `register` chaining does not beat JPR alone** on office
   (9.24 / 10.00 vs 8.68). On already-globally-consistent scans, prepending the
   global stage only gives JPR a worse local starting point. Not shipped as a
   default; both stages remain independently invocable.

6. **P2 loop edges (shipped infrastructure, C) do not help the current benchmark
   scans** — because those scans have no wide-baseline drift to fix. Detection
   is fast and correct (honka: ~4k candidate edges in ~20 s), but on honka the
   edges are ~neutral-to-slightly-negative on GT (0.795 → ~0.789), and
   over-confident edge sigmas actively hurt (0.770). Loop closure is OFF by
   default with conservative defaults. Its payoff requires (a) a genuinely
   loopy/drifting scan with GT — see #221 Tier 2 (ARKitScenes / TLS importer) —
   and/or (b) the learned-matcher upgrade (EfficientLoFTR / LightGlue+ALIKED)
   the `LoopClosure` matcher interface is built to accept.

### Loop-closure detection follow-up (2026-09-02)

Prompted by the office scan actually having start→end drift that no loop closure
would align. Findings, all measured:

1. **Spatial (pose-based) proposal is structurally blind to drift loops.** It
   shortlists pairs by camera-centre proximity *in the seed poses*, but drift
   pulls the true start/end partners far apart there, so that pair is never
   proposed. Fixed with pose-INDEPENDENT proposal: an appearance bag-of-words
   over the frames' ORB descriptors, plus an `exhaustive` all-pairs mode, plus
   `auto` (exhaustive on small scans, appearance on large). On the office scan
   the BoW shortlist *still* misses the loop under indoor perceptual aliasing,
   so `auto` correctly falls back to exhaustive there.

2. **Detection was the wrong suspect; verification was the bottleneck.**
   Exhaustive proposal tried all 17k office pairs and accepted 0 edges at the
   old thresholds. Loosening ORB verification (3000 features, 0.10 m 3D-3D
   threshold — iPad depth is noisy) surfaced a genuine start/end loop:
   frame-gap ~215, ~90 RANSAC inliers, ~12 m disagreement with the drifted seed.

3. **GNC discards drift-correcting loop edges by construction.** A loop edge
   that corrects large drift has a huge residual at the drifted seed, which
   GNC-TLS classifies as an outlier and zeros — so `--loop-closure` alone (GNC)
   detects but does not apply big corrections. `--loop-trust` (GNC known-inlier
   + Huber) + a looser `--odometry-sigma-trans` lets it flow.

4. **Two safeguards make detection usable:** PCM (keep the largest mutually
   consistent edge set — rejects aliasing false positives) and a LOWER
   seed-disagreement gate (drop edges that already agree with the seed, so loop
   closure is a no-op on well-aligned scans — recovered honka GT 0.73 → 0.79).

5. **Open limit:** on a GT-less scan with repeated structure, applying the
   detected loop (`--loop-trust`) currently *degrades* the GT-free flatness/
   thickness metrics on the office scan (17.9 / 29.0 mm vs 12.5 / 20.4) with a
   ~16 m correction. Whether that is overshoot/aliasing or a globally-correct
   alignment the *local* metric penalises is unknowable without absolute GT.
   Safe automatic application needs (a) GT to validate (#221 Tier 2) and (b) a
   discriminative learned matcher (EfficientLoFTR / LightGlue+ALIKED) to cut the
   aliasing false-positive rate the ORB front-end suffers on repetitive interiors.

### Recommended next steps (revised)

- **#221 Tier 2 GT importer** is now the critical path: every back-end lever
  (P2 loop edges, aggressive plane weighting) is only measurable on a scan that
  actually drifts *and* has absolute GT. The current fixtures are already
  near the sensor floor, so they cannot show the wins these levers target.
- **Learned matcher** for `LoopClosure` (commercial-safe EfficientLoFTR or
  LightGlue+ALIKED via the TensorRT backend) once (1) is in place.
- Keep `register` for flatness-only work and `optimize` for GT-safe global
  refinement; document the trade-off (done in the CLI help).
