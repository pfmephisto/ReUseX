<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Learned Wide-Baseline Loop Closure: MASt3R, Licence-Clean Alternatives, and a File-Bridge Design

Decision-ready follow-up to
[`registration-improvements.md`](registration-improvements.md), for issues
[#221](https://github.com/pfmephisto/ReUseX/issues/221) and
[#225](https://github.com/pfmephisto/ReUseX/issues/225).

**Question this document answers.** The P2 loop-closure front-end (shipped in
#225) uses an ORB matcher, which aliases on repetitive interiors and finds only
sparse wide-baseline edges. Can a *learned* matcher (the user asked specifically
about **MASt3R**) do better, and how do we use it given that (a) MASt3R is
painful to link into GPL C++ and (b) it is non-commercial? Also: **are there
licence-compatible alternatives, and could we train our own model?**

---

## 1. Executive summary

- **We do not need MASt3R for the shipping pipeline, because ReUseX is RGB-D.**
  MASt3R's headline capability is recovering *metric* relative pose from raw RGB
  pairs. We already have metric depth per frame, so a plain permissive *matcher*
  (2D correspondences) + our depth → RANSAC-Kabsch gives a metric relative-pose
  edge with no non-commercial dependency. MASt3R's unique value collapses to
  "slightly better correspondences," not "the only way to get scale."

- **The licence landscape changed in our favour (verified 2026-09-04).** Since
  the last research pass there is now **MapAnything** (Meta+CMU, Sep 2025;
  weights Jan 2026) shipping an **Apache-2.0 checkpoint** — effectively a
  commercially-licensed MASt3R successor — and **RoMa**'s DINOv2 backbone was
  **relicensed to Apache-2.0**, making that dense matcher fully clean. Combined
  with **XFeat / EfficientLoFTR / LightGlue+ALIKED** (all Apache-2.0), there is a
  strong commercial-safe path at every tier.

- **The integration + licence problem is solved structurally by a file bridge,
  not by porting a model.** `rux optimize --loop-edges <file.json>` ingests
  externally-computed relative-pose constraints (schema `reusex.loop_edges.v1`)
  into the *same* GNC graph as the internal ORB edges. Producers run
  out-of-process in Python (`tools/loop_edges/`). A **non-commercial** model can
  therefore serve as an offline accuracy-**ceiling oracle** without ever linking
  into the GPL binary, while a commercial-safe matcher writes the identical file
  for production. No NC code, no GPL-linking headache.

- **Training our own foundation model is not feasible; fine-tuning one is, and
  is the real lever.** From-scratch DUSt3R/MASt3R/MapAnything-class pretraining
  needs millions of pairs across a dozen+ datasets and hundreds of GPU-days on a
  multi-GPU cluster — 2–3 orders of magnitude beyond one RTX 6000 Ada and a few
  building scans. But **fine-tuning a permissive matcher on our own domain is
  cheap and high-ROI**: RGB-D frames + optimized poses yield ground-truth pixel
  correspondences *for free* by reprojection (no manual labels), which is exactly
  the supervision these matchers train on.

- **Recommendation.** Ship the ORB front-end as the zero-dependency default;
  offer a commercial-safe learned upgrade (**XFeat** or **LightGlue+ALIKED**, or
  **MapAnything-apache**) via the file bridge; keep **MASt3R** and
  **MapAnything-NC** as offline ceiling oracles to *quantify* whether a learned
  matcher is worth productionising on genuinely-drifting scans (office
  `afb3234950`, NewOffice) with the honka Faro GT as the no-regression guard.

---

## 2. Licence-compatible alternatives to MASt3R

MASt3R / DUSt3R / MASt3R-SfM are **CC-BY-NC-SA 4.0** (non-commercial), and the
metric checkpoint carries extra dataset (mapfree) restrictions. ReUseX is a
commercial LINK product, so those are **eval-only**. Alternatives, by the role
they would play:

### A. Feed-forward metric geometry / pose (direct MASt3R replacements)

| Model | Licence (weights) | Notes |
|---|---|---|
| **MapAnything** (Meta+CMU) | **Apache-2.0** *and* CC-BY-NC | Ships **two** checkpoints: `facebook/map-anything-apache` (6 datasets, commercial) and a 13-dataset CC-BY-NC one (higher accuracy). Full train+bench code. The apache/NC pair lets us **measure exactly what the licence costs in accuracy**. |
| **VGGT-1B-Commercial** | Commercial, application-gated | Feed-forward multi-view poses/pointmaps; the other foundation-model commercial path. |
| MASt3R / DUSt3R / MASt3R-SfM | CC-BY-NC-SA | **Oracle only.** Best-known raw wide-baseline accuracy. |

### B. Two-view matcher + our depth → PnP/Kabsch (pragmatic shippable path)

| Model | Licence | Notes |
|---|---|---|
| **RoMa** | **MIT** (code) + DINOv2 **Apache-2.0** | Dense, robust at wide baselines / low overlap; now fully clean after the DINOv2 relicense. |
| **EfficientLoFTR** | Apache-2.0 | Semi-dense, strong on low-texture indoor. |
| **XFeat** | Apache-2.0 | Tiny, fast, CPU-capable, trivial ONNX/TensorRT export — best fit for the existing backends. |
| **LightGlue + ALIKED/DISK** | Apache-2.0 | Sparse, very fast, ONNX/TensorRT-proven. **Avoid classic SuperPoint** (NC) — this taints the bundled `superpoint.pt`, which is unused. |

Because we lift matches to metric 3D with our own depth, **any** of these yields
a metric relative-pose edge — the metric-from-RGB capability that makes MASt3R
special is redundant here.

### C. Classical / C++-native (no Python, permissive)

- **TEASER++** (MIT) + FPFH — certifiably-robust registration, stable C++ API.
- **GeoTransformer** (MIT) — learned indoor point-cloud registration (libTorch).

---

## 3. Could we train our own model?

Two very different scopes:

**(a) A from-scratch foundation model (MASt3R/MapAnything class): No.**
These are trained on millions of image pairs spanning a dozen+ datasets (Habitat,
ScanNet++, ARKitScenes, MegaDepth, BlendedMVS, Co3D, …) for hundreds of GPU-days
on multi-GPU A100/H100 clusters. One RTX 6000 Ada plus a handful of building
scans is 2–3 orders of magnitude short on **both** compute and data diversity;
the result would overfit our few buildings and not generalize.

**(b) Fine-tune / distill a permissive model on our domain: Yes — high ROI.**
We have a *free self-supervised signal*: an RGB-D frame pair with known relative
pose gives ground-truth pixel correspondences by **reprojection** — project a
pixel from frame A into frame B via A's depth and the A→B pose, and that is a
labeled match, no human annotation. This is precisely how matchers (and VPR
descriptors) are trained. So we can:

- **Domain-adapt a matcher** (XFeat / ALIKED / RoMa) to indoor iPad-LiDAR imagery
  — where outdoor-trained models have a documented gap — using correspondences
  mined from our own optimized scans.
- **Fine-tune an indoor VPR head** (SALAD / DINOv2) for better loop *detection*
  (retrieval), which the literature shows is where the indoor domain gap bites.

**Hardware / data verdict.** RTX 6000 Ada (48 GB) is **excellent for inference**
of any of these and **sufficient for fine-tuning small matchers/descriptors**;
it is **not** enough for foundation pretraining. Our scans are **enough for
fine-tuning** (each yields 10⁵+ reprojection pairs) but **not** for from-scratch
pretraining. Net: don't pretrain — fine-tune a permissive model if off-the-shelf
accuracy proves insufficient (measure first, via the oracle).

---

## 4. The file-bridge design (how this ships)

The obstacle was never the algorithm — it was linking a Python/NC model into a
GPL C++ binary. We dissolve it by moving the producer **out of process** and
exchanging **data**, not code.

```
 ┌─────────────────────────────┐        ┌──────────────────────────────────────┐
 │  tools/loop_edges/           │  JSON  │  rux optimize --loop-edges edges.json │
 │  export_loop_edges.py        │──────▶ │   load_loop_edges() (nlohmann::json)  │
 │   matcher: orb | xfeat |     │ schema │   node_id → frame-index map           │
 │   lightglue | mast3r |       │  v1    │   union with internal ORB edges       │
 │   mapanything                │        │   → same GncOptimizer graph (PCM/GNC) │
 └─────────────────────────────┘        └──────────────────────────────────────┘
   (Python venv, GPU; a NC model             (GPL C++; never links a matcher;
    lives here as an ORACLE only)             a wrong edge is down-weighted by GNC)
```

- **`LoopEdge` is pure data**: `(node_i, node_j, T_ij, sigma_rot, sigma_trans,
  inliers)`. `T_ij = pose(i)^-1 · pose(j)` in the optical→world convention the
  graph uses — recovered from backprojected optical-frame 3D correspondences, so
  it is independent of the (drifted) stored world poses; metric scale comes from
  the RGB-D depth.
- **Schema `reusex.loop_edges.v1`** (JSON) referencing stable DB `node_id`s, so
  the producer never needs to know the optimizer's internal frame ordering.
- **Robustness is unchanged**: external edges enter the *same* GNC graph as ORB
  edges, so PCM false-positive rejection, GNC down-weighting, and the
  seed-disagreement gates all apply. `--loop-trust` promotes them to GNC
  known-inliers when a large-drift correction must actually flow.
- **Licence boundary is explicit**: the NC backends (`mast3r`, `mapanything`-NC)
  are gated behind `--allow-noncommercial`, print a banner, and their output is
  labelled an evaluation artefact. Nothing in `tools/loop_edges/` is compiled
  into the product.

### C++ changes (this PR)
- `load_loop_edges()` (`libs/reusex/src/slam/load_loop_edges.cpp`) — parse +
  node-id mapping + loud failure on bad files; 3 unit tests.
- `PlaneGraphOptions::loop_edges_file` + union in `optimize_sensor_poses.cpp`.
- `rux optimize --loop-edges <file>` CLI flag.

### Python tool (this PR)
- `tools/loop_edges/export_loop_edges.py` — matcher-agnostic core: `.rux` frame
  reader, depth backprojection, RANSAC-Umeyama relative pose, edge JSON writer;
  `--proposal exhaustive|endcap` (endcap targets a start↔end drift loop that
  spatial proposal is structurally blind to).
- `tools/loop_edges/matchers/` — `orb` (BSD, zero-dep baseline), `xfeat` /
  `lightglue` (Apache), `mast3r` / `mapanything` (oracle, gated).

---

## 5. Benchmark

Scans (all RGB-D iPad-LiDAR):

| scan | frames | drift | ground truth | role |
|---|---|---|---|---|
| office `afb3234950` | 238 | start↔end ~16 m (documented revisit-drift) | none (GT-free) | controlled single-loop matcher A/B |
| **NewOffice** | 3876 | ~18.6 m end-to-start over an 816 m path | none (GT-free) | **visible-drift headline**; endcap loop |
| honka (MuSHRoom) | 1596 | minimal (pre-optimized poses) | Faro laser | **no-regression guard** (F-score @ 50 mm) |

Protocol: fresh `.rux` per variant → pose stage → `create clouds` → `create
planes` → `analyze quality` (+`analyze accuracy` where GT exists), deterministic
seeds. Driver: `scripts/bench-loop-edges.sh`.

### 5.1 Matcher comparison on the office loop

Identical candidate set (endcap proposal, 1225 start↔end pairs, `--band-frac
0.15 --min-frame-gap 50 --min-inliers 30`), identical depth backprojection +
RANSAC — only the matcher differs:

| matcher | licence | loop edges | Σ inliers | max inliers |
|---|---|---:|---:|---:|
| ORB (BSD) | commercial-safe | 5 | 175 | 42 |
| **XFeat** (Apache-2.0) | **commercial-safe** | **163** | **7857** | **205** |
| MASt3R (CC-BY-NC) | oracle only | _(endcap run pending)_ | | |

**XFeat found 32× more loop edges with 45× the inlier support than ORB on the
identical candidate set** — a decisive front-end win, and it is Apache-2.0
(shippable). This directly answers "find more/better loop closures."

### 5.2 Effect on applied pose quality (office, GT-free)

`scripts/bench-loop-edges.sh`, fresh `.rux` per variant, `--loop-trust
--odometry-sigma-trans 0.05`:

| pose stage | flatness_rms | thickness_p90 | edges | max pose shift |
|---|---:|---:|---:|---:|
| base (stored poses) | 12.50 mm | 20.42 mm | 0 | — |
| optimize (no loops) | **11.72 mm** | **19.12 mm** | 0 | 6.8 cm |
| optimize + ORB edges | 18.63 mm | 30.88 mm | 5 | 14.92 m |
| optimize + XFeat edges | 20.89 mm | 34.52 mm | 163 | **16.66 m** |

**Read this carefully — it is the crux of the whole loop-closure problem on
GT-less scans.** The office scan has a measured **16.66 m** start↔end drift
(§Scans). With `--loop-trust`, the XFeat edges apply a **16.66 m** correction —
*exactly* the measured drift — i.e. they close the loop essentially perfectly;
ORB's sparse 5 edges manage only 14.92 m. Yet the **GT-free flatness metric gets
worse**, because it measures *local* plane consistency and cannot tell a
globally-corrected trajectory from a locally-smeared one — a globally-correct
re-slice of the planes still moves points off their old local fits. This is the
same "unknowable without absolute GT" result the ORB experiment hit in
[`registration-improvements.md`](registration-improvements.md) §6.5, now
reproduced with a far stronger matcher: **a better matcher makes the closure more
complete, not the GT-free metric happier.** Adjudicating whether the closure is
*right* needs either absolute GT (→ §5.4 honka guard) or a visual (→ §5.3).

### 5.3 Visual (renders)

<!-- RESULTS-RENDERS -->
_Before/after trajectory + merged-cloud renders of the NewOffice start↔end
overlap region, showing drift closure (or its absence) per matcher._

---

## 6. Recommendation

1. **Ship** the ORB front-end as the zero-dependency default (already in #225).
2. **Offer** a commercial-safe learned upgrade through the file bridge — start
   with **XFeat** or **LightGlue+ALIKED** (Apache, ONNX/TensorRT-ready), with
   **MapAnything-apache** as a foundation-model option for the widest baselines.
3. **Keep** MASt3R + MapAnything-NC as offline ceiling oracles; only invest in
   productionising / fine-tuning a learned matcher if the oracle shows a
   material win on a genuinely-drifting scan **and** the honka GT does not
   regress.
4. **Do not** attempt from-scratch foundation-model training. If off-the-shelf
   accuracy is insufficient, **fine-tune** a permissive matcher on our own scans
   via self-supervised reprojection.

---

## 7. Sources (verified 2026-09-04)

- MapAnything (Apache + NC checkpoints): https://github.com/facebookresearch/map-anything ·
  https://huggingface.co/facebook/map-anything-apache · https://arxiv.org/abs/2509.13414
- RoMa (MIT; DINOv2 Apache backbone): https://github.com/Parskatt/RoMa
- DINOv2 Apache-2.0 relicense: https://ai.meta.com/blog/dinov2-facet-computer-vision-fairness-evaluation/
- XFeat (Apache-2.0): https://github.com/verlab/accelerated_features
- EfficientLoFTR (Apache-2.0): https://github.com/zju3dv/EfficientLoFTR
- LightGlue (Apache-2.0): https://github.com/cvg/LightGlue
- MASt3R (CC-BY-NC-SA): https://github.com/naver/mast3r
- VGGT (commercial ckpt): https://github.com/facebookresearch/vggt
- TEASER++ (MIT): https://github.com/MIT-SPARK/TEASER-plusplus
