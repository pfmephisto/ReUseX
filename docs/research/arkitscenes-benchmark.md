<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ARKitScenes as an External Ground-Truth Benchmark

Results note for issue
[#224](https://github.com/pfmephisto/ReUseX/issues/224) (Tier 2 of the
[#221](https://github.com/pfmephisto/ReUseX/issues/221) ground-truth plan).

[ARKitScenes](https://github.com/apple/ARKitScenes) (Apple, Dehghan et al.,
NeurIPS 2021) provides room-scale RGB-D sequences captured with iPad-Pro LiDAR —
the **same sensor class** as ReUseX's own iPad scans — each paired with a
ground-truth mesh. `rux import arkitscenes <scene-dir>` maps one scene into a
`.rux` ProjectDB once; the whole pipeline (`create clouds/planes`,
`analyze quality`, `analyze accuracy`) then runs natively, identical to RTABMap
and MuSHRoom imports.

> ARKitScenes carries a **research-only** license. Scene data is **not** committed
> to this repository; `scripts/bench-arkitscenes.sh` downloads it on demand. Only
> derived, geometry-only quality visualizations are checked in.

## On-disk format (lowres LiDAR streams)

| Stream | File | Notes |
|---|---|---|
| RGB | `lowres_wide/<vid>_<ts>.png` | 256×192 |
| Depth | `lowres_depth/<vid>_<ts>.png` | CV_16UC1, **millimeters** (passed through) |
| Confidence | `confidence/<vid>_<ts>.png` | CV_8UC1, values 0/1/2 (higher = better) |
| Intrinsics | `lowres_wide_intrinsics/<vid>_<ts>.pincam` | one line: `width height fx fy cx cy` |
| Poses | `lowres_wide.traj` | one line: `ts rx ry rz tx ty tz` |
| GT mesh | `<vid>_3dod_mesh.ply` | ARKit trajectory frame |

The trajectory is subsampled (~10 fps) relative to depth (~60 fps), so each depth
frame is matched to the nearest trajectory pose and `.pincam` within a 20 ms
tolerance; unmatched frames are skipped.

## Pose convention (the one subtle part)

A `.traj` line is the **world→camera** extrinsic `E = [R(rodrigues) | t]` (Apple's
`TrajStringToMatrix` builds `E` and then inverts it), so camera→world = `E⁻¹`.
ARKitScenes' camera frame is **already** the OpenCV optical convention
(x-right, y-down, z-forward) — the same frame `reconstruct` back-projects into —
so the stored pose is `c2w` **with no axis flip**.

Nerfstudio's ARKitScenes parser flips Y/Z, but only because its internal frame is
OpenGL/y-up. Inserting that flip here fans the reconstructed walls into a radial
spiral. The failure is unmistakable in a thin horizontal wall slice:

| Pose handling | flatness_rms | thickness_p90 | planes |
|---|---|---|---|
| `c2w · diag(1,−1,−1,1)` (OpenGL flip — **wrong**) | 55.6 mm | 89.3 mm | 85 (over-fragmented) |
| `c2w` (no flip — **correct**) | 17.9 mm | 28.0 mm | 9 |

## Results (three Validation scenes)

The ARKit trajectory frame coincides with the `<vid>_3dod_mesh.ply` frame, so
`rux analyze accuracy` scores the reconstruction against GT **with no
registration step**.

| Scene | Frames | Points | Planes | flatness_rms | thickness_p90 | F-score@5cm | acc. median | compl. median |
|---|---|---|---|---|---|---|---|---|
| 41069050 | 933 / 1899 | 24.7k | 9 | 17.9 mm | 28.0 mm | 0.887 | 17.0 mm | 24.0 mm |
| 41069048 | 990 / 2012 | 27.7k | 10 | 17.5 mm | 28.2 mm | 0.894 | 15.1 mm | 22.8 mm |
| 41069051 | 1926 / 4000 | 28.7k | 10 | 23.1 mm | 39.4 mm | 0.895 | 18.3 mm | 23.6 mm |

For reference, the MuSHRoom `honka` iPhone capture scores flatness_rms 25.5 mm /
thickness_p90 43.1 mm with its shipped poses — ARKitScenes' internal metrics land
in the same regime, and the GT accuracy (F ≈ 0.89 @5 cm) gives an absolute,
drift-sensitive quality number the internal metrics alone cannot.

![Wall-slice comparison, reconstruction vs GT mesh](arkitscenes/wall-slice-comparison.png)

*Horizontal wall slice (z ≈ 1 m). Top row: ReUseX reconstruction from imported
ARKit poses. Bottom row: ARKitScenes ground-truth mesh. Matching, crisp wall
lines confirm correct pose handling.*

## Reproduce

```bash
scripts/bench-arkitscenes.sh                       # default 3 Validation scenes
scripts/bench-arkitscenes.sh ~/data 41069050       # custom dir / scene ids
```

The script clones Apple's `download_data.py`, fetches the lowres streams + GT
mesh for each scene, then runs import → reconstruct → segment → quality/accuracy
and prints a one-line summary per scene.
