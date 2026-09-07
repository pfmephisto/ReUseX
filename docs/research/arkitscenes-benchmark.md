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

All timestamps (filenames and `.traj`) are ARKit **device-relative** seconds
(uptime since boot, not a Unix epoch), so they are only comparable within a scene.

### Pose/frame synchronization

The trajectory is subsampled (~10 fps) relative to depth (~60 fps), so only one
depth frame in six lands on a trajectory sample. Matching each frame to its
*nearest* pose therefore hands five frames in six a pose up to ~17 ms stale —
at the measured hand-held motion for these scenes (p50 0.23 m/s, 29.6 °/s) that
is ~26 mm of lateral error at 3 m — the same order as the flatness residual the
benchmark is trying to measure. It also reused every pose for three frames, so
the extra frames carried no new pose information. (The *measured* effect on the
scores turned out to be small; see *Effect of pose interpolation* below.)

Instead each frame's pose is **interpolated to its own timestamp** between the
bracketing trajectory samples — SLERP on the rotation, linear interpolation on
the camera position, both applied to the `c2w` pose (lerping the world→camera
translation would mix in the rotation and bend the camera path). Frames outside
the trajectory's time span are skipped (no extrapolation), as are frames whose
bracketing samples straddle a gap > 0.5 s (ARKit tracking loss). In practice the
only losses are the frames captured before the first / after the last pose
(39, 38 and 155 frames in the three scenes below, almost all of them *leading*
frames — depth recording starts before the tracker converges). Intrinsics still use nearest-`.pincam` matching within 20 ms (there is
one `.pincam` per RGB frame, and intrinsics drift far more slowly than pose).

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

Both rows of that A/B were measured on scene 41069050 with the earlier
nearest-pose matching, which is what makes them comparable to each other; the
current interpolating importer scores the no-flip case at 17.6 mm / 28.5 mm
(see below). The flip verdict is unaffected — it is a 3× effect.

## Results (three Validation scenes)

The ARKit trajectory frame coincides with the `<vid>_3dod_mesh.ply` frame, so
`rux analyze accuracy` scores the reconstruction against GT **with no
registration step**.

The **Frames** column is `imported / depth PNGs on disk`. Every frame that has a
bracketing pose pair is imported — the shortfall is entirely frames captured
outside the trajectory's time span (see *Pose/frame synchronization* above), not
a matching tolerance.

| Scene | Frames | Points | Planes | flatness_rms | thickness_p90 | F-score@5cm | acc. median | compl. median |
|---|---|---|---|---|---|---|---|---|
| 41069050 | 1860 / 1899 | 26.1k | 10 | 17.6 mm | 28.5 mm | 0.893 | 17.6 mm | 23.7 mm |
| 41069048 | 1974 / 2012 | 30.0k | 12 | 17.5 mm | 28.1 mm | 0.892 | 15.7 mm | 22.5 mm |
| 41069051 | 3845 / 4000 | 29.5k | 10 | 24.8 mm | 41.3 mm | 0.889 | 18.6 mm | 23.5 mm |

### Effect of pose interpolation

These numbers replace an earlier run that matched each frame to its nearest pose
within 20 ms, which imported ~½ the frames (933 / 990 / 1926) because a 10 Hz
trajectory only covers one depth frame in three at that tolerance:

| Scene | Frames (nearest → interp) | flatness_rms | thickness_p90 | F-score@5cm |
|---|---|---|---|---|
| 41069050 | 933 → 1860 | 17.9 → **17.6** mm | 28.0 → 28.5 mm | 0.887 → **0.893** |
| 41069048 | 990 → 1974 | 17.5 → 17.5 mm | 28.2 → **28.1** mm | 0.894 → 0.892 |
| 41069051 | 1926 → 3845 | 23.1 → 24.8 mm | 39.4 → 41.3 mm | 0.895 → 0.889 |

The aggregate metrics barely move (≤ 1.7 mm, ≤ 0.006 F either way) — so contrary
to expectation the stale-pose artifact was **not** dominating the benchmark, even
though it was a real ~26 mm per-frame error. Two reasons: the temporal offset is
near zero-mean across frames, so it partly averages out of a plane-residual RMS
rather than biasing it; and both the point clouds and the plane fits are already
limited by ARKit's own pose drift and LiDAR depth noise. Doubling the frame count
also adds depth noise as fast as it adds pose information, which is why the
densest scene (41069051) gets marginally *worse*.

What the fix buys is therefore correctness rather than a better score: the
reported quality is now a property of the reconstruction pipeline instead of
partly a property of the importer's frame-to-pose assignment, and the frame
count is no longer an artifact of the tolerance.

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
