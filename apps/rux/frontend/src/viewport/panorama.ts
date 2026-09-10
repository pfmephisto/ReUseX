// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Everything panorama mode decides that does not need a GPU (#265, Phase 5).
 *
 * Split from `PanoramaScene.ts` for the same reason `gsplatLayer.ts` is split
 * from `SplatScene.ts`: that module imports three.js and a WebGL context, while
 * the questions that decide whether the picture is *correct* — where a panorama
 * goes, which way it faces, and which pixel a direction lands on — are plain
 * arithmetic and are tested in `src/test/panorama.test.ts`.
 *
 * ## The two conventions this module encodes
 *
 * **Equirect pixels.** `libs/reusex/include/geometry/EquirectProjection.hpp` is
 * the definition every other consumer of these images uses, and it is restated
 * in `docs/gui/openapi.yaml` under `GET /panoramas/{id}/image`:
 *
 * ```
 *   longitude theta in [-pi, pi]   -> column u in [0, W)
 *   latitude  phi   in [-pi/2,pi/2] -> row    v in [0, H), v = 0 at the north pole
 *   bearing(theta, phi) = ( sin(theta)cos(phi), -sin(phi), cos(theta)cos(phi) )
 * ```
 *
 * so the panorama's own frame is the pipeline's optical convention: **+x right,
 * +y down, +z forward**. Getting the sign of y wrong mirrors the world, which
 * looks entirely plausible and is entirely wrong.
 *
 * **Poses.** `pose` and `frame_pose` are row-major 4x4 *camera-to-world*
 * matrices in that same optical convention — the pipeline's own
 * (`panorama_features.cpp` composes `sensor_frame_pose()` with pinhole-local
 * points directly, with no axis change). The world is gravity-aligned Z-up.
 */

import type { PanoramaInfo } from '../api/types';

/** Where a panorama's orientation comes from, and therefore how far to trust it. */
export type PanoramaHeading =
  /** Resected by `rux align 360`: position and heading are both measured. */
  | 'resected'
  /**
   * Borrowed from the timestamp-matched sensor frame: the position is that
   * frame's, and the heading is **not known at all** — see `LEVELLED_BASIS`.
   */
  | 'levelled';

/** A panorama placed in the world, ready for the scene to consume. */
export interface PanoramaPlacement {
  /** World position, metres. */
  position: [number, number, number];
  /**
   * Rotation taking a direction in the panorama's optical frame into world,
   * column-major (the order `THREE.Matrix4.fromArray` wants for a basis).
   */
  basis: number[];
  heading: PanoramaHeading;
  /** The sensor frame the position came from, when it was borrowed. */
  fromFrame?: number;
}

/**
 * The heading-free orientation used for a panorama that has not been resected.
 *
 * A 360 camera on a monopod is level, so *up* is known: the panorama's +y
 * (down) axis is world -Z. Its heading is not — the camera's yaw has no fixed
 * relation to the phone's, which is the whole reason `rux align 360` exists —
 * so +z (forward) is parked on world +X.
 *
 * Borrowing the matched frame's full rotation was rejected. It would import the
 * phone's tilt into a panorama that was level, and a tilted horizon reads as a
 * broken viewer rather than as missing information; an arbitrary heading with a
 * true horizon reads as what it is, and the UI says so.
 *
 * Columns are the panorama's x, y, z axes in world. Right-handed: x = y x z.
 */
export const LEVELLED_BASIS: number[] = [
  // +x (right)      +y (down)        +z (forward)
  0, -1, 0, /*    */ 0, 0, -1, /*  */ 1, 0, 0,
];

function isUsablePose(pose: number[] | undefined): pose is number[] {
  return (
    Array.isArray(pose) && pose.length === 16 && pose.every((value) => Number.isFinite(value))
  );
}

/** Translation of a row-major 4x4. */
function translationOf(pose: number[]): [number, number, number] {
  return [pose[3], pose[7], pose[11]];
}

/**
 * Rotation of a row-major 4x4, as a **column-major** 3x3.
 *
 * The transpose is not a mistake: a row-major matrix's rotation read
 * column-by-column is the same numbers a column-major consumer expects, which
 * is why this looks like an identity relabelling and is not.
 */
function basisOf(pose: number[]): number[] {
  return [pose[0], pose[4], pose[8], pose[1], pose[5], pose[9], pose[2], pose[6], pose[10]];
}

/**
 * Where to draw a panorama, or null when the project cannot say.
 *
 * Order is deliberate and is the point of the whole `frame_pose` contract
 * addition: a resected pose wins, a borrowed frame position is the fallback,
 * and a panorama with neither is **not drawn at the origin**. Every panorama in
 * a project that has only been imported is in the middle case, so treating it
 * as unplaceable would make panorama mode useless before `rux align 360`; and
 * silently stacking the unplaceable ones on (0,0,0) would put a room's worth of
 * imagery somewhere the scan never was.
 */
export function resolvePlacement(pano: PanoramaInfo): PanoramaPlacement | null {
  if (pano.has_pose && isUsablePose(pano.pose)) {
    return {
      position: translationOf(pano.pose),
      basis: basisOf(pano.pose),
      heading: 'resected',
    };
  }
  if (pano.has_frame_pose && isUsablePose(pano.frame_pose)) {
    return {
      position: translationOf(pano.frame_pose),
      basis: [...LEVELLED_BASIS],
      heading: 'levelled',
      fromFrame: pano.node_id,
    };
  }
  return null;
}

/** A unit bearing in the panorama's optical frame. */
export function bearingAt(theta: number, phi: number): [number, number, number] {
  const cp = Math.cos(phi);
  return [Math.sin(theta) * cp, -Math.sin(phi), Math.cos(theta) * cp];
}

/**
 * Bearing -> normalised texture coordinates, `u` across and `v` **down** from
 * the north pole — the row/column convention of the stored image, divided by
 * its size. Mirrors `geometry::bearing_to_pixel`.
 */
export function bearingToUv(x: number, y: number, z: number): [number, number] {
  const length = Math.hypot(x, y, z) || 1;
  const theta = Math.atan2(x / length, z / length);
  const phi = Math.asin(Math.min(1, Math.max(-1, -y / length)));
  return [(theta + Math.PI) / (2 * Math.PI), (Math.PI / 2 - phi) / Math.PI];
}

/** Vertex buffers of a unit sphere carrying the equirect mapping. */
export interface SphereBuffers {
  /** xyz triples, in the panorama's optical frame, on the unit sphere. */
  positions: Float32Array;
  /** uv pairs, in WebGL orientation (`v = 1` is the image's top row). */
  uvs: Float32Array;
  indices: Uint32Array;
}

/**
 * A sphere whose UVs are generated from the grid, not from vertex positions.
 *
 * Written out here rather than taking `THREE.SphereGeometry` and rewriting its
 * `uv` attribute, for two reasons that both show up on screen:
 *
 *  * **The seam.** Deriving `u` from a vertex position gives the duplicated
 *    seam column the same `u` at both copies, so the quads bridging longitude
 *    ±pi interpolate the whole texture backwards across one strip. Deriving it
 *    from the column index gives the two copies `u = 0` and `u = 1`, and the
 *    seam closes.
 *  * **The convention.** The positions here come from `bearingAt`, so the
 *    geometry's local frame *is* the panorama frame and the pose can be applied
 *    to the mesh unmodified. Three's own sphere is Y-up with its own UV layout,
 *    which would need a correction rotation nobody could later verify.
 *
 * Wound outward and drawn with `THREE.BackSide`, so the viewer at the centre
 * sees the inside of the sphere with positions still meaning what they say.
 *
 * `v = 1 - row / rows` because three.js uploads images with `flipY` on, making
 * `v = 1` the image's first row — which is the north pole.
 */
export function equirectSphere(columns = 96, rows = 48): SphereBuffers {
  const positions = new Float32Array((columns + 1) * (rows + 1) * 3);
  const uvs = new Float32Array((columns + 1) * (rows + 1) * 2);

  for (let row = 0; row <= rows; row += 1) {
    const v = row / rows;
    const phi = Math.PI / 2 - v * Math.PI;
    for (let col = 0; col <= columns; col += 1) {
      const u = col / columns;
      const [x, y, z] = bearingAt(u * 2 * Math.PI - Math.PI, phi);
      const vertex = row * (columns + 1) + col;
      positions[vertex * 3] = x;
      positions[vertex * 3 + 1] = y;
      positions[vertex * 3 + 2] = z;
      uvs[vertex * 2] = u;
      uvs[vertex * 2 + 1] = 1 - v;
    }
  }

  const indices: number[] = [];
  for (let row = 0; row < rows; row += 1) {
    for (let col = 0; col < columns; col += 1) {
      const a = row * (columns + 1) + col;
      const b = a + columns + 1;
      // Wound so the face normal points AWAY from the centre. With
      // `THREE.BackSide` that is what makes the sphere visible from the
      // inside and invisible from the outside; the opposite winding culls
      // exactly the faces the viewer standing at the centre can see, which
      // renders as an empty viewport rather than as an obvious mistake.
      //
      // The pole rows collapse to a point, so one triangle of each quad there
      // has zero area. Skipped rather than uploaded, exactly as three.js does.
      if (row !== 0) indices.push(a, a + 1, b);
      if (row !== rows - 1) indices.push(b, a + 1, b + 1);
    }
  }

  return { positions, uvs, indices: new Uint32Array(indices) };
}

/**
 * The next panorama when stepping by @p delta, wrapping at both ends.
 *
 * Wrapping rather than stopping: this backs the `[` / `]` keys `rux view`
 * already trains users on, and a capture is a loop through a building — the
 * panorama after the last one is the first one.
 *
 * Returns null when there is nothing to step to, so a caller cannot
 * accidentally index an empty list.
 */
export function stepPanorama(
  ids: number[],
  current: number | null,
  delta: number,
): number | null {
  if (ids.length === 0) return null;
  const index = current === null ? -1 : ids.indexOf(current);
  if (index < 0) return ids[delta >= 0 ? 0 : ids.length - 1];
  const next = (((index + delta) % ids.length) + ids.length) % ids.length;
  return ids[next];
}

/** One-line description of a panorama's placement provenance. */
export function describePlacement(
  pano: PanoramaInfo,
  placement: PanoramaPlacement | null,
): string {
  if (!placement) {
    return pano.node_id >= 0
      ? `Not placeable — frame ${pano.node_id} has no pose`
      : 'Not placeable — matched to no sensor frame';
  }
  if (placement.heading === 'resected') {
    const parts = ['Aligned'];
    if (Number.isFinite(pano.align_inliers) && (pano.align_inliers ?? -1) >= 0)
      parts.push(`${pano.align_inliers} inliers`);
    if (Number.isFinite(pano.align_rms) && (pano.align_rms ?? -1) >= 0)
      parts.push(`${(pano.align_rms ?? 0).toFixed(2)}° RMS`);
    return parts.join(' · ');
  }
  return `Position from frame ${placement.fromFrame} · heading unknown`;
}

/**
 * The caveat shown while looking through an unresected panorama.
 *
 * Null for a resected one: a permanent banner over a picture that is correct
 * would train the user to ignore the banner that matters.
 */
export function headingCaveat(placement: PanoramaPlacement | null): string | null {
  if (!placement || placement.heading === 'resected') return null;
  return (
    'This panorama has not been aligned. It is drawn at its matched frame’s ' +
    'position with a level horizon and an arbitrary heading — run `rux align 360` ' +
    'to resect it against the scan.'
  );
}

/**
 * What to say when the panorama section has no rows.
 *
 * Same ordering rule as `gsplatNote`: a transport failure is about the server,
 * and reporting "this project has no panoramas" on top of a request that never
 * arrived would state something the client does not know.
 */
export function panoramaNote(
  panoramas: PanoramaInfo[] | null | undefined,
  error?: Error | null,
): string | null {
  if (error) return `Could not list this project's panoramas: ${error.message}`;
  if (!panoramas) return null;
  if (panoramas.length > 0) return null;
  return 'No 360 panoramas in this project. Import some with `rux import 360`.';
}
