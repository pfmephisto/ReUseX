// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import type { TileInfo } from '../api/types';

/**
 * Discrete LOD fractions used for within-tile streaming (#396).
 *
 * Each level represents the fraction of a tile's points to fetch. Snapping to
 * discrete values is deliberate: continuous fractions would issue a new request
 * on every camera micro-movement, while four levels keep request counts low and
 * cache-friendly. The values roughly double each step so the jump from any level
 * to the next is never more than 2× the current budget.
 */
export const LOD_FRACTIONS = [0.1, 0.25, 0.5, 1.0] as const;
export type LodFraction = (typeof LOD_FRACTIONS)[number];

/**
 * Decide what fraction of a tile's points to show based on its angular size.
 *
 * Angular size ≈ tile_diagonal / distance_to_tile_centre, in radians.
 * The four thresholds roughly correspond to: object fills the screen (≥20°),
 * room-scale object (~10°), distant element (~5°), barely visible (<5°).
 *
 * `origin` is the scene-recentring offset subtracted from world coordinates
 * before upload to the GPU; the tile bbox (world space) is shifted by it
 * before computing the distance.
 */
export function computeTileFraction(
  tile: TileInfo,
  cameraPosition: THREE.Vector3,
  origin: THREE.Vector3 | null,
): LodFraction {
  const ox = origin?.x ?? 0;
  const oy = origin?.y ?? 0;
  const oz = origin?.z ?? 0;

  const cx = (tile.min[0] + tile.max[0]) / 2 - ox;
  const cy = (tile.min[1] + tile.max[1]) / 2 - oy;
  const cz = (tile.min[2] + tile.max[2]) / 2 - oz;

  const dx = tile.max[0] - tile.min[0];
  const dy = tile.max[1] - tile.min[1];
  const dz = tile.max[2] - tile.min[2];
  const diagonal = Math.sqrt(dx * dx + dy * dy + dz * dz);

  const distSq = (cx - cameraPosition.x) ** 2 + (cy - cameraPosition.y) ** 2 + (cz - cameraPosition.z) ** 2;
  const dist = Math.sqrt(distSq);

  // Avoid division by zero; when camera is inside the tile, show everything.
  if (dist < 1e-6 || diagonal < 1e-6) return 1.0;

  // Angular size in radians.
  const angular = diagonal / dist;

  if (angular >= 0.349) return 1.0;   // ~20°
  if (angular >= 0.175) return 0.5;   // ~10°
  if (angular >= 0.087) return 0.25;  // ~5°
  return 0.1;
}

/**
 * How many points to request for one tile at a given LOD fraction.
 *
 * Always at least 1 so an empty result is not requested.
 */
export function desiredCount(tileCount: number, fraction: LodFraction): number {
  return Math.max(1, Math.ceil(tileCount * fraction));
}
