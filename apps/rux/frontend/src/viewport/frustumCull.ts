// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import type { TileInfo } from '../api/types';

/**
 * Test whether a tile's world-space bbox intersects a camera frustum.
 *
 * The frustum is in scene-local coordinates (camera already applied).
 * The tile bbox is in world coordinates. The recentring origin (subtracted
 * from world positions before upload to the GPU) must be applied here too.
 */
export function tileIntersectsFrustum(
  tile: TileInfo,
  frustum: THREE.Frustum,
  origin: THREE.Vector3 | null,
): boolean {
  const ox = origin?.x ?? 0;
  const oy = origin?.y ?? 0;
  const oz = origin?.z ?? 0;
  const box = new THREE.Box3(
    new THREE.Vector3(tile.min[0] - ox, tile.min[1] - oy, tile.min[2] - oz),
    new THREE.Vector3(tile.max[0] - ox, tile.max[1] - oy, tile.max[2] - oz),
  );
  return frustum.intersectsBox(box);
}

/**
 * Ids of tiles that intersect the frustum, plus their order (nearest first by
 * centre distance to camera, so foreground loads first).
 */
export function visibleTileIds(
  tiles: TileInfo[],
  frustum: THREE.Frustum,
  cameraPosition: THREE.Vector3,
  origin: THREE.Vector3 | null,
): number[] {
  const ox = origin?.x ?? 0;
  const oy = origin?.y ?? 0;
  const oz = origin?.z ?? 0;

  const visible: Array<{ id: number; dist: number }> = [];
  for (const tile of tiles) {
    const box = new THREE.Box3(
      new THREE.Vector3(tile.min[0] - ox, tile.min[1] - oy, tile.min[2] - oz),
      new THREE.Vector3(tile.max[0] - ox, tile.max[1] - oy, tile.max[2] - oz),
    );
    if (!frustum.intersectsBox(box)) continue;
    const centre = box.getCenter(new THREE.Vector3());
    const dist = centre.distanceToSquared(cameraPosition);
    visible.push({ id: tile.id, dist });
  }
  visible.sort((a, b) => a.dist - b.dist);
  return visible.map((v) => v.id);
}
