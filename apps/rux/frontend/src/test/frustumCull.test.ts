// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { tileIntersectsFrustum, visibleTileIds } from '../viewport/frustumCull';
import type { TileInfo } from '../api/types';

function makeFrustumLookingAt(direction: THREE.Vector3): THREE.Frustum {
  const camera = new THREE.PerspectiveCamera(60, 1, 0.1, 1000);
  camera.position.set(0, 0, 5);
  camera.lookAt(direction);
  camera.updateMatrixWorld();
  const mat = new THREE.Matrix4().multiplyMatrices(
    camera.projectionMatrix,
    camera.matrixWorldInverse,
  );
  return new THREE.Frustum().setFromProjectionMatrix(mat);
}

describe('tileIntersectsFrustum', () => {
  it('includes a tile directly in front of the camera', () => {
    const frustum = makeFrustumLookingAt(new THREE.Vector3(0, 0, 0));
    const tile: TileInfo = { id: 0, count: 100, min: [-1, -1, -1], max: [1, 1, 1] };
    expect(tileIntersectsFrustum(tile, frustum, null)).toBe(true);
  });

  it('excludes a tile behind the camera', () => {
    const frustum = makeFrustumLookingAt(new THREE.Vector3(0, 0, 0));
    const tile: TileInfo = { id: 0, count: 100, min: [-1, -1, 50], max: [1, 1, 52] };
    expect(tileIntersectsFrustum(tile, frustum, null)).toBe(false);
  });

  it('applies the recentring origin', () => {
    const frustum = makeFrustumLookingAt(new THREE.Vector3(0, 0, 0));
    // Tile at world coords [99,99,99] to [101,101,101] but origin is (100,100,100)
    // → scene coords [-1,-1,-1] to [1,1,1] → should be visible
    const tile: TileInfo = { id: 0, count: 100, min: [99, 99, 99], max: [101, 101, 101] };
    const origin = new THREE.Vector3(100, 100, 100);
    expect(tileIntersectsFrustum(tile, frustum, origin)).toBe(true);
  });
});

describe('visibleTileIds', () => {
  it('returns ids sorted nearest-first', () => {
    const frustum = makeFrustumLookingAt(new THREE.Vector3(0, 0, 0));
    const camera = new THREE.Vector3(0, 0, 5);
    const tiles: TileInfo[] = [
      { id: 0, count: 10, min: [-0.5, -0.5, -5], max: [0.5, 0.5, -3] }, // far
      { id: 1, count: 10, min: [-0.5, -0.5, -1], max: [0.5, 0.5, 1] }, // near
    ];
    const ids = visibleTileIds(tiles, frustum, camera, null);
    expect(ids).toContain(0);
    expect(ids).toContain(1);
    expect(ids.indexOf(1)).toBeLessThan(ids.indexOf(0)); // near before far
  });

  it('excludes invisible tiles', () => {
    const frustum = makeFrustumLookingAt(new THREE.Vector3(0, 0, 0));
    const camera = new THREE.Vector3(0, 0, 5);
    const tiles: TileInfo[] = [
      { id: 0, count: 10, min: [-0.5, -0.5, -1], max: [0.5, 0.5, 1] }, // visible
      { id: 1, count: 10, min: [-0.5, -0.5, 50], max: [0.5, 0.5, 52] }, // behind
    ];
    const ids = visibleTileIds(tiles, frustum, camera, null);
    expect(ids).toContain(0);
    expect(ids).not.toContain(1);
  });
});
