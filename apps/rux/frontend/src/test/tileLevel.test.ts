// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import type { TileInfo } from '../api/types';
import { LOD_FRACTIONS, computeTileFraction, desiredCount } from '../viewport/tileLevel';

/** A unit-cube tile centred at world origin. */
const UNIT_CUBE: TileInfo = {
  id: 0,
  count: 1000,
  min: [-0.5, -0.5, -0.5],
  max: [0.5, 0.5, 0.5],
};

describe('computeTileFraction', () => {
  it('returns 1.0 for a camera inside a tile', () => {
    const cam = new THREE.Vector3(0, 0, 0);
    expect(computeTileFraction(UNIT_CUBE, cam, null)).toBe(1.0);
  });

  it('returns 1.0 for a very close camera (large angular size)', () => {
    // Camera at (0, 0, 0.6) looking at the unit-cube tile centred at origin.
    // Diagonal ≈ sqrt(3) ≈ 1.73, distance ≈ 0.6 → angular ≈ 2.9 rad >> 0.35.
    const cam = new THREE.Vector3(0, 0, 0.6);
    expect(computeTileFraction(UNIT_CUBE, cam, null)).toBe(1.0);
  });

  it('returns 0.5 at medium distance', () => {
    // Diagonal ≈ 1.73, distance ≈ 7 → angular ≈ 0.247 (between 0.175 and 0.349).
    const cam = new THREE.Vector3(0, 0, 7);
    const f = computeTileFraction(UNIT_CUBE, cam, null);
    expect(f).toBe(0.5);
  });

  it('returns 0.25 at a moderate distance', () => {
    // Diagonal ≈ 1.73, distance ≈ 14 → angular ≈ 0.124 (between 0.087 and 0.175).
    const cam = new THREE.Vector3(0, 0, 14);
    const f = computeTileFraction(UNIT_CUBE, cam, null);
    expect(f).toBe(0.25);
  });

  it('returns 0.1 for a far small tile', () => {
    // Camera at origin, large tile centred at (50,50,50).
    // Diagonal ≈ 17.3, distance ≈ 86.6 → angular ≈ 0.2 ... wait, let me recalc.
    // Centre of LARGE_FAR_TILE = (50,50,50), distance from origin = sqrt(7500) ≈ 86.6.
    // Diagonal = 10*sqrt(3) ≈ 17.32, angular ≈ 17.32/86.6 ≈ 0.2 → 0.5 bucket.
    // Use a very far camera instead.
    const veryFarCam = new THREE.Vector3(0, 0, 1000);
    // Diagonal ≈ 1.73, distance ≈ 1000 → angular ≈ 0.00173 < 0.087.
    const f = computeTileFraction(UNIT_CUBE, veryFarCam, null);
    expect(f).toBe(0.1);
  });

  it('applies the recentring origin', () => {
    // Tile in world coords [99..100]^3, camera at world (100.6, 100, 100).
    // With origin=(100,100,100): tile scene centre=(−0.5,−0.5,−0.5), cam scene=(0.6,0,0).
    const worldTile: TileInfo = {
      id: 2,
      count: 100,
      min: [99, 99, 99],
      max: [100, 100, 100],
    };
    const cam = new THREE.Vector3(100.6, 100, 100);
    const origin = new THREE.Vector3(100, 100, 100);
    const f = computeTileFraction(worldTile, cam, origin);
    expect(LOD_FRACTIONS).toContain(f);
  });

  it('always returns a value in LOD_FRACTIONS', () => {
    for (const dist of [0.1, 1, 5, 20, 100, 500]) {
      const cam = new THREE.Vector3(dist, 0, 0);
      const f = computeTileFraction(UNIT_CUBE, cam, null);
      expect(LOD_FRACTIONS).toContain(f);
    }
  });
});

describe('desiredCount', () => {
  it('returns at least 1 for any fraction', () => {
    for (const f of LOD_FRACTIONS) {
      expect(desiredCount(0, f)).toBeGreaterThanOrEqual(1);
      expect(desiredCount(1, f)).toBeGreaterThanOrEqual(1);
    }
  });

  it('returns the full count at fraction 1.0', () => {
    expect(desiredCount(1000, 1.0)).toBe(1000);
  });

  it('returns ceiling of count * fraction', () => {
    expect(desiredCount(100, 0.25)).toBe(25);
    expect(desiredCount(101, 0.25)).toBe(26); // ceil(101*0.25) = ceil(25.25) = 26
    expect(desiredCount(100, 0.1)).toBe(10);
    expect(desiredCount(101, 0.1)).toBe(11);
  });
});
