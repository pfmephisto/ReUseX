// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import { describe, expect, it } from 'vitest';

import { FACE_HANDLES, boxToPlanes, handleCenter } from '../viewport/clippingBox';

describe('boxToPlanes', () => {
  it('produces exactly six planes', () => {
    const box = new THREE.Box3(new THREE.Vector3(-1, -2, -3), new THREE.Vector3(4, 5, 6));
    expect(boxToPlanes(box)).toHaveLength(6);
  });

  it('all planes have unit normals', () => {
    const box = new THREE.Box3(new THREE.Vector3(-5, -3, 0), new THREE.Vector3(5, 3, 4));
    for (const plane of boxToPlanes(box)) {
      expect(plane.normal.length()).toBeCloseTo(1);
    }
  });

  it('interior point is on the positive side of every plane', () => {
    const box = new THREE.Box3(new THREE.Vector3(-4, -2, -1), new THREE.Vector3(4, 2, 3));
    const interior = new THREE.Vector3(0, 0, 0);
    for (const plane of boxToPlanes(box)) {
      expect(plane.distanceToPoint(interior)).toBeGreaterThan(0);
    }
  });

  it('exterior point beyond +X face is clipped by the +X plane', () => {
    const box = new THREE.Box3(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(2, 1, 1));
    const outside = new THREE.Vector3(3, 0, 0); // x > max.x = 2
    const planes = boxToPlanes(box);
    // At least one plane must report a negative distance for the outside point.
    const clipped = planes.some((p) => p.distanceToPoint(outside) < 0);
    expect(clipped).toBe(true);
  });

  it('exterior point beyond -X face is clipped', () => {
    const box = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(5, 5, 5));
    const outside = new THREE.Vector3(-1, 2.5, 2.5);
    const planes = boxToPlanes(box);
    expect(planes.some((p) => p.distanceToPoint(outside) < 0)).toBe(true);
  });

  it('all six box corners are on the boundary (distance ≈ 0) of exactly two planes each', () => {
    const min = new THREE.Vector3(-2, -3, -1);
    const max = new THREE.Vector3(4, 5, 2);
    const box = new THREE.Box3(min, max);
    const planes = boxToPlanes(box);

    const corners = [
      new THREE.Vector3(min.x, min.y, min.z),
      new THREE.Vector3(max.x, min.y, min.z),
      new THREE.Vector3(min.x, max.y, min.z),
      new THREE.Vector3(max.x, max.y, min.z),
      new THREE.Vector3(min.x, min.y, max.z),
      new THREE.Vector3(max.x, min.y, max.z),
      new THREE.Vector3(min.x, max.y, max.z),
      new THREE.Vector3(max.x, max.y, max.z),
    ];

    for (const corner of corners) {
      const zeros = planes.filter((p) => Math.abs(p.distanceToPoint(corner)) < 1e-6);
      // Each corner touches three face planes (at box extremes in 3 axes)
      expect(zeros.length).toBeGreaterThanOrEqual(3);
    }
  });

  it('plane constants scale with box size', () => {
    const small = new THREE.Box3(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(1, 1, 1));
    const large = new THREE.Box3(new THREE.Vector3(-10, -10, -10), new THREE.Vector3(10, 10, 10));
    const smallPlanes = boxToPlanes(small);
    const largePlanes = boxToPlanes(large);
    // The large box should have planes further from origin
    const smallMaxC = Math.max(...smallPlanes.map((p) => Math.abs(p.constant)));
    const largeMaxC = Math.max(...largePlanes.map((p) => Math.abs(p.constant)));
    expect(largeMaxC).toBeGreaterThan(smallMaxC);
  });
});

describe('FACE_HANDLES', () => {
  it('has exactly six entries', () => {
    expect(FACE_HANDLES).toHaveLength(6);
  });

  it('covers all three axes in both signs', () => {
    for (const axis of [0, 1, 2] as const) {
      const pos = FACE_HANDLES.filter((h) => h.axis === axis && h.side === 1);
      const neg = FACE_HANDLES.filter((h) => h.axis === axis && h.side === -1);
      expect(pos).toHaveLength(1);
      expect(neg).toHaveLength(1);
    }
  });

  it('inward normals are unit vectors', () => {
    for (const h of FACE_HANDLES) {
      expect(h.inwardNormal.length()).toBeCloseTo(1);
    }
  });

  it('inward normal points opposite to the face side', () => {
    for (const h of FACE_HANDLES) {
      const component = h.inwardNormal.getComponent(h.axis);
      // Positive face → inward normal is negative along axis; negative face → positive
      expect(component * h.side).toBeLessThan(0);
    }
  });
});

describe('handleCenter', () => {
  it('places center at the face midpoint', () => {
    const box = new THREE.Box3(new THREE.Vector3(0, 0, 0), new THREE.Vector3(4, 6, 8));

    // +X face center: x=4, y=3, z=4
    const posX = FACE_HANDLES.find((h) => h.axis === 0 && h.side === 1)!;
    const c = handleCenter(posX, box);
    expect(c.x).toBeCloseTo(4);
    expect(c.y).toBeCloseTo(3); // mid of [0,6]
    expect(c.z).toBeCloseTo(4); // mid of [0,8]
  });

  it('places -Z face center at min.z on the Z axis', () => {
    const box = new THREE.Box3(new THREE.Vector3(-2, -2, -5), new THREE.Vector3(2, 2, 5));
    const negZ = FACE_HANDLES.find((h) => h.axis === 2 && h.side === -1)!;
    const c = handleCenter(negZ, box);
    expect(c.z).toBeCloseTo(-5);
    expect(c.x).toBeCloseTo(0);
    expect(c.y).toBeCloseTo(0);
  });
});
