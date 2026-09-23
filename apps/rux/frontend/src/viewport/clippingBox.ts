// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';

/**
 * Pure geometry for the viewport clipping box (#444).
 *
 * Separated from `ClippingBoxLayer.ts` for the same reason `cameraViews.ts` is
 * kept apart from `PointCloudScene.ts`: the conversion from a bounding box to
 * six clip planes is a pure function of its inputs, unit-testable without a
 * WebGL context.
 */

/**
 * Convert a `Box3` into six clipping planes that together keep only the
 * interior of the box visible.
 *
 * Three.js `renderer.clippingPlanes` discards fragments on the *negative* side
 * of each plane. A plane defined by `(normal, constant)` satisfies
 * `dot(normal, v) + constant ≥ 0` for visible points. Six planes, one per face,
 * invert the box's outward normals so that being inside the box is the
 * "positive" side of every plane simultaneously.
 */
export function boxToPlanes(box: THREE.Box3): THREE.Plane[] {
  return [
    // +X face: keep x ≤ max.x → normal (-1,0,0), constant = max.x
    new THREE.Plane(new THREE.Vector3(-1, 0, 0), box.max.x),
    // -X face: keep x ≥ min.x → normal (1,0,0), constant = -min.x
    new THREE.Plane(new THREE.Vector3(1, 0, 0), -box.min.x),
    // +Y face: keep y ≤ max.y
    new THREE.Plane(new THREE.Vector3(0, -1, 0), box.max.y),
    // -Y face: keep y ≥ min.y
    new THREE.Plane(new THREE.Vector3(0, 1, 0), -box.min.y),
    // +Z face: keep z ≤ max.z
    new THREE.Plane(new THREE.Vector3(0, 0, -1), box.max.z),
    // -Z face: keep z ≥ min.z
    new THREE.Plane(new THREE.Vector3(0, 0, 1), -box.min.z),
  ];
}

/**
 * The six face handles of a clipping box, one per face.
 *
 * Each handle describes which axis and sign (positive = max face, negative =
 * min face) it controls, and what the dragging-plane normal is (the inward face
 * normal, pointing toward box interior). Used by `ClippingBoxLayer` to map a
 * dragged handle to the box parameter it should move.
 */
export interface FaceHandle {
  /** 0=X, 1=Y, 2=Z */
  axis: 0 | 1 | 2;
  /** +1 = max face, -1 = min face */
  side: 1 | -1;
  /** Unit normal pointing inward from this face. */
  inwardNormal: THREE.Vector3;
}

export const FACE_HANDLES: readonly FaceHandle[] = [
  { axis: 0, side: 1, inwardNormal: new THREE.Vector3(-1, 0, 0) },
  { axis: 0, side: -1, inwardNormal: new THREE.Vector3(1, 0, 0) },
  { axis: 1, side: 1, inwardNormal: new THREE.Vector3(0, -1, 0) },
  { axis: 1, side: -1, inwardNormal: new THREE.Vector3(0, 1, 0) },
  { axis: 2, side: 1, inwardNormal: new THREE.Vector3(0, 0, -1) },
  { axis: 2, side: -1, inwardNormal: new THREE.Vector3(0, 0, 1) },
];

/** Face-center position in scene-local space for a given handle and box. */
export function handleCenter(handle: FaceHandle, box: THREE.Box3): THREE.Vector3 {
  const center = new THREE.Vector3();
  box.getCenter(center);
  const facePos = handle.side > 0 ? box.max.getComponent(handle.axis) : box.min.getComponent(handle.axis);
  center.setComponent(handle.axis, facePos);
  return center;
}
