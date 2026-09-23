// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';

import type { PointCloudScene } from './PointCloudScene';
import { FACE_HANDLES, boxToPlanes, handleCenter } from './clippingBox';

/**
 * Three.js scene layer for the interactive clipping box (#444).
 *
 * Draws a wireframe box outline and six spherical face handles in the scene.
 * When a handle is dragged the corresponding face moves, the six renderer
 * clipping planes update live, and `onChange` is called on pointer-up so the
 * React layer panel can synchronise its sliders.
 *
 * All coordinates are in scene-local space (world − recentring origin), the
 * same frame every other object in the scene inhabits.
 */

/** Minimum allowed size per axis, in metres. Prevents degenerate boxes. */
const MIN_AXIS_SIZE = 0.01;

/** Handle sphere colour — accent blue. */
const HANDLE_COLOR = 0x6ea8fe;
/** Wireframe box colour — dimmed accent. */
const WIRE_COLOR = 0x3b5a8a;
/** Handle colour while dragging. */
const HANDLE_DRAG_COLOR = 0xffa500;
/** Handle sphere radius in metres. */
const HANDLE_BASE_RADIUS = 0.08;

export class ClippingBoxLayer {
  private readonly host: PointCloudScene;
  private readonly canvas: HTMLCanvasElement;
  private readonly onChange: (box: THREE.Box3) => void;

  private readonly group = new THREE.Group();
  private readonly handles: THREE.Mesh[] = [];
  private wireframe: THREE.LineSegments | null = null;

  private box = new THREE.Box3();
  private enabled = false;

  // Drag state
  private drag: {
    handleIndex: number;
    dragPlane: THREE.Plane;
    startFacePos: number;
    startHitPos: number;
  } | null = null;

  constructor(host: PointCloudScene, onChange: (box: THREE.Box3) => void) {
    this.host = host;
    this.canvas = host.getCanvas();
    this.onChange = onChange;

    this.buildHandles();
    this.group.visible = false;
    host.sceneRoot().add(this.group);

    this.canvas.addEventListener('pointerdown', this.onPointerDown);
    this.canvas.addEventListener('pointermove', this.onPointerMove);
    this.canvas.addEventListener('pointerup', this.onPointerUp);
    this.canvas.addEventListener('pointercancel', this.onPointerUp);
  }

  // --- public API ----------------------------------------------------------

  setEnabled(enabled: boolean): void {
    this.enabled = enabled;
    this.group.visible = enabled;
    if (enabled) {
      if (this.box.isEmpty()) this.initFromSceneBounds();
      this.applyPlanes();
    } else {
      this.host.setClippingPlanes([]);
    }
  }

  setBox(nextBox: THREE.Box3): void {
    if (this.box.equals(nextBox)) return;
    this.box.copy(nextBox);
    this.rebuildWireframe();
    this.updateHandlePositions();
    if (this.enabled) this.applyPlanes();
  }

  resetToSceneBounds(): void {
    this.initFromSceneBounds();
    if (this.enabled) this.applyPlanes();
    this.onChange(this.box.clone());
  }

  dispose(): void {
    this.canvas.removeEventListener('pointerdown', this.onPointerDown);
    this.canvas.removeEventListener('pointermove', this.onPointerMove);
    this.canvas.removeEventListener('pointerup', this.onPointerUp);
    this.canvas.removeEventListener('pointercancel', this.onPointerUp);

    this.host.setClippingPlanes([]);
    this.host.setControlsEnabled(true);
    this.host.sceneRoot().remove(this.group);
    this.disposeGroup();
  }

  // --- scene construction --------------------------------------------------

  private buildHandles(): void {
    const geo = new THREE.SphereGeometry(1, 8, 6);
    for (let i = 0; i < FACE_HANDLES.length; i++) {
      const mat = new THREE.MeshBasicMaterial({ color: HANDLE_COLOR });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.userData['handleIndex'] = i;
      this.handles.push(mesh);
      this.group.add(mesh);
    }
  }

  private rebuildWireframe(): void {
    if (this.wireframe) {
      this.group.remove(this.wireframe);
      this.wireframe.geometry.dispose();
      (this.wireframe.material as THREE.Material).dispose();
      this.wireframe = null;
    }
    const size = this.box.getSize(new THREE.Vector3());
    const center = this.box.getCenter(new THREE.Vector3());
    const boxGeo = new THREE.BoxGeometry(size.x, size.y, size.z);
    const edgesGeo = new THREE.EdgesGeometry(boxGeo);
    boxGeo.dispose();
    const mat = new THREE.LineBasicMaterial({ color: WIRE_COLOR });
    this.wireframe = new THREE.LineSegments(edgesGeo, mat);
    this.wireframe.position.copy(center);
    this.group.add(this.wireframe);
  }

  private updateHandlePositions(): void {
    const radius = this.handleRadius();
    for (let i = 0; i < FACE_HANDLES.length; i++) {
      const c = handleCenter(FACE_HANDLES[i], this.box);
      this.handles[i].position.copy(c);
      this.handles[i].scale.setScalar(radius);
    }
  }

  private handleRadius(): number {
    return Math.max(HANDLE_BASE_RADIUS, this.box.getBoundingSphere(new THREE.Sphere()).radius * 0.03);
  }

  // --- internal helpers ----------------------------------------------------

  private initFromSceneBounds(): void {
    const sceneBounds = this.host.getSceneBounds();
    if (sceneBounds && !sceneBounds.isEmpty()) {
      this.box.copy(sceneBounds);
    } else {
      // Fallback when no content is loaded yet
      this.box.set(new THREE.Vector3(-5, -5, -2), new THREE.Vector3(5, 5, 3));
    }
    this.rebuildWireframe();
    this.updateHandlePositions();
  }

  private applyPlanes(): void {
    this.host.setClippingPlanes(boxToPlanes(this.box));
  }

  private ndcFromEvent(event: PointerEvent): THREE.Vector2 {
    const rect = this.canvas.getBoundingClientRect();
    return new THREE.Vector2(
      ((event.clientX - rect.left) / rect.width) * 2 - 1,
      -((event.clientY - rect.top) / rect.height) * 2 + 1,
    );
  }

  private raycastPlane(ndc: THREE.Vector2, plane: THREE.Plane): THREE.Vector3 | null {
    const camera = this.host.getCamera();
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, camera);
    const target = new THREE.Vector3();
    return raycaster.ray.intersectPlane(plane, target);
  }

  private disposeGroup(): void {
    for (const mesh of this.handles) {
      mesh.geometry.dispose();
      (mesh.material as THREE.Material).dispose();
    }
    if (this.wireframe) {
      this.wireframe.geometry.dispose();
      (this.wireframe.material as THREE.Material).dispose();
    }
    this.group.clear();
  }

  // --- pointer event handlers ----------------------------------------------

  private readonly onPointerDown = (event: PointerEvent): void => {
    if (!this.enabled || event.button !== 0) return;

    const ndc = this.ndcFromEvent(event);
    const hits = this.host.pick(ndc, this.handles);
    if (hits.length === 0) return;

    const hitMesh = hits[0].object as THREE.Mesh;
    const handleIndex = hitMesh.userData['handleIndex'] as number;
    if (handleIndex === undefined) return;

    event.stopPropagation();
    this.canvas.setPointerCapture(event.pointerId);
    this.host.setControlsEnabled(false);

    // Build a drag plane facing the camera through the handle center
    const camera = this.host.getCamera();
    const camDir = new THREE.Vector3();
    camera.getWorldDirection(camDir);
    const planeNormal = camDir.negate().normalize();
    const center = handleCenter(FACE_HANDLES[handleIndex], this.box);
    const dragPlane = new THREE.Plane().setFromNormalAndCoplanarPoint(planeNormal, center);

    // Project start hit onto the constraint axis
    const startHit = this.raycastPlane(ndc, dragPlane);
    const axisVec = FACE_HANDLES[handleIndex].inwardNormal.clone().negate(); // outward = constraint axis
    const startHitPos = startHit ? axisVec.dot(startHit) : axisVec.dot(center);
    const startFacePos = axisVec.dot(center); // current face position along axis

    this.drag = { handleIndex, dragPlane, startFacePos, startHitPos };

    // Visual: highlight dragged handle
    const mat = hitMesh.material as THREE.MeshBasicMaterial;
    mat.color.setHex(HANDLE_DRAG_COLOR);
  };

  private readonly onPointerMove = (event: PointerEvent): void => {
    if (!this.drag) return;

    const ndc = this.ndcFromEvent(event);
    const hit = this.raycastPlane(ndc, this.drag.dragPlane);
    if (!hit) return;

    const faceHandle = FACE_HANDLES[this.drag.handleIndex];
    const axisVec = faceHandle.inwardNormal.clone().negate(); // outward normal = constraint axis
    const currentHitPos = axisVec.dot(hit);
    const delta = currentHitPos - this.drag.startHitPos;
    const newFacePos = this.drag.startFacePos + delta;

    const nextBox = this.box.clone();
    const KEYS: readonly ['x', 'y', 'z'] = ['x', 'y', 'z'];
    const key = KEYS[faceHandle.axis];

    if (faceHandle.side > 0) {
      nextBox.max[key] = Math.max(newFacePos, nextBox.min[key] + MIN_AXIS_SIZE);
    } else {
      nextBox.min[key] = Math.min(newFacePos, nextBox.max[key] - MIN_AXIS_SIZE);
    }

    this.box.copy(nextBox);
    this.rebuildWireframe();
    this.updateHandlePositions();
    if (this.enabled) this.applyPlanes();
  };

  private readonly onPointerUp = (event: PointerEvent): void => {
    if (!this.drag) return;

    // Restore handle colour
    const dragIndex = this.drag.handleIndex;
    const mat = this.handles[dragIndex].material as THREE.MeshBasicMaterial;
    mat.color.setHex(HANDLE_COLOR);

    this.drag = null;
    this.canvas.releasePointerCapture(event.pointerId);
    this.host.setControlsEnabled(true);

    // Notify React so sliders sync to the final drag position
    this.onChange(this.box.clone());
  };
}
