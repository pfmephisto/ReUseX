// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';

import type { PointCloudScene } from './PointCloudScene';

/**
 * Mesh layer of the viewport (#265, review point 2).
 *
 * Loads a PLY geometry blob from the API and adds it to the host
 * {@link PointCloudScene}. Works exactly like `SplatScene`: the host scene and
 * camera are shared, so the mesh is depth-sorted correctly against the point
 * cloud without a second renderer.
 *
 * ## Material
 *
 * `MeshNormalMaterial` is the default — it requires no lights and gives an
 * immediate read of the surface orientation, which is what "does this mesh look
 * right?" requires. A wireframe toggle switches to `MeshBasicMaterial` so the
 * triangle topology is visible (the cell-complex cells are large triangulated
 * quads; their count is a legible structural hint about room boundaries).
 *
 * ## Coordinates
 *
 * PLY coordinates are in the same world frame as the point cloud — both
 * originate from the same `ProjectDB`. The host scene applies a centroid
 * offset to keep float32 precision; the mesh group position is set to the
 * negated origin so the mesh lands on the same grid as the cloud.
 *
 * ## Dynamic import
 *
 * `PLYLoader` is dynamic-imported inside `load()`, as the splat library is in
 * `SplatScene`. The mesh blob is unlikely on most projects, so the loader
 * must not inflate the initial bundle.
 */
export class MeshScene {
  private group: THREE.Group | null = null;
  private surfaceMaterial: THREE.MeshNormalMaterial | null = null;
  private wireframeMaterial: THREE.MeshBasicMaterial | null = null;

  private _visible = true;
  private _wireframe = false;
  private disposed = false;

  constructor(private readonly host: PointCloudScene) {}

  /**
   * Download the PLY blob at @p url and add the mesh to the scene.
   *
   * Resolves once the mesh is on screen. Rejects with the loader error,
   * consistent with `SplatScene.load` — a mesh that silently fails to appear
   * is indistinguishable from one that never existed.
   */
  async load(url: string): Promise<void> {
    if (this.disposed) return;

    const { PLYLoader } = await import('three/examples/jsm/loaders/PLYLoader.js');
    if (this.disposed) return;

    const geometry = await new Promise<THREE.BufferGeometry>((resolve, reject) => {
      new PLYLoader().load(url, resolve, undefined, (error) =>
        reject(error instanceof Error ? error : new Error(String(error))),
      );
    });

    if (this.disposed) {
      geometry.dispose();
      return;
    }

    const posAttr = geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    if (!posAttr || posAttr.count === 0) {
      geometry.dispose();
      return;
    }

    geometry.computeBoundingBox();
    const box = geometry.boundingBox ?? new THREE.Box3().setFromBufferAttribute(posAttr);

    const origin = this.host.registerExternalBounds(box.clone());

    const surfaceMaterial = new THREE.MeshNormalMaterial({ side: THREE.DoubleSide });
    const wireframeMaterial = new THREE.MeshBasicMaterial({
      color: 0x88aacc,
      side: THREE.DoubleSide,
      wireframe: true,
    });

    const mesh = new THREE.Mesh(geometry, this._wireframe ? wireframeMaterial : surfaceMaterial);

    const group = new THREE.Group();
    group.position.copy(origin).negate();
    group.visible = this._visible;
    group.add(mesh);

    this.host.sceneRoot().add(group);
    this.surfaceMaterial = surfaceMaterial;
    this.wireframeMaterial = wireframeMaterial;
    this.group = group;
  }

  /** Show or hide the layer without unloading the geometry. */
  setVisible(visible: boolean): void {
    this._visible = visible;
    if (this.group) this.group.visible = visible;
  }

  /** Toggle between normal-vector colouring and wireframe mode. */
  setWireframe(wireframe: boolean): void {
    this._wireframe = wireframe;
    if (!this.group) return;
    this.group.traverse((obj) => {
      if (obj instanceof THREE.Mesh) {
        obj.material = wireframe
          ? (this.wireframeMaterial ?? obj.material)
          : (this.surfaceMaterial ?? obj.material);
      }
    });
  }

  dispose(): void {
    this.disposed = true;
    const group = this.group;
    this.group = null;
    if (group) {
      group.removeFromParent();
      group.traverse((obj) => {
        if (obj instanceof THREE.Mesh) obj.geometry.dispose();
      });
    }
    this.surfaceMaterial?.dispose();
    this.wireframeMaterial?.dispose();
    this.surfaceMaterial = null;
    this.wireframeMaterial = null;
  }
}
