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
 * The default surface is a lit `MeshStandardMaterial` in a neutral gray taken
 * from the `--mesh-surface` design token. Cell-complex meshes are exported
 * without vertex colours *or* normals, and the earlier `MeshNormalMaterial`
 * rendered them flat black — `normalize((0,0,0))` on a missing normal is `NaN`,
 * which the shader packs as black (#441). `flatShading` sidesteps the missing
 * normals by deriving a per-face normal in the fragment shader, so the facets
 * read under lighting instead of collapsing to a single tone. A mesh that *does*
 * carry usable vertex colours keeps them (`vertexColors`), so this only supplies
 * a colour where the geometry has none. A wireframe toggle switches to
 * `MeshBasicMaterial` so the triangle topology is visible (the cell-complex
 * cells are large triangulated quads; their count is a legible structural hint
 * about room boundaries).
 *
 * ## Lighting
 *
 * The lit surface needs light, and the host {@link PointCloudScene} has none of
 * its own — points and splats draw with unlit materials, so they are unaffected
 * by the hemisphere + directional pair added here on {@link load}. The lights
 * live only as long as the mesh does and are removed in {@link dispose}.
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
/**
 * Fallback surface gray when the `--mesh-surface` token cannot be resolved.
 *
 * A mid neutral gray that reads as lit solid geometry against the near-black
 * viewport canvas. Kept in sync with the token's placeholder value.
 */
const DEFAULT_MESH_GRAY = '#8a8f99';

export class MeshScene {
  private group: THREE.Group | null = null;
  private surfaceMaterial: THREE.MeshStandardMaterial | null = null;
  private wireframeMaterial: THREE.MeshBasicMaterial | null = null;
  private lights: THREE.Light[] = [];

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

    const hasColors = hasUsableVertexColors(geometry);
    const surfaceMaterial = new THREE.MeshStandardMaterial({
      // White base lets vertex colours through unmodulated; the neutral gray is
      // only the fallback for geometry that carries no colour of its own.
      color: hasColors
        ? new THREE.Color(0xffffff)
        : this.host.resolveColorToken('--mesh-surface', DEFAULT_MESH_GRAY),
      vertexColors: hasColors,
      side: THREE.DoubleSide,
      metalness: 0,
      roughness: 0.85,
      flatShading: true,
    });
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
    this.addLights();
    this.surfaceMaterial = surfaceMaterial;
    this.wireframeMaterial = wireframeMaterial;
    this.group = group;
  }

  /**
   * Add the lights the lit surface material needs, once.
   *
   * A hemisphere light gives an even sky/ground fill aligned with the scan's
   * gravity axis (Z up), and an oblique directional key makes adjacent facets
   * differ so the geometry reads as solid rather than flat. Both are added to
   * the shared scene root; position-independent (hemisphere) and
   * direction-only (directional) lighting means the group's recentring offset
   * does not affect them.
   */
  private addLights(): void {
    if (this.lights.length > 0) return;

    const hemisphere = new THREE.HemisphereLight(0xdfe4ec, 0x14171c, 1.4);
    hemisphere.position.set(0, 0, 1);

    const key = new THREE.DirectionalLight(0xffffff, 2.2);
    key.position.set(4, -6, 8);

    const root = this.host.sceneRoot();
    root.add(hemisphere);
    root.add(key);
    this.lights = [hemisphere, key];
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
    for (const light of this.lights) {
      light.removeFromParent();
      light.dispose();
    }
    this.lights = [];
    this.surfaceMaterial?.dispose();
    this.wireframeMaterial?.dispose();
    this.surfaceMaterial = null;
    this.wireframeMaterial = null;
  }
}

/**
 * Whether a geometry carries vertex colours worth showing.
 *
 * Presence of a `color` attribute is not enough: a common export default is an
 * all-black colour attribute, which is exactly the case that makes a mesh
 * invisible. Treat anything darker than a few percent on every channel of every
 * vertex as "no colour", so the neutral-gray fallback applies to it too.
 */
function hasUsableVertexColors(geometry: THREE.BufferGeometry): boolean {
  const color = geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
  if (!color || color.count === 0) return false;

  const data = color.array as ArrayLike<number>;
  // Integer colour attributes (e.g. Uint8 from PLY) are 0..255; float ones are
  // 0..1. ~3% of full scale is the "effectively black" cut in each encoding.
  const threshold = color.normalized || !(color.array instanceof Float32Array) ? 8 : 8 / 255;
  for (let i = 0; i < data.length; i += 1) {
    if (data[i] > threshold) return true;
  }
  return false;
}
