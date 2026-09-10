// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import { labelColorIndex, paletteToFloats, readLabelPalette } from './labelColors';

export type ColorMode = 'rgb' | 'label';

/** One decoded page, in the form the scene consumes. */
export interface PageBuffers {
  /** xyz triples, in world coordinates as stored. */
  positions: Float32Array;
  /** rgb triples in 0..1, or null for a cloud with no colour. */
  rgb: Float32Array | null;
  /** Per-point label values, or null when no label source is selected. */
  labels: Uint32Array | null;
}

interface Page {
  points: THREE.Points;
  rgb: Float32Array | null;
  labels: Uint32Array | null;
}

interface Layer {
  group: THREE.Group;
  pages: Page[];
}

/**
 * Imperative three.js scene for streamed point clouds.
 *
 * Kept entirely outside React on purpose. A cloud arrives as dozens to hundreds
 * of pages, and re-rendering a component tree per page — or holding GPU buffers
 * in component state — would make React's reconciler the bottleneck in
 * something that is really just "append a buffer and redraw". The React side
 * (`Viewport.tsx`) owns the canvas element and the lifecycle; everything after
 * that is method calls.
 *
 * ## Coordinate handling
 *
 * Positions are recentred on the first page's bounding-box centre before they
 * reach the GPU. Scans can be georeferenced, and a `Float32Array` holding
 * coordinates in the hundreds of thousands of metres has decimetre resolution
 * left — the cloud visibly quantises onto a lattice and the depth buffer
 * shatters. The library has the same problem and solves it the same way
 * (`CellComplex` recentres to its bbox centroid, STANDARDS §4). The offset is
 * fixed by the first page so that later pages stay registered to it.
 */
export class PointCloudScene {
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly layers = new Map<string, Layer>();

  private readonly bounds = new THREE.Box3();
  private origin: THREE.Vector3 | null = null;

  private colorMode: ColorMode = 'rgb';
  private pointSize = 0.02;
  private frameHandle: number | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private disposed = false;

  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.camera = new THREE.PerspectiveCamera(60, 1, 0.01, 5000);
    // Building scans are gravity-aligned with Z up; three.js defaults to Y up,
    // which would make the orbit gimbal fight the data on every drag.
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(6, -6, 4);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.screenSpacePanning = true;

    this.applyBackgroundFromTokens(canvas);
    this.observeResize(canvas);
    this.loop();
  }

  /**
   * Take the canvas colour from `--color-canvas` rather than hard-coding it.
   *
   * The viewport background is a design token like any other; reading it here
   * is what keeps the 3D view inside the design system instead of beside it.
   */
  private applyBackgroundFromTokens(canvas: HTMLCanvasElement): void {
    const value = getComputedStyle(canvas).getPropertyValue('--color-canvas').trim();
    this.scene.background = new THREE.Color(value || '#0a0b0d');
  }

  private observeResize(canvas: HTMLCanvasElement): void {
    const resize = () => {
      const parent = canvas.parentElement;
      if (!parent) return;
      const { clientWidth, clientHeight } = parent;
      if (clientWidth === 0 || clientHeight === 0) return;
      this.renderer.setSize(clientWidth, clientHeight, false);
      this.camera.aspect = clientWidth / clientHeight;
      this.camera.updateProjectionMatrix();
    };
    resize();
    this.resizeObserver = new ResizeObserver(resize);
    if (canvas.parentElement) this.resizeObserver.observe(canvas.parentElement);
  }

  private loop = (): void => {
    if (this.disposed) return;
    this.frameHandle = requestAnimationFrame(this.loop);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  };

  /** Append one decoded page to a layer, creating the layer if needed. */
  addPage(layerId: string, page: PageBuffers): void {
    if (this.disposed) return;

    const count = Math.floor(page.positions.length / 3);
    if (count === 0) return;

    if (!this.origin) this.origin = centroidOf(page.positions);
    const local = recentre(page.positions, this.origin);

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(local, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(count * 3), 3));
    geometry.computeBoundingSphere();

    const material = new THREE.PointsMaterial({
      size: this.pointSize,
      sizeAttenuation: true,
      vertexColors: true,
    });

    const points = new THREE.Points(geometry, material);
    const record: Page = { points, rgb: page.rgb, labels: page.labels };
    this.writeColors(record);

    let layer = this.layers.get(layerId);
    if (!layer) {
      layer = { group: new THREE.Group(), pages: [] };
      this.scene.add(layer.group);
      this.layers.set(layerId, layer);
    }
    layer.group.add(points);
    layer.pages.push(record);

    this.growBounds(page.positions);
  }

  /**
   * Fill a page's colour attribute for the current mode.
   *
   * Colours are recomputed from the retained source arrays rather than refetched
   * when the mode changes: a 20-million-point cloud costs one pass over memory
   * to recolour and a full reload to refetch, and the user toggling RGB/label to
   * compare them is the normal way this control gets used.
   */
  private writeColors(page: Page): void {
    const attribute = page.points.geometry.getAttribute('color') as THREE.BufferAttribute;
    const target = attribute.array as Float32Array;
    const count = attribute.count;

    if (this.colorMode === 'label' && page.labels) {
      const palette = paletteToFloats(readLabelPalette());
      for (let i = 0; i < count; i += 1) {
        const slot = labelColorIndex(page.labels[i], palette.colors.length);
        const [r, g, b] = slot < 0 ? palette.unlabeled : palette.colors[slot];
        target[i * 3] = r;
        target[i * 3 + 1] = g;
        target[i * 3 + 2] = b;
      }
    } else if (page.rgb) {
      target.set(page.rgb.subarray(0, count * 3));
    } else {
      // A PointXYZ cloud has no colour of its own. Flat neutral rather than
      // white: white points on a near-black canvas bloom and hide structure.
      const [r, g, b] = paletteToFloats(readLabelPalette()).unlabeled;
      for (let i = 0; i < count; i += 1) {
        target[i * 3] = r;
        target[i * 3 + 1] = g;
        target[i * 3 + 2] = b;
      }
    }
    attribute.needsUpdate = true;
  }

  setColorMode(mode: ColorMode): void {
    if (this.colorMode === mode) return;
    this.colorMode = mode;
    for (const layer of this.layers.values()) {
      for (const page of layer.pages) this.writeColors(page);
    }
  }

  currentColorMode(): ColorMode {
    return this.colorMode;
  }

  setPointSize(size: number): void {
    this.pointSize = size;
    for (const layer of this.layers.values()) {
      for (const page of layer.pages) {
        (page.points.material as THREE.PointsMaterial).size = size;
      }
    }
  }

  setLayerVisible(layerId: string, visible: boolean): void {
    const layer = this.layers.get(layerId);
    if (layer) layer.group.visible = visible;
  }

  /** True when at least one point has been added. */
  hasContent(): boolean {
    return !this.bounds.isEmpty();
  }

  /**
   * The scene root, so a sibling layer can draw with this camera (#322).
   *
   * The Gaussian-splat layer is the caller: it renders through its own shader
   * but must share the camera and `OrbitControls`, or the viewport would have
   * two cameras the user has to orbit separately.
   */
  sceneRoot(): THREE.Scene {
    return this.scene;
  }

  /**
   * Register world-space bounds for content this scene draws but does not own,
   * and get back the offset that content must be positioned by.
   *
   * Two things at once because they are two halves of the same fact. The
   * recentring described above applies to *everything* in this scene, not just
   * to points: a splat left at its georeferenced coordinates while the cloud is
   * recentred lands kilometres away. And bounds a layer never contributed are
   * bounds `frameAll` cannot frame — which is exactly the splat-only case,
   * where no page has run `growBounds`.
   *
   * @param box World-space bounds, in the same frame the point pages arrive in.
   * @returns The recentring origin, adopted from @p box if none is set yet.
   */
  registerExternalBounds(box: THREE.Box3): THREE.Vector3 {
    if (!this.origin) this.origin = box.getCenter(new THREE.Vector3());
    this.bounds.union(box);
    return this.origin.clone();
  }

  removeLayer(layerId: string): void {
    const layer = this.layers.get(layerId);
    if (!layer) return;
    for (const page of layer.pages) disposePoints(page.points);
    this.scene.remove(layer.group);
    this.layers.delete(layerId);
  }

  /** Drop every layer and reset the recentring origin. */
  clear(): void {
    for (const id of [...this.layers.keys()]) this.removeLayer(id);
    this.bounds.makeEmpty();
    this.origin = null;
  }

  private growBounds(positions: Float32Array): void {
    const point = new THREE.Vector3();
    for (let i = 0; i < positions.length; i += 3) {
      point.set(positions[i], positions[i + 1], positions[i + 2]);
      this.bounds.expandByPoint(point);
    }
  }

  /** Move the camera so the whole loaded cloud is in frame. */
  frameAll(): void {
    if (this.bounds.isEmpty() || !this.origin) return;

    const sphere = new THREE.Sphere();
    this.bounds.getBoundingSphere(sphere);
    sphere.center.sub(this.origin);

    const radius = Math.max(sphere.radius, 0.5);
    const fov = THREE.MathUtils.degToRad(this.camera.fov);
    const distance = (radius / Math.sin(fov / 2)) * 1.15;

    // Approach from an oblique angle rather than an axis: an axis-aligned view
    // of a corridor scan is a wall of points with no depth cue at all.
    const direction = new THREE.Vector3(0.7, -0.7, 0.45).normalize();
    this.camera.position.copy(sphere.center).addScaledVector(direction, distance);
    this.camera.near = Math.max(radius / 1000, 0.01);
    this.camera.far = distance + radius * 4;
    this.camera.updateProjectionMatrix();

    this.controls.target.copy(sphere.center);
    this.controls.update();
  }

  dispose(): void {
    this.disposed = true;
    if (this.frameHandle !== null) cancelAnimationFrame(this.frameHandle);
    this.resizeObserver?.disconnect();
    this.clear();
    this.controls.dispose();
    this.renderer.dispose();
  }
}

function centroidOf(positions: Float32Array): THREE.Vector3 {
  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    minX = Math.min(minX, positions[i]);
    maxX = Math.max(maxX, positions[i]);
    minY = Math.min(minY, positions[i + 1]);
    maxY = Math.max(maxY, positions[i + 1]);
    minZ = Math.min(minZ, positions[i + 2]);
    maxZ = Math.max(maxZ, positions[i + 2]);
  }
  return new THREE.Vector3((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
}

function recentre(positions: Float32Array, origin: THREE.Vector3): Float32Array {
  const out = new Float32Array(positions.length);
  for (let i = 0; i < positions.length; i += 3) {
    out[i] = positions[i] - origin.x;
    out[i + 1] = positions[i + 1] - origin.y;
    out[i + 2] = positions[i + 2] - origin.z;
  }
  return out;
}

function disposePoints(points: THREE.Points): void {
  points.geometry.dispose();
  const material = points.material;
  if (Array.isArray(material)) material.forEach((entry) => entry.dispose());
  else material.dispose();
}
