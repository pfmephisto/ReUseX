// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import {
  DEFAULT_LIGHTING,
  lightDirection,
  orthoHalfHeight,
  presetOrientation,
  type CameraProjection,
  type LightingState,
  type ViewPreset,
} from './cameraViews';
import { labelColorIndex, paletteToFloats, readLabelPalette } from './labelColors';

export type ColorMode = 'rgb' | 'label';

/** Oblique framing direction: an axis view of a corridor has no depth cue. */
const OBLIQUE_DIRECTION = new THREE.Vector3(0.7, -0.7, 0.45).normalize();

/** World up for the gravity-aligned scan. */
const WORLD_UP = new THREE.Vector3(0, 0, 1);

/**
 * Radius of the panorama backdrop, metres.
 *
 * Far enough that a room's geometry fits inside it — a sphere smaller than the
 * scan would clip through the walls the user is comparing it against — and
 * near enough to stay well inside the far plane at ordinary building scale.
 */
export const PANORAMA_RADIUS = 20;

/** Fallback look direction when a caller supplies a degenerate one. */
const FORWARD_X = new THREE.Vector3(1, 0, 0);

/** The orbit camera as it was before panorama mode took it over. */
interface SavedCamera {
  position: THREE.Vector3;
  target: THREE.Vector3;
  up: THREE.Vector3;
  near: number;
  far: number;
  fov: number;
  projection: CameraProjection;
}

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
  private readonly canvas: HTMLCanvasElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly scene = new THREE.Scene();
  private readonly perspectiveCamera: THREE.PerspectiveCamera;
  private readonly orthographicCamera: THREE.OrthographicCamera;
  private camera: THREE.PerspectiveCamera | THREE.OrthographicCamera;
  private projection: CameraProjection = 'perspective';
  private readonly controls: OrbitControls;
  private readonly layers = new Map<string, Layer>();

  // Viewport lighting rig (#443). One key + one hemisphere fill live on the
  // scene, not per-mesh: a viewport has one light model, adjustable from the
  // panel and unchanged by which meshes are toggled on. Points and splats draw
  // with unlit materials, so the lights only ever reach a lit mesh.
  private readonly hemisphere: THREE.HemisphereLight;
  private readonly keyLight: THREE.DirectionalLight;
  private lighting: LightingState = { ...DEFAULT_LIGHTING };

  private readonly bounds = new THREE.Box3();
  private origin: THREE.Vector3 | null = null;

  private saved: SavedCamera | null = null;
  private colorMode: ColorMode = 'rgb';
  private pointSize = 0.02;
  private frameHandle: number | null = null;
  private resizeObserver: ResizeObserver | null = null;
  private disposed = false;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.perspectiveCamera = new THREE.PerspectiveCamera(60, 1, 0.01, 5000);
    // Building scans are gravity-aligned with Z up; three.js defaults to Y up,
    // which would make the orbit gimbal fight the data on every drag.
    this.perspectiveCamera.up.copy(WORLD_UP);
    this.perspectiveCamera.position.set(6, -6, 4);

    // Orthographic peer. Its frustum is (re)sized from the perspective view on
    // every projection toggle and reframing, so these placeholder extents are
    // overwritten before it is ever the active camera.
    this.orthographicCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 5000);
    this.orthographicCamera.up.copy(WORLD_UP);
    this.orthographicCamera.position.copy(this.perspectiveCamera.position);

    this.camera = this.perspectiveCamera;

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.screenSpacePanning = true;

    this.hemisphere = new THREE.HemisphereLight(0xdfe4ec, 0x14171c, this.lighting.ambientIntensity);
    this.hemisphere.position.copy(WORLD_UP);
    this.keyLight = new THREE.DirectionalLight(0xffffff, this.lighting.keyIntensity);
    this.scene.add(this.hemisphere);
    this.scene.add(this.keyLight);
    this.applyLighting();

    this.applyBackgroundFromTokens();
    this.observeResize(canvas);
    this.loop();
  }

  /**
   * Take the canvas colour from `--color-canvas` rather than hard-coding it.
   *
   * The viewport background is a design token like any other; reading it here
   * is what keeps the 3D view inside the design system instead of beside it.
   */
  private applyBackgroundFromTokens(): void {
    this.scene.background = this.resolveColorToken('--color-canvas', '#0a0b0d');
  }

  /**
   * Resolve a CSS custom property to a {@link THREE.Color}, from the canvas's
   * own cascade so themed overrides apply.
   *
   * Shared with sibling layers ({@link MeshScene}) that draw into this scene and
   * must take their colours from the same design tokens rather than hard-coding
   * them. Falls back to @p fallback when the property is unset or empty.
   */
  resolveColorToken(name: string, fallback: string): THREE.Color {
    const value = getComputedStyle(this.canvas).getPropertyValue(name).trim();
    return new THREE.Color(value || fallback);
  }

  private observeResize(canvas: HTMLCanvasElement): void {
    const resize = () => {
      const parent = canvas.parentElement;
      if (!parent) return;
      const { clientWidth, clientHeight } = parent;
      if (clientWidth === 0 || clientHeight === 0) return;
      this.renderer.setSize(clientWidth, clientHeight, false);
      const aspect = clientWidth / clientHeight;
      this.perspectiveCamera.aspect = aspect;
      this.perspectiveCamera.updateProjectionMatrix();
      // Hold the orthographic vertical extent and refit the horizontal one, so
      // a resize never changes how tall the scene reads under either camera.
      const halfHeight = (this.orthographicCamera.top - this.orthographicCamera.bottom) / 2;
      this.orthographicCamera.left = -halfHeight * aspect;
      this.orthographicCamera.right = halfHeight * aspect;
      this.orthographicCamera.updateProjectionMatrix();
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

  // --- camera projection (#443) -------------------------------------------

  /** Whether the orbit camera is currently perspective or orthographic. */
  currentProjection(): CameraProjection {
    return this.projection;
  }

  /**
   * Switch the orbit camera between perspective and orthographic, preserving
   * the framing the user was looking at.
   *
   * The two cameras share a target and view direction; only the projection
   * differs. Going to orthographic, the frustum is sized to show the same
   * vertical extent the perspective camera spanned at the target distance;
   * going back, the perspective camera is stood off at the distance that
   * reproduces the orthographic camera's visible height. Either way the scene
   * stays the same size at the plane the user was focused on, so the switch
   * reads as a change of projection rather than a jump.
   *
   * In panorama mode the look-around is always perspective, so a request there
   * is only remembered and applied when the orbit camera returns
   * (see {@link exitFirstPerson}).
   */
  setProjection(next: CameraProjection): void {
    if (this.projection === next) return;
    if (this.saved) {
      this.saved.projection = next;
      return;
    }

    const target = this.controls.target;
    const persp = this.perspectiveCamera;
    const ortho = this.orthographicCamera;

    if (next === 'orthographic') {
      ortho.position.copy(persp.position);
      ortho.up.copy(persp.up);
      ortho.near = persp.near;
      ortho.far = persp.far;
      const distance = persp.position.distanceTo(target);
      this.sizeOrtho(orthoHalfHeight(persp.fov, distance));
      this.camera = ortho;
    } else {
      persp.up.copy(ortho.up);
      persp.near = ortho.near;
      persp.far = ortho.far;
      const halfHeight = (ortho.top - ortho.bottom) / 2 / ortho.zoom;
      const distance = Math.max(
        halfHeight / Math.tan(THREE.MathUtils.degToRad(persp.fov) / 2),
        persp.near * 2,
      );
      const direction = ortho.position.clone().sub(target);
      if (direction.lengthSq() === 0) direction.copy(OBLIQUE_DIRECTION);
      direction.normalize();
      persp.position.copy(target).addScaledVector(direction, distance);
      persp.updateProjectionMatrix();
      this.camera = persp;
    }

    this.controls.object = this.camera;
    this.projection = next;
    this.controls.update();
  }

  /**
   * Reorient the camera to a named axis-aligned view (Top / Front / …),
   * framing the whole loaded scene along that axis.
   *
   * Orbit-only: inside a panorama there is no scene to frame, and the page
   * leaves the panorama before asking for a preset. The orientation convention
   * is the CLI's — see {@link presetOrientation}.
   */
  setView(preset: ViewPreset): void {
    if (this.saved) return;
    const { direction, up } = presetOrientation(preset);
    this.frameFromDirection(direction, up);
  }

  // --- lighting (#443) -----------------------------------------------------

  /** Current key/fill light settings. */
  currentLighting(): LightingState {
    return { ...this.lighting };
  }

  /** Update one or more lighting parameters and drive the scene lights. */
  setLighting(next: Partial<LightingState>): void {
    this.lighting = { ...this.lighting, ...next };
    this.applyLighting();
  }

  private applyLighting(): void {
    this.hemisphere.intensity = this.lighting.ambientIntensity;
    this.keyLight.intensity = this.lighting.keyIntensity;
    this.keyLight.position.copy(lightDirection(this.lighting.azimuth, this.lighting.elevation));
  }

  /** Size the orthographic frustum to a half-height, keeping the aspect ratio. */
  private sizeOrtho(halfHeight: number): void {
    const ortho = this.orthographicCamera;
    const half = Math.max(halfHeight, 1e-3);
    ortho.top = half;
    ortho.bottom = -half;
    ortho.left = -half * this.perspectiveCamera.aspect;
    ortho.right = half * this.perspectiveCamera.aspect;
    ortho.zoom = 1;
    ortho.updateProjectionMatrix();
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

  /**
   * World -> scene coordinates, adopting @p world as the recentring origin if
   * none is set yet.
   *
   * The lighter half of {@link registerExternalBounds}: a panorama's sphere is
   * 20 m of backdrop that must not enter the framing bounds, but its centre
   * still has to be recentred like everything else or it lands at the scan's
   * absolute coordinates. Callers whose content *should* be framable use
   * `registerExternalBounds` instead.
   */
  toSceneLocal(world: THREE.Vector3): THREE.Vector3 {
    if (!this.origin) this.origin = world.clone();
    return world.clone().sub(this.origin);
  }

  /**
   * Current camera frustum, position, and recentring origin.
   *
   * Passed to {@link useCloudStream} so it can frustum-cull the tile index and
   * fetch only what the camera sees. The frustum is computed from the camera's
   * current projection × view matrices — the render loop keeps them current, so
   * this is always valid to call during a camera-change callback.
   */
  getCameraState(): {
    frustum: THREE.Frustum;
    position: THREE.Vector3;
    origin: THREE.Vector3 | null;
  } {
    this.camera.updateWorldMatrix(true, false);
    const projScreenMatrix = new THREE.Matrix4().multiplyMatrices(
      this.camera.projectionMatrix,
      this.camera.matrixWorldInverse,
    );
    return {
      frustum: new THREE.Frustum().setFromProjectionMatrix(projScreenMatrix),
      position: this.camera.position.clone(),
      origin: this.origin,
    };
  }

  /**
   * Register a listener that fires whenever the orbit camera moves.
   *
   * Returns a cleanup function that removes the listener. Used by the tile
   * streaming path to dispatch `rux-camera-move` so {@link useCloudStream}
   * can re-evaluate the visible tile set.
   */
  onCameraChange(listener: () => void): () => void {
    this.controls.addEventListener('change', listener);
    return () => this.controls.removeEventListener('change', listener);
  }

  /**
   * Objects under the pointer, nearest first.
   *
   * Exposed so a sibling layer can be clickable without a second camera or a
   * copy of this one: picking needs the camera that drew the frame, and that
   * camera stays private.
   *
   * @param ndc Pointer position in normalised device coordinates (-1..1).
   */
  pick(ndc: THREE.Vector2, objects: THREE.Object3D[]): THREE.Intersection[] {
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, this.camera);
    return raycaster.intersectObjects(objects, false);
  }

  /**
   * Pick the nearest point-cloud point under the pointer and return its
   * world-space position, or `null` when nothing is hit.
   *
   * Uses the same camera as {@link pick}: the raycaster needs the camera that
   * drew the frame. The threshold is two point radii in world units — large
   * enough to be usable at typical point densities, small enough not to snap
   * across large gaps.
   *
   * The hit position is in **world** coordinates (scene-local + recentring
   * origin), so it can be passed directly to the visibility API which speaks
   * the same world frame as the stored sensor poses.
   *
   * @param ndc Pointer position in normalised device coordinates (-1..1).
   */
  pickCloudPoint(ndc: THREE.Vector2): THREE.Vector3 | null {
    if (!this.origin) return null;

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, this.camera);
    raycaster.params.Points = { threshold: this.pointSize * 2 };

    const objects: THREE.Object3D[] = [];
    for (const layer of this.layers.values()) {
      for (const page of layer.pages) objects.push(page.points);
    }

    const hits = raycaster.intersectObjects(objects, false);
    if (hits.length === 0) return null;

    // Convert scene-local hit back to world coordinates.
    return hits[0].point.clone().add(this.origin);
  }

  /**
   * Stand the camera at @p position looking along @p forward (both in scene
   * coordinates), and remember where it was.
   *
   * This is panorama mode's camera: `OrbitControls` orbiting a target 10 cm
   * ahead is a look-around, and inverting `rotateSpeed` makes a drag grab the
   * image rather than swing around it — the motion every 360 viewer uses.
   * Panning is switched off because there is nowhere to pan to from inside a
   * sphere, and dollying because it would push the camera through the wall.
   *
   * Idempotent: entering twice does not overwrite the saved orbit pose with
   * the immersive one, so stepping panorama-to-panorama still returns the user
   * to where they were standing.
   */
  enterFirstPerson(position: THREE.Vector3, forward: THREE.Vector3): void {
    if (!this.saved) {
      this.saved = {
        position: this.camera.position.clone(),
        target: this.controls.target.clone(),
        up: this.camera.up.clone(),
        near: this.perspectiveCamera.near,
        far: this.perspectiveCamera.far,
        fov: this.perspectiveCamera.fov,
        projection: this.projection,
      };
    }

    // A 360 look-around is perspective by nature; force it here and restore the
    // orthographic choice, if any, on exit.
    this.camera = this.perspectiveCamera;
    this.controls.object = this.perspectiveCamera;
    this.projection = 'perspective';

    const persp = this.perspectiveCamera;
    const direction = forward.lengthSq() > 0 ? forward.clone().normalize() : FORWARD_X.clone();
    persp.position.copy(position);
    persp.up.copy(WORLD_UP);
    persp.near = 0.01;
    persp.far = PANORAMA_RADIUS * 4;
    persp.updateProjectionMatrix();

    this.controls.target.copy(position).addScaledVector(direction, 0.1);
    this.controls.enablePan = false;
    this.controls.enableZoom = false;
    this.controls.rotateSpeed = -0.3;
    this.controls.update();
  }

  /** Put the orbit camera back where {@link enterFirstPerson} found it. */
  exitFirstPerson(): void {
    this.controls.enablePan = true;
    this.controls.enableZoom = true;
    this.controls.rotateSpeed = 1;

    const saved = this.saved;
    this.saved = null;
    if (!saved) return;

    const persp = this.perspectiveCamera;
    persp.position.copy(saved.position);
    persp.up.copy(saved.up);
    persp.near = saved.near;
    persp.far = saved.far;
    persp.fov = saved.fov;
    persp.updateProjectionMatrix();

    this.camera = persp;
    this.controls.object = persp;
    this.projection = 'perspective';
    this.controls.target.copy(saved.target);
    this.controls.update();

    // Rebuild the orthographic projection the user had before, from the
    // now-restored perspective pose so the framing carries across.
    if (saved.projection === 'orthographic') this.setProjection('orthographic');
  }

  // --- clipping box (#444) -------------------------------------------------

  /**
   * Scene-local bounds of all loaded content, or `null` if nothing has been
   * added yet.
   *
   * "Scene-local" means the recentred frame: world coordinates minus the
   * recentring `origin`, the same frame every `THREE.Object3D` in the scene
   * lives in. The returned box is a copy — mutating it has no effect here.
   */
  getSceneBounds(): THREE.Box3 | null {
    if (this.bounds.isEmpty()) return null;
    if (!this.origin) return this.bounds.clone();
    return this.bounds.clone().translate(this.origin.clone().negate());
  }

  /**
   * Apply a set of world-space clipping planes to the renderer.
   *
   * `renderer.clippingPlanes` clips all geometry globally without requiring
   * per-material opt-in (`localClippingEnabled`). Pass an empty array to
   * disable clipping.
   */
  setClippingPlanes(planes: THREE.Plane[]): void {
    this.renderer.clippingPlanes = planes;
  }

  /** Enable or disable orbit-camera interaction without disposing the controls. */
  setControlsEnabled(enabled: boolean): void {
    this.controls.enabled = enabled;
  }

  /**
   * The active camera, for use by scene layers that need to construct a ray.
   *
   * Exposed narrowly so `ClippingBoxLayer` can compute a drag plane without
   * keeping its own camera reference.
   */
  getCamera(): THREE.PerspectiveCamera | THREE.OrthographicCamera {
    return this.camera;
  }

  /** The canvas element the renderer draws into. */
  getCanvas(): HTMLCanvasElement {
    return this.canvas;
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
    // Approach from an oblique angle rather than an axis: an axis-aligned view
    // of a corridor scan is a wall of points with no depth cue at all.
    this.frameFromDirection(OBLIQUE_DIRECTION, WORLD_UP);
  }

  /**
   * Frame the whole loaded scene along a given view direction and up vector.
   *
   * Shared by {@link frameAll} (oblique) and {@link setView} (axis presets).
   * @p direction points from the framed centre toward the camera.
   */
  private frameFromDirection(direction: THREE.Vector3, up: THREE.Vector3): void {
    if (this.bounds.isEmpty() || !this.origin) return;

    const sphere = new THREE.Sphere();
    this.bounds.getBoundingSphere(sphere);
    sphere.center.sub(this.origin);

    const radius = Math.max(sphere.radius, 0.5);
    const fov = THREE.MathUtils.degToRad(this.perspectiveCamera.fov);
    const distance = (radius / Math.sin(fov / 2)) * 1.15;

    const dir = direction.lengthSq() > 0 ? direction.clone().normalize() : OBLIQUE_DIRECTION.clone();
    const position = sphere.center.clone().addScaledVector(dir, distance);
    const near = Math.max(radius / 1000, 0.01);
    const far = distance + radius * 4;

    // In panorama mode the camera is standing inside a sphere, and the framing
    // request is almost always the cloud's own first page arriving a moment
    // after the backdrop. Flying out of the picture the user asked for would
    // be the wrong reading of it — so the framed pose is written into the
    // remembered orbit camera, and exiting lands on it.
    if (this.saved) {
      this.saved.position.copy(position);
      this.saved.target.copy(sphere.center);
      this.saved.up.copy(up);
      this.saved.near = near;
      this.saved.far = far;
      return;
    }

    for (const cam of [this.perspectiveCamera, this.orthographicCamera]) {
      cam.position.copy(position);
      cam.up.copy(up);
      cam.near = near;
      cam.far = far;
    }
    // The orthographic camera holds the same bounding sphere with the same 15%
    // margin the perspective distance uses, so a toggle after framing is seamless.
    this.sizeOrtho(radius * 1.15);
    this.perspectiveCamera.updateProjectionMatrix();

    this.controls.target.copy(sphere.center);
    this.controls.update();
  }

  dispose(): void {
    this.disposed = true;
    if (this.frameHandle !== null) cancelAnimationFrame(this.frameHandle);
    this.resizeObserver?.disconnect();
    this.clear();
    this.scene.remove(this.hemisphere);
    this.scene.remove(this.keyLight);
    this.hemisphere.dispose();
    this.keyLight.dispose();
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
