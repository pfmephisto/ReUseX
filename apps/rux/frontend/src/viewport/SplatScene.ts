// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';
import type { DropInViewer } from '@mkkellogg/gaussian-splats-3d';

import type { PointCloudScene } from './PointCloudScene';

/**
 * The Gaussian-splat layer of the viewport (#322).
 *
 * ## Why this renderer
 *
 * `@mkkellogg/gaussian-splats-3d` (MIT), because it reads the INRIA `.ply` that
 * `rux create gsplat` writes with no conversion step, it is pure three.js
 * against the `three` version this app already pins (its peer range is
 * `>=0.160`), and its `DropInViewer` is designed for exactly this case — a host
 * application that owns the canvas. The alternative considered was
 * `luma-web`: broader format support and a WebGPU path, but it is built around
 * Luma's own hosted captures and its Three.js integration wants to own the
 * render loop. Neither advantage applies to a local file in a viewport that
 * already exists.
 *
 * ## Embedded, not overlaid
 *
 * The issue leaves open whether to embed in the existing scene or stack a
 * second canvas. This embeds: `DropInViewer` extends `THREE.Group` and runs its
 * depth sort from an `onBeforeRender` hook on an invisible child, so it draws
 * with whatever camera drew the frame. One canvas, one camera, one
 * `OrbitControls` — the user orbits the splat and the point cloud together
 * because they *are* together, and the depth buffer is shared, so a point cloud
 * inside the splat volume occludes correctly instead of always winning or
 * always losing. A second canvas would have needed the camera mirrored every
 * frame and could never have interleaved depth at all.
 *
 * ## Coordinates
 *
 * `PointCloudScene` recentres points on the first page's bbox centre (float32
 * precision — see its class comment). The splat is in the same world frame as
 * the seed cloud, so it needs the same offset or it lands wherever the scan's
 * absolute coordinates put it. The offset is only knowable after the file is
 * parsed, which is why it is applied on the group *after* the load resolves
 * rather than passed as a scene `position`.
 *
 * ## SharedArrayBuffer
 *
 * `sharedMemoryForWorkers` is forced off. The library defaults it on and has no
 * fallback for a document that is not cross-origin-isolated; `rux gui` sends no
 * COOP/COEP headers (and should not — they would break the rest of the page for
 * one layer), so leaving the default would throw inside the sort worker.
 *
 * ## Loaded on demand
 *
 * The renderer is `import()`ed inside `load` rather than at module scope, so it
 * becomes its own chunk. It is ~600 kB of the bundle and most projects have no
 * splat at all; a static import would make every dashboard visit pay for a
 * viewer that will never be constructed.
 */

/**
 * Cap on how many splat centres are read to derive the layer's bounds.
 *
 * A bounding box needs the extremes, not every point, and a scene of a few
 * million Gaussians would otherwise spend a visible fraction of a second in a
 * loop that only feeds the framing button. Striding across the whole range
 * keeps the sample spread over the scene rather than over its first corner.
 */
const MAX_BOUNDS_SAMPLES = 200_000;

export interface SplatSceneOptions {
  /** Load progress in 0..1. Called during download and parse. */
  onProgress?: (fraction: number) => void;
}

/** Minimal view of the library's `SplatMesh`; see `gaussian-splats-3d.d.ts`. */
interface SplatCentres {
  getSplatCount(): number;
  getSplatCenter(index: number, out: THREE.Vector3, applySceneTransform?: boolean): void;
}

function hasSplatCentres(mesh: unknown): mesh is SplatCentres {
  const candidate = mesh as Partial<SplatCentres> | null;
  return (
    typeof candidate?.getSplatCount === 'function' &&
    typeof candidate?.getSplatCenter === 'function'
  );
}

/** World-space bounds of a loaded splat mesh, or null when it holds nothing. */
function splatBounds(mesh: unknown): THREE.Box3 | null {
  if (!hasSplatCentres(mesh)) return null;
  const count = mesh.getSplatCount();
  if (count <= 0) return null;

  const stride = Math.max(1, Math.ceil(count / MAX_BOUNDS_SAMPLES));
  const box = new THREE.Box3();
  const centre = new THREE.Vector3();
  for (let i = 0; i < count; i += stride) {
    mesh.getSplatCenter(i, centre, true);
    box.expandByPoint(centre);
  }
  // The last splat explicitly: a stride that does not divide the count evenly
  // would otherwise never look at the end of the scene.
  mesh.getSplatCenter(count - 1, centre, true);
  box.expandByPoint(centre);

  return box.isEmpty() ? null : box;
}

export class SplatScene {
  private viewer: DropInViewer | null = null;
  private visible = true;
  private disposed = false;

  constructor(
    private readonly host: PointCloudScene,
    private readonly options: SplatSceneOptions = {},
  ) {}

  /**
   * Download and add the splat at @p url.
   *
   * Resolves once it is on screen. Rejects with the loader's error, which the
   * caller surfaces — a splat that silently fails to appear is indistinguishable
   * from one that rendered as nothing.
   */
  async load(url: string): Promise<void> {
    if (this.disposed || this.viewer) return;

    const { DropInViewer, SceneFormat, SceneRevealMode } = await import(
      '@mkkellogg/gaussian-splats-3d'
    );
    if (this.disposed) return;

    const viewer = new DropInViewer({
      sharedMemoryForWorkers: false,
      gpuAcceleratedSort: true,
      dynamicScene: false,
      // The scan is the subject, not a reveal animation; and a gradual reveal
      // on a layer the user just switched on reads as a slow load.
      sceneRevealMode: SceneRevealMode.Instant,
      freeIntermediateSplatData: true,
    });
    viewer.visible = this.visible;
    this.viewer = viewer;
    this.host.sceneRoot().add(viewer);

    try {
      await viewer.addSplatScene(url, {
        // The URL is `/api/v1/gsplat/data` — no extension for the loader to
        // sniff, so the format is stated rather than guessed.
        format: SceneFormat.Ply,
        // This app has its own progress UI in the layer panel; the library's
        // spinner would be a second, differently-styled one over the canvas.
        showLoadingUI: false,
        onProgress: (percent: number) => {
          if (!this.disposed) this.options.onProgress?.(percent / 100);
        },
      });
    } catch (error) {
      // Leaving a half-constructed viewer in the scene would keep its sort
      // worker alive and its group in the graph for a splat that never loaded.
      await this.dispose();
      throw error;
    }

    if (this.disposed) return;
    this.applyRecentring(viewer);
  }

  /** Line the splat up with the point cloud's recentred coordinates. */
  private applyRecentring(viewer: DropInViewer): void {
    const box = splatBounds(viewer.splatMesh);
    if (!box) return;
    const origin = this.host.registerExternalBounds(box);
    viewer.position.copy(origin).negate();
  }

  /**
   * Show or hide the layer.
   *
   * Hiding does not unload: the splat is one large download, and a toggle that
   * costs it again is a toggle nobody uses twice. An invisible group is skipped
   * by three.js's traversal, so the depth sort stops running too.
   */
  setVisible(visible: boolean): void {
    this.visible = visible;
    if (this.viewer) this.viewer.visible = visible;
  }

  /** True once a splat is loaded and drawable. */
  hasContent(): boolean {
    return this.viewer !== null && this.viewer.getSceneCount() > 0;
  }

  async dispose(): Promise<void> {
    this.disposed = true;
    const viewer = this.viewer;
    this.viewer = null;
    if (!viewer) return;
    viewer.removeFromParent();
    // Terminates the sort worker and frees the GPU buffers; without it a
    // navigation away from the viewport leaks both.
    await viewer.dispose();
  }
}
