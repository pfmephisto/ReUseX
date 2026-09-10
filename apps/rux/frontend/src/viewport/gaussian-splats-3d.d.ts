// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Type declarations for `@mkkellogg/gaussian-splats-3d` (MIT), which ships
 * JavaScript with no `types` entry.
 *
 * Deliberately **not** a full mirror of the package's API: only the surface
 * `SplatScene.ts` uses is declared, so this file stays small enough to keep
 * honest against the upstream build. Anything else is a compile error rather
 * than an `any`, which is the point.
 */
declare module '@mkkellogg/gaussian-splats-3d' {
  import * as THREE from 'three';

  export const SceneFormat: {
    readonly Splat: number;
    readonly KSplat: number;
    readonly Ply: number;
    readonly Spz: number;
  };

  export const SceneRevealMode: {
    readonly Default: number;
    readonly Gradual: number;
    readonly Instant: number;
  };

  export interface AddSplatSceneOptions {
    /** Overrides the format the loader would otherwise guess from the path. */
    format?: number;
    /** Scene offset, applied on top of the object's own transform. */
    position?: [number, number, number];
    rotation?: [number, number, number, number];
    scale?: [number, number, number];
    /** Drop Gaussians whose alpha is below this (0–255). */
    splatAlphaRemovalThreshold?: number;
    /** The package's own DOM spinner. Off: this app has its own progress UI. */
    showLoadingUI?: boolean;
    progressiveLoad?: boolean;
    onProgress?: (percent: number, percentLabel: string, stage: number) => void;
  }

  export interface DropInViewerOptions {
    /**
     * Must be `false` here: a `SharedArrayBuffer` needs a cross-origin-isolated
     * document, and `rux gui` sends no COOP/COEP headers.
     */
    sharedMemoryForWorkers?: boolean;
    gpuAcceleratedSort?: boolean;
    integerBasedSort?: boolean;
    halfPrecisionCovariancesOnGPU?: boolean;
    dynamicScene?: boolean;
    antialiased?: boolean;
    sphericalHarmonicsDegree?: number;
    sceneRevealMode?: number;
    logLevel?: number;
    freeIntermediateSplatData?: boolean;
  }

  /**
   * A splat viewer that is itself a `THREE.Group`.
   *
   * Renders inside a host application's own scene, camera and renderer: it
   * hooks `onBeforeRender` on an invisible child mesh to run its depth sort
   * against whatever camera drew it. That is what lets the splat share the
   * viewport's `OrbitControls` instead of running a second canvas.
   */
  export class DropInViewer extends THREE.Group {
    constructor(options?: DropInViewerOptions);
    addSplatScene(path: string, options?: AddSplatSceneOptions): Promise<void>;
    removeSplatScene(index: number, showLoadingUI?: boolean): Promise<void>;
    getSceneCount(): number;
    dispose(): Promise<void>;
    splatMesh: THREE.Object3D | null;
  }
}
