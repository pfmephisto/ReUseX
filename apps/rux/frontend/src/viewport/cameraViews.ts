// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';

/**
 * Camera and lighting maths for the viewport controls (#443).
 *
 * Kept apart from `PointCloudScene.ts` for the same reason `frustumCull.ts` is
 * kept apart from the scene: preset orientations, the ortho/perspective frustum
 * match, and the key-light direction are pure functions of their inputs, so
 * they are unit-testable without a WebGL context. `PointCloudScene` is the only
 * consumer; it wires these into the live camera and lights.
 */

/** Which projection the orbit camera draws with. */
export type CameraProjection = 'perspective' | 'orthographic';

/** A named axis-aligned viewpoint, matching `rux render --view`. */
export type ViewPreset = 'top' | 'bottom' | 'front' | 'back' | 'left' | 'right';

/** The preset buttons, in the order the panel lays them out. */
export const VIEW_PRESETS: { readonly preset: ViewPreset; readonly label: string }[] = [
  { preset: 'top', label: 'Top' },
  { preset: 'bottom', label: 'Bottom' },
  { preset: 'front', label: 'Front' },
  { preset: 'back', label: 'Back' },
  { preset: 'left', label: 'Left' },
  { preset: 'right', label: 'Right' },
];

/** How a preset places the camera relative to the framed content. */
export interface PresetOrientation {
  /** Unit vector from the target to the camera, in scene coords (Z up). */
  direction: THREE.Vector3;
  /** Camera up vector for the shot. */
  up: THREE.Vector3;
}

/**
 * The camera direction and up vector for a named preset.
 *
 * The convention is the CLI's (`place_preset_camera` in
 * `libs/reusex/src/visualize/render_view.cpp`): building scans are gravity
 * aligned with world **Z up**, `front` looks along +Y from the −Y side with Z
 * up, and `top` looks straight down −Z with +Y up the page (Z would be
 * degenerate when it is also the view axis). The side and back views extend the
 * same convention so a shot named here matches the one `rux render` produces.
 *
 * `direction` points *from the target toward the camera*, so the scene places
 * the camera at `target + direction * distance`.
 */
export function presetOrientation(preset: ViewPreset): PresetOrientation {
  switch (preset) {
    case 'top':
      return { direction: new THREE.Vector3(0, 0, 1), up: new THREE.Vector3(0, 1, 0) };
    case 'bottom':
      return { direction: new THREE.Vector3(0, 0, -1), up: new THREE.Vector3(0, 1, 0) };
    case 'front':
      return { direction: new THREE.Vector3(0, -1, 0), up: new THREE.Vector3(0, 0, 1) };
    case 'back':
      return { direction: new THREE.Vector3(0, 1, 0), up: new THREE.Vector3(0, 0, 1) };
    case 'right':
      return { direction: new THREE.Vector3(1, 0, 0), up: new THREE.Vector3(0, 0, 1) };
    case 'left':
      return { direction: new THREE.Vector3(-1, 0, 0), up: new THREE.Vector3(0, 0, 1) };
  }
}

/**
 * Half-height of the orthographic frustum that shows the same vertical extent
 * as a perspective camera of field of view @p fovDeg at focal distance
 * @p distance.
 *
 * This is what makes the projection toggle seamless at the plane the user was
 * looking at: `tan(fov/2) * distance` is the half-height the perspective camera
 * spans there, and giving the orthographic camera the same half-height leaves
 * everything at that depth the same size on screen.
 */
export function orthoHalfHeight(fovDeg: number, distance: number): number {
  return Math.tan(THREE.MathUtils.degToRad(fovDeg) / 2) * distance;
}

/** Adjustable state of the viewport's key + fill lights (#443). */
export interface LightingState {
  /** Directional key-light intensity. */
  keyIntensity: number;
  /** Hemisphere fill intensity — the "ambient" of an outdoor sky/ground pair. */
  ambientIntensity: number;
  /** Key-light bearing around the world Z axis, in degrees. */
  azimuth: number;
  /** Key-light height above the horizon, in degrees (−90..90). */
  elevation: number;
}

/**
 * Default lighting — the rig `MeshScene` used before the controls existed.
 *
 * The azimuth/elevation reproduce the old fixed key position `(4, −6, 8)`; the
 * intensities are that rig's hemisphere (1.4) and directional (2.2) values, so
 * a project opens looking exactly as it did before this control was added.
 */
export const DEFAULT_LIGHTING: LightingState = {
  keyIntensity: 2.2,
  ambientIntensity: 1.4,
  azimuth: -56,
  elevation: 48,
};

/**
 * Unit direction a key light points *from*, for a bearing and elevation in
 * degrees, in the scene's Z-up frame.
 *
 * A directional light is parallel, so only this direction matters: setting the
 * light's position to the returned vector makes its rays arrive from that
 * bearing. Elevation is measured up from the horizon, azimuth around +Z from
 * the +X axis toward +Y.
 */
export function lightDirection(azimuthDeg: number, elevationDeg: number): THREE.Vector3 {
  const az = THREE.MathUtils.degToRad(azimuthDeg);
  const el = THREE.MathUtils.degToRad(elevationDeg);
  const cosEl = Math.cos(el);
  return new THREE.Vector3(cosEl * Math.cos(az), cosEl * Math.sin(az), Math.sin(el));
}
