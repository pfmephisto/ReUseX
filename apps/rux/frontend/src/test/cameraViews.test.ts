// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Tests for the viewport camera/lighting maths (#443).
 *
 * The live camera and lights need a WebGL context (`PointCloudScene.ts`), so
 * what is pinned here is the pure geometry the controls are built on: the
 * preset orientations matching the CLI convention, the ortho/perspective
 * frustum match that makes the projection toggle seamless, and the key-light
 * direction.
 */

import { describe, expect, it } from 'vitest';
import * as THREE from 'three';

import {
  DEFAULT_LIGHTING,
  VIEW_PRESETS,
  lightDirection,
  orthoHalfHeight,
  presetOrientation,
  type ViewPreset,
} from '../viewport/cameraViews';

/** Every preset the panel offers has an orientation, and no duplicates. */
describe('VIEW_PRESETS', () => {
  it('Presets_EveryButton_ResolvesToAnOrientation', () => {
    for (const { preset } of VIEW_PRESETS) {
      const { direction, up } = presetOrientation(preset);
      expect(direction.length()).toBeCloseTo(1);
      expect(up.length()).toBeCloseTo(1);
    }
  });

  it('Presets_ButtonList_HasNoDuplicatePresets', () => {
    const seen = new Set<ViewPreset>(VIEW_PRESETS.map((entry) => entry.preset));
    expect(seen.size).toBe(VIEW_PRESETS.length);
  });
});

describe('presetOrientation', () => {
  it('Top_LooksStraightDown_WithYUpMatchingTheCli', () => {
    // `rux render --view top`: camera above centre (+Z), +Y up the page.
    const { direction, up } = presetOrientation('top');
    expect(direction.toArray()).toEqual([0, 0, 1]);
    expect(up.toArray()).toEqual([0, 1, 0]);
  });

  it('Front_LooksAlongPlusY_WithWorldZUp', () => {
    // `rux render --view front`: camera on the −Y side, world Z up.
    const { direction, up } = presetOrientation('front');
    expect(direction.toArray()).toEqual([0, -1, 0]);
    expect(up.toArray()).toEqual([0, 0, 1]);
  });

  it('LeftAndRight_AreOpposedAlongX_WithZUp', () => {
    const left = presetOrientation('left');
    const right = presetOrientation('right');
    expect(left.direction.clone().add(right.direction).length()).toBeCloseTo(0);
    expect(left.up.toArray()).toEqual([0, 0, 1]);
    expect(right.up.toArray()).toEqual([0, 0, 1]);
  });

  it('TopAndBottom_AreOpposedAlongZ', () => {
    const top = presetOrientation('top');
    const bottom = presetOrientation('bottom');
    expect(top.direction.clone().add(bottom.direction).length()).toBeCloseTo(0);
  });

  it('Orientation_ViewAxisAndUp_AreAlwaysPerpendicular', () => {
    // A degenerate up parallel to the view axis makes the camera basis
    // singular; every preset must avoid it.
    for (const { preset } of VIEW_PRESETS) {
      const { direction, up } = presetOrientation(preset);
      expect(direction.dot(up)).toBeCloseTo(0);
    }
  });
});

describe('orthoHalfHeight', () => {
  it('HalfHeight_MatchesPerspectiveVerticalExtentAtDistance', () => {
    // A 60° camera spans tan(30°)·distance above the axis at the focal plane.
    const distance = 10;
    const expected = Math.tan(THREE.MathUtils.degToRad(30)) * distance;
    expect(orthoHalfHeight(60, distance)).toBeCloseTo(expected);
  });

  it('HalfHeight_ScalesLinearlyWithDistance', () => {
    expect(orthoHalfHeight(45, 20)).toBeCloseTo(orthoHalfHeight(45, 10) * 2);
  });
});

describe('lightDirection', () => {
  it('Direction_IsAlwaysUnitLength', () => {
    for (const az of [-180, -56, 0, 90, 180]) {
      for (const el of [0, 30, 48, 90]) {
        expect(lightDirection(az, el).length()).toBeCloseTo(1);
      }
    }
  });

  it('Elevation90_PointsStraightUpTheZAxis', () => {
    const dir = lightDirection(0, 90);
    expect(dir.x).toBeCloseTo(0);
    expect(dir.y).toBeCloseTo(0);
    expect(dir.z).toBeCloseTo(1);
  });

  it('AzimuthZeroElevationZero_PointsAlongPlusX', () => {
    const dir = lightDirection(0, 0);
    expect(dir.x).toBeCloseTo(1);
    expect(dir.y).toBeCloseTo(0);
    expect(dir.z).toBeCloseTo(0);
  });

  it('DefaultLighting_ReproducesTheOldFixedKeyDirection', () => {
    // The rig `MeshScene` used before the controls existed placed the key at
    // (4, −6, 8). The default azimuth/elevation must point the same way so a
    // project opens looking as it did.
    const dir = lightDirection(DEFAULT_LIGHTING.azimuth, DEFAULT_LIGHTING.elevation);
    const old = new THREE.Vector3(4, -6, 8).normalize();
    expect(dir.angleTo(old)).toBeLessThan(THREE.MathUtils.degToRad(2));
  });
});
