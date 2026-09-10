// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Tests for panorama mode's non-WebGL half (#265, Phase 5).
 *
 * The renderer needs a GPU and a canvas, so what is pinned here is everything
 * that decides whether the picture is *correct*: where a panorama is drawn,
 * which way it faces, and which pixel a direction lands on. Those are the
 * failures a screenshot cannot catch — a mirrored equirect looks like a room.
 *
 * The equirect convention under test is the one
 * `libs/reusex/include/geometry/EquirectProjection.hpp` defines and
 * `docs/gui/openapi.yaml` restates; these cases are its cardinal directions.
 */

import { describe, expect, it } from 'vitest';

import { RuxApiClient } from '../api/client';
import type { PanoramaInfo } from '../api/types';
import {
  LEVELLED_BASIS,
  bearingAt,
  bearingToUv,
  describePlacement,
  equirectSphere,
  headingCaveat,
  panoramaNote,
  resolvePlacement,
  stepPanorama,
} from '../viewport/panorama';

/** Row-major 4x4 with the given translation and an identity rotation. */
function poseAt(x: number, y: number, z: number): number[] {
  return [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
}

const UNALIGNED: PanoramaInfo = {
  id: 3,
  filename: 'R0010003.JPG',
  node_id: 2893,
  has_pose: false,
  pose: poseAt(0, 0, 0),
  pose_source: 'timestamp',
  align_inliers: -1,
  align_rms: -1,
  has_frame_pose: true,
  frame_pose: poseAt(4, -1, 1.5),
};

describe('resolvePlacement', () => {
  it('ResolvePlacement_AlignedPanorama_UsesTheResectedPose', () => {
    const pano: PanoramaInfo = {
      ...UNALIGNED,
      has_pose: true,
      pose: poseAt(9, 9, 9),
      pose_source: 'aligned',
    };

    const placement = resolvePlacement(pano);

    expect(placement?.heading).toBe('resected');
    expect(placement?.position).toEqual([9, 9, 9]);
    // The resected pose wins outright — a frame pose is present here and must
    // not dilute a measurement with a borrowed guess.
    expect(placement?.fromFrame).toBeUndefined();
  });

  it('ResolvePlacement_UnalignedButMatchedToAPosedFrame_BorrowsThePositionOnly', () => {
    // The normal state of a project that has only been imported.
    const placement = resolvePlacement(UNALIGNED);

    expect(placement?.heading).toBe('levelled');
    expect(placement?.position).toEqual([4, -1, 1.5]);
    expect(placement?.fromFrame).toBe(2893);
    // The frame's rotation is deliberately NOT adopted: a 360 camera's yaw has
    // no fixed relation to the phone's, and importing the phone's tilt would
    // put a false slope on a horizon that was level.
    expect(placement?.basis).toEqual(LEVELLED_BASIS);
  });

  it('ResolvePlacement_NoAlignedPoseAndNoFramePose_IsNotPlaceable', () => {
    // Null rather than the origin: a panorama drawn at (0,0,0) is a
    // photograph asserted to have been taken somewhere the building is not.
    expect(
      resolvePlacement({ ...UNALIGNED, has_frame_pose: false, frame_pose: undefined }),
    ).toBeNull();
  });

  it('ResolvePlacement_MalformedPose_IsRejectedRatherThanUsedPartially', () => {
    const truncated = { ...UNALIGNED, has_pose: true, pose: [1, 0, 0, 5] };
    // Falls through to the frame pose, which is intact.
    expect(resolvePlacement(truncated)?.heading).toBe('levelled');

    const notFinite = { ...UNALIGNED, frame_pose: poseAt(Number.NaN, 0, 0) };
    expect(resolvePlacement(notFinite)).toBeNull();
  });

  it('ResolvePlacement_RotatedPose_TransposesRowMajorIntoAColumnMajorBasis', () => {
    // A 90-degree rotation about world Z: pano +x -> world +y, +y -> world -x.
    // Asymmetric on purpose — a symmetric matrix cannot catch a transpose bug,
    // and a transpose here mirrors the panorama's heading.
    const pose = [0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    const placement = resolvePlacement({
      ...UNALIGNED,
      has_pose: true,
      pose,
      pose_source: 'aligned',
    });

    // Column 0 (the panorama's +x axis in world) is world +y.
    expect(placement?.basis.slice(0, 3)).toEqual([0, 1, 0]);
    // Column 1 (its +y, i.e. down) is world -x.
    expect(placement?.basis.slice(3, 6)).toEqual([-1, 0, 0]);
  });
});

describe('LEVELLED_BASIS', () => {
  it('LevelledBasis_PanoramaDownAxis_PointsAtWorldNegativeZ', () => {
    // The world is gravity-aligned Z-up and the panorama's +y is down, so this
    // is what "level horizon" means numerically.
    expect(LEVELLED_BASIS.slice(3, 6)).toEqual([0, 0, -1]);
  });

  it('LevelledBasis_Axes_AreOrthonormalAndRightHanded', () => {
    const x = LEVELLED_BASIS.slice(0, 3);
    const y = LEVELLED_BASIS.slice(3, 6);
    const z = LEVELLED_BASIS.slice(6, 9);
    const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const cross = (a: number[], b: number[]) => [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];

    for (const axis of [x, y, z]) expect(dot(axis, axis)).toBeCloseTo(1);
    expect(dot(x, y)).toBeCloseTo(0);
    expect(dot(y, z)).toBeCloseTo(0);
    expect(dot(x, z)).toBeCloseTo(0);
    // x = y x z is the optical convention (right = down x forward). A
    // left-handed basis would mirror the whole panorama.
    expect(cross(y, z)).toEqual(x);
  });
});

describe('bearingToUv', () => {
  it('BearingToUv_Forward_IsTheCentreColumnOnTheHorizon', () => {
    const [u, v] = bearingToUv(0, 0, 1);
    expect(u).toBeCloseTo(0.5);
    expect(v).toBeCloseTo(0.5);
  });

  it('BearingToUv_Right_IsThreeQuartersAcross', () => {
    // theta = +pi/2 for +x, and theta spans [-pi, pi] left to right.
    expect(bearingToUv(1, 0, 0)[0]).toBeCloseTo(0.75);
    expect(bearingToUv(-1, 0, 0)[0]).toBeCloseTo(0.25);
  });

  it('BearingToUv_UpAndDown_AreTheFirstAndLastRow', () => {
    // +y is DOWN in the panorama frame. Getting this sign wrong flips the
    // image vertically, which reads as a camera mounted upside down.
    expect(bearingToUv(0, -1, 0)[1]).toBeCloseTo(0);
    expect(bearingToUv(0, 1, 0)[1]).toBeCloseTo(1);
  });

  it('BearingToUv_UnnormalisedInput_IsNormalisedFirst', () => {
    expect(bearingToUv(0, 0, 7)).toEqual(bearingToUv(0, 0, 1));
  });

  it('BearingToUv_RoundTripThroughBearingAt_IsTheIdentity', () => {
    for (const [theta, phi] of [
      [0, 0],
      [1.1, 0.4],
      [-2.0, -0.9],
      [3.0, 1.2],
    ]) {
      const [x, y, z] = bearingAt(theta, phi);
      const [u, v] = bearingToUv(x, y, z);
      expect(u * 2 * Math.PI - Math.PI).toBeCloseTo(theta);
      expect(Math.PI / 2 - v * Math.PI).toBeCloseTo(phi);
    }
  });
});

describe('equirectSphere', () => {
  const columns = 8;
  const rows = 4;
  const sphere = equirectSphere(columns, rows);

  it('EquirectSphere_EveryVertex_LiesOnTheUnitSphere', () => {
    for (let i = 0; i < sphere.positions.length; i += 3) {
      const length = Math.hypot(
        sphere.positions[i],
        sphere.positions[i + 1],
        sphere.positions[i + 2],
      );
      expect(length).toBeCloseTo(1);
    }
  });

  it('EquirectSphere_SeamColumns_GetUZeroAndUOneAtTheSamePosition', () => {
    // The whole reason the UVs are generated from the grid rather than from
    // the vertex positions: both copies of the seam sit at longitude -pi, so
    // a position-derived `u` would give them the same value and the quads
    // between them would interpolate the entire texture backwards.
    const stride = columns + 1;
    const first = stride * 2; // an equatorial row
    const last = first + columns;
    expect(sphere.uvs[first * 2]).toBeCloseTo(0);
    expect(sphere.uvs[last * 2]).toBeCloseTo(1);
    for (let axis = 0; axis < 3; axis += 1) {
      expect(sphere.positions[first * 3 + axis]).toBeCloseTo(
        sphere.positions[last * 3 + axis],
      );
    }
  });

  it('EquirectSphere_NorthPole_IsTheTopOfTheImage', () => {
    // three.js uploads images with flipY on, so v = 1 is the first row — and
    // the first row of an equirect is the north pole.
    expect(sphere.positions[1]).toBeCloseTo(-1); // +y is down, so up is -1
    expect(sphere.uvs[1]).toBeCloseTo(1);
  });

  it('EquirectSphere_EveryTriangle_IsWoundOutward', () => {
    // Regression test for a bug that renders as *nothing at all*: with the
    // opposite winding, `THREE.BackSide` culls exactly the faces a viewer
    // standing at the centre can see, so the panorama is visible from outside
    // the sphere — where nobody stands — and the immersive view is empty.
    const at = (index: number) => [
      sphere.positions[index * 3],
      sphere.positions[index * 3 + 1],
      sphere.positions[index * 3 + 2],
    ];

    for (let t = 0; t < sphere.indices.length; t += 3) {
      const [a, b, c] = [at(sphere.indices[t]), at(sphere.indices[t + 1]), at(sphere.indices[t + 2])];
      const e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
      const normal = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
      ];
      // On a sphere centred at the origin, "outward" is simply "along the
      // vertex", so the face normal must agree with the vertex it starts at.
      const outward = normal[0] * a[0] + normal[1] * a[1] + normal[2] * a[2];
      expect(outward).toBeGreaterThan(0);
    }
  });

  it('EquirectSphere_Indices_AddressExistingVerticesAndSkipTheDegenerateCaps', () => {
    const vertices = (columns + 1) * (rows + 1);
    for (const index of sphere.indices) expect(index).toBeLessThan(vertices);
    // Two triangles per quad, minus the one collapsed triangle at each pole
    // row: 2*C*R - 2*C.
    expect(sphere.indices.length / 3).toBe(2 * columns * rows - 2 * columns);
  });
});

describe('stepPanorama', () => {
  it('StepPanorama_PastTheEnd_WrapsToTheStart', () => {
    // A capture is a loop through a building; the panorama after the last one
    // is the first one.
    expect(stepPanorama([1, 2, 3], 3, 1)).toBe(1);
    expect(stepPanorama([1, 2, 3], 1, -1)).toBe(3);
  });

  it('StepPanorama_FromInsideTheList_MovesOneEitherWay', () => {
    expect(stepPanorama([1, 2, 3], 2, 1)).toBe(3);
    expect(stepPanorama([1, 2, 3], 2, -1)).toBe(1);
  });

  it('StepPanorama_NothingActive_StartsAtTheNearestEnd', () => {
    expect(stepPanorama([1, 2, 3], null, 1)).toBe(1);
    expect(stepPanorama([1, 2, 3], null, -1)).toBe(3);
  });

  it('StepPanorama_EmptyList_IsNull', () => {
    // Returning an id here would index an empty array at the call site.
    expect(stepPanorama([], null, 1)).toBeNull();
  });
});

describe('describePlacement', () => {
  it('DescribePlacement_Aligned_ReportsInlierCountAndAngularRms', () => {
    const pano: PanoramaInfo = {
      ...UNALIGNED,
      has_pose: true,
      pose_source: 'aligned',
      align_inliers: 182,
      align_rms: 0.418,
    };
    const text = describePlacement(pano, resolvePlacement(pano));
    expect(text).toContain('Aligned');
    expect(text).toContain('182 inliers');
    expect(text).toContain('0.42° RMS');
  });

  it('DescribePlacement_Borrowed_NamesTheFrameAndAdmitsTheHeadingIsUnknown', () => {
    const text = describePlacement(UNALIGNED, resolvePlacement(UNALIGNED));
    expect(text).toContain('frame 2893');
    expect(text).toContain('heading unknown');
  });

  it('DescribePlacement_Unplaceable_SaysWhichOfTheTwoReasonsItIs', () => {
    const noFrame: PanoramaInfo = {
      ...UNALIGNED,
      node_id: -1,
      has_frame_pose: false,
      frame_pose: undefined,
    };
    expect(describePlacement(noFrame, null)).toContain('no sensor frame');

    const poselessFrame: PanoramaInfo = {
      ...UNALIGNED,
      has_frame_pose: false,
      frame_pose: undefined,
    };
    expect(describePlacement(poselessFrame, null)).toContain('frame 2893 has no pose');
  });
});

describe('headingCaveat', () => {
  it('HeadingCaveat_BorrowedPlacement_NamesTheCommandThatFixesIt', () => {
    const caveat = headingCaveat(resolvePlacement(UNALIGNED));
    expect(caveat).toContain('rux align 360');
  });

  it('HeadingCaveat_ResectedPlacement_IsSilent', () => {
    // A permanent banner over a picture that is correct trains the user to
    // ignore the banner that matters.
    const pano: PanoramaInfo = { ...UNALIGNED, has_pose: true, pose_source: 'aligned' };
    expect(headingCaveat(resolvePlacement(pano))).toBeNull();
  });
});

describe('panoramaNote', () => {
  it('PanoramaNote_ProjectWithPanoramas_HasNothingToSay', () => {
    expect(panoramaNote([UNALIGNED], null)).toBeNull();
  });

  it('PanoramaNote_ProjectWithout_NamesTheImportCommand', () => {
    expect(panoramaNote([], null)).toContain('rux import 360');
  });

  it('PanoramaNote_RequestFailed_ReportsTheTransportErrorNotAnEmptyProject', () => {
    const note = panoramaNote([], new Error('project database is busy'));
    expect(note).toContain('project database is busy');
    expect(note).not.toContain('rux import 360');
  });

  it('PanoramaNote_ListNotYetLoaded_IsNull', () => {
    expect(panoramaNote(null, null)).toBeNull();
  });
});

describe('RuxApiClient panorama image URLs', () => {
  it('PanoramaImageUrl_NoOptions_IsTheBarePath', () => {
    expect(new RuxApiClient().panoramaImageUrl(3)).toBe('/api/v1/panoramas/3/image');
  });

  it('PanoramaImageUrl_MaxSize_AsksForAThumbnail', () => {
    // A picker that omits this downloads several megabytes per row.
    expect(new RuxApiClient().panoramaImageUrl(3, { maxSize: 128 })).toBe(
      '/api/v1/panoramas/3/image?max_size=128',
    );
  });
});
