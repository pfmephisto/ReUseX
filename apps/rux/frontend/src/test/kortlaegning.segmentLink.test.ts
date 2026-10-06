// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { SurveyPart, SurveyType, VisibleFrame } from '../api/types';
import { tableAction } from '../kortlaegning/keys';
import { segmentHrefFromFrames, segmentTargetPart } from '../kortlaegning/segmentLink';

const part = (code: string, instance_id: number | null): SurveyPart =>
  ({ code, type_id: 1, cloud: instance_id === null ? null : 'instances', instance_id }) as SurveyPart;

describe('Segmentér from Kortlægning', () => {
  it('targets the selected instance-backed part, or a type row\'s first one', () => {
    const manual = part('RX-2', null);
    const scanned = part('RX-3', 7);
    const type = { id: 1, parts: [manual, scanned] } as SurveyType;
    expect(segmentTargetPart(type, scanned)).toBe(scanned);
    expect(segmentTargetPart(type, manual)).toBeNull();
    expect(segmentTargetPart(type, null)).toBe(scanned);
    expect(segmentTargetPart(null, null)).toBeNull();
  });

  it('opens the best frame with the centroid pixel seeded', () => {
    const frames = [{ frame_id: 42, u: 120.4, v: 88.6 } as VisibleFrame, { frame_id: 9, u: 1, v: 1 } as VisibleFrame];
    expect(segmentHrefFromFrames(frames)).toBe('/segmentering?frame=42&u=120&v=89');
    expect(segmentHrefFromFrames([])).toBeNull();
  });

  it('binds S in the table, never while typing or with a modifier', () => {
    expect(tableAction({ key: 's', inField: false })).toEqual({ type: 'segment' });
    expect(tableAction({ key: 'S', inField: false })).toEqual({ type: 'segment' });
    expect(tableAction({ key: 's', inField: true })).toBeNull();
    expect(tableAction({ key: 's', inField: false, ctrlKey: true })).toBeNull();
  });
});
