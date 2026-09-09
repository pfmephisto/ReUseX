// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { FrameInfo, FrameList } from '../api/types';
import {
  describeFrameCounts,
  formatFrameTimestamp,
  frameImageSlots,
  parseFrameFilter,
  parseSelectedFrame,
  poseRows,
  poseTranslation,
  segmentedParam,
  stepSelection,
} from '../data/framesModel';
import { FRAMES } from './fixtures';

const ids = FRAMES.ids;

describe('parseFrameFilter', () => {
  it('accepts the three documented filters', () => {
    expect(parseFrameFilter('all')).toBe('all');
    expect(parseFrameFilter('segmented')).toBe('segmented');
    expect(parseFrameFilter('unsegmented')).toBe('unsegmented');
  });

  it('falls back to all for junk in the URL', () => {
    for (const raw of ['', null, undefined, 'nonsense']) {
      expect(parseFrameFilter(raw)).toBe('all');
    }
  });
});

describe('segmentedParam', () => {
  it('omits the parameter entirely for the unfiltered view', () => {
    // undefined, so buildQuery drops it — the server then returns every frame.
    expect(segmentedParam('all')).toBeUndefined();
  });

  it('maps the two filters onto the boolean the contract defines', () => {
    expect(segmentedParam('segmented')).toBe(true);
    expect(segmentedParam('unsegmented')).toBe(false);
  });
});

describe('parseSelectedFrame', () => {
  it('resolves an id that is on screen', () => {
    expect(parseSelectedFrame('1997', ids)).toBe(1997);
  });

  it('drops a selection the current filter excludes', () => {
    // Otherwise the detail pane shows a frame the grid is not listing.
    expect(parseSelectedFrame('1997', [1995, 1996])).toBeNull();
  });

  it('rejects a non-integer without selecting anything', () => {
    for (const raw of ['', null, undefined, 'abc', '19.5']) {
      expect(parseSelectedFrame(raw, ids)).toBeNull();
    }
  });
});

describe('stepSelection', () => {
  it('starts at the first frame when nothing is selected', () => {
    expect(stepSelection(ids, null, 1)).toBe(ids[0]);
  });

  it('moves by the given offset', () => {
    expect(stepSelection(ids, 1997, 1)).toBe(1998);
    expect(stepSelection(ids, 1997, -1)).toBe(1996);
    expect(stepSelection(ids, 1995, 4)).toBe(1999);
  });

  it('clamps at both ends rather than wrapping', () => {
    // Wrapping several hundred rows reads as a reload, not as a cursor move.
    expect(stepSelection(ids, ids[0], -1)).toBe(ids[0]);
    expect(stepSelection(ids, ids[ids.length - 1], 1)).toBe(ids[ids.length - 1]);
  });

  it('recovers to the first frame when the selection has vanished', () => {
    expect(stepSelection(ids, 999999, 1)).toBe(ids[0]);
  });

  it('selects nothing in an empty list', () => {
    expect(stepSelection([], 1997, 1)).toBeNull();
  });
});

describe('describeFrameCounts', () => {
  it('reports the whole scan when unfiltered', () => {
    expect(describeFrameCounts(FRAMES, 'all')).toBe('10 frames · 0 segmented');
  });

  it('reports the filtered size against the whole-scan total', () => {
    // total_count and segmented_count describe the whole scan even when `ids`
    // is filtered, so the shown count must come from ids.length. Getting this
    // backwards prints "10 of 10" on a filter showing two frames.
    const filtered: FrameList = { ...FRAMES, ids: [1995, 1996] };
    expect(describeFrameCounts(filtered, 'segmented')).toBe('2 segmented of 10 frames');
    expect(describeFrameCounts(filtered, 'unsegmented')).toBe('2 unsegmented of 10 frames');
  });
});

describe('formatFrameTimestamp', () => {
  it('formats a real capture time', () => {
    expect(formatFrameTimestamp(1773126992)).toMatch(/^2026-03-10 /);
  });

  it('reports the contract sentinel as no timestamp at all', () => {
    // -1 through Date is 31 December 1969: a plausible-looking date that is
    // entirely fictional, and it would appear on every frame of a scan that
    // recorded no times.
    expect(formatFrameTimestamp(-1)).toBeNull();
    expect(formatFrameTimestamp(0)).toBeNull();
    expect(formatFrameTimestamp(undefined)).toBeNull();
    expect(formatFrameTimestamp(Number.NaN)).toBeNull();
  });
});

describe('frameImageSlots', () => {
  const frame: FrameInfo = {
    id: 1997,
    pose: [],
    has_depth: true,
    has_confidence: true,
    has_segmentation: false,
  };

  it('offers colour unconditionally and the rest by availability', () => {
    const slots = frameImageSlots(frame);
    const byKind = Object.fromEntries(slots.map((s) => [s.kind, s.available]));
    expect(byKind.color).toBe(true);
    expect(byKind.depth).toBe(true);
    expect(byKind.confidence).toBe(true);
    expect(byKind.segmentation).toBe(false);
  });

  it('normalizes every kind except colour, which is already displayable', () => {
    const slots = frameImageSlots(frame);
    for (const slot of slots) {
      expect(slot.normalized).toBe(slot.kind !== 'color');
    }
  });

  it('lists an absent kind rather than hiding it', () => {
    // Four panels for one frame and three for the next teaches nothing.
    expect(frameImageSlots(frame)).toHaveLength(4);
    expect(frameImageSlots({ ...frame, has_depth: false })).toHaveLength(4);
  });
});

describe('pose helpers', () => {
  const pose = [1, 0, 0, 2, 0, 1, 0, 3, 0, 0, 1, 4, 0, 0, 0, 1];

  it('splits a row-major 4x4 into rows', () => {
    expect(poseRows(pose)).toEqual([
      [1, 0, 0, 2],
      [0, 1, 0, 3],
      [0, 0, 1, 4],
      [0, 0, 0, 1],
    ]);
  });

  it('reads the translation out of the last column', () => {
    expect(poseTranslation(pose)).toEqual([2, 3, 4]);
  });

  it('refuses a pose that is not sixteen numbers', () => {
    for (const bad of [undefined, [], [1, 2, 3]]) {
      expect(poseRows(bad)).toBeNull();
      expect(poseTranslation(bad)).toBeNull();
    }
  });
});
