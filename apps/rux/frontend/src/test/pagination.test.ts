// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Chunk-planning arithmetic for `GET /clouds/{name}/points`.
 *
 * This is the code that silently drops or double-loads points when it is
 * wrong — a viewport missing its last 2500 points looks like a rendering bug,
 * not an off-by-one in a loop bound — so the invariants are asserted directly:
 * every page contiguous, the limits summing to the total, no page past the end.
 */

import { describe, expect, it } from 'vitest';
import {
  clampPageSize,
  DEFAULT_PAGE_SIZE,
  fieldIndices,
  hasColors,
  hasPositions,
  loadFraction,
  MAX_PAGE_SIZE,
  planPages,
} from '../viewport/pagination';
import { CLOUD_POINTS_PAGE, LABEL_POINTS_PAGE } from './fixtures';

describe('planPages', () => {
  it('makes the last page short rather than over-reading', () => {
    // The recorded corridor scan: 10500 points.
    const pages = planPages(10500, 4000);
    expect(pages).toEqual([
      { index: 0, offset: 0, limit: 4000 },
      { index: 1, offset: 4000, limit: 4000 },
      { index: 2, offset: 8000, limit: 2500 },
    ]);
  });

  it('covers the cloud exactly — no gaps, no overlap, no over-read', () => {
    for (const [total, pageSize] of [
      [10500, 4000],
      [10500, 100_000],
      [1, 1],
      [7, 3],
      [999_999, 1000],
    ] as const) {
      const pages = planPages(total, pageSize);
      const covered = pages.reduce((sum, page) => sum + page.limit, 0);
      expect(covered).toBe(total);

      let expectedOffset = 0;
      pages.forEach((page, index) => {
        expect(page.index).toBe(index);
        expect(page.offset).toBe(expectedOffset);
        expect(page.limit).toBeGreaterThan(0);
        expect(page.offset + page.limit).toBeLessThanOrEqual(total);
        expectedOffset += page.limit;
      });
    }
  });

  it('produces no trailing empty page for an exact multiple', () => {
    expect(planPages(8000, 4000)).toHaveLength(2);
    expect(planPages(8000, 4000).at(-1)).toEqual({ index: 1, offset: 4000, limit: 4000 });
    expect(planPages(4000, 4000)).toHaveLength(1);
  });

  it('plans a single page when the cloud fits in one', () => {
    expect(planPages(10500)).toEqual([{ index: 0, offset: 0, limit: 10500 }]);
  });

  it('requests nothing at all for an empty cloud', () => {
    // A request for points of a cloud that has none is a wasted round trip at
    // best; the caller must render an empty cloud instead.
    expect(planPages(0, 4000)).toEqual([]);
    expect(planPages(0)).toEqual([]);
  });

  it('requests nothing for a nonsensical total', () => {
    expect(planPages(-1, 4000)).toEqual([]);
    expect(planPages(Number.NaN, 4000)).toEqual([]);
    expect(planPages(Number.POSITIVE_INFINITY, 4000)).toEqual([]);
  });

  it('clamps the page size before planning', () => {
    expect(planPages(10, 0)).toHaveLength(10);
    expect(planPages(10, Number.NaN)).toEqual([{ index: 0, offset: 0, limit: 10 }]);
  });
});

describe('clampPageSize', () => {
  it('clamps to the contract maximum', () => {
    expect(MAX_PAGE_SIZE).toBe(1_000_000);
    expect(clampPageSize(2_000_000)).toBe(MAX_PAGE_SIZE);
    expect(clampPageSize(MAX_PAGE_SIZE)).toBe(MAX_PAGE_SIZE);
    expect(clampPageSize(MAX_PAGE_SIZE + 1)).toBe(MAX_PAGE_SIZE);
  });

  it('floors to at least one point', () => {
    expect(clampPageSize(0)).toBe(1);
    expect(clampPageSize(-500)).toBe(1);
    expect(clampPageSize(0.9)).toBe(1);
  });

  it('truncates a fractional page size', () => {
    expect(clampPageSize(1000.7)).toBe(1000);
  });

  it('falls back to the default for a non-finite size', () => {
    expect(clampPageSize(Number.NaN)).toBe(DEFAULT_PAGE_SIZE);
    expect(clampPageSize(Number.POSITIVE_INFINITY)).toBe(DEFAULT_PAGE_SIZE);
    expect(clampPageSize(Number.NEGATIVE_INFINITY)).toBe(DEFAULT_PAGE_SIZE);
  });

  it('passes an ordinary size through unchanged', () => {
    expect(clampPageSize(4000)).toBe(4000);
    expect(clampPageSize(DEFAULT_PAGE_SIZE)).toBe(DEFAULT_PAGE_SIZE);
  });
});

describe('loadFraction', () => {
  it('is indeterminate while the total is unknown', () => {
    // null, not 0: the caller renders a spinner rather than a bar pinned at
    // zero — the same distinction the job-progress contract makes for total 0.
    expect(loadFraction(0, undefined)).toBeNull();
    expect(loadFraction(500, undefined)).toBeNull();
    expect(loadFraction(500, 0)).toBeNull();
    expect(loadFraction(500, -1)).toBeNull();
    expect(loadFraction(500, Number.NaN)).toBeNull();
  });

  it('reports the ratio for a known total', () => {
    expect(loadFraction(0, 10500)).toBe(0);
    expect(loadFraction(4000, 10500)).toBeCloseTo(4000 / 10500, 10);
    expect(loadFraction(10500, 10500)).toBe(1);
  });

  it('clamps into [0, 1]', () => {
    expect(loadFraction(20000, 10500)).toBe(1);
    expect(loadFraction(-5, 10500)).toBe(0);
  });
});

describe('field resolution', () => {
  // The four field sets the contract defines, one per cloud type.
  const XYZRGB = ['x', 'y', 'z', 'r', 'g', 'b'];
  const XYZ = ['x', 'y', 'z'];
  const NORMAL = ['nx', 'ny', 'nz'];
  const LABEL = ['label'];

  it('maps each field to its column index', () => {
    expect(fieldIndices(XYZRGB)).toEqual({ x: 0, y: 1, z: 2, r: 3, g: 4, b: 5 });
    expect(fieldIndices(XYZ)).toEqual({ x: 0, y: 1, z: 2 });
    expect(fieldIndices(NORMAL)).toEqual({ nx: 0, ny: 1, nz: 2 });
    expect(fieldIndices(LABEL)).toEqual({ label: 0 });
    expect(fieldIndices([])).toEqual({});
  });

  it('recognises positional geometry only where it exists', () => {
    expect(hasPositions(XYZRGB)).toBe(true);
    expect(hasPositions(XYZ)).toBe(true);
    // A Normal cloud's nx/ny/nz are directions, not positions, and a Label
    // cloud has one column that is emphatically not `x`.
    expect(hasPositions(NORMAL)).toBe(false);
    expect(hasPositions(LABEL)).toBe(false);
    expect(hasPositions([])).toBe(false);
  });

  it('recognises colour only on PointXYZRGB', () => {
    expect(hasColors(XYZRGB)).toBe(true);
    expect(hasColors(XYZ)).toBe(false);
    expect(hasColors(NORMAL)).toBe(false);
    expect(hasColors(LABEL)).toBe(false);
  });

  it('agrees with the recorded pages', () => {
    expect(CLOUD_POINTS_PAGE.fields).toEqual(XYZRGB);
    expect(hasPositions(CLOUD_POINTS_PAGE.fields)).toBe(true);
    expect(hasColors(CLOUD_POINTS_PAGE.fields)).toBe(true);

    expect(LABEL_POINTS_PAGE.fields).toEqual(LABEL);
    expect(hasPositions(LABEL_POINTS_PAGE.fields)).toBe(false);
    expect(hasColors(LABEL_POINTS_PAGE.fields)).toBe(false);
  });
});
