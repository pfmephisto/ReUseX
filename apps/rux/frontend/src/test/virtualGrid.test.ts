// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import { columnsForWidth, gridWindow, scrollToIndex } from '../data/virtualGrid';

/** A grid of 100 cells, 4 across, 120px rows, showing 600px at a time. */
const base = {
  scrollTop: 0,
  viewportHeight: 600,
  rowHeight: 120,
  columns: 4,
  count: 100,
  overscan: 2,
};

describe('gridWindow', () => {
  it('measures the full grid regardless of where it is scrolled', () => {
    // 100 cells / 4 columns = 25 rows of 120px.
    for (const scrollTop of [0, 500, 3000, 99999]) {
      const win = gridWindow({ ...base, scrollTop });
      expect(win.rowCount).toBe(25);
      expect(win.totalHeight).toBe(3000);
    }
  });

  it('renders from the top with only trailing overscan at rest', () => {
    const win = gridWindow(base);
    expect(win.firstRow).toBe(0);
    expect(win.startIndex).toBe(0);
    // 5 visible rows + 2 overscan.
    expect(win.endRow).toBe(7);
    expect(win.endIndex).toBe(28);
    expect(win.paddingTop).toBe(0);
  });

  it('keeps padding and rendered rows summing to the full height', () => {
    // This is what stops the scrollbar twitching as the window moves.
    for (const scrollTop of [0, 130, 700, 1450, 2999]) {
      const win = gridWindow({ ...base, scrollTop });
      const rendered = (win.endRow - win.firstRow) * base.rowHeight;
      expect(win.paddingTop + rendered + win.paddingBottom).toBe(win.totalHeight);
    }
  });

  it('never starts before the first cell on a negative scrollTop', () => {
    // Rubber-band overscroll reports a negative offset; a negative startIndex
    // would slice from the end of the array and show the wrong frames.
    const win = gridWindow({ ...base, scrollTop: -400 });
    expect(win.firstRow).toBe(0);
    expect(win.startIndex).toBe(0);
    expect(win.paddingTop).toBe(0);
  });

  it('never ends past the last cell when scrolled beyond the content', () => {
    // Happens for one frame after a filter shrinks the list.
    const win = gridWindow({ ...base, scrollTop: 100000 });
    expect(win.endIndex).toBe(base.count);
    expect(win.endRow).toBe(win.rowCount);
    expect(win.startIndex).toBeLessThan(win.endIndex);
    expect(win.paddingBottom).toBe(0);
  });

  it('clamps the last page to the real cell count, not the row capacity', () => {
    // 10 cells over 4 columns is 3 rows, whose capacity is 12.
    const win = gridWindow({ ...base, count: 10, scrollTop: 0 });
    expect(win.rowCount).toBe(3);
    expect(win.endIndex).toBe(10);
  });

  it('draws nothing for an empty grid', () => {
    const win = gridWindow({ ...base, count: 0 });
    expect(win.startIndex).toBe(0);
    expect(win.endIndex).toBe(0);
    expect(win.rowCount).toBe(0);
    expect(win.totalHeight).toBe(0);
  });

  it('still renders a row before the viewport has been measured', () => {
    // A zero height is what the first paint reports; blanking the grid then
    // would flash empty on every mount.
    const win = gridWindow({ ...base, viewportHeight: 0 });
    expect(win.endIndex).toBeGreaterThan(0);
  });

  it('survives NaN measurements rather than propagating them', () => {
    const win = gridWindow({
      ...base,
      scrollTop: Number.NaN,
      viewportHeight: Number.NaN,
      rowHeight: Number.NaN,
      columns: Number.NaN,
    });
    expect(Number.isFinite(win.totalHeight)).toBe(true);
    expect(win.startIndex).toBe(0);
    expect(win.endIndex).toBeGreaterThan(0);
  });

  it('treats a zero column count as one column instead of dividing by it', () => {
    const win = gridWindow({ ...base, columns: 0, count: 5 });
    expect(win.rowCount).toBe(5);
  });
});

describe('columnsForWidth', () => {
  it('fits cells with the gap counted between them, not after the last', () => {
    // 4 * 160 + 3 * 16 = 688, so 700 fits four and 680 does not.
    expect(columnsForWidth(700, 160, 16)).toBe(4);
    expect(columnsForWidth(680, 160, 16)).toBe(3);
  });

  it('never reports zero columns for a container narrower than a cell', () => {
    expect(columnsForWidth(40, 160, 16)).toBe(1);
    expect(columnsForWidth(0, 160, 16)).toBe(1);
    expect(columnsForWidth(Number.NaN, 160, 16)).toBe(1);
  });
});

describe('scrollToIndex', () => {
  it('leaves an already-visible cell alone', () => {
    expect(scrollToIndex(4, base)).toBeNull();
  });

  it('scrolls up to a cell above the window', () => {
    // Row 5 starts at 600.
    expect(scrollToIndex(20, { ...base, scrollTop: 1200 })).toBe(600);
  });

  it('scrolls down just far enough to reveal a cell below the window', () => {
    // Row 8 ends at 1080; showing it in a 600px viewport needs 480.
    expect(scrollToIndex(35, { ...base, scrollTop: 0 })).toBe(480);
  });

  it('refuses an index that is not in the grid', () => {
    expect(scrollToIndex(-1, base)).toBeNull();
    expect(scrollToIndex(base.count, base)).toBeNull();
  });
});
