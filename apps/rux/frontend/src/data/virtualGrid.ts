// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Window arithmetic for a fixed-cell virtualised grid.
 *
 * Hand-rolled rather than pulled from `react-window`, and kept pure and apart
 * from the component for the same reason `viewport/pagination.ts` is: this is
 * the part that silently shows the wrong cells or falls off the end of the
 * array, and it must be testable without a DOM. The tests run under
 * `environment: 'node'` — there is no element to scroll.
 *
 * The grid is uniform: every cell is `rowHeight` tall, every row holds
 * `columns` cells, and the last row may be short. That uniformity is what lets
 * the scroll position be turned into an index range by division rather than by
 * measuring anything, which in turn is what makes a scan of several hundred
 * frame thumbnails cost a screenful of `<img>` elements instead of all of them.
 */

export interface GridWindowInput {
  /** Current scroll offset of the scrolling element, in pixels. */
  scrollTop: number;
  /** Visible height of the scrolling element, in pixels. */
  viewportHeight: number;
  /** Height of one row, including the gap below it, in pixels. */
  rowHeight: number;
  /** Cells per row. */
  columns: number;
  /** Total number of cells. */
  count: number;
  /** Extra rows rendered above and below, to hide scroll latency. */
  overscan?: number;
}

export interface GridWindow {
  /** First rendered row, 0-based. */
  firstRow: number;
  /** One past the last rendered row. */
  endRow: number;
  /** First rendered cell index. */
  startIndex: number;
  /** One past the last rendered cell index. Never exceeds `count`. */
  endIndex: number;
  /** Pixels of empty space to hold above the rendered rows. */
  paddingTop: number;
  /** Pixels of empty space to hold below them. */
  paddingBottom: number;
  /** Height of the full, unvirtualised grid — what the scrollbar measures. */
  totalHeight: number;
  /** Total rows the grid would have. */
  rowCount: number;
}

/** An empty grid: the answer whenever the inputs describe nothing to draw. */
const EMPTY: GridWindow = {
  firstRow: 0,
  endRow: 0,
  startIndex: 0,
  endIndex: 0,
  paddingTop: 0,
  paddingBottom: 0,
  totalHeight: 0,
  rowCount: 0,
};

/** Coerce a possibly-`NaN` measurement into a usable non-negative number. */
function finite(value: number, fallback: number): number {
  return Number.isFinite(value) ? value : fallback;
}

/**
 * Which cells to render for a given scroll position.
 *
 * Clamped at both ends, always. The two failure modes this guards against are
 * asymmetric but equally real: a negative `scrollTop` (rubber-band overscroll
 * on macOS, which browsers do report) would otherwise produce a negative
 * `startIndex` and slice from the end of the array, and a `scrollTop` past the
 * content — which happens for one frame after the filter shrinks the list —
 * would produce an `endIndex` beyond `count` and index past the end.
 *
 * `paddingTop + rendered rows + paddingBottom` always sums to `totalHeight`, so
 * the scrollbar never twitches as the window moves.
 */
export function gridWindow(input: GridWindowInput): GridWindow {
  const columns = Math.max(1, Math.floor(finite(input.columns, 1)));
  const rowHeight = Math.max(1, finite(input.rowHeight, 1));
  const count = Math.max(0, Math.floor(finite(input.count, 0)));
  if (count === 0) return EMPTY;

  const overscan = Math.max(0, Math.floor(finite(input.overscan ?? 2, 2)));
  const scrollTop = Math.max(0, finite(input.scrollTop, 0));
  const viewportHeight = Math.max(0, finite(input.viewportHeight, 0));

  const rowCount = Math.ceil(count / columns);
  const totalHeight = rowCount * rowHeight;

  // `min(rowCount - 1, ...)` before the overscan, not after: scrolled past the
  // end, the first *visible* row is the last one, and subtracting the overscan
  // from that still leaves a window that ends at the content rather than a
  // window that starts beyond it and renders nothing.
  const firstVisible = Math.min(rowCount - 1, Math.floor(scrollTop / rowHeight));
  const firstRow = Math.max(0, firstVisible - overscan);

  // A zero viewport height (measured before layout, or a hidden tab) still
  // renders one row, so the grid is never blank while the browser catches up.
  const visibleRows = Math.max(1, Math.ceil(viewportHeight / rowHeight));
  const endRow = Math.min(rowCount, firstVisible + visibleRows + overscan);

  const startIndex = firstRow * columns;
  const endIndex = Math.min(count, endRow * columns);

  return {
    firstRow,
    endRow,
    startIndex,
    endIndex,
    paddingTop: firstRow * rowHeight,
    paddingBottom: Math.max(0, (rowCount - endRow) * rowHeight),
    totalHeight,
    rowCount,
  };
}

/**
 * How many cells fit across a container of `width` pixels.
 *
 * At least one, so a container narrower than a single cell renders that cell
 * clipped rather than dividing by a zero column count further down.
 */
export function columnsForWidth(width: number, cellWidth: number, gap: number): number {
  const usable = finite(width, 0);
  const cell = Math.max(1, finite(cellWidth, 1));
  const spacing = Math.max(0, finite(gap, 0));
  if (usable <= 0) return 1;
  return Math.max(1, Math.floor((usable + spacing) / (cell + spacing)));
}

/**
 * Scroll offset that brings a cell into view, or `null` if it already is.
 *
 * Used when the selection moves by keyboard: the selected thumbnail must not be
 * allowed to sit outside the window, because the next arrow press would then
 * move a selection the user cannot see.
 */
export function scrollToIndex(
  index: number,
  input: Pick<GridWindowInput, 'scrollTop' | 'viewportHeight' | 'rowHeight' | 'columns' | 'count'>,
): number | null {
  const columns = Math.max(1, Math.floor(finite(input.columns, 1)));
  const rowHeight = Math.max(1, finite(input.rowHeight, 1));
  const count = Math.max(0, Math.floor(finite(input.count, 0)));
  if (index < 0 || index >= count) return null;

  const row = Math.floor(index / columns);
  const top = row * rowHeight;
  const bottom = top + rowHeight;
  const scrollTop = Math.max(0, finite(input.scrollTop, 0));
  const viewportHeight = Math.max(0, finite(input.viewportHeight, 0));

  if (top < scrollTop) return top;
  if (bottom > scrollTop + viewportHeight) return Math.max(0, bottom - viewportHeight);
  return null;
}
