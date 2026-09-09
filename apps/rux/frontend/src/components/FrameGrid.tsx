// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

import { api } from '../api/client';
import { columnsForWidth, gridWindow, scrollToIndex } from '../data/virtualGrid';
import styles from './FrameGrid.module.css';

/**
 * Cell geometry, in pixels.
 *
 * These are the one place in this frontend where a size is a number rather
 * than a token, and it is not an oversight: a virtualiser turns a scroll offset
 * into an index range by dividing by the row height, so it must *know* that
 * height numerically before layout happens. Reading it back out of the computed
 * style would make the first frame of every scroll depend on a measurement that
 * has not been taken yet.
 *
 * They are therefore layout constants owned by this module and applied as
 * inline dimensions, not design values. Everything the design system does own
 * here — colour, type, radius, the padding of the surrounding surface — still
 * comes from `tokens.css` in the stylesheet.
 */
const CELL_WIDTH = 160;
const THUMB_HEIGHT = 120;
const CAPTION_HEIGHT = 20;
const GAP = 8;
const CELL_HEIGHT = THUMB_HEIGHT + CAPTION_HEIGHT;
const ROW_HEIGHT = CELL_HEIGHT + GAP;

/**
 * Thumbnail edge requested from the server.
 *
 * Twice the rendered width, so the grid stays sharp on a 2× display, and no
 * more: this is the number that decides whether four hundred frames is one
 * screen's worth of traffic or a hundred megabytes of it.
 */
const THUMB_MAX_SIZE = 320;

export interface FrameGridProps {
  /** Frame ids to show, already filtered by the server. */
  ids: number[];
  selected: number | null;
  onSelect: (id: number) => void;
}

/**
 * A virtualised filmstrip of frame thumbnails.
 *
 * A scan holds hundreds to thousands of frames and `GET /frames` returns all
 * their ids at once, so the list is cheap and the *images* are not. Only the
 * cells inside the scroll window (plus an overscan band) are mounted, which is
 * what keeps the number of in-flight image requests proportional to the screen
 * rather than to the scan.
 *
 * The window arithmetic itself lives in `data/virtualGrid.ts` and is tested
 * there: it is the part that silently renders the wrong cells or indexes past
 * the end of the array, and tests here cannot render anything — vitest runs
 * with `environment: 'node'` and there is no DOM to scroll.
 */
export function FrameGrid({ ids, selected, onSelect }: FrameGridProps) {
  const scrollerRef = useRef<HTMLDivElement | null>(null);
  const [scrollTop, setScrollTop] = useState(0);
  const [size, setSize] = useState({ width: 0, height: 0 });

  // Measured rather than assumed: the grid sits beside a detail pane that
  // appears and disappears with the selection, so its width changes without
  // the window resizing.
  useEffect(() => {
    const element = scrollerRef.current;
    if (!element || typeof ResizeObserver === 'undefined') return;

    const observer = new ResizeObserver(() => {
      setSize({ width: element.clientWidth, height: element.clientHeight });
    });
    observer.observe(element);
    setSize({ width: element.clientWidth, height: element.clientHeight });
    return () => observer.disconnect();
  }, []);

  const columns = useMemo(
    () => columnsForWidth(size.width, CELL_WIDTH, GAP),
    [size.width],
  );

  const window_ = useMemo(
    () =>
      gridWindow({
        scrollTop,
        viewportHeight: size.height,
        rowHeight: ROW_HEIGHT,
        columns,
        count: ids.length,
        overscan: 2,
      }),
    [scrollTop, size.height, columns, ids.length],
  );

  const handleScroll = useCallback((event: React.UIEvent<HTMLDivElement>) => {
    setScrollTop(event.currentTarget.scrollTop);
  }, []);

  // Keep the selection on screen when it moves by keyboard. Without this the
  // next arrow press would move a cursor the user cannot see.
  useEffect(() => {
    const element = scrollerRef.current;
    if (!element || selected === null) return;
    const index = ids.indexOf(selected);
    const target = scrollToIndex(index, {
      scrollTop: element.scrollTop,
      viewportHeight: element.clientHeight,
      rowHeight: ROW_HEIGHT,
      columns,
      count: ids.length,
    });
    if (target !== null) element.scrollTop = target;
  }, [selected, ids, columns]);

  const handleKeyDown = useCallback(
    (event: React.KeyboardEvent<HTMLDivElement>) => {
      const deltas: Record<string, number> = {
        ArrowRight: 1,
        ArrowLeft: -1,
        ArrowDown: columns,
        ArrowUp: -columns,
      };
      const delta = deltas[event.key];
      if (delta === undefined) return;

      event.preventDefault();
      const index = selected === null ? 0 : ids.indexOf(selected);
      const next = Math.min(ids.length - 1, Math.max(0, index + delta));
      if (ids[next] !== undefined) onSelect(ids[next]);
    },
    [columns, ids, selected, onSelect],
  );

  const visible = ids.slice(window_.startIndex, window_.endIndex);

  return (
    <div
      ref={scrollerRef}
      className={styles.scroller}
      onScroll={handleScroll}
      onKeyDown={handleKeyDown}
      tabIndex={0}
      role="listbox"
      aria-label="Sensor frames"
      aria-activedescendant={selected === null ? undefined : `frame-cell-${selected}`}
    >
      <div style={{ height: window_.totalHeight, position: 'relative' }}>
        <div
          className={styles.grid}
          style={{
            transform: `translateY(${window_.paddingTop}px)`,
            gridTemplateColumns: `repeat(${columns}, ${CELL_WIDTH}px)`,
            gap: `${GAP}px`,
          }}
        >
          {visible.map((id) => (
            <FrameCell
              key={id}
              id={id}
              selected={id === selected}
              onSelect={onSelect}
            />
          ))}
        </div>
      </div>
    </div>
  );
}

interface FrameCellProps {
  id: number;
  selected: boolean;
  onSelect: (id: number) => void;
}

/**
 * One thumbnail.
 *
 * The colour image is requested without `normalize`: it is already displayable,
 * and the contract ignores the flag for it. A frame with no colour image
 * answers 404, which lands in `onError` and shows the id against an empty tile
 * rather than a browser's broken-image glyph.
 */
function FrameCell({ id, selected, onSelect }: FrameCellProps) {
  const [failed, setFailed] = useState(false);

  return (
    <button
      type="button"
      id={`frame-cell-${id}`}
      role="option"
      aria-selected={selected}
      className={`${styles.cell} ${selected ? styles.selected : ''}`}
      style={{ width: CELL_WIDTH, height: CELL_HEIGHT }}
      onClick={() => onSelect(id)}
    >
      <span className={styles.thumb} style={{ height: THUMB_HEIGHT }}>
        {failed ? (
          <span className={styles.missing}>no image</span>
        ) : (
          <img
            className={styles.image}
            src={api.frameImageUrl(id, 'color', { maxSize: THUMB_MAX_SIZE })}
            alt={`Sensor frame ${id}`}
            decoding="async"
            onError={() => setFailed(true)}
          />
        )}
      </span>
      <span className={`${styles.caption} mono`} style={{ height: CAPTION_HEIGHT }}>
        {id}
      </span>
    </button>
  );
}
