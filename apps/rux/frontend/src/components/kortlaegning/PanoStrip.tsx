// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * A 360° equirect as a horizontally pannable strip, opened centred on the
 * part's `u` with a marker at `(u, v)` (spec A5). The image repeats
 * horizontally, so panning wraps round the full circle the way the camera saw
 * it. Drag with a pointer, or focus the strip and use ←/→; the arithmetic is
 * `kortlaegning/pano.ts`.
 *
 * Arrow keys stop here: the page's table keys (← fold, → unfold) would
 * otherwise act on the table while the user is looking round the room.
 */

import { useEffect, useRef, useState } from 'react';
import type { CSSProperties, KeyboardEvent, PointerEvent } from 'react';

import { panoBackgroundX, panoImageWidth, panoKeyStep, panoMarkerX } from '../../kortlaegning/pano';
import styles from './PanoStrip.module.css';

export interface PanoStripProps {
  url: string;
  u: number;
  v: number;
  /** Accessible name, e.g. "360°-optagelse 5 · Mødelokale". */
  label: string;
  /** False for a levelled panorama: its heading, and so `u`, is a guess. */
  marker: boolean;
  /** Called when the image fails to load. */
  onError: () => void;
  className?: string;
}

export function PanoStrip({ url, u, v, label, marker, onError, className }: PanoStripProps) {
  const ref = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ width: 0, height: 0 });
  const [pan, setPan] = useState(0);
  const drag = useRef<{ x: number; pan: number } | null>(null);

  // Re-centre on the part whenever the panorama or the part changes.
  useEffect(() => setPan(0), [url, u, v]);

  useEffect(() => {
    const el = ref.current;
    if (!el) return;
    const measure = () => setSize({ width: el.clientWidth, height: el.clientHeight });
    measure();
    if (typeof ResizeObserver === 'undefined') return;
    const observer = new ResizeObserver(measure);
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  // The background never reports a load error, so probe the URL once with an
  // Image: a failed equirect falls back to the caller's empty state.
  useEffect(() => {
    let live = true;
    const probe = new Image();
    probe.onerror = () => {
      if (live) onError();
    };
    probe.src = url;
    return () => {
      live = false;
    };
    // onError is a fresh closure per render; the URL is what matters.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [url]);

  const imageWidth = panoImageWidth(size.height);
  const ready = size.width > 0 && imageWidth > 0;
  const style: CSSProperties = {
    backgroundImage: `url("${url}")`,
    backgroundPositionX: ready ? `${panoBackgroundX(u, size.width, imageWidth, pan)}px` : undefined,
  };

  function onPointerDown(e: PointerEvent<HTMLDivElement>) {
    drag.current = { x: e.clientX, pan };
    e.currentTarget.setPointerCapture(e.pointerId);
  }
  function onPointerMove(e: PointerEvent<HTMLDivElement>) {
    if (!drag.current) return;
    setPan(drag.current.pan + (e.clientX - drag.current.x));
  }
  function onPointerUp(e: PointerEvent<HTMLDivElement>) {
    drag.current = null;
    if (e.currentTarget.hasPointerCapture(e.pointerId)) e.currentTarget.releasePointerCapture(e.pointerId);
  }
  function onKeyDown(e: KeyboardEvent<HTMLDivElement>) {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
    e.preventDefault();
    e.stopPropagation();
    const step = panoKeyStep(size.width);
    // ← looks left: the image moves right.
    setPan((p) => p + (e.key === 'ArrowLeft' ? step : -step));
  }

  return (
    <div
      ref={ref}
      className={`${styles.strip} ${className ?? ''}`}
      style={style}
      role="img"
      aria-label={`${label} — træk eller brug ← → for at se dig om`}
      tabIndex={0}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerUp}
      onKeyDown={onKeyDown}
    >
      {ready && marker && (
        <span
          className={styles.marker}
          style={{ left: `${panoMarkerX(size.width, imageWidth, pan)}px`, top: `${v * 100}%` }}
          aria-hidden="true"
        />
      )}
    </div>
  );
}
