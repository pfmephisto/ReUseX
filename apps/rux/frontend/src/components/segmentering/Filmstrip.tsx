// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef } from 'react';

import { api } from '../../api/client';
import { filmstripWindow } from '../../data/segmentView';
import styles from './Filmstrip.module.css';

/** Thumbnails either side of the current frame. */
const RADIUS = 6;
/** 2x a thumbnail's displayed edge, for HiDPI. */
const THUMB_MAX_SIZE = 160;

export interface FilmstripProps {
  ids: readonly number[];
  current: number | null;
  segmented: ReadonlySet<number>;
  onSelect: (id: number) => void;
}

/**
 * A window of frames around the current one, with prev/next and a scrubber
 * over the whole scan — a scan has thousands of frames, so only the window is
 * ever rendered. A dot marks frames that already have a saved segmentation.
 */
export function Filmstrip({ ids, current, segmented, onSelect }: FilmstripProps) {
  const at = current === null ? -1 : ids.indexOf(current);
  const visible = filmstripWindow(ids, current, RADIUS);
  const currentRef = useRef<HTMLButtonElement>(null);
  // Keep the current thumbnail in view when the strip is narrower than the window.
  useEffect(() => {
    currentRef.current?.scrollIntoView({ block: 'nearest', inline: 'center' });
  }, [current]);
  return (
    <nav className={styles.strip} aria-label="Billeder">
      <div className={styles.thumbs}>
        <button
          type="button"
          className={styles.step}
          onClick={() => at > 0 && onSelect(ids[at - 1])}
          disabled={at <= 0}
          aria-label="Forrige billede (←)"
          title="Forrige billede (←)"
        >
          ‹
        </button>
        <ol className={styles.list}>
          {visible.map((id) => (
            <li key={id}>
              <button
                ref={id === current ? currentRef : undefined}
                type="button"
                className={`${styles.thumb} ${id === current ? styles.current : ''}`}
                aria-current={id === current ? 'true' : undefined}
                onClick={() => onSelect(id)}
                title={`Billede ${id}${segmented.has(id) ? ' · segmenteret' : ''}`}
              >
                <img src={api.frameImageUrl(id, 'color', { maxSize: THUMB_MAX_SIZE })} alt="" loading="lazy" />
                <span className={styles.id}>{id}</span>
                {segmented.has(id) && <span className={styles.dot} aria-label="segmenteret" />}
              </button>
            </li>
          ))}
        </ol>
        <button
          type="button"
          className={styles.step}
          onClick={() => at >= 0 && at < ids.length - 1 && onSelect(ids[at + 1])}
          disabled={at < 0 || at >= ids.length - 1}
          aria-label="Næste billede (→)"
          title="Næste billede (→)"
        >
          ›
        </button>
      </div>
      {ids.length > 1 && (
        <label className={styles.scrub}>
          <span className={styles.position}>
            {at >= 0 ? at + 1 : '–'} / {ids.length}
          </span>
          <input
            type="range"
            min={0}
            max={ids.length - 1}
            value={Math.max(0, at)}
            onChange={(e) => onSelect(ids[Number(e.target.value)])}
            aria-label="Spol gennem scanningens billeder"
          />
        </label>
      )}
    </nav>
  );
}
