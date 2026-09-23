// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';

import { api } from '../api/client';
import type { PanoramaInfo } from '../api/types';
import styles from './PanoramaGrid.module.css';

/**
 * Cell geometry, in pixels.
 *
 * Panoramas are wide-aspect equirectangulars so the thumb is landscape.
 * Same reasoning as FrameGrid for using literal numbers: these drive the
 * grid template and cannot be CSS tokens the browser has not measured yet.
 */
const CELL_WIDTH = 160;
const THUMB_HEIGHT = 90;
const CAPTION_HEIGHT = 28;
const GAP = 8;

/**
 * Thumbnail edge requested from the server.
 *
 * A full equirectangular is routinely 8192 × 4096; this cap is what keeps a
 * strip of twenty panorama thumbs from pulling tens of megabytes.
 */
const THUMB_MAX_SIZE = 320;

export interface PanoramaGridProps {
  panoramas: PanoramaInfo[];
  selected: number | null;
  onSelect: (id: number) => void;
}

/**
 * A simple (non-virtualised) grid of 360° panorama thumbnails.
 *
 * Not virtualised because projects carry tens of panoramas, not thousands.
 * Keyboard navigation supports left/right (the natural axis for a strip of
 * wide-aspect images); up/down is omitted because the column count is
 * determined by auto-fill and is not known without measuring.
 */
export function PanoramaGrid({ panoramas, selected, onSelect }: PanoramaGridProps) {
  const handleKeyDown = useCallback(
    (event: React.KeyboardEvent<HTMLDivElement>) => {
      const deltas: Record<string, number> = {
        ArrowRight: 1,
        ArrowLeft: -1,
      };
      const delta = deltas[event.key];
      if (delta === undefined) return;

      event.preventDefault();
      const index = selected === null ? 0 : panoramas.findIndex((p) => p.id === selected);
      const next = Math.min(panoramas.length - 1, Math.max(0, (index < 0 ? 0 : index) + delta));
      const pano = panoramas[next];
      if (pano !== undefined) onSelect(pano.id);
    },
    [panoramas, selected, onSelect],
  );

  return (
    <div
      className={styles.grid}
      style={{
        gridTemplateColumns: `repeat(auto-fill, ${CELL_WIDTH}px)`,
        gap: GAP,
      }}
      onKeyDown={handleKeyDown}
      tabIndex={0}
      role="listbox"
      aria-label="360° panoramas"
      aria-activedescendant={selected === null ? undefined : `pano-cell-${selected}`}
    >
      {panoramas.map((pano) => (
        <PanoramaCell
          key={pano.id}
          pano={pano}
          selected={pano.id === selected}
          onSelect={onSelect}
        />
      ))}
    </div>
  );
}

interface PanoramaCellProps {
  pano: PanoramaInfo;
  selected: boolean;
  onSelect: (id: number) => void;
}

/**
 * One panorama thumbnail.
 *
 * Shows the filename (truncated) and a small "aligned" indicator when
 * `rux align 360` has resected this panorama — an unaligned panorama in the
 * viewer renders with an arbitrary heading, which is worth calling out.
 */
function PanoramaCell({ pano, selected, onSelect }: PanoramaCellProps) {
  const [failed, setFailed] = useState(false);

  return (
    <button
      type="button"
      id={`pano-cell-${pano.id}`}
      role="option"
      aria-selected={selected}
      className={`${styles.cell} ${selected ? styles.selected : ''}`}
      style={{ width: CELL_WIDTH }}
      onClick={() => onSelect(pano.id)}
    >
      <span className={styles.thumb} style={{ height: THUMB_HEIGHT }}>
        {failed ? (
          <span className={styles.missing}>no image</span>
        ) : (
          <img
            className={styles.image}
            src={api.panoramaImageUrl(pano.id, { maxSize: THUMB_MAX_SIZE })}
            alt={pano.filename}
            decoding="async"
            onError={() => setFailed(true)}
          />
        )}
      </span>
      <span className={styles.caption} style={{ height: CAPTION_HEIGHT }}>
        <span className={styles.filename} title={pano.filename}>
          {pano.filename}
        </span>
        {pano.has_pose && (
          <span className={styles.aligned} title="Pose aligned">
            aligned
          </span>
        )}
      </span>
    </button>
  );
}
