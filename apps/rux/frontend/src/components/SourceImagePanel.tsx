// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Source-image cross-reference panel (#454).
 *
 * Shows the ranked list of sensor frames that see a picked world-space point,
 * with thumbnail previews and a one-click shortcut to the best source image.
 * Rendered into the viewport's `overlay` slot so it floats over the canvas.
 */

import { api } from '../api/client';
import type { FrameVisibilityList } from '../api/types';
import { bestFrame, formatScore, rankLabel } from '../data/sourceImage';
import styles from './SourceImagePanel.module.css';

/** Max thumbnail edge in pixels — 2× the 120px cell for HiDPI. */
const THUMB_MAX_SIZE = 240;

export interface SourceImagePanelProps {
  /**
   * Ranked visibility result from `/frames/visibility` (or the instance variant).
   * Null while a fetch is in flight; undefined when no point has been picked yet.
   */
  visibility: FrameVisibilityList | null | undefined;
  /** A fetch error, if any. */
  error: Error | null;
  /** Close the panel (the host sets `pickedPoint` back to null). */
  onClose: () => void;
  /**
   * Open a frame in the detail view.
   *
   * The host wires this to whatever frame-detail surface already exists on the
   * page (e.g. setting `selectedFrame` in `FramesPage`, or navigating to a
   * `/frames?frame=<id>` deep-link).
   */
  onOpenFrame: (frameId: number) => void;
}

export function SourceImagePanel({
  visibility,
  error,
  onClose,
  onOpenFrame,
}: SourceImagePanelProps) {
  const best = visibility ? bestFrame(visibility.frames) : null;

  return (
    <div className={styles.panel} role="complementary" aria-label="Source images">
      <header className={styles.header}>
        <span className={styles.title}>Source images</span>
        {visibility && (
          <span className={styles.total}>{visibility.total} visible</span>
        )}
        <button type="button" className={styles.close} onClick={onClose} aria-label="Close">
          ×
        </button>
      </header>

      <div className={styles.body}>
        {error && (
          <p className={styles.error}>Could not load frames: {error.message}</p>
        )}

        {!error && visibility === null && (
          <p className={styles.loading}>Loading…</p>
        )}

        {!error && visibility && visibility.frames.length === 0 && (
          <p className={styles.empty}>No frames see this point.</p>
        )}

        {!error && visibility && visibility.frames.length > 0 && (
          <>
            {best && (
              <button
                type="button"
                className={styles.bestButton}
                onClick={() => onOpenFrame(best.frame_id)}
              >
                Open best source image
              </button>
            )}

            <div className={styles.grid}>
              {visibility.frames.map((frame, index) => (
                <button
                  key={frame.frame_id}
                  type="button"
                  className={styles.thumb}
                  onClick={() => onOpenFrame(frame.frame_id)}
                  title={`Frame ${frame.frame_id} — score ${formatScore(frame.score)} — depth ${frame.depth.toFixed(1)} m`}
                >
                  <img
                    src={api.frameImageUrl(frame.frame_id, 'color', { maxSize: THUMB_MAX_SIZE })}
                    alt={`Frame ${frame.frame_id}`}
                    className={styles.thumbImg}
                    loading="lazy"
                  />
                  <span className={`${styles.badge}${index === 0 ? ` ${styles.bestBadge}` : ''}`}>
                    {rankLabel(index)}
                  </span>
                  <span className={styles.score}>{formatScore(frame.score)}</span>
                </button>
              ))}
            </div>
          </>
        )}
      </div>
    </div>
  );
}
