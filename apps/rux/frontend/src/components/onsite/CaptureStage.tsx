// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import { PHOTO_TEXT, type Reticle } from '../../onsite/model';
import styles from './CaptureStage.module.css';

export interface CaptureStageProps {
  photoUrl: string | null;
  placeholder: string;
  reticle: Reticle | null;
  title: string;
  detail: string;
  /** The part's code, for the photo's alt text. */
  code: string;
  /** The frame aspect to hold; `null` keeps the 4/3 default. */
  aspect: { width: number; height: number } | null;
}

/**
 * The prototype's camera frame, as a stored photo (R6): the part's best
 * sensor frame, the reticle on its instance, and the detection chip. Takes
 * the frame's aspect ratio, so the reticle's percentages land on the photo.
 * A photo, not the 3D canvas: the stage is themed like any other surface.
 *
 * `aspect` is the caller's call: OnsitePage passes the frame aspect while the
 * frames lookup is loading and once a photo arrives, so Videre onto a part
 * with a photo never resizes the stage (and moves the sheet) when it loads.
 * For an unlinked part, no frame or a failed lookup it passes `null` and the
 * stage keeps 4/3 — a portrait frame's height would push the sheet under the
 * fold for nothing. An image that fails to load falls back to 4/3 too.
 *
 * With no photo the placeholder deliberately borrows the 3D canvas tone
 * (`--color-canvas`), so it reads as a dark camera viewfinder in both themes
 * (R6: dark placeholder).
 */
export function CaptureStage({ photoUrl, placeholder, reticle, title, detail, code, aspect }: CaptureStageProps) {
  const [failedUrl, setFailedUrl] = useState<string | null>(null);
  const imageFailed = photoUrl !== null && failedUrl === photoUrl;
  const showPhoto = photoUrl !== null && !imageFailed;
  return (
    <div
      className={`${styles.stage} ${showPhoto ? '' : styles.empty}`}
      style={aspect && !imageFailed ? { aspectRatio: `${aspect.width} / ${aspect.height}` } : undefined}
    >
      {showPhoto ? (
        <img className={styles.photo} src={photoUrl} alt={`Bedste foto af ${code}, ${title}`} onError={() => setFailedUrl(photoUrl)} />
      ) : (
        <p className={styles.placeholder}>{photoUrl !== null ? PHOTO_TEXT.failed : placeholder}</p>
      )}
      {/* Without the frame's size the stage is 4/3, so the percentages would miss. */}
      {showPhoto && reticle && aspect && (
        <div
          className={styles.reticle}
          aria-hidden="true"
          style={{ left: `${reticle.left}%`, top: `${reticle.top}%`, width: `${reticle.width}%`, height: `${reticle.height}%` }}
        />
      )}
      <div className={styles.chip}>
        <span className={styles.chipTitle}>{title}</span>
        <small className={styles.chipDetail}>{detail}</small>
      </div>
    </div>
  );
}
