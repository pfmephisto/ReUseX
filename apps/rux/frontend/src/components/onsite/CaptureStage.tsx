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
  aspect: { width: number; height: number } | null;
}

/**
 * The prototype's camera frame, as a stored photo (R6): the part's best
 * sensor frame, the reticle on its instance, and the detection chip. Takes
 * the frame's aspect ratio, so the reticle's percentages land on the photo.
 * A photo, not the 3D canvas: the stage is themed like any other surface.
 * With no photo it keeps 4/3 — a portrait frame's height would push the sheet
 * under the fold for nothing — and is dark in both themes, as a camera
 * viewfinder is (R6).
 */
export function CaptureStage({ photoUrl, placeholder, reticle, title, detail, code, aspect }: CaptureStageProps) {
  const [failedUrl, setFailedUrl] = useState<string | null>(null);
  const showPhoto = photoUrl !== null && failedUrl !== photoUrl;
  return (
    <div
      className={`${styles.stage} ${showPhoto ? '' : styles.empty}`}
      style={showPhoto && aspect ? { aspectRatio: `${aspect.width} / ${aspect.height}` } : undefined}
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
