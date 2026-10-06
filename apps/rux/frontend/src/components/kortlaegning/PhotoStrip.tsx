// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The "Fotos (n)" strip: the sensor frames that see the selected part (or a
 * type's first linked part), most central first, up to `PHOTO_STRIP_MAX`
 * thumbnails and a `+n` overflow. Shared by the edit dialog and the detail
 * panel (spec A4); what it shows is `photoStripModel` in `kortlaegning/photo`.
 *
 * The label and field classes come from the host so the strip reads as one of
 * its fields rather than bringing a look of its own.
 */

import { useState } from 'react';

import { api } from '../../api/client';
import type { SurveyPart, SurveyType } from '../../api/types';
import { useAsync } from '../../app/useAsync';
import {
  type FrameLookup,
  hasInstanceLink,
  instanceKey,
  PHOTO_STRIP_THUMB_SIZE,
  photoStripModel,
} from '../../kortlaegning/photo';
import { resolveHighlightPart } from './EvidencePanel';
import styles from './PhotoStrip.module.css';

/** One Fotos thumbnail; a sunken placeholder when the image fails to load. */
function PhotoThumb({ frameId }: { frameId: number }) {
  const [errored, setErrored] = useState(false);
  if (errored) {
    return (
      <span className={styles.photoFallback} title={`Ramme ${frameId} kunne ikke hentes`}>
        Intet billede
      </span>
    );
  }
  return (
    <img
      className={styles.photo}
      src={api.frameImageUrl(frameId, 'color', { maxSize: PHOTO_STRIP_THUMB_SIZE })}
      alt={`Ramme ${frameId}`}
      loading="lazy"
      onError={() => setErrored(true)}
    />
  );
}

export interface PhotoStripProps {
  type: SurveyType;
  part: SurveyPart | null;
  /** The host's field wrapper and label classes. */
  fieldClassName: string;
  labelClassName: string;
}

export function PhotoStrip({ type, part, fieldClassName, labelClassName }: PhotoStripProps) {
  const highlight = resolveHighlightPart(type, part);
  const linked = hasInstanceLink(highlight);
  const currentKey = linked ? instanceKey(highlight.cloud, highlight.instance_id) : null;
  const frames = useAsync<FrameLookup>(
    async (signal) => {
      if (!linked) return { key: '', frames: [], failed: false };
      const key = instanceKey(highlight.cloud, highlight.instance_id);
      try {
        const result = await api.instanceFrames(highlight.cloud, highlight.instance_id, signal);
        return { key, frames: result, failed: false };
      } catch (cause) {
        if (signal.aborted) throw cause;
        return { key, frames: [], failed: true };
      }
    },
    [highlight?.cloud, highlight?.instance_id],
  );
  const { strip, count, message } = photoStripModel(currentKey, frames.data);

  return (
    <div className={fieldClassName}>
      <span className={labelClassName}>Fotos{count !== null && ` (${count})`}</span>
      {strip ? (
        <div className={styles.photos}>
          {strip.visible.map((f) => (
            <PhotoThumb key={f.frame_id} frameId={f.frame_id} />
          ))}
          {strip.overflow > 0 && <span className={styles.photoMore}>+{strip.overflow}</span>}
        </div>
      ) : (
        <span className={styles.photoEmpty}>{message}</span>
      )}
    </div>
  );
}
