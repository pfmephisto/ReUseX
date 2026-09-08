// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useMemo } from 'react';

import { labelColorIndex, readLabelPalette } from '../viewport/labelColors';
import styles from './LabelLegend.module.css';

export interface LabelLegendProps {
  /** Label id → name, exactly as `CloudInfo.labels` delivers it. */
  labels: Record<string, string>;
  /** Cap the list; the rest are summarised. */
  limit?: number;
}

/**
 * Legend for the viewport's label colour mode.
 *
 * Reads the same palette the renderer does, through the same module, so a swatch
 * here can never disagree with a point out there. Key `"0"` is never present in
 * the payload — 0 means unlabeled — so no entry for it is rendered either.
 */
export function LabelLegend({ labels, limit = 24 }: LabelLegendProps) {
  const palette = useMemo(() => readLabelPalette(), []);

  const entries = useMemo(
    () =>
      Object.entries(labels)
        .map(([id, name]) => ({ id: Number(id), name }))
        .filter((entry) => Number.isFinite(entry.id) && entry.id >= 1)
        .sort((a, b) => a.id - b.id),
    [labels],
  );

  if (entries.length === 0) return null;
  const shown = entries.slice(0, limit);
  const hidden = entries.length - shown.length;

  return (
    <div className={styles.legend}>
      {shown.map((entry) => {
        const slot = labelColorIndex(entry.id, palette.colors.length);
        return (
          <div key={entry.id} className={styles.entry}>
            <span
              className={styles.swatch}
              style={{ background: slot < 0 ? palette.unlabeled : palette.colors[slot] }}
              aria-hidden="true"
            />
            <span className={styles.name} title={entry.name}>
              {entry.name}
            </span>
            <span className={`${styles.id} mono`}>{entry.id}</span>
          </div>
        );
      })}
      {hidden > 0 && <p className={styles.more}>+{hidden} more</p>}
    </div>
  );
}
