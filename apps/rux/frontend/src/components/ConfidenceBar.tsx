// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import styles from './ConfidenceBar.module.css';

export interface ConfidenceBarProps {
  /** A 0–100 confidence figure, or null when there is nothing to show yet. */
  percent: number | null;
}

/** A short inline confidence meter: a filled track plus the figure, or "—" when unknown. */
export function ConfidenceBar({ percent }: ConfidenceBarProps) {
  if (percent === null) {
    return <span className={styles.none}>—</span>;
  }
  return (
    <span className={styles.conf}>
      <span className={styles.bar}>
        <i style={{ width: `${percent}%` }} />
      </span>
      {percent} %
    </span>
  );
}
