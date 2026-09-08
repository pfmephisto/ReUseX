// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import styles from './StatCard.module.css';

export interface StatCardProps {
  label: string;
  value: string | number;
  /** Secondary figure — a breakdown of `value`, not a second headline. */
  hint?: string;
  /** `muted` for a tile that is present but carries nothing yet. */
  tone?: 'default' | 'muted';
}

/**
 * One figure from the project inventory.
 *
 * Presentational only, and the value arrives pre-formatted: thousands
 * separators are the caller's job because only the caller knows whether the
 * number is a count, a byte size or a duration. Doing it here would push a
 * formatting policy into every consumer that does not want one.
 */
export function StatCard({ label, value, hint, tone = 'default' }: StatCardProps) {
  return (
    <div className={`${styles.card} ${tone === 'muted' ? styles.muted : ''}`}>
      <span className={`${styles.value} mono`}>{value}</span>
      <span className={styles.label}>{label}</span>
      {hint && <span className={styles.hint}>{hint}</span>}
    </div>
  );
}
