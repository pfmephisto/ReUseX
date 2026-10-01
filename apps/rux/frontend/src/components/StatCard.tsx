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
  /** Overblik's KPI look: a big display-face figure. */
  kpi?: boolean;
  /** A unit set small after the figure, e.g. "%". */
  unit?: string;
  /** Figure colour for a number that asks for action. */
  ink?: 'warn' | 'crit';
}

/**
 * One figure from the project inventory.
 *
 * Presentational only, and the value arrives pre-formatted: thousands
 * separators are the caller's job because only the caller knows whether the
 * number is a count, a byte size or a duration. Doing it here would push a
 * formatting policy into every consumer that does not want one.
 */
export function StatCard({ label, value, hint, tone = 'default', kpi = false, unit, ink }: StatCardProps) {
  const cls = [styles.card, tone === 'muted' ? styles.muted : '', kpi ? styles.kpi : '', ink ? styles[ink] : '']
    .filter(Boolean)
    .join(' ');
  return (
    <div className={cls}>
      <span className={`${styles.value} ${kpi ? '' : 'mono'}`}>
        {value}
        {unit && <small className={styles.unit}> {unit}</small>}
      </span>
      <span className={styles.label}>{label}</span>
      {hint && <span className={styles.hint}>{hint}</span>}
    </div>
  );
}
