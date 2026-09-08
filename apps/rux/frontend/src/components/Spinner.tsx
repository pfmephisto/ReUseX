// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import styles from './Spinner.module.css';

export interface SpinnerProps {
  label?: string;
}

/**
 * Inline busy indicator for a request in flight.
 *
 * `aria-live="polite"` rather than `assertive`: this appears on every route
 * load, and interrupting a screen reader each time would make the app hostile
 * to use. The ring itself is decorative — the label carries the meaning, which
 * is also why an unlabelled spinner announces nothing at all.
 */
export function Spinner({ label }: SpinnerProps) {
  return (
    <div className={styles.wrap} role="status" aria-live="polite">
      <span className={styles.ring} aria-hidden="true" />
      {label && <span className={styles.label}>{label}</span>}
    </div>
  );
}
