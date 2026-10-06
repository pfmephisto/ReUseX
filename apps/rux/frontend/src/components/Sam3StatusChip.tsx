// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Sam3View } from '../data/sam3Provisioning';
import styles from './Sam3StatusChip.module.css';

const TONE: Record<Sam3View['phase'], string> = {
  unknown: styles.wait,
  ready: styles.good,
  'first-run': styles.warn,
  preparing: styles.accent,
  error: styles.crit,
};

/**
 * The managed SAM3 model's state: a chip, plus a progress bar and a line of
 * explanation while it is downloaded or built, or when it failed.
 */
export function Sam3StatusChip({ view, compact = false }: { view: Sam3View; compact?: boolean }) {
  return (
    <div className={styles.wrap} role="status" aria-live="polite">
      <span className={`${styles.chip} ${TONE[view.phase]}`}>
        <span className={styles.dot} aria-hidden="true" />
        SAM3 · {view.label}
      </span>
      {view.progress !== null && (
        <span
          className={styles.bar}
          role="progressbar"
          aria-label={view.label}
          aria-valuemin={0}
          aria-valuemax={100}
          aria-valuenow={Math.round(view.progress * 100)}
        >
          <span className={styles.fill} style={{ inlineSize: `${Math.round(view.progress * 100)}%` }} />
        </span>
      )}
      {!compact && view.message && <p className={styles.message}>{view.message}</p>}
    </div>
  );
}
