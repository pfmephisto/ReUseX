// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';

import styles from './EmptyState.module.css';

export interface EmptyStateProps {
  title: string;
  /** What would put something here — the stage that produces it. */
  detail?: string;
  action?: ReactNode;
}

/**
 * "Nothing here yet", said without alarm.
 *
 * Deliberately low-key and visually distinct from `ErrorBanner`. A freshly
 * imported project legitimately has no meshes, no components and no material
 * passports — that is the normal starting state of the pipeline, not a fault,
 * and styling it like one would train the user to ignore real failures.
 */
export function EmptyState({ title, detail, action }: EmptyStateProps) {
  return (
    <div className={styles.empty}>
      <p className={styles.title}>{title}</p>
      {detail && <p className={styles.detail}>{detail}</p>}
      {action && <div className={styles.action}>{action}</div>}
    </div>
  );
}
