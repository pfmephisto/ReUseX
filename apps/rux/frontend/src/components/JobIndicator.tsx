// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ConnectionStatus } from '../api/events';
import styles from './JobIndicator.module.css';

export interface JobIndicatorProps {
  connection: ConnectionStatus;
  activeJobCount: number;
}

const CONNECTION_LABEL: Record<ConnectionStatus, string> = {
  connecting: 'Forbinder til hændelseskanalen…',
  open: 'Live',
  closed: 'Hændelseskanalen er afbrudt — viste resultater kan være forældede',
};

/**
 * Global "is anything running?" indicator.
 *
 * Deliberately reports two independent facts. A disconnected event channel is
 * not the same as an idle server — if it were collapsed into one dot, a dropped
 * socket during a 20-minute `create clouds` would read as "nothing is
 * happening", which is the single most misleading thing this bar could say.
 */
export function JobIndicator({ connection, activeJobCount }: JobIndicatorProps) {
  const busy = activeJobCount > 0;

  return (
    <div className={styles.indicator}>
      <span
        className={`${styles.dot} ${styles[connection]} ${busy ? styles.busy : ''}`}
        aria-hidden="true"
      />
      <span className={styles.label}>
        {busy ? `${activeJobCount} job kører` : CONNECTION_LABEL[connection]}
      </span>
    </div>
  );
}
