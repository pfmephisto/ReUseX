// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ConnectionStatus } from '../api/events';
import { JobIndicator } from './JobIndicator';
import styles from './TitleBar.module.css';

export interface TitleBarProps {
  /** Project file name, as reported by `GET /health`. Never a full path. */
  projectName?: string;
  /** False when the server could not open the database. */
  projectOpen?: boolean;
  schemaVersion?: number;
  /** ReUseX build version of the server. */
  version?: string;
  /** Which backend is answering — `rux-gui` today, `ruxd` in Phase 6. */
  implementation?: string;
  connection: ConnectionStatus;
  activeJobCount: number;
  /** True when even `GET /health` failed. */
  unreachable?: boolean;
}

/**
 * Identity bar: which project, served by which backend, in what state.
 *
 * It reports three distinguishable failures rather than one, because they need
 * different reactions from the user: the server is unreachable (restart it),
 * the server is up but could not open the database (wrong `-p`, or the file is
 * corrupt), or the event channel dropped while REST still works (results are
 * current, live progress is not).
 */
export function TitleBar({
  projectName,
  projectOpen,
  schemaVersion,
  version,
  implementation,
  connection,
  activeJobCount,
  unreachable = false,
}: TitleBarProps) {
  const title = unreachable
    ? 'Server unreachable'
    : (projectName ?? 'Loading…');

  return (
    <header className={styles.bar}>
      <div className={styles.identity}>
        <span className={styles.product}>ReUseX</span>
        <span className={styles.divider} aria-hidden="true" />
        <span className={styles.project} title={title}>
          {title}
        </span>
        {projectOpen === false && (
          <span className={styles.badgeWarn} title="The server could not open this database">
            not open
          </span>
        )}
        {schemaVersion !== undefined && (
          <span className={`${styles.meta} mono`}>schema v{schemaVersion}</span>
        )}
      </div>

      <div className={styles.status}>
        <JobIndicator connection={connection} activeJobCount={activeJobCount} />
        {implementation && <span className={styles.meta}>{implementation}</span>}
        {version && <span className={`${styles.meta} mono`}>{version}</span>}
      </div>
    </header>
  );
}
