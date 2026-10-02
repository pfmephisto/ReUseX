// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { RefObject } from 'react';

import type { ConnectionStatus } from '../api/events';
import { JobIndicator } from './JobIndicator';
import { ThemeToggle } from './ThemeToggle';
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
  /** Below 900px: the sidebar drawer is open (R5). */
  menuOpen?: boolean;
  /** Below 900px: toggles the sidebar drawer. Without it no Menu button is drawn. */
  onMenu?: () => void;
  menuRef?: RefObject<HTMLButtonElement | null>;
  /** The drawer's id. */
  menuControls?: string;
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
  menuOpen = false,
  onMenu,
  menuRef,
  menuControls,
}: TitleBarProps) {
  const title = unreachable
    ? 'Server utilgængelig'
    : (projectName ?? 'Indlæser…');

  return (
    <header className={styles.bar}>
      <div className={styles.identity}>
        {onMenu && (
          <button
            ref={menuRef}
            type="button"
            className={styles.menu}
            aria-label="Menu"
            aria-expanded={menuOpen}
            aria-controls={menuControls}
            onClick={onMenu}
          >
            <span aria-hidden="true">☰</span>
          </button>
        )}
        <span className={styles.product}>
          ReUse<em className={styles.x}>X</em>
        </span>
        <span className={styles.divider} aria-hidden="true" />
        <span className={styles.project} title={title}>
          {title}
        </span>
        {projectOpen === false && (
          <span className={styles.badgeWarn} title="Serveren kunne ikke åbne databasen">
            ikke åben
          </span>
        )}
        {schemaVersion !== undefined && (
          <span className={`${styles.meta} mono`}>skema v{schemaVersion}</span>
        )}
      </div>

      <div className={styles.status}>
        <JobIndicator connection={connection} activeJobCount={activeJobCount} />
        {implementation && <span className={styles.meta}>{implementation}</span>}
        {version && <span className={`${styles.meta} mono`}>{version}</span>}
        {/* Below the breakpoint the theme control lives in the drawer. */}
        <span className={styles.theme}>
          <ThemeToggle />
        </span>
      </div>
    </header>
  );
}
