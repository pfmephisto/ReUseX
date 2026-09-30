// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link, NavLink } from 'react-router-dom';

import {
  ALL_CASES_PATH,
  ALL_CASES_PENDING,
  badgeText,
  entriesIn,
  type NavBadge,
  type NavEntry,
} from '../app/navigation';
import styles from './Sidebar.module.css';

export interface SidebarProps {
  /** Shown under the PROJEKT eyebrow; a placeholder dash while loading. */
  projectName?: string;
  /** Live counts for entries that carry a badge; absent or 0 hides it. */
  badges?: Partial<Record<NavBadge, number>>;
}

function Entry({ entry, count }: { entry: NavEntry; count?: number }) {
  const badge = badgeText(count);
  const inner = (
    <>
      <span className={styles.dot} aria-hidden="true" />
      <span className={styles.label}>{entry.label}</span>
      {badge && (
        <span className={`${styles.count} ${entry.badge === 'reviewQueue' ? styles.hot : ''}`}>{badge}</span>
      )}
    </>
  );
  if (entry.pending) {
    return (
      <span className={`${styles.item} ${styles.pending}`} title={entry.pending} aria-disabled="true">
        {inner}
      </span>
    );
  }
  return (
    <NavLink
      to={entry.to}
      end={entry.end}
      className={({ isActive }) => `${styles.item} ${isActive ? styles.active : ''}`}
    >
      {inner}
    </NavLink>
  );
}

/**
 * The navy case sidebar: which project, the case workflow, the technical tools,
 * and the way back to the case list.
 */
export function Sidebar({ projectName, badges = {} }: SidebarProps) {
  return (
    <aside className={styles.sidebar}>
      <div className={styles.eyebrow}>Projekt</div>
      <div className={styles.project} title={projectName}>
        {projectName ?? '—'}
      </div>
      <nav className={styles.nav} aria-label="Sag">
        {entriesIn('sag').map((e) => (
          <Entry key={e.to} entry={e} count={e.badge ? badges[e.badge] : undefined} />
        ))}
      </nav>
      <div className={styles.groupLabel}>Værktøjer</div>
      <nav className={styles.nav} aria-label="Værktøjer">
        {entriesIn('tools').map((e) => (
          <Entry key={e.to} entry={e} />
        ))}
      </nav>
      <div className={styles.back}>
        {ALL_CASES_PENDING ? (
          <span className={styles.backPending} title={ALL_CASES_PENDING} aria-disabled="true">
            ← Alle sager
          </span>
        ) : (
          <Link to={ALL_CASES_PATH}>← Alle sager</Link>
        )}
      </div>
    </aside>
  );
}
