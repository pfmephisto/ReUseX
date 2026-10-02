// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { MouseEvent, RefObject } from 'react';
import { NavLink } from 'react-router-dom';

import {
  ALL_CASES_PATH,
  badgeText,
  drawerClickCloses,
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
  /** The drawer's id, for the title bar's `aria-controls`. */
  id?: string;
  /** Below 900px the sidebar is a drawer (Phase 6 R5); this opens it. */
  open?: boolean;
  /** A link was clicked inside the open drawer. */
  onClose?: () => void;
  /** The drawer takes focus when it opens. */
  navRef?: RefObject<HTMLElement | null>;
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

/** Tag names from the click target up to (not including) the drawer. */
function tagsUpTo(e: MouseEvent<HTMLElement>): string[] {
  const tags: string[] = [];
  for (let el = e.target as Element | null; el && el !== e.currentTarget; el = el.parentElement) {
    tags.push(el.tagName);
  }
  return tags;
}

/**
 * The navy case sidebar: which project, the case workflow, the technical tools,
 * and the way back to the case list. Below 900px it is a drawer the title
 * bar's Menu button opens (R5); closed, CSS hides it from the tab order.
 */
export function Sidebar({ projectName, badges = {}, id, open = false, onClose, navRef }: SidebarProps) {
  return (
    <aside
      id={id}
      ref={navRef}
      className={styles.sidebar}
      data-open={open || undefined}
      tabIndex={-1}
      aria-label={open ? 'Navigation' : undefined}
      onClick={(e) => {
        if (open && drawerClickCloses(tagsUpTo(e))) onClose?.();
      }}
    >
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
        <NavLink to={ALL_CASES_PATH} className={({ isActive }) => (isActive ? styles.backActive : undefined)}>
          ← Alle sager
        </NavLink>
      </div>
    </aside>
  );
}
