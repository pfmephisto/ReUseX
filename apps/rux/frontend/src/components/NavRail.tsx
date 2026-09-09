// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { NavLink } from 'react-router-dom';

import styles from './NavRail.module.css';

interface NavEntry {
  to: string;
  label: string;
  /** Set when the destination does not exist yet; renders inert with a reason. */
  pending?: string;
}

/**
 * The navigation model for the whole app.
 *
 * Not-yet-built destinations are listed and visibly inert rather than hidden: a
 * user who cannot see that a place will exist reasonably concludes the GUI
 * cannot do that thing at all, which is a worse lie than an item saying when it
 * arrives. Nothing is pending as of Phase 4 — `/frames` and `/data` are the
 * last two the rail was promising — but the mechanism stays, because the next
 * phase will want it and re-deriving it from an empty rail would be guesswork.
 */
const ENTRIES: NavEntry[] = [
  { to: '/', label: 'Overview' },
  { to: '/viewport', label: 'Viewport' },
  { to: '/pipeline', label: 'Pipeline' },
  { to: '/frames', label: 'Frames' },
  { to: '/data', label: 'Data' },
];

export function NavRail() {
  return (
    <nav className={styles.rail} aria-label="Primary">
      {ENTRIES.map((entry) =>
        entry.pending ? (
          <span key={entry.to} className={styles.pending} title={entry.pending}>
            {entry.label}
            <span className={styles.pendingMark} aria-hidden="true">
              ◦
            </span>
          </span>
        ) : (
          <NavLink
            key={entry.to}
            to={entry.to}
            end={entry.to === '/'}
            className={({ isActive }) => `${styles.link} ${isActive ? styles.active : ''}`}
          >
            {entry.label}
          </NavLink>
        ),
      )}
    </nav>
  );
}
