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
 * The not-yet-built destinations are listed and visibly inert rather than
 * hidden. The pipeline is the product's core object (design brief), so a user
 * who cannot see that "Pipeline" is a place this app will have would reasonably
 * conclude the GUI simply cannot run stages — which is a worse lie than an
 * item that says when it arrives.
 */
const ENTRIES: NavEntry[] = [
  { to: '/', label: 'Overview' },
  { to: '/viewport', label: 'Viewport' },
  { to: '/pipeline', label: 'Pipeline' },
  { to: '/data', label: 'Data', pending: 'Components, passports, frames — Phase 4' },
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
