// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useId, useRef, useState } from 'react';

import type { AuthUser } from '../api/types';
import { initials } from '../app/auth';
import styles from './UserMenu.module.css';

export interface UserMenuProps {
  user: AuthUser;
  /** Signs out (ends the session server-side, then the login page). */
  onLogout: () => void;
}

/**
 * Who is signed in, in the title bar: initials and name; open it for the
 * email and "Log ud". Server mode only — local mode has no login.
 */
export function UserMenu({ user, onLogout }: UserMenuProps) {
  const [open, setOpen] = useState(false);
  const [leaving, setLeaving] = useState(false);
  const rootRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);
  const panelId = useId();
  const name = user.display_name || user.email;

  // Escape or a click elsewhere closes it; Escape returns focus to the button.
  useEffect(() => {
    if (!open) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return;
      e.preventDefault();
      setOpen(false);
      buttonRef.current?.focus();
    };
    const onPointer = (e: PointerEvent) => {
      if (!rootRef.current?.contains(e.target as Node)) setOpen(false);
    };
    document.addEventListener('keydown', onKey);
    document.addEventListener('pointerdown', onPointer);
    return () => {
      document.removeEventListener('keydown', onKey);
      document.removeEventListener('pointerdown', onPointer);
    };
  }, [open]);

  return (
    <div ref={rootRef} className={styles.root}>
      <button
        ref={buttonRef}
        type="button"
        className={styles.button}
        aria-expanded={open}
        aria-controls={panelId}
        aria-label={`Bruger: ${name}`}
        onClick={() => setOpen((o) => !o)}
      >
        <span className={styles.avatar} aria-hidden="true">
          {initials(name)}
        </span>
        <span className={styles.name}>{name}</span>
        <span className={styles.caret} aria-hidden="true">
          ▾
        </span>
      </button>
      {open && (
        <div id={panelId} className={styles.panel} role="group" aria-label="Bruger">
          <div className={styles.who}>
            <span className={styles.whoName}>{name}</span>
            <span className={styles.whoEmail}>{user.email}</span>
            {user.is_admin && <span className={styles.badge}>Administrator</span>}
          </div>
          <button
            type="button"
            className={styles.logout}
            disabled={leaving}
            onClick={() => {
              setLeaving(true);
              onLogout();
            }}
          >
            {leaving ? 'Logger ud…' : 'Log ud'}
          </button>
        </div>
      )}
    </div>
  );
}
