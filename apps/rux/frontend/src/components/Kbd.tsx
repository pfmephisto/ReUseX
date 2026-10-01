// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';

import styles from './Kbd.module.css';

export interface KbdProps {
  children: ReactNode;
}

/** A keyboard-shortcut label, e.g. in a tooltip or a help panel. */
export function Kbd({ children }: KbdProps) {
  return <kbd className={styles.kbd}>{children}</kbd>;
}
