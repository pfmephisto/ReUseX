// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';

import type { Treatment } from '../api/types';
import type { Tone } from '../kortlaegning/vocab';
import styles from './Pill.module.css';

export interface PillProps {
  /** A semantic tone (good/warn/wait/crit/accent). */
  tone?: Tone;
  /** Or a waste-hierarchy step, drawn in its --circ-* colour. */
  treatment?: Treatment;
  title?: string;
  children: ReactNode;
}

/** A small status label: miljøstatus, behandling, BIM7AA, "Godkendt ✓". */
export function Pill({ tone = 'wait', treatment, title, children }: PillProps) {
  const cls = treatment ? `${styles.pill} ${styles[`circ_${treatment}`]}` : `${styles.pill} ${styles[tone]}`;
  return (
    <span className={cls} title={title}>
      {children}
    </span>
  );
}
