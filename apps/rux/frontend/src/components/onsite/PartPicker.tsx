// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId } from 'react';

import type { PickerGroup } from '../../onsite/model';
import styles from './PartPicker.module.css';

export interface PartPickerProps {
  groups: PickerGroup[];
  value: string;
  onChange: (code: string) => void;
}

/**
 * Jump to any bygningsdel, grouped by room in walk order (R6). One button per
 * part, named by its label (code · type); the current part is
 * `aria-current`.
 */
export function PartPicker({ groups, value, onChange }: PartPickerProps) {
  const id = useId();
  return (
    <nav className={styles.picker} aria-labelledby={`${id}-heading`}>
      <h2 id={`${id}-heading`} className={styles.heading}>
        Bygningsdele
      </h2>
      {groups.map((g, i) => (
        <section key={g.room} className={styles.group} aria-labelledby={`${id}-room-${i}`}>
          <h3 id={`${id}-room-${i}`} className={styles.room}>
            {g.room}
          </h3>
          <ul className={styles.list}>
            {g.options.map((o) => (
              <li key={o.code}>
                <button
                  type="button"
                  className={styles.option}
                  aria-current={o.code === value ? 'true' : undefined}
                  onClick={() => onChange(o.code)}
                >
                  {o.label}
                </button>
              </li>
            ))}
          </ul>
        </section>
      ))}
    </nav>
  );
}
