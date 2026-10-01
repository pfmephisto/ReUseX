// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { Kpi } from '../../overblik/model';
import { StatCard } from '../StatCard';
import styles from './KpiRow.module.css';

export interface KpiRowProps {
  kpis: Kpi[];
}

/** The five KPI tiles; they wrap rather than shrink below a readable width. */
export function KpiRow({ kpis }: KpiRowProps) {
  return (
    <ul className={styles.row} aria-label="Nøgletal">
      {kpis.map((k) => (
        <li key={k.key}>
          <StatCard kpi label={k.label} value={k.value} unit={k.unit} ink={k.ink} hint={k.hint} />
        </li>
      ))}
    </ul>
  );
}
