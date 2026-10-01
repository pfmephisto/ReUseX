// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import type { SurveyFractions } from '../../api/types';
import { FOOT_STATUS_ID, footStatus, fractionRows } from '../../indberetning/model';
import { formatTonnes } from '../../kortlaegning/vocab';
import { Pill } from '../Pill';
import styles from './FractionTable.module.css';

export interface FractionTableProps {
  fractions: SurveyFractions;
}

/**
 * EAK-kode · Fraktion · Behandling · Mængde · Status. Ready fractions first,
 * then the types that block sending, muted, each linking to that type in
 * Kortlægning. The footer shows the server's `total_t` — never a client sum.
 * Not a DataTable: that has no `<tfoot>` and no per-row class (muted blockers).
 */
export function FractionTable({ fractions }: FractionTableProps) {
  const foot = footStatus(fractions);
  return (
    <div className={styles.scroll}>
      <table className={styles.table}>
        <thead>
          <tr>
            <th scope="col">EAK-kode</th>
            <th scope="col">Fraktion</th>
            <th scope="col">Behandling</th>
            <th scope="col" className={styles.num}>
              Mængde
            </th>
            <th scope="col">Status</th>
          </tr>
        </thead>
        <tbody>
          {fractionRows(fractions).map((r) =>
            r.kind === 'ready' ? (
              <tr key={r.key}>
                <td className="mono">{r.eak}</td>
                <td>
                  {r.fraction} {r.contaminated && <Pill tone="crit">Forurenet</Pill>}
                </td>
                <td>{r.treatment}</td>
                <td className={`${styles.num} ${styles.amount} mono`}>{r.amount}</td>
                <td>
                  <Pill tone="good">Klar ✓</Pill>
                </td>
              </tr>
            ) : (
              <tr key={r.key} className={styles.blocking}>
                <td className="mono">{r.eak}</td>
                <td>
                  <Link to={r.href} className={styles.typeLink}>
                    {r.name}
                  </Link>
                </td>
                <td>{r.treatment}</td>
                <td className={`${styles.num} mono`}>{r.amount}</td>
                <td>
                  <Pill tone={r.status.tone}>{r.status.text}</Pill>
                </td>
              </tr>
            ),
          )}
        </tbody>
        <tfoot>
          <tr>
            <td colSpan={3}>I alt (godkendt)</td>
            <td className={`${styles.num} mono`}>{formatTonnes(fractions.total_t)}</td>
            <td>
              <span id={FOOT_STATUS_ID}>
                <Pill tone={foot.tone}>{foot.text}</Pill>
              </span>
            </td>
          </tr>
        </tfoot>
      </table>
    </div>
  );
}
