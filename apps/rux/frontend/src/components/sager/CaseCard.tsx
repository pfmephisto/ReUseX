// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useId, useState } from 'react';

import type { CaseStat, CaseStatus } from '../../sager/model';
import { NO_SURVEY_TEXT } from '../../sager/model';
import { Pill } from '../Pill';
import styles from './CaseCard.module.css';

export interface CaseCardProps {
  /**
   * The case's Overblik. A plain href, not a router link: entering a case is
   * a page load (a fresh client and events socket per case, `app/cases.ts`).
   */
  href: string;
  name: string;
  subline: string;
  stats: CaseStat[] | null;
  status: CaseStatus;
  date: string;
  /** Small print: the file and its size. */
  fileLine?: string;
  /** The case is archived. */
  archived?: boolean;
  /**
   * A server-rendered plan (`GET /renders?view=plan`). It answers 422 when the
   * project has no cloud to draw and 503 when the server has no renderer; on
   * either (or any other failure) the striped thumb shows the status (R2).
   */
  thumbUrl: string;
}

/** One case on Sager: the prototype's card, linking to the case's Overblik. */
export function CaseCard({ href, name, subline, stats, status, date, fileLine, archived, thumbUrl }: CaseCardProps) {
  const [thumbFailed, setThumbFailed] = useState(false);
  const id = useId();
  // Named by the case alone; the subline, status and date describe it.
  return (
    <a href={href} className={styles.card} aria-label={name} aria-describedby={`${id}-sub ${id}-foot`}>
      <div className={styles.thumb} data-plain={thumbFailed || undefined}>
        {thumbFailed ? (
          <span className={styles.thumbLabel}>{status.label}</span>
        ) : (
          <img
            className={styles.thumbImg}
            src={thumbUrl}
            alt="Plan af punktskyen"
            loading="lazy"
            onError={() => setThumbFailed(true)}
          />
        )}
      </div>
      <div className={styles.body}>
        <h2 className={styles.name}>{name}</h2>
        <p id={`${id}-sub`} className={styles.addr}>
          {subline}
        </p>
        <p className={styles.stats}>
          {stats
            ? stats.map((s) => (
                <span key={s.key}>
                  <b className={styles.figure}>{s.value}</b> {s.label}
                </span>
              ))
            : NO_SURVEY_TEXT}
        </p>
        {fileLine && <p className={`${styles.file} mono`}>{fileLine}</p>}
        <div id={`${id}-foot`} className={styles.foot}>
          <Pill tone={status.tone}>{status.label}</Pill>
          {archived && <Pill tone="wait">Arkiveret</Pill>}
          <span className={styles.date}>{date}</span>
        </div>
      </div>
    </a>
  );
}
