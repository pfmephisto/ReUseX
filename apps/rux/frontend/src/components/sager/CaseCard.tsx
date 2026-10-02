// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';
import { Link } from 'react-router-dom';

import type { CaseStat, CaseStatus } from '../../sager/model';
import { NO_SURVEY_TEXT } from '../../sager/model';
import { Pill } from '../Pill';
import styles from './CaseCard.module.css';

export interface CaseCardProps {
  to: string;
  name: string;
  subline: string;
  stats: CaseStat[] | null;
  status: CaseStatus;
  date: string;
  /**
   * A server-rendered plan (`GET /renders?view=plan`). It answers 422 when the
   * project has no cloud to draw and 503 when the server has no renderer; on
   * either (or any other failure) the striped thumb shows the status (R2).
   */
  thumbUrl: string;
}

/** One case on Sager: the prototype's card, linking to the case's Overblik. */
export function CaseCard({ to, name, subline, stats, status, date, thumbUrl }: CaseCardProps) {
  const [thumbFailed, setThumbFailed] = useState(false);
  return (
    <Link to={to} className={styles.card} aria-label={name}>
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
        <h3 className={styles.name}>{name}</h3>
        <p className={styles.addr}>{subline}</p>
        <p className={styles.stats}>
          {stats
            ? stats.map((s) => (
                <span key={s.key}>
                  <b className={styles.figure}>{s.value}</b> {s.label}
                </span>
              ))
            : NO_SURVEY_TEXT}
        </p>
        <div className={styles.foot}>
          <Pill tone={status.tone}>{status.label}</Pill>
          <span className={styles.date}>{date}</span>
        </div>
      </div>
    </Link>
  );
}
