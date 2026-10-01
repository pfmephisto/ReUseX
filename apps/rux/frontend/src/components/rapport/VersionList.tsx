// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReportPdfVersion } from '../../api/types';
import {
  formatBytesDa,
  parseServerTime,
  UNKNOWN_STATUS,
  versionDate,
  versionDateTime,
  versionStatus,
  versionTitle,
} from '../../rapport/model';
import { Pill } from '../Pill';
import styles from './VersionList.module.css';

export interface VersionListProps {
  /** Newest first, as the server lists them. */
  versions: ReportPdfVersion[];
  pdfUrl: (id: number) => string;
  /** The live material-passport CSV (`/exports/csv`): the prototype's Inventarliste. */
  inventoryUrl: string;
}

/** Stored PDF versions, then the live inventory export as a last, fixed row (R7). */
export function VersionList({ versions, pdfUrl, inventoryUrl }: VersionListProps) {
  return (
    <ul className={styles.list} aria-label="Rapportversioner">
      {versions.map((v) => {
        const status = versionStatus(v);
        const title = versionTitle(v);
        const at = parseServerTime(v.created_at);
        return (
          <li key={v.id} className={styles.row}>
            <span className={styles.format}>PDF</span>
            <span className={styles.main}>
              <span className={styles.name}>{title}</span>
              <span className={`${styles.meta} mono`}>
                <time dateTime={at?.toISOString()} title={versionDateTime(v.created_at)}>
                  {versionDate(v.created_at)}
                </time>{' '}
                · {formatBytesDa(v.size_bytes)}
              </span>
            </span>
            <span className={styles.end}>
              {status ? (
                <Pill tone={status.tone} title={status.title}>
                  {status.text}
                </Pill>
              ) : (
                <span className={styles.unknown} title={UNKNOWN_STATUS.title}>
                  {UNKNOWN_STATUS.text}
                </span>
              )}
              <a
                className={styles.download}
                href={pdfUrl(v.id)}
                download={`ressourcekortlaegning-v${v.version}.pdf`}
                aria-label={`Hent ${title}`}
              >
                ↓
              </a>
            </span>
          </li>
        );
      })}
      <li className={styles.row}>
        <span className={styles.format}>CSV</span>
        <span className={styles.main}>
          <span className={styles.name}>Inventarliste</span>
          <span className={styles.meta}>Aktuel eksport af materialepassene</span>
        </span>
        <span className={styles.end}>
          <Pill tone="accent">Eksport</Pill>
          <a className={styles.download} href={inventoryUrl} download="inventarliste.csv" aria-label="Hent inventarliste (CSV)">
            ↓
          </a>
        </span>
      </li>
    </ul>
  );
}
