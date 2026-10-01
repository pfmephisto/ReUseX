// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useMemo, useState } from 'react';

import { api } from '../api/client';
import { useAsync } from '../app/useAsync';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { FractionTable } from '../components/indberetning/FractionTable';
import {
  canSend,
  CSV_FILENAME,
  FOOT_STATUS_ID,
  footStatus,
  FRACTION_NOTE,
  fractionsCsvHref,
  SEND_NOTICE,
} from '../indberetning/model';
import styles from './IndberetningPage.module.css';

/**
 * Indberetning — approved tonnes per EAK fraction for bygningsaffald.dk
 * (prototype 02e). Read-only: the fractions, the blocking types and readiness
 * are all `GET /survey/fractions`.
 *
 * The send gate is the prototype's: `Send til bygningsaffald.dk` is enabled
 * exactly when nothing blocks. v1 posts nothing, so a click only says so and
 * points to the CSV (R9), which is always available. Submission is a follow-up.
 */
export function IndberetningPage() {
  const { data, error, loading, reload } = useAsync((s) => api.surveyFractions(s), []);
  const [sendNotice, setSendNotice] = useState(false);
  const csvHref = useMemo(() => (data ? fractionsCsvHref(data) : null), [data]);

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="affaldsfraktionerne" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser fraktioner…" />
      </div>
    );
  }
  if (!data || !csvHref) return null;

  const sendable = canSend(data);
  const empty = data.fractions.length === 0 && data.blocking.length === 0;

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Indberetning</h2>
        <span className={styles.sub}>Affaldsfraktioner til bygningsaffald.dk</span>
        <div className={styles.actions}>
          <a className={styles.btnGhost} href={csvHref} download={CSV_FILENAME}>
            Hent fraktioner (CSV)
          </a>
          {!sendable && <span className={styles.gateHint}>{footStatus(data).text}</span>}
          <button
            type="button"
            className={styles.btnPrimary}
            disabled={!sendable}
            title={sendable ? undefined : footStatus(data).text}
            aria-describedby={sendable ? undefined : FOOT_STATUS_ID}
            onClick={() => setSendNotice(true)}
          >
            Send til bygningsaffald.dk
          </button>
        </div>
      </header>

      {sendNotice && (
        <p className={styles.notice} role="status">
          {SEND_NOTICE}
        </p>
      )}

      <p className={styles.note}>{FRACTION_NOTE}</p>

      {empty ? (
        <EmptyState
          title="Ingen typer i kortlægningen endnu"
          detail="Fraktionerne opstår, når typerne i Kortlægning er godkendt med tonnage."
        />
      ) : (
        <FractionTable fractions={data} />
      )}
    </div>
  );
}
