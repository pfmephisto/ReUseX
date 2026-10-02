// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { api } from '../api/client';
import { OVERBLIK_PATH } from '../app/links';
import { useAsync } from '../app/useAsync';
import { appWriteChain } from '../app/writeChain';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { CaseCard } from '../components/sager/CaseCard';
import { caseName } from '../overblik/model';
import { cardDate, cardSubline, caseStats, caseStatus, OPEN_ANOTHER_COMMAND } from '../sager/model';
import styles from './SagerPage.module.css';

/**
 * Sager — the case list (R1). `rux gui` serves one project, so the list is
 * that project's card, plus how to open another. The grid is the
 * prototype's, so a longer list from a multi-case server drops in without a
 * layout change.
 */
export function SagerPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.projectSummary(s), api.surveySummary(s)])),
    [],
  );
  // Only the status needs the fractions; a failure reads as "not done" (R3).
  const fractions = useAsync((s) => appWriteChain.idle().then(() => api.surveyFractions(s)), []);

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="sagslisten" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser sager…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey] = data;
  const record = summary.projects[0];
  const cards = [
    {
      key: summary.path,
      name: caseName(summary, record),
      subline: cardSubline(record),
      stats: caseStats(survey),
      status: caseStatus(survey, fractions.error ? null : fractions.data),
      date: cardDate(record),
    },
  ];

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Sager</h1>
        <span className={styles.count}>{cards.length}</span>
      </header>

      <ul className={styles.cards} aria-label="Sager">
        {cards.map((c) => (
          <li key={c.key}>
            <CaseCard
              to={OVERBLIK_PATH}
              name={c.name}
              subline={c.subline}
              stats={c.stats}
              status={c.status}
              date={c.date}
              thumbUrl={api.renderUrl({ view: 'plan', width: 640, height: 248 })}
            />
          </li>
        ))}
      </ul>

      <section className={styles.panel} aria-labelledby="sager-andre">
        <h2 id="sager-andre" className={styles.panelHeading}>
          Åbn en anden sag
        </h2>
        <p className={styles.text}>
          Denne server viser én sag — den projektfil, den blev startet med. Start den med en anden fil:
        </p>
        <code className={styles.command}>{OPEN_ANOTHER_COMMAND}</code>
        <p className={styles.muted}>En sagsliste på tværs af projekter hører til serverudgaven (ruxd).</p>
      </section>
    </div>
  );
}
