// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useMemo, type ReactNode } from 'react';

import { api } from '../api/client';
import type { Health, ProjectSummary, SurveySummary } from '../api/types';
import { TitleBar } from '../components/TitleBar';
import { Sidebar } from '../components/Sidebar';
import { JobToaster } from '../components/JobToaster';
import { useAsync } from './useAsync';
import { useJobs } from './JobsContext';
import { SurveyCountsProvider } from './SurveyCountsContext';
import { displayProjectName } from './navigation';
import styles from './AppShell.module.css';

/**
 * Title bar + sidebar + content region.
 *
 * The shell resolves the project identity once, from `GET /health`, rather than
 * from `/project`: health is the cheap call, it is the one the contract
 * designates for the version handshake, and it reports `project.open === false`
 * when the database could not be opened — which is exactly the state a title
 * bar must not render as if everything were fine.
 *
 * The Kortlægning and Miljø & prøver badges both come from
 * `GET /survey/summary` (`counts.queue`, `pending_samples`: samples not yet at
 * *svar*, i.e. those that can hold a type at *afventer prøve*). A server that
 * predates the survey routes answers 404; the shell then shows no badge rather
 * than an error — the badge is a hint, not something to block the app on.
 */
export function AppShell({ children }: { children: ReactNode }) {
  const { data: health, error } = useAsync<Health>((signal) => api.health(signal), []);
  const { data: summary } = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);
  const survey = useAsync<SurveySummary>((signal) => api.surveySummary(signal), []);
  const { active, status } = useJobs();
  const reviewQueue = survey.error ? undefined : survey.data?.counts.queue;
  const pendingSamples = survey.error ? undefined : survey.data?.pending_samples;
  const surveyCounts = useMemo(() => ({ refresh: survey.reload }), [survey.reload]);

  return (
    <div className={styles.shell}>
      <TitleBar
        projectName={health?.project.name}
        projectOpen={health?.project.open}
        schemaVersion={health?.project.schema_version}
        version={health?.version}
        implementation={health?.implementation}
        connection={status}
        activeJobCount={active.length}
        unreachable={Boolean(error)}
      />
      <div className={styles.body}>
        <Sidebar projectName={displayProjectName(summary, health)} badges={{ reviewQueue, pendingSamples }} />
        <main className={styles.content}>
          <SurveyCountsProvider value={surveyCounts}>{children}</SurveyCountsProvider>
        </main>
      </div>
      <JobToaster />
    </div>
  );
}
