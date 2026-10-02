// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { ProjectInfo } from '../api/types';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { appWriteChain } from '../app/writeChain';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { CircularityBar } from '../components/CircularityBar';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { CaseHero } from '../components/overblik/CaseHero';
import { KpiRow } from '../components/overblik/KpiRow';
import { ProjectMetaForm } from '../components/overblik/ProjectMetaForm';
import { QuickLinks } from '../components/overblik/QuickLinks';
import {
  caseName,
  circularitySegments,
  EMPTY_NAME_TOAST,
  heroSubline,
  INVALID_YEAR_TOAST,
  kpis,
  newRecordId,
  quickLinks,
  type ProjectPatch,
} from '../overblik/model';
import styles from './OverblikPage.module.css';

/**
 * Overblik — the case dashboard and the app's landing route: the case hero
 * with an in-place editor for its details, the KPI row, the circularity bar
 * and a link to each case screen with what waits there.
 *
 * Four reads at mount (project, survey summary, report versions, fractions —
 * the last only for Indberetning's blocking count, so its failure falls back
 * to the approved count rather than failing the page). The busy fix in
 * `rux gui` (Phase 5 R1) is what keeps them from racing into 503s.
 * Metadata edits run on the page's mutation queue, and each settles by
 * re-reading the shell's project summary, so the sidebar name follows.
 */
export function OverblikPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => appWriteChain.idle().then(() => Promise.all([api.projectSummary(s), api.surveySummary(s)])),
    [],
  );
  const versions = useAsync((s) => api.listReportVersions(s), []);
  const fractions = useAsync((s) => appWriteChain.idle().then(() => api.surveyFractions(s)), []);
  const { refreshProject } = useSurveyCounts();
  const toast = useToast(2600);
  const { mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refreshProject,
  });

  // The record being edited. A ref mirror, so a queued commit reads the id the
  // previous commit's response produced, not a stale render's.
  const [project, setProjectState] = useState<ProjectInfo | undefined>(undefined);
  const projectRef = useRef<ProjectInfo | undefined>(undefined);
  const setProject = useCallback((p: ProjectInfo | undefined) => {
    projectRef.current = p;
    setProjectState(p);
  }, []);
  useEffect(() => {
    if (data) setProject(data[0].projects[0]);
  }, [data, setProject]);

  // PATCH /projects/{id} upserts, so a project with no record yet gets one on
  // its first edit. The id is minted lazily, on that first commit, and kept
  // for the visit; `newRecordId` falls back when `crypto.randomUUID` is
  // missing (plain http on a LAN address).
  const newId = useRef<string | null>(null);
  const commit = useCallback(
    (patch: ProjectPatch) => {
      mutate(async () => {
        const id = projectRef.current?.id ?? (newId.current ??= newRecordId());
        setProject(await api.patchProject(id, patch));
      });
    },
    [mutate, setProject],
  );

  const [editing, setEditing] = useState(false);
  const toggleRef = useRef<HTMLButtonElement>(null);
  const closeEditor = useCallback(() => {
    setEditing(false);
    toggleRef.current?.focus(); // never drop focus to <body>
  }, []);

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="sagsoverblikket" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser sagen…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey] = data;
  const segments = circularitySegments(survey.circularity);

  return (
    <div className={styles.page}>
      <CaseHero
        name={caseName(summary, project)}
        subline={heroSubline(project)}
        editing={editing}
        onToggle={() => (editing ? closeEditor() : setEditing(true))}
        toggleRef={toggleRef}
      />
      {editing && (
        <ProjectMetaForm
          project={project}
          onCommit={commit}
          onInvalidYear={() => toast.show(INVALID_YEAR_TOAST)}
          onInvalidName={() => toast.show(EMPTY_NAME_TOAST)}
          onClose={closeEditor}
        />
      )}

      <KpiRow kpis={kpis(survey)} />

      <section className={styles.panel} aria-labelledby="cirk-heading">
        <h3 id="cirk-heading" className={styles.panelHeading}>
          Cirkularitetsoversigt
        </h3>
        {segments.length > 0 ? (
          <CircularityBar segments={segments} />
        ) : (
          <EmptyState
            bare
            title="Ingen mængder endnu"
            detail="Tonnage pr. type sættes i Kortlægning; bjælken viser den fordelt på affaldshierarkiet."
          />
        )}
      </section>

      <QuickLinks
        links={quickLinks(
          survey,
          versions.error ? null : versions.data,
          fractions.error ? null : fractions.data?.blocking_types,
        )}
      />
      <Toast message={toast.message} />
    </div>
  );
}
