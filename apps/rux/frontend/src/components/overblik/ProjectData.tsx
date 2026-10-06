// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';
import { Link } from 'react-router-dom';

import { api } from '../../api/client';
import type { ProjectSummary } from '../../api/types';
import { useAsync } from '../../app/useAsync';
import { componentTypeRows, projectDataSummary } from '../../overblik/projectData';
import { DataTable } from '../DataTable';
import { EmptyState } from '../EmptyState';
import { ErrorBanner } from '../ErrorBanner';
import { PipelineLogList } from '../PipelineLogList';
import { Spinner } from '../Spinner';
import { CLOUD_COLUMNS, MESH_COLUMNS, TYPE_COLUMNS } from './projectDataColumns';
import styles from './ProjectData.module.css';

/** How many recent runs the section shows. The full history is at /pipeline/log. */
const LOG_LIMIT = 5;

/**
 * Projektdata — the technical inventory at the foot of Overblik (spec A2),
 * formerly its own Værktøjer screen. Quiet on purpose: a closed disclosure
 * with a one-line summary, for the user who needs the figures, out of the way
 * of the case work above it.
 *
 * The body mounts only while open, so the run log is read on demand. It is
 * its own failure surface: a 503 there must not touch the rest of Overblik.
 */
export function ProjectData({ summary }: { summary: ProjectSummary }) {
  const [open, setOpen] = useState(false);
  return (
    <details className={styles.section} onToggle={(e) => setOpen(e.currentTarget.open)}>
      <summary className={styles.summary}>
        <span className={styles.chevron} aria-hidden="true" />
        <span className={styles.heading}>Projektdata</span>
        <span className={styles.line}>{projectDataSummary(summary)}</span>
      </summary>
      {open && <ProjectDataBody summary={summary} />}
    </details>
  );
}

function ProjectDataBody({ summary }: { summary: ProjectSummary }) {
  const log = useAsync((signal) => api.pipelineLog(LOG_LIMIT, signal), []);
  const types = componentTypeRows(summary);
  return (
    <div className={styles.body}>
      <section className={styles.block} aria-labelledby="pd-log">
        <div className={styles.blockHead}>
          <h4 id="pd-log" className={styles.blockHeading}>
            Seneste aktivitet
          </h4>
          <Link to="/pipeline/log" className={styles.more}>
            Hele kørselsloggen
          </Link>
        </div>
        {log.error ? (
          <ErrorBanner error={log.error} onRetry={log.reload} context="kørselsloggen" />
        ) : log.data ? (
          <PipelineLogList entries={log.data} compact />
        ) : (
          <Spinner label="Læser kørselsloggen…" />
        )}
      </section>

      <section className={styles.block} aria-labelledby="pd-clouds">
        <h4 id="pd-clouds" className={styles.blockHeading}>
          Punktskyer
        </h4>
        <DataTable
          columns={CLOUD_COLUMNS}
          rows={summary.clouds}
          rowKey={(c) => c.name}
          empty={
            <EmptyState bare
              title="Ingen punktskyer"
              detail="`rux create clouds` projicerer de importerede dybdebilleder til en samlet punktsky."
            />
          }
        />
      </section>

      <section className={styles.block} aria-labelledby="pd-meshes">
        <h4 id="pd-meshes" className={styles.blockHeading}>
          Meshes
        </h4>
        <DataTable
          columns={MESH_COLUMNS}
          rows={summary.meshes}
          rowKey={(m) => m.name}
          empty={<EmptyState bare title="Ingen meshes" detail="`rux create mesh` løser cellekomplekset til en lukket flade." />}
        />
      </section>

      <section className={styles.block} aria-labelledby="pd-types">
        <h4 id="pd-types" className={styles.blockHeading}>
          Komponenter pr. type
        </h4>
        <DataTable
          columns={TYPE_COLUMNS}
          rows={types}
          rowKey={(r) => r.type}
          empty={
            <EmptyState bare
              title="Ingen bygningskomponenter"
              detail="Komponenter dukker op, når rekonstruktionen har klassificeret fladerne."
            />
          }
        />
      </section>
    </div>
  );
}
