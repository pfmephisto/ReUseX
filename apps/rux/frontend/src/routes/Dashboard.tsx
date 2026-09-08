// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Link } from 'react-router-dom';

import { api } from '../api/client';
import type { CloudInfo, MeshInfo, ProjectInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { DataTable, type Column } from '../components/DataTable';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { PipelineLogList } from '../components/PipelineLogList';
import { Spinner } from '../components/Spinner';
import { StatCard } from '../components/StatCard';
import styles from './Dashboard.module.css';

/** How much durable history the overview shows. Deeper history is Phase 4. */
const LOG_LIMIT = 15;

/**
 * Counts here run to eight digits — a 20-million-point cloud must not read as
 * `20000000`, which is unparseable at a glance and the single most common
 * figure on this screen.
 */
const COUNT = new Intl.NumberFormat();

/** A stored value the contract represents as empty-string-means-absent. */
function present(value?: string): string | null {
  const trimmed = value?.trim();
  return trimmed ? trimmed : null;
}

const CLOUD_COLUMNS: Column<CloudInfo>[] = [
  {
    key: 'name',
    header: 'Cloud',
    // The viewport route reads `?cloud=`; linking from the name keeps the row
    // itself inert, so selecting text in a wide table does not navigate.
    render: (cloud) => (
      <Link to={`/viewport?cloud=${encodeURIComponent(cloud.name)}`}>{cloud.name}</Link>
    ),
  },
  { key: 'type', header: 'Type', render: (cloud) => cloud.type },
  {
    key: 'point_count',
    header: 'Points',
    numeric: true,
    render: (cloud) => COUNT.format(cloud.point_count),
  },
  {
    key: 'dims',
    header: 'W × H',
    numeric: true,
    render: (cloud) => `${COUNT.format(cloud.width)} × ${COUNT.format(cloud.height)}`,
  },
  {
    key: 'organized',
    header: 'Organized',
    render: (cloud) => (cloud.organized ? 'yes' : 'no'),
  },
];

const MESH_COLUMNS: Column<MeshInfo>[] = [
  { key: 'name', header: 'Mesh', render: (mesh) => mesh.name },
  { key: 'format', header: 'Format', render: (mesh) => mesh.format ?? '—' },
  {
    key: 'vertex_count',
    header: 'Vertices',
    numeric: true,
    render: (mesh) => COUNT.format(mesh.vertex_count),
  },
  {
    key: 'face_count',
    header: 'Faces',
    numeric: true,
    render: (mesh) => COUNT.format(mesh.face_count),
  },
  {
    key: 'texture_count',
    header: 'Textures',
    numeric: true,
    render: (mesh) => COUNT.format(mesh.texture_count ?? 0),
  },
  {
    // Shown as stored. The contract types it as a bare string with no zone, so
    // re-rendering it in the viewer's local time would shift it by an unknown
    // offset — see the same reasoning in PipelineLogList.
    key: 'created_at',
    header: 'Created',
    render: (mesh) => present(mesh.created_at) ?? '—',
  },
];

interface TypeCount {
  type: string;
  count: number;
}

const TYPE_COLUMNS: Column<TypeCount>[] = [
  { key: 'type', header: 'Component type', render: (row) => row.type },
  { key: 'count', header: 'Count', numeric: true, render: (row) => COUNT.format(row.count) },
];

/**
 * Project overview — the app's landing route.
 *
 * Two independent requests, and deliberately two independent failure surfaces:
 * `/project` is the screen, so losing it replaces the screen, while
 * `/pipeline-log` is one panel and a 503 there must not blank out an inventory
 * that loaded perfectly well. Both retry through `useAsync`'s `reload`, which is
 * the whole point of the 503 branch in `ErrorBanner`.
 */
export function Dashboard() {
  const summary = useAsync((signal) => api.projectSummary(signal), []);
  const log = useAsync((signal) => api.pipelineLog(LOG_LIMIT, signal), []);

  if (summary.error) {
    return (
      <div className={styles.page}>
        <ErrorBanner
          error={summary.error}
          onRetry={summary.reload}
          context="the project summary"
        />
      </div>
    );
  }

  if (!summary.data) {
    return (
      <div className={styles.page}>
        <Spinner label="Reading the project…" />
      </div>
    );
  }

  const data = summary.data;
  const totalPoints = data.clouds.reduce((sum, cloud) => sum + cloud.point_count, 0);
  const componentTypes: TypeCount[] = Object.entries(data.components.count_by_type)
    .map(([type, count]) => ({ type, count }))
    .sort((a, b) => b.count - a.count);

  return (
    <div className={styles.page}>
      <section className={styles.stats}>
        <StatCard
          label="Point clouds"
          value={COUNT.format(data.clouds.length)}
          hint={`${COUNT.format(totalPoints)} points total`}
          tone={data.clouds.length === 0 ? 'muted' : 'default'}
        />
        <StatCard
          label="Meshes"
          value={COUNT.format(data.meshes.length)}
          tone={data.meshes.length === 0 ? 'muted' : 'default'}
        />
        <StatCard
          label="Sensor frames"
          value={COUNT.format(data.sensor_frames.total_count)}
          hint={`${COUNT.format(data.sensor_frames.segmented_count)} segmented`}
          tone={data.sensor_frames.total_count === 0 ? 'muted' : 'default'}
        />
        <StatCard
          label="Panoramas"
          value={COUNT.format(data.panoramic_images.total_count)}
          hint={`${COUNT.format(data.panoramic_images.matched_count)} matched to a frame`}
          tone={data.panoramic_images.total_count === 0 ? 'muted' : 'default'}
        />
        <StatCard
          label="Components"
          value={COUNT.format(data.components.total_count)}
          tone={data.components.total_count === 0 ? 'muted' : 'default'}
        />
        <StatCard
          label="Material passports"
          value={COUNT.format(data.materials.length)}
          tone={data.materials.length === 0 ? 'muted' : 'default'}
        />
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Project</h2>
        <p className={styles.subheading}>
          <span className="mono">{data.path}</span> · schema v{data.schema_version}
        </p>
        {data.projects.length === 0 ? (
          <EmptyState
            title="No project metadata record"
            detail="Address, survey date and organisation are set by an import that carries them, or by `rux set`."
          />
        ) : (
          <div className={styles.metaGrid}>
            {data.projects.map((project) => (
              <ProjectCard key={project.id} project={project} />
            ))}
          </div>
        )}
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Clouds</h2>
        <DataTable
          columns={CLOUD_COLUMNS}
          rows={data.clouds}
          rowKey={(cloud) => cloud.name}
          empty={
            <EmptyState
              title="No point clouds"
              detail="`rux create clouds` back-projects the imported depth frames into a fused cloud."
            />
          }
        />
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Meshes</h2>
        <DataTable
          columns={MESH_COLUMNS}
          rows={data.meshes}
          rowKey={(mesh) => mesh.name}
          empty={
            <EmptyState
              title="No meshes"
              detail="`rux create mesh` solves the cell complex into a watertight surface."
            />
          }
        />
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Components by type</h2>
        <DataTable
          columns={TYPE_COLUMNS}
          rows={componentTypes}
          rowKey={(row) => row.type}
          empty={
            <EmptyState
              title="No building components"
              detail="Components appear once the reconstruction stages have classified surfaces."
            />
          }
        />
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Recent stages</h2>
        {log.error ? (
          <ErrorBanner error={log.error} onRetry={log.reload} context="the pipeline log" />
        ) : log.data ? (
          <PipelineLogList entries={log.data} />
        ) : (
          <Spinner label="Reading stage history…" />
        )}
      </section>
    </div>
  );
}

function ProjectCard({ project }: { project: ProjectInfo }) {
  // `0` is the contract's "not set" for year_of_construction, so it is dropped
  // rather than printed — a building constructed in year 0 is not the claim.
  const year =
    project.year_of_construction && project.year_of_construction > 0
      ? String(project.year_of_construction)
      : null;

  const fields: [string, string | null][] = [
    ['Address', present(project.building_address)],
    ['Surveyed', present(project.survey_date)],
    ['Surveyor', present(project.survey_organisation)],
    ['Built', year],
  ];
  const known = fields.filter((field): field is [string, string] => field[1] !== null);

  return (
    <article className={styles.metaCard}>
      <h3 className={styles.metaName}>{project.name}</h3>
      {known.length === 0 ? (
        <p className={styles.metaNone}>No metadata recorded.</p>
      ) : (
        <dl className={styles.metaList}>
          {known.map(([label, value]) => (
            <div key={label} className={styles.metaRow}>
              <dt className={styles.metaLabel}>{label}</dt>
              <dd className={styles.metaValue}>{value}</dd>
            </div>
          ))}
        </dl>
      )}
      {present(project.notes) && <p className={styles.metaNotes}>{project.notes}</p>}
    </article>
  );
}
