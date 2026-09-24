// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';

import { api } from '../api/client';
import type {
  MaterialDetail,
  ProjectInfo,
  ProjectSummary,
  PropertyDefinition,
  ReportPdfVersion,
} from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { WriteBanner } from '../components/WriteBanner';
import { describeWriteFailure } from '../data/writeState';
import type { WriteFailure } from '../data/writeState';
import styles from './ExportPage.module.css';

/**
 * BEK 496 "Ressourcekortlægning" export (#265, #456, #457).
 *
 * BEK 496 (Bekendtgørelse om ressourcekortlægning) is Danish building-waste
 * legislation requiring a resource survey report listing each identified
 * material with quantity, reusability class (A–D), and hazardous status.
 *
 * The authoritative artefact is now the server-side Typst PDF (POST
 * /reports/ressourcekortlaegning). The on-page HTML preview remains for
 * quick reference; the stored versions panel lists every generated PDF with
 * a direct download link.
 */
export function ExportPage() {
  const summary = useAsync<ProjectSummary>((signal) => api.projectSummary(signal), []);

  if (summary.error)
    return (
      <ErrorBanner error={summary.error} onRetry={summary.reload} context="project summary" />
    );
  if (!summary.data) return <Spinner label="Loading project…" />;

  const project = summary.data.projects[0] ?? null;
  const materialInfos = summary.data.materials;

  return (
    <ReportView project={project} guids={materialInfos.map((m) => m.guid)} />
  );
}

// ---------------------------------------------------------------------------
// Internal: report assembled once all passport details and column definitions
// are loaded.
// ---------------------------------------------------------------------------

/**
 * Format a property value for display in the report, according to the column
 * type set on the material page. Keeps formatting consistent with how the
 * material table renders each type (boolean → Ja/Nej, date → Danish locale).
 */
function formatValue(value: string | undefined, type: PropertyDefinition['type']): string {
  if (value === undefined || value === '') return '—';
  switch (type) {
    case 'boolean':
      return value === 'true' || value === '1' || value.toLowerCase() === 'ja' ? 'Ja' : 'Nej';
    case 'date': {
      const d = new Date(value);
      return Number.isNaN(d.getTime())
        ? value
        : d.toLocaleDateString('da-DK', { year: 'numeric', month: 'long', day: 'numeric' });
    }
    default:
      return value;
  }
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
}

function formatTimestamp(iso: string): string {
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return iso;
  return d.toLocaleString('da-DK', {
    year: 'numeric',
    month: 'short',
    day: 'numeric',
    hour: '2-digit',
    minute: '2-digit',
  });
}

function ReportView({
  project,
  guids,
}: {
  project: ProjectInfo | null;
  guids: string[];
}) {
  // User-defined column definitions — the same source as the material table.
  const definitionsAsync = useAsync<PropertyDefinition[]>(
    (signal) => api.propertyDefinitions(signal),
    [],
  );

  const detailsAsync = useAsync<MaterialDetail[]>(
    async (signal) => {
      if (guids.length === 0) return [];
      return Promise.all(guids.map((guid) => api.material(guid, signal)));
    },
    [guids.join(',')],
  );

  // Version history — reload key bumped after each successful generate.
  const [versionsKey, setVersionsKey] = useState(0);
  const versionsAsync = useAsync<ReportPdfVersion[]>(
    (signal) => api.listReportVersions(signal),
    [versionsKey],
  );

  // Generate PDF state.
  const [generating, setGenerating] = useState(false);
  const [generateFailure, setGenerateFailure] = useState<WriteFailure | null>(null);

  const handleGenerate = useCallback(async () => {
    setGenerating(true);
    setGenerateFailure(null);
    try {
      await api.generateReport();
      setVersionsKey((k) => k + 1);
    } catch (err) {
      setGenerateFailure(describeWriteFailure(err instanceof Error ? err : new Error(String(err)), 'the PDF report'));
    } finally {
      setGenerating(false);
    }
  }, []);

  const handleRetryGenerate = useCallback(() => {
    void handleGenerate();
  }, [handleGenerate]);

  const handleDismissFailure = useCallback(() => {
    setGenerateFailure(null);
  }, []);

  if (definitionsAsync.error)
    return (
      <ErrorBanner
        error={definitionsAsync.error}
        onRetry={definitionsAsync.reload}
        context="material columns"
      />
    );
  if (detailsAsync.error)
    return (
      <ErrorBanner
        error={detailsAsync.error}
        onRetry={detailsAsync.reload}
        context="material passports"
      />
    );
  if (!definitionsAsync.data || !detailsAsync.data)
    return <Spinner label="Loading passports…" />;

  // Sort by sort_order — the same order the material table uses.
  const columns = [...definitionsAsync.data].sort((a, b) => a.sort_order - b.sort_order);

  const today = new Date().toLocaleDateString('da-DK', {
    year: 'numeric',
    month: 'long',
    day: 'numeric',
  });

  return (
    <div className={styles.page}>
      {/* Screen-only toolbar */}
      <div className={styles.toolbar}>
        <div className={styles.toolbarLeft}>
          <p className={styles.hint}>
            Click <strong>Generate PDF</strong> to create a server-side PDF using Typst.
            Previous versions remain available for download below.
          </p>
        </div>
        <button
          type="button"
          className={styles.printButton}
          onClick={() => void handleGenerate()}
          disabled={generating}
        >
          {generating ? 'Generating…' : 'Generate PDF'}
        </button>
      </div>

      {/* Generate failure banner */}
      {generateFailure && (
        <WriteBanner
          failure={generateFailure}
          onRetry={generateFailure.retryable ? handleRetryGenerate : undefined}
          onDismiss={handleDismissFailure}
        />
      )}

      {/* Version history */}
      <section className={styles.versionsSection}>
        <h2 className={styles.versionsHeading}>Generated PDFs</h2>
        {versionsAsync.error ? (
          <ErrorBanner
            error={versionsAsync.error}
            onRetry={versionsAsync.reload}
            context="report versions"
          />
        ) : !versionsAsync.data ? (
          <Spinner label="Loading versions…" />
        ) : versionsAsync.data.length === 0 ? (
          <p className={styles.versionsEmpty}>
            No PDFs generated yet. Click <strong>Generate PDF</strong> to create the first one.
          </p>
        ) : (
          <table className={styles.versionsTable}>
            <thead>
              <tr>
                <th className={styles.versionsTh}>Generated</th>
                <th className={styles.versionsTh}>Label</th>
                <th className={styles.versionsTh}>Size</th>
                <th className={styles.versionsTh}></th>
              </tr>
            </thead>
            <tbody>
              {versionsAsync.data.map((v) => (
                <tr key={v.id} className={styles.versionsTr}>
                  <td className={styles.versionsTd}>{formatTimestamp(v.created_at)}</td>
                  <td className={styles.versionsTd}>{v.label}</td>
                  <td className={`${styles.versionsTd} ${styles.versionsSize}`}>
                    {formatBytes(v.size_bytes)}
                  </td>
                  <td className={styles.versionsTd}>
                    <a
                      href={api.reportPdfUrl(v.id)}
                      download={`ressourcekortlaegning-${v.id}.pdf`}
                      className={styles.downloadLink}
                    >
                      Download
                    </a>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </section>

      {/* On-page HTML preview — print CSS shows only this section */}
      <article className={styles.report}>
        {/* Cover page */}
        <header className={styles.cover}>
          <h1 className={styles.coverTitle}>Ressourcekortlægning</h1>
          <p className={styles.coverSubtitle}>BEK nr. 496 af 11/05/2022</p>
          <dl className={styles.coverMeta}>
            {project?.name && (
              <>
                <dt>Projekt</dt>
                <dd>{project.name}</dd>
              </>
            )}
            {project?.building_address && (
              <>
                <dt>Adresse</dt>
                <dd>{project.building_address}</dd>
              </>
            )}
            {project?.year_of_construction ? (
              <>
                <dt>Byggeår</dt>
                <dd>{project.year_of_construction}</dd>
              </>
            ) : null}
            {project?.survey_date && (
              <>
                <dt>Registreringsdato</dt>
                <dd>{project.survey_date}</dd>
              </>
            )}
            {project?.survey_organisation && (
              <>
                <dt>Udarbejdet af</dt>
                <dd>{project.survey_organisation}</dd>
              </>
            )}
            <dt>Eksporteret</dt>
            <dd>{today}</dd>
          </dl>
          {project?.notes && <p className={styles.coverNotes}>{project.notes}</p>}
        </header>

        {/* Material table */}
        <section className={styles.tableSection}>
          {detailsAsync.data.length === 0 ? (
            <p className={styles.empty}>
              Ingen materialeregistreringer fundet. Kør{' '}
              <code>rux import materialepas</code> eller{' '}
              <code>rux create materials</code> for at oprette registreringer.
            </p>
          ) : columns.length === 0 ? (
            <p className={styles.empty}>
              Ingen brugerdefinererede kolonner fundet. Tilføj kolonner via materialesiden for at
              se egenskaber i eksporten.
            </p>
          ) : (
            <table className={styles.table}>
              <thead>
                <tr>
                  {/* Fixed structural columns — not material properties */}
                  <th className={styles.th}>Nr.</th>
                  <th className={styles.th}>Id</th>
                  {/* User-defined material columns, in the same order as the material table */}
                  {columns.map((col) => (
                    <th key={col.id} className={styles.th}>
                      {col.name}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {detailsAsync.data.map((detail, index) => {
                  const props = detail.properties ?? {};
                  return (
                    <tr key={detail.guid} className={styles.tr}>
                      <td className={styles.td}>{index + 1}</td>
                      <td className={`${styles.td} ${styles.tdId}`} title={detail.guid}>
                        {detail.id ?? detail.guid.slice(0, 8)}
                      </td>
                      {columns.map((col) => (
                        <td key={col.id} className={styles.td}>
                          {formatValue(props[col.name], col.type)}
                        </td>
                      ))}
                    </tr>
                  );
                })}
              </tbody>
            </table>
          )}
        </section>
      </article>
    </div>
  );
}
