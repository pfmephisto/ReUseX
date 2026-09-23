// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';

import { api } from '../api/client';
import type {
  MaterialDetail,
  ProjectInfo,
  ProjectSummary,
  PropertyDefinition,
} from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import styles from './ExportPage.module.css';

/**
 * BEK 496 "Ressourcekortlægning" export (#265, review pt 9, #458).
 *
 * BEK 496 (Bekendtgørelse om ressourcekortlægning) is Danish building-waste
 * legislation requiring a resource survey report listing each identified
 * material with quantity, reusability class (A–D), and hazardous status.
 *
 * Implementation note: the report is rendered as a print-optimised HTML page.
 * The user clicks "Print / Save as PDF" and the browser's native print dialog
 * produces a PDF — no new npm dependency needed, and the output renders
 * correctly in every major PDF viewer. The `@media print` rules in the module
 * CSS hide the page chrome (nav, button) and lay the report out for A4 paper.
 *
 * Column mapping (#458): property columns are driven by the same
 * `PropertyDefinition[]` that power the material table (`GET /material-columns`,
 * sorted by `sort_order`). A value edited on the material page is stored under
 * `properties[col.name]` and appears here immediately. Fixed report columns
 * (Nr., Id) are structural identifiers and not material properties; they are
 * always present regardless of which user columns exist.
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

  const handlePrint = useCallback(() => {
    window.print();
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
        <p className={styles.hint}>
          Click <strong>Print / Save as PDF</strong> to export the report. In the print dialog,
          select "Save as PDF" as the destination, A4 paper, and portrait orientation.
        </p>
        <button type="button" className={styles.printButton} onClick={handlePrint}>
          Print / Save as PDF
        </button>
      </div>

      {/* The report itself — the print CSS shows only this */}
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
