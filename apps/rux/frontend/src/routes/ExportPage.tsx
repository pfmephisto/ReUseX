// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';

import { api } from '../api/client';
import type { MaterialDetail, ProjectInfo, ProjectSummary } from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import styles from './ExportPage.module.css';

/**
 * BEK 496 "Ressourcekortlægning" export (#265, review pt 9).
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
 * BEK 496 field mapping:
 *   - `material_type`       → Materialetype
 *   - `quantity_estimate`   → Mængde (estimate)
 *   - `unit`                → Enhed
 *   - `location`            → Beliggenhed
 *   - `reusability_class`   → Genanvendelsespotentiale (A/B/C/D)
 *   - `hazardous`           → Farlige stoffer (ja/nej)
 *   - `condition`           → Tilstand
 *   - `notes` / `comment`   → Kommentarer
 *
 * Keys that are absent from a passport are shown as "—". Passports imported
 * via `rux import materialepas` may already carry these fields under their
 * MaterialEPAS names; `rux create materials` derives passports from instances
 * and the field names depend on the annotation pipeline used.
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
// Internal: report assembled once all passport details are loaded
// ---------------------------------------------------------------------------

/** BEK 496 property keys to look for in each passport, in column order. */
const BEK_FIELDS: { key: string; label: string }[] = [
  { key: 'material_type', label: 'Materialetype' },
  { key: 'quantity_estimate', label: 'Mængde' },
  { key: 'unit', label: 'Enhed' },
  { key: 'location', label: 'Beliggenhed' },
  { key: 'reusability_class', label: 'Genanvendelsespotentiale (A–D)' },
  { key: 'hazardous', label: 'Farlige stoffer' },
  { key: 'condition', label: 'Tilstand' },
  { key: 'notes', label: 'Kommentarer' },
];

/** Aliases: MaterialEPAS may store the same fact under a slightly different key. */
const ALIASES: Record<string, string[]> = {
  material_type: ['materialtype', 'type', 'material'],
  quantity_estimate: ['quantity', 'maengde', 'mængde', 'amount'],
  unit: ['enhed'],
  location: ['beliggenhed', 'location_reference', 'placering'],
  reusability_class: ['genanvendelsespotentiale', 'reusability', 'reuse_class', 'class'],
  hazardous: ['farlige_stoffer', 'farlige stoffer', 'hazardous_materials'],
  condition: ['tilstand', 'state'],
  notes: ['comment', 'comments', 'kommentarer', 'note', 'bemærkninger'],
};

function lookupField(props: Record<string, string>, key: string): string {
  const direct = props[key];
  if (direct !== undefined) return direct;
  for (const alias of ALIASES[key] ?? []) {
    const found = props[alias];
    if (found !== undefined) return found;
  }
  return '—';
}

function ReportView({
  project,
  guids,
}: {
  project: ProjectInfo | null;
  guids: string[];
}) {
  const details = useAsync<MaterialDetail[]>(
    async (signal) => {
      if (guids.length === 0) return [];
      return Promise.all(guids.map((guid) => api.material(guid, signal)));
    },
    [guids.join(',')],
  );

  const handlePrint = useCallback(() => {
    window.print();
  }, []);

  if (details.error)
    return (
      <ErrorBanner
        error={details.error}
        onRetry={details.reload}
        context="material passports"
      />
    );
  if (!details.data) return <Spinner label="Loading passports…" />;

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
          {details.data.length === 0 ? (
            <p className={styles.empty}>
              Ingen materialeregistreringer fundet. Kør{' '}
              <code>rux import materialepas</code> eller{' '}
              <code>rux create materials</code> for at oprette registreringer.
            </p>
          ) : (
            <table className={styles.table}>
              <thead>
                <tr>
                  <th className={styles.th}>Nr.</th>
                  <th className={styles.th}>Id</th>
                  {BEK_FIELDS.map((f) => (
                    <th key={f.key} className={styles.th}>
                      {f.label}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {details.data.map((detail, index) => {
                  const props = detail.properties ?? {};
                  return (
                    <tr key={detail.guid} className={styles.tr}>
                      <td className={styles.td}>{index + 1}</td>
                      <td className={`${styles.td} ${styles.tdId}`} title={detail.guid}>
                        {detail.id ?? detail.guid.slice(0, 8)}
                      </td>
                      {BEK_FIELDS.map((f) => (
                        <td key={f.key} className={styles.td}>
                          {lookupField(props, f.key)}
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
