// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useMemo, useState } from 'react';

import { api } from '../api/client';
import type {
  ExportTemplate,
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
import {
  configToSelection,
  selectionToConfig,
  selectionToQueryParam,
  STRUCTURAL_COLUMNS,
} from '../data/csvExport';
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

  const definitions = definitionsAsync.data;

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

      {/* CSV Export section */}
      <CsvExportSection definitions={definitions} />

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

// ---------------------------------------------------------------------------
// CsvExportSection — column selection, named templates, and download.
// ---------------------------------------------------------------------------

const GROUP_LABELS: Record<string, string> = {
  common: 'Common',
  component: 'Component fields',
  passport: 'Passport metadata',
  property: 'Material properties',
};

function CsvExportSection({ definitions }: { definitions: PropertyDefinition[] }) {
  // All available columns: structural (from server source) + user-defined.
  const allColumns = useMemo(() => {
    const propertyColumns = definitions.map((def) => ({
      key: def.name,
      label: def.name,
      group: 'property' as const,
    }));
    return [...STRUCTURAL_COLUMNS, ...propertyColumns];
  }, [definitions]);

  const allKeys = useMemo(() => allColumns.map((c) => c.key), [allColumns]);

  // Column selection state — defaults to all columns selected.
  const [selected, setSelected] = useState<Set<string>>(() => new Set(allKeys));

  // Active template id — tracks which saved template matches the current selection.
  const [activeTemplateId, setActiveTemplateId] = useState<number | null>(null);

  // Template list — refetched when a template is created/updated/deleted.
  const [templatesKey, setTemplatesKey] = useState(0);
  const templatesAsync = useAsync<ExportTemplate[]>(
    (signal) => api.listExportTemplates(signal),
    [templatesKey],
  );

  // "Save as new" input state.
  const [saveName, setSaveName] = useState('');
  const [saving, setSaving] = useState(false);
  const [saveError, setSaveError] = useState<string | null>(null);

  // Per-template rename state: maps template id to the in-progress name string.
  const [renaming, setRenaming] = useState<Record<number, string>>({});

  const toggle = useCallback(
    (key: string) => {
      setSelected((prev) => {
        const next = new Set(prev);
        if (next.has(key)) next.delete(key);
        else next.add(key);
        return next;
      });
      setActiveTemplateId(null);
    },
    [],
  );

  const selectAll = useCallback(() => {
    setSelected(new Set(allKeys));
    setActiveTemplateId(null);
  }, [allKeys]);

  const selectNone = useCallback(() => {
    setSelected(new Set());
    setActiveTemplateId(null);
  }, []);

  const applyTemplate = useCallback(
    (template: ExportTemplate) => {
      setSelected(configToSelection(template.config, allKeys));
      setActiveTemplateId(template.id);
    },
    [allKeys],
  );

  const saveAsNew = useCallback(async () => {
    const name = saveName.trim();
    if (!name) return;
    setSaving(true);
    setSaveError(null);
    try {
      const config = selectionToConfig(selected, allKeys);
      const created = await api.createExportTemplate(name, config);
      setActiveTemplateId(created.id);
      setSaveName('');
      setTemplatesKey((k) => k + 1);
    } catch (err) {
      setSaveError(err instanceof Error ? err.message : String(err));
    } finally {
      setSaving(false);
    }
  }, [saveName, selected, allKeys]);

  const updateActiveTemplate = useCallback(async () => {
    if (activeTemplateId === null) return;
    setSaving(true);
    setSaveError(null);
    try {
      const config = selectionToConfig(selected, allKeys);
      await api.updateExportTemplate(activeTemplateId, { config });
      setTemplatesKey((k) => k + 1);
    } catch (err) {
      setSaveError(err instanceof Error ? err.message : String(err));
    } finally {
      setSaving(false);
    }
  }, [activeTemplateId, selected, allKeys]);

  const commitRename = useCallback(
    async (id: number) => {
      const name = renaming[id]?.trim();
      if (!name) {
        setRenaming((prev) => {
          const next = { ...prev };
          delete next[id];
          return next;
        });
        return;
      }
      try {
        await api.updateExportTemplate(id, { name });
        setTemplatesKey((k) => k + 1);
      } catch (err) {
        setSaveError(err instanceof Error ? err.message : String(err));
      } finally {
        setRenaming((prev) => {
          const next = { ...prev };
          delete next[id];
          return next;
        });
      }
    },
    [renaming],
  );

  const deleteTemplate = useCallback(
    async (id: number) => {
      setSaveError(null);
      try {
        await api.deleteExportTemplate(id);
        if (activeTemplateId === id) setActiveTemplateId(null);
        setTemplatesKey((k) => k + 1);
      } catch (err) {
        setSaveError(err instanceof Error ? err.message : String(err));
      }
    },
    [activeTemplateId],
  );

  const columnsParam = selectionToQueryParam(selected, allKeys);
  const downloadUrl = api.csvExportUrl(columnsParam ? columnsParam.split(',') : undefined);

  const templates = templatesAsync.data ?? [];

  // Group structural columns by their group field.
  const structuralGroups = useMemo(() => {
    const map = new Map<string, typeof STRUCTURAL_COLUMNS>();
    for (const col of STRUCTURAL_COLUMNS) {
      const arr = map.get(col.group) ?? [];
      arr.push(col);
      map.set(col.group, arr);
    }
    return map;
  }, []);

  return (
    <section className={styles.csvSection}>
      <h2 className={styles.versionsHeading}>CSV Export</h2>

      {/* Saved templates */}
      {templates.length > 0 && (
        <div className={styles.csvTemplates}>
          <span className={styles.csvTemplateLabel}>Saved templates</span>
          <div className={styles.csvTemplateList}>
            {templates.map((t) => {
              const isActive = activeTemplateId === t.id;
              const isRenaming = t.id in renaming;
              return (
                <div
                  key={t.id}
                  className={`${styles.csvTemplateItem} ${isActive ? styles.csvTemplateItemActive : ''}`}
                >
                  {isRenaming ? (
                    <input
                      type="text"
                      className={styles.csvTemplateRenameInput}
                      value={renaming[t.id]}
                      onChange={(e) =>
                        setRenaming((prev) => ({ ...prev, [t.id]: e.target.value }))
                      }
                      onBlur={() => void commitRename(t.id)}
                      onKeyDown={(e) => {
                        if (e.key === 'Enter') void commitRename(t.id);
                        if (e.key === 'Escape')
                          setRenaming((prev) => {
                            const next = { ...prev };
                            delete next[t.id];
                            return next;
                          });
                      }}
                      autoFocus
                    />
                  ) : (
                    <button
                      type="button"
                      className={styles.csvTemplateApply}
                      onClick={() => applyTemplate(t)}
                      title="Apply this template"
                    >
                      {t.name}
                    </button>
                  )}
                  <button
                    type="button"
                    className={styles.csvTemplateDelete}
                    title="Rename"
                    onClick={() =>
                      setRenaming((prev) => ({ ...prev, [t.id]: t.name }))
                    }
                  >
                    ✎
                  </button>
                  <button
                    type="button"
                    className={styles.csvTemplateDelete}
                    title={`Delete "${t.name}"`}
                    onClick={() => void deleteTemplate(t.id)}
                  >
                    ×
                  </button>
                </div>
              );
            })}
          </div>
        </div>
      )}

      {/* Column selection */}
      <div className={styles.csvColumns}>
        <div className={styles.csvColumnsHeader}>
          <span className={styles.csvColumnsTitle}>
            Columns ({selected.size} / {allKeys.length})
          </span>
          <div className={styles.csvColumnsActions}>
            <button type="button" className={styles.csvActionButton} onClick={selectAll}>
              All
            </button>
            <button type="button" className={styles.csvActionButton} onClick={selectNone}>
              None
            </button>
          </div>
        </div>
        <div className={styles.csvColumnGroups}>
          {/* Structural groups */}
          {(['common', 'component', 'passport'] as const).map((group) => {
            const cols = structuralGroups.get(group);
            if (!cols?.length) return null;
            return (
              <div key={group} className={styles.csvColumnGroup}>
                <span className={styles.csvColumnGroupLabel}>{GROUP_LABELS[group]}</span>
                <div className={styles.csvColumnCheckboxes}>
                  {cols.map((col) => (
                    <label key={col.key} className={styles.csvColumnLabel}>
                      <input
                        type="checkbox"
                        checked={selected.has(col.key)}
                        onChange={() => toggle(col.key)}
                      />
                      <span>{col.label}</span>
                    </label>
                  ))}
                </div>
              </div>
            );
          })}
          {/* User-defined material property columns */}
          {definitions.length > 0 && (
            <div className={styles.csvColumnGroup}>
              <span className={styles.csvColumnGroupLabel}>
                {GROUP_LABELS['property']}
              </span>
              <div className={styles.csvColumnCheckboxes}>
                {definitions.map((def) => (
                  <label key={def.id} className={styles.csvColumnLabel}>
                    <input
                      type="checkbox"
                      checked={selected.has(def.name)}
                      onChange={() => toggle(def.name)}
                    />
                    <span>{def.name}</span>
                  </label>
                ))}
              </div>
            </div>
          )}
        </div>
      </div>

      {/* Template save / update row */}
      <div className={styles.csvSaveRow}>
        {activeTemplateId !== null && (
          <button
            type="button"
            className={styles.csvActionButtonPrimary}
            onClick={() => void updateActiveTemplate()}
            disabled={saving}
          >
            {saving ? 'Saving…' : 'Update template'}
          </button>
        )}
        <input
          type="text"
          className={styles.csvNameInput}
          placeholder="New template name…"
          value={saveName}
          onChange={(e) => setSaveName(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter') void saveAsNew();
          }}
        />
        <button
          type="button"
          className={styles.csvActionButtonPrimary}
          onClick={() => void saveAsNew()}
          disabled={saving || !saveName.trim()}
        >
          {saving ? 'Saving…' : 'Save as template'}
        </button>
      </div>

      {saveError && <p className={styles.csvError}>{saveError}</p>}

      {/* Download */}
      <div className={styles.csvDownloadRow}>
        <a
          href={downloadUrl}
          download="elements.csv"
          className={`${styles.printButton} ${selected.size === 0 ? styles.printButtonDisabled : ''}`}
          onClick={selected.size === 0 ? (e) => e.preventDefault() : undefined}
        >
          Download CSV
        </a>
        {selected.size === 0 && (
          <span className={styles.hint}>Select at least one column to download.</span>
        )}
      </div>
    </section>
  );
}
