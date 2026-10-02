// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { api } from '../api/client';
import type { ReportPdfVersion, Template, TemplateCsv } from '../api/types';
import { explainLoadError } from '../app/errorCopy';
import { createOnceGuard } from '../app/onceGuard';
import { useAsync } from '../app/useAsync';
import { appWriteChain } from '../app/writeChain';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { CircularityBar } from '../components/CircularityBar';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { DataExportPanel } from '../components/rapport/DataExportPanel';
import { TemplateSelect } from '../components/rapport/TemplateSelect';
import { VersionList } from '../components/rapport/VersionList';
import { caseName, circularitySegments } from '../overblik/model';
import { type CsvOptions, writeCsvOptions } from '../rapport/csvOptions';
import {
  csvWriteOutcome,
  defaultExportTemplateId,
  draftNotice,
  generatedToast,
  generateErrorMessage,
  HERO_SCOPE,
  LIST_REFRESH_FAILED,
  REPORT_FOOTNOTE,
  reportHeroSub,
  ressourcetabelHint,
  validChoice,
} from '../rapport/model';
import { createLatestGate, replaceTemplate, templateErrorMessage } from '../skabeloner/model';
import styles from './RapportPage.module.css';

/**
 * Rapport — the Ressourcekortlægning report versions (prototype 02d).
 *
 * The hero sums the whole case up — every non-rejected type, captioned so it
 * is not read as the PDF's approved-only figures — and the list holds every stored PDF, newest first,
 * marked complete or draft by how many types still blocked it when it was
 * generated. `Generér ny version` runs on the page's mutation queue. The same
 * queued task re-reads the list, so the new row and its number come from the
 * server. A generation that succeeded is confirmed before that re-read, and a
 * failed re-read is reported on its own — it never reads as a failed
 * generation (F22).
 *
 * Rapport also holds the template choices (spec §6.3): the head's
 * `Ressourcetabel` select adds a table built from a template to the next
 * generated version (starts at Ingen, R10), and the Data-eksport panel
 * downloads the resources CSV for a template. Its delimiter / encoding /
 * header are saved back to that template on the app write chain, and the
 * download waits while such a write is pending.
 */
export function RapportPage() {
  const { data, error, loading, reload } = useAsync(
    (s) =>
      appWriteChain
        .idle()
        .then(() => Promise.all([api.projectSummary(s), api.surveySummary(s), api.surveyFractions(s)])),
    [],
  );
  const listed = useAsync((s) => appWriteChain.idle().then(() => api.listReportVersions(s)), []);
  const [versions, setVersions] = useState<ReportPdfVersion[] | null>(null);
  const [stale, setStale] = useState(false);
  useEffect(() => {
    if (listed.data) {
      setVersions(listed.data);
      setStale(false);
    }
  }, [listed.data]);

  const tpl = useAsync((s) => appWriteChain.idle().then(() => api.templates(s)), []);
  const [templates, setTemplates] = useState<Template[]>([]);
  const [pdfTemplate, setPdfTemplate] = useState<number | null>(null);
  const [exportTemplate, setExportTemplate] = useState<number | null>(null);
  useEffect(() => {
    const list = tpl.data;
    if (!list) return;
    setTemplates(list);
    setExportTemplate((id) => validChoice(list, id) ?? defaultExportTemplateId(list));
    setPdfTemplate((id) => validChoice(list, id));
  }, [tpl.data]);

  const toast = useToast(3200);
  const { busy, mutate } = useMutationQueue({
    scope: 'page',
    onError: (cause) => toast.show(generateErrorMessage(cause)),
  });
  // Template writes (CSV options) join the app chain: they change project
  // state the other screens read.
  const csvQueue = useMutationQueue({ onError: (cause) => toast.show(templateErrorMessage(cause)) });
  // Only the newest CSV write per template may overwrite the optimistic copy,
  // so an earlier response cannot flash an older option back.
  const csvGate = useRef(createLatestGate()).current;
  const onCsvChange = (patch: Partial<CsvOptions>) => {
    const t = templates.find((x) => x.id === exportTemplate);
    if (!t) return;
    const csv = writeCsvOptions(t.csv, patch) as TemplateCsv;
    const ticket = csvGate.next(t.id);
    setTemplates((prev) => replaceTemplate(prev, { ...t, csv }));
    void csvQueue.mutate(async () => {
      try {
        const saved = await api.patchTemplate(t.id, { csv });
        if (csvWriteOutcome(csvGate.isLatest(t.id, ticket), true) === 'apply') {
          setTemplates((prev) => replaceTemplate(prev, saved));
        }
      } catch (cause) {
        // Only the newest write reconciles: re-read the truth, and if that
        // fails too, undo the optimistic edit. An older failure leaves a newer
        // queued edit's preview alone; that edit settles the state.
        if (csvWriteOutcome(csvGate.isLatest(t.id, ticket), false) === 'reconcile') {
          const fresh = await api.templates().catch(() => null);
          setTemplates((prev) => fresh ?? replaceTemplate(prev, t));
        }
        throw cause;
      }
    });
  };

  // `busy` only disables the button after React re-renders, so a double click
  // inside one frame would queue two generations. The guard closes that gap.
  const [generating] = useState(createOnceGuard);
  // Until the first list read lands, a generate could be overwritten by that
  // late read; the button waits for it.
  const listReady = versions !== null;
  const generate = () => {
    if (!listReady) return;
    generating.run(() =>
      mutate(async () => {
        const created = await api.generateReport(validChoice(templates, pdfTemplate));
        setVersions((prev) => [created, ...(prev ?? []).filter((v) => v.id !== created.id)]);
        toast.show(generatedToast(created));
        try {
          setVersions(await api.listReportVersions());
          setStale(false);
        } catch {
          setStale(true);
        }
      }),
    );
  };

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="rapportgrundlaget" />
      </div>
    );
  }
  if (loading && !data) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser rapporten…" />
      </div>
    );
  }
  if (!data) return null;

  const [summary, survey, fractions] = data;
  const notice = draftNotice(fractions);
  const segments = circularitySegments(survey.circularity);

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Rapport</h2>
        <span className={styles.sub}>Ressourcekortlægningsrapport</span>
        <div className={styles.actions}>
          <div className={styles.ressourcetabel}>
            <TemplateSelect
              id="rapport-ressourcetabel"
              label="Ressourcetabel"
              templates={templates}
              value={validChoice(templates, pdfTemplate)}
              onChange={setPdfTemplate}
              allowNone
              disabled={busy}
            />
          </div>
          <button type="button" className={styles.btnPrimary} onClick={generate} disabled={busy || !listReady}>
            {busy ? 'Genererer…' : 'Generér ny version'}
          </button>
        </div>
      </header>
      <p className={styles.hint}>
        {ressourcetabelHint(templates.find((t) => t.id === validChoice(templates, pdfTemplate)) ?? null)}
      </p>

      {notice && <p className={styles.notice}>{notice}</p>}

      <section className={styles.hero} aria-labelledby="rapport-hero">
        <h3 id="rapport-hero" className={styles.heroTitle}>
          Ressourcekortlægning — {caseName(summary)}
        </h3>
        <p className={styles.heroSub}>{reportHeroSub(survey)}</p>
        {segments.length > 0 && <CircularityBar segments={segments} legend={false} />}
        <p className={styles.heroScope}>{HERO_SCOPE}</p>
      </section>

      {stale && (
        <p className={styles.notice} role="status">
          {LIST_REFRESH_FAILED}{' '}
          <button type="button" className={styles.textBtn} onClick={listed.reload} disabled={listed.loading}>
            {listed.loading ? 'Henter listen…' : 'Hent listen igen'}
          </button>
          {listed.error && !listed.loading && (
            <span className={styles.noticeDetail}>{explainLoadError(listed.error, 'rapportversionerne').message}</span>
          )}
        </p>
      )}

      {versions === null && listed.error ? (
        <ErrorBanner error={listed.error} onRetry={listed.reload} context="rapportversionerne" />
      ) : versions === null ? (
        <Spinner label="Indlæser versioner…" />
      ) : (
        <>
          {versions.length === 0 && (
            <EmptyState
              title="Ingen versioner endnu"
              detail="Generér den første version — den gemmes i projektet og kan hentes her igen."
            />
          )}
          <VersionList versions={versions} pdfUrl={(id) => api.reportPdfUrl(id)} inventoryUrl={api.csvExportUrl()} />
        </>
      )}

      {tpl.error && templates.length === 0 ? (
        <ErrorBanner error={tpl.error} onRetry={tpl.reload} context="skabelonerne" />
      ) : !tpl.data && templates.length === 0 ? (
        <Spinner label="Indlæser skabeloner…" />
      ) : (
        <DataExportPanel
          templates={templates}
          selectedId={validChoice(templates, exportTemplate)}
          onSelect={setExportTemplate}
          onCsvChange={onCsvChange}
          writing={csvQueue.busy}
          csvUrl={(id) => api.resourcesExportCsvUrl(id)}
          onDownloadError={toast.show}
        />
      )}

      <p className={styles.footnote}>{REPORT_FOOTNOTE}</p>
      <Toast message={toast.message} />
    </div>
  );
}
