// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';
import type { KeyboardEvent } from 'react';
import { useLocation } from 'react-router-dom';

import { api } from '../api/client';
import type { Sample, SurveyPart, SurveySummary, SurveyType } from '../api/types';
import { isControl, isField } from '../app/keyTargets';
import { parseTypeQuery } from '../app/links';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { DetailPanel, pendingSampleList } from '../components/kortlaegning/DetailPanel';
import { EditDialog, primaryDisabled } from '../components/kortlaegning/EditDialog';
import { EvidencePanel } from '../components/kortlaegning/EvidencePanel';
import { SurveyTable } from '../components/kortlaegning/SurveyTable';
import { dialogAction, tableAction, type EvidenceTab, type KortAction } from '../kortlaegning/keys';
import {
  NO_FILTERS,
  flattenRows,
  initialViewFor,
  moveSelection,
  nextInQueue,
  partOf,
  replacePart,
  replaceType,
  roomOptions,
  sameSelection,
  tabCounts,
  typeOf,
  visibleTypes,
  type Filters,
  type Selection,
  type Tab,
} from '../kortlaegning/model';
import { formatNumber } from '../kortlaegning/vocab';
import styles from './KortlaegningPage.module.css';

/**
 * The toast after a refused approval: names the samples still awaiting an
 * answer, the same way the gate note does (`pendingSampleList`).
 */
export function blockedMessage(type: SurveyType, samples: Sample[]): string {
  const names = pendingSampleList(type, samples);
  return names
    ? `Kan ikke godkendes — afventer prøvesvar (${names})`
    : 'Kan ikke godkendes — afventer prøvesvar';
}

/** The toast after an approval, counting what is left in the queue. */
export function approvedMessage(name: string, types: SurveyType[]): string {
  return `✓ ${name} godkendt · ${tabCounts(types).queue} tilbage i køen`;
}

/**
 * The coverage notice's clauses after `Dækning:`, or `[]` when there is
 * nothing to warn about. A clause whose figure is absent is left out.
 */
export function coverageParts(summary: SurveySummary): string[] {
  const parts: string[] = [];
  if (summary.unlabeled_points) {
    parts.push(`${formatNumber(summary.unlabeled_points)} punkter uklassificeret`);
  }
  const rooms = summary.rooms_without_parts;
  if (rooms.length > 0) {
    parts.push(`${rooms.length} rum uden registrerede bygningsdele (${rooms.join(', ')})`);
  }
  return parts;
}

/**
 * Kortlægning — the surveyor's review workbench: survey types grouped by
 * review tab, expandable into their building parts, with the evidence and
 * detail panels for the selection and an edit dialog for keyboard review.
 *
 * The page owns every piece of state; the components are presentational.
 * Each PATCH response replaces the local copy of what it changed, and each
 * mutation asks the shell to re-read the survey summary for the sidebar badge.
 */
export function KortlaegningPage() {
  const { data, error, loading, reload } = useAsync(
    (s) => Promise.all([api.survey(s), api.samples(s), api.surveySummary(s)]),
    [],
  );
  const { refresh } = useSurveyCounts();
  const toast = useToast();
  // Every write runs on one serial chain (see useMutationQueue): two PATCHes
  // to one type — a note blur racing an approve — can never land out of order.
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });
  const location = useLocation();
  // `/kortlaegning?type=<id>` (from a sample's "Koblet:" link) selects that
  // type once, in the tab it lives in. Applied in the same effect that seeds
  // `types`, so the first render with data already has the right tab and the
  // keep-selection-visible effect below finds the selection shown.
  const deepLinkType = useRef(parseTypeQuery(location.search));

  const [types, setTypesState] = useState<SurveyType[]>([]);
  // Mirrors `types` synchronously, so a mutation's follow-up (the next queued
  // type, the count in the toast) reads the state its own response produced.
  const typesRef = useRef<SurveyType[]>([]);
  const setTypes = useCallback((update: (prev: SurveyType[]) => SurveyType[]) => {
    typesRef.current = update(typesRef.current);
    setTypesState(typesRef.current);
    return typesRef.current;
  }, []);

  const [tab, setTab] = useState<Tab>('queue');
  const [filters, setFilters] = useState<Filters>(NO_FILTERS);
  const [open, setOpen] = useState<ReadonlySet<number>>(() => new Set());
  const [selection, setSelection] = useState<Selection>(null);
  const [evidenceTab, setEvidenceTab] = useState<EvidenceTab>('plan');
  const [dialogOpen, setDialogOpen] = useState(false);
  const [syncing, setSyncing] = useState(false);
  const [syncError, setSyncError] = useState<Error | null>(null);
  const tableRef = useRef<HTMLDivElement>(null);

  const samples = data?.[1] ?? [];
  const summary = data?.[2];

  const visible = visibleTypes(types, tab, filters);
  // The tab and filters as of now, for a queued mutation's follow-up that
  // runs after later renders (queued via `useMutationQueue`).
  const viewRef = useRef({ tab, filters });
  viewRef.current = { tab, filters };
  const shownIn = (list: SurveyType[]) => visibleTypes(list, viewRef.current.tab, viewRef.current.filters);
  const rows = flattenRows(visible, open);
  const counts = tabCounts(types);
  const rooms = roomOptions(types);
  const selType = typeOf(types, selection);
  const selPart = partOf(types, selection);

  /** Select, and open the selected type so its parts show. */
  const select = useCallback((sel: Selection) => {
    setSelection(sel);
    if (sel) setOpen((o) => (o.has(sel.typeId) ? o : new Set([...o, sel.typeId])));
  }, []);

  // Fresh data (first load, or a reload after sync) replaces the local copy.
  // A `/kortlaegning?type=<id>` deep link selects and opens that type, in the
  // tab it lives in, the first time data arrives (see `initialViewFor`); an
  // unknown or rejected id, or no `?type=` at all, leaves the usual
  // first-load behaviour (the repair effect below picks row 0) untouched.
  const [loadedOnce, setLoadedOnce] = useState(false);
  useEffect(() => {
    if (!data) return;
    setTypes(() => data[0].types);
    const want = deepLinkType.current;
    if (want !== null) {
      deepLinkType.current = null;
      const view = initialViewFor(data[0].types, want);
      if (view) {
        setTab(view.tab);
        setFilters(NO_FILTERS);
        select(view.selection);
      }
    }
    setLoadedOnce(true);
  }, [data, setTypes, select]);

  // Keep the selection on a row that is actually shown: first load, a tab or
  // filter change, or an approval that moved the type out of this tab.
  const rowsKey = rows.map((r) => (r.kind === 'type' ? `t${r.typeId}` : r.partCode)).join(',');
  useEffect(() => {
    if (!loadedOnce) return;
    if (rows.some((r) => sameSelection(r, selection))) return;
    // A part folded away falls back to its own type; anything else to the top.
    const typeShown = selection !== null && rows.some((r) => r.kind === 'type' && r.typeId === selection.typeId);
    if (typeShown) setSelection({ typeId: selection.typeId, partCode: null });
    else select(rows[0] ? { typeId: rows[0].typeId, partCode: null } : null);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [loadedOnce, rowsKey]);

  // Focus the table once it exists, so the arrow keys work without a click:
  // on first load, and when the first sync replaces the empty state.
  const hasTypes = types.length > 0;
  useEffect(() => {
    if (loadedOnce && hasTypes) tableRef.current?.focus({ preventScroll: true });
  }, [loadedOnce, hasTypes]);

  // The dialog does not hand focus back on close; the page does.
  const wasOpen = useRef(false);
  useEffect(() => {
    if (wasOpen.current && !dialogOpen) tableRef.current?.focus({ preventScroll: true });
    wasOpen.current = dialogOpen;
  }, [dialogOpen]);

  // A selection that vanished (last queued type approved) leaves nothing to edit.
  useEffect(() => {
    if (dialogOpen && !selType) setDialogOpen(false);
  }, [dialogOpen, selType]);

  // ------------------------------------------------------------ mutations --

  // Approve and reject are gated on `busy` (a held G/A must not stack
  // requests); field commits deliberately are not.
  function approve() {
    const t = selType;
    if (busy || !t || t.review_status !== 'queue') return;
    void mutate(
      async () => {
        const body = await api.patchSurveyType(t.id, { review_status: 'approved' });
        const next = setTypes((prev) => replaceType(prev, body));
        toast.show(approvedMessage(body.name, next));
        select(nextInQueue(next, t.id, shownIn(next)));
      },
      () => toast.show(blockedMessage(t, samples)),
    );
  }

  function reject() {
    const t = selType;
    if (busy || !t || t.review_status === 'rejected') return;
    void mutate(async () => {
      const body = await api.patchSurveyType(t.id, { review_status: 'rejected' });
      const next = setTypes((prev) => replaceType(prev, body));
      toast.show('Afvist som fejldetektion — fjernet fra listen');
      select(nextInQueue(next, t.id, shownIn(next)));
    });
  }

  function reopen() {
    const t = selType;
    if (!t) return;
    void mutate(async () => {
      const body = await api.patchSurveyType(t.id, { review_status: 'queue' });
      setTypes((prev) => replaceType(prev, body));
    });
  }

  function patchType(t: SurveyType, patch: Parameters<typeof api.patchSurveyType>[1]) {
    void mutate(async () => {
      const body = await api.patchSurveyType(t.id, patch);
      setTypes((prev) => replaceType(prev, body));
    });
  }

  function patchPart(p: SurveyPart, patch: Parameters<typeof api.patchSurveyPart>[1]) {
    void mutate(async () => {
      const body = await api.patchSurveyPart(p.code, patch);
      setTypes((prev) => replacePart(prev, body));
    });
  }

  function star() {
    if (selPart) patchPart(selPart, { starred: !selPart.starred });
    else if (selType) patchType(selType, { starred: !selType.starred });
  }

  function setQuantity(quantity: number) {
    if (selPart) patchPart(selPart, { quantity });
    else if (selType) patchType(selType, { quantity });
  }

  function setNote(note: string) {
    if (selPart) patchPart(selPart, { note });
    else if (selType) patchType(selType, { note });
  }

  async function sync() {
    setSyncing(true);
    setSyncError(null);
    try {
      const report = await api.syncSurvey();
      if (report.parts_orphaned > 0) {
        toast.show(`${report.parts_orphaned} del(e) peger på instanser der ikke findes længere`);
      } else if (report.types_created === 0) {
        toast.show('Ingen nye typer — ingen instanser at kortlægge');
      }
      reload();
      refresh();
    } catch (cause) {
      setSyncError(cause instanceof Error ? cause : new Error(String(cause)));
    } finally {
      setSyncing(false);
    }
  }

  // ------------------------------------------------------------- keyboard --

  /** PgUp/PgDn in the dialog walk the selected type's parts too. */
  function dialogMove(delta: number) {
    const walk = selection ? new Set([...open, selection.typeId]) : open;
    select(moveSelection(flattenRows(visible, walk), selection, delta));
  }

  /** The actions both key maps share. Returns false when nothing happened. */
  function runShared(action: KortAction): boolean {
    switch (action.type) {
      case 'approve':
        approve();
        return true;
      case 'reject':
        reject();
        return true;
      case 'star':
        star();
        return true;
      case 'evidence':
        setEvidenceTab(action.tab);
        return true;
      default:
        return false;
    }
  }

  function onTableKeyDown(e: KeyboardEvent) {
    const action = tableAction({
      key: e.key,
      metaKey: e.metaKey,
      ctrlKey: e.ctrlKey,
      altKey: e.altKey,
      inField: isField(e.target),
      isControl: isControl(e.target),
    });
    if (!action) return;
    e.preventDefault();
    switch (action.type) {
      case 'move':
        setSelection(moveSelection(rows, selection, action.delta));
        return;
      case 'expand':
        if (selection) setOpen((o) => new Set([...o, selection.typeId]));
        return;
      case 'collapse':
        if (!selection) return;
        setOpen((o) => {
          const next = new Set(o);
          next.delete(selection.typeId);
          return next;
        });
        if (selection.partCode !== null) setSelection({ typeId: selection.typeId, partCode: null });
        return;
      case 'open':
        if (selection) setDialogOpen(true);
        return;
      case 'blur':
        (e.target as HTMLElement).blur();
        tableRef.current?.focus();
        return;
      default:
        runShared(action);
    }
  }

  function onDialogKeyDown(e: KeyboardEvent) {
    const action = dialogAction({
      key: e.key,
      metaKey: e.metaKey,
      ctrlKey: e.ctrlKey,
      altKey: e.altKey,
      inField: isField(e.target),
    });
    if (!action) return;
    e.preventDefault();
    switch (action.type) {
      case 'close':
        setDialogOpen(false);
        return;
      case 'approveNext':
        approveNext();
        return;
      case 'move':
        dialogMove(action.delta);
        return;
      default:
        runShared(action);
    }
  }

  /** `Godkend & næste`, or `Godkendt ✓ — næste` for an approved type. */
  function approveNext() {
    if (!selType) return;
    if (primaryDisabled(selType, busy)) {
      if (!busy) toast.show(blockedMessage(selType, samples));
      return;
    }
    if (selType.review_status === 'approved') dialogMove(1);
    else approve();
  }

  // --------------------------------------------------------------- render --

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="Kortlægning" />
      </div>
    );
  }
  // Until the local copy is seeded from `data`, `types` is still [] — showing
  // the empty state for that one frame would flash "Ingen kortlægning".
  if ((loading && !data) || (data && !loadedOnce)) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser kortlægning…" />
      </div>
    );
  }

  const coverage = summary ? coverageParts(summary) : [];

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Kortlægning</h2>
        <span className={styles.sub}>
          Ressourcekortlægning · bygningsdele pr. rum, grupperet pr. type
        </span>
        {types.length > 0 && (
          <a className={styles.btnGhost} href={api.csvExportUrl()} download>
            Eksport (XLS)
          </a>
        )}
      </header>

      {coverage.length > 0 && (
        <p className={styles.notice}>
          <b>Dækning:</b> {coverage.join(' · ')}
        </p>
      )}

      {types.length === 0 ? (
        <>
          {syncError && (
            <p className={styles.notice} role="alert">
              Kunne ikke oprette kortlægning: {syncError.message}
            </p>
          )}
          <EmptyState
            title="Ingen kortlægning endnu"
            detail="Opret typer og bygningsdele ud fra projektets instanser. Kræver at rux create instances er kørt."
            action={
              <button type="button" className={styles.btnPrimary} onClick={sync} disabled={syncing}>
                Opret kortlægning fra instanser
              </button>
            }
          />
        </>
      ) : (
        <div className={styles.bench}>
          <SurveyTable
            types={visible}
            counts={counts}
            tab={tab}
            onTab={setTab}
            filters={filters}
            onFilters={setFilters}
            rooms={rooms}
            open={open}
            selection={selection}
            onSelect={select}
            onToggle={(typeId) =>
              setOpen((o) => {
                const next = new Set(o);
                if (!next.delete(typeId)) next.add(typeId);
                return next;
              })
            }
            onOpenDialog={(sel) => {
              select(sel);
              setDialogOpen(true);
            }}
            onKeyDown={onTableKeyDown}
            tableRef={tableRef}
          />
          <aside className={styles.aside}>
            <EvidencePanel
              type={selType}
              part={selPart}
              tab={evidenceTab}
              onTab={setEvidenceTab}
              variant="panel"
            />
            <DetailPanel
              type={selType}
              part={selPart}
              samples={samples}
              busy={busy}
              onQuantity={setQuantity}
              onTreatment={(treatment) => selType && patchType(selType, { treatment })}
              onNote={setNote}
              onStar={star}
              onApprove={approve}
              onReject={reject}
              onReopen={reopen}
              onDone={() => tableRef.current?.focus({ preventScroll: true })}
            />
          </aside>
        </div>
      )}

      {dialogOpen && selType && (
        <EditDialog
          type={selType}
          part={selPart}
          samples={samples}
          tab={evidenceTab}
          onTab={setEvidenceTab}
          busy={busy}
          onSelectPart={(code) => select({ typeId: selType.id, partCode: code })}
          onPrev={() => dialogMove(-1)}
          onNext={() => dialogMove(1)}
          onClose={() => setDialogOpen(false)}
          onApproveNext={approveNext}
          onReject={reject}
          onQuantity={setQuantity}
          onTreatment={(treatment) => patchType(selType, { treatment })}
          onNote={setNote}
          onStar={star}
          onKeyDown={onDialogKeyDown}
        />
      )}

      <Toast message={toast.message} />
    </div>
  );
}
