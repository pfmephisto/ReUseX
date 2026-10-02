// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type { KeyboardEvent } from 'react';
import { useLocation } from 'react-router-dom';

import { api } from '../api/client';
import type {
  Resource,
  ResourceCreate,
  ResourceKey,
  Sample,
  SurveyPart,
  SurveySummary,
  SurveySyncReport,
  SurveyType,
  Template,
} from '../api/types';
import { isControl, isField } from '../app/keyTargets';
import { parseTypeQuery } from '../app/links';
import { createOnceGuard } from '../app/onceGuard';
import { errorMessage, saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { appWriteChain } from '../app/writeChain';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useMutationQueue } from '../app/useMutationQueue';
import { useToast } from '../app/useToast';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { AddColumnDialog } from '../components/kortlaegning/AddColumnDialog';
import { AddResourceDialog } from '../components/kortlaegning/AddResourceDialog';
import { DetailPanel, pendingSampleList } from '../components/kortlaegning/DetailPanel';
import { EditDialog, primaryDisabled } from '../components/kortlaegning/EditDialog';
import { EvidencePanel } from '../components/kortlaegning/EvidencePanel';
import { SurveyTable } from '../components/kortlaegning/SurveyTable';
import {
  columnCreateBody,
  columnCreateConflict,
  columnPartialFailureMessage,
  duplicateFirst,
  type ColumnDraft,
} from '../kortlaegning/columnDraft';
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
import {
  isManual,
  patchedResources,
  replaceResources,
  resourceColumnKeyId,
  resourceIndex,
  templateColumns,
  touchesSurvey,
  viewForNewResource,
} from '../kortlaegning/resources';
import {
  appendKeyMember,
  pickTemplate,
  projectIdentity,
  readStoredTemplateId,
  writeStoredTemplateId,
  type ProjectIdentity,
} from '../kortlaegning/templatePick';
import { cellErrorMessage, invalidValueMessage, refreshAfterSave } from '../kortlaegning/writeOutcome';
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

/** How many room names the coverage notice spells out before "… og N flere". */
export const COVERAGE_ROOMS_SHOWN = 5;

/** `names` joined, capped at `shown`: "A, B, C … og 451 flere". */
export function cappedList(names: string[], shown = COVERAGE_ROOMS_SHOWN): string {
  if (names.length <= shown) return names.join(', ');
  return `${names.slice(0, shown).join(', ')} … og ${formatNumber(names.length - shown)} flere`;
}

/**
 * The coverage notice's clauses after `Dækning:`, or `[]` when there is
 * nothing to warn about. A clause whose figure is absent is left out. The
 * rooms clause waits until the survey has types: before the first sync every
 * room lacks parts, which says nothing.
 */
export function coverageParts(summary: SurveySummary): string[] {
  const parts: string[] = [];
  if (summary.unlabeled_points) {
    parts.push(`${formatNumber(summary.unlabeled_points)} punkter uklassificeret`);
  }
  const rooms = summary.rooms_without_parts;
  if (rooms.length > 0 && summary.counts.all > 0) {
    parts.push(
      `${formatNumber(rooms.length)} rum uden registrerede bygningsdele (${cappedList(rooms)})`,
    );
  }
  return parts;
}

/** The toast after "Opret kortlægning fra instanser": what sync did, and why not more. */
export function syncMessage(report: SurveySyncReport): string {
  if (report.parts_orphaned > 0) {
    return `${report.parts_orphaned} del(e) peger på instanser der ikke findes længere`;
  }
  const parts = report.parts_created;
  if (parts > 0) {
    const noun = parts === 1 ? 'bygningsdel' : 'bygningsdele';
    const types = report.types_created;
    if (types === 0) return `${formatNumber(parts)} ${noun} tilføjet til eksisterende typer`;
    return `${formatNumber(parts)} ${noun} oprettet i ${formatNumber(types)} ${types === 1 ? 'ny type' : 'nye typer'}`;
  }
  if (report.instances_seen === 0) {
    return 'Ingen instanser at kortlægge — opret instanser først';
  }
  return `Ingen nye bygningsdele — alle ${formatNumber(report.instances_seen)} instanser er allerede kortlagt`;
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
    (s) =>
      appWriteChain
        .idle()
        .then(() =>
          Promise.all([
            api.survey(s),
            api.samples(s),
            api.surveySummary(s),
            api.resourceKeys(s),
            api.templates(s),
            api.resources(undefined, s),
            api.health(s),
            api.projects(s),
          ]),
        ),
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

  // The resource catalogue, the templates and every resource's values (R1):
  // a template switch only rebuilds columns, it never re-fetches.
  const [keys, setKeys] = useState<ResourceKey[]>([]);
  const [templates, setTemplates] = useState<Template[]>([]);
  const [templateId, setTemplateId] = useState<number | null>(null);
  const [resources, setResourcesState] = useState<Resource[]>([]);
  // Mirrors `resources` synchronously, like `typesRef`, so queued commits
  // fold their responses into the state the previous one produced.
  const resourcesRef = useRef<Resource[]>([]);
  const setResources = useCallback((update: (prev: Resource[]) => Resource[]) => {
    resourcesRef.current = update(resourcesRef.current);
    setResourcesState(resourcesRef.current);
  }, []);
  // The project's identity keys the remembered template choice (templatePick).
  const projectRef = useRef<ProjectIdentity>({ id: null, name: '' });
  // A write succeeded but the re-read after it failed: the view may be stale.
  const [refreshNotice, setRefreshNotice] = useState<string | null>(null);

  const [tab, setTab] = useState<Tab>('queue');
  const [filters, setFilters] = useState<Filters>(NO_FILTERS);
  const [open, setOpen] = useState<ReadonlySet<number>>(() => new Set());
  const [selection, setSelection] = useState<Selection>(null);
  const [evidenceTab, setEvidenceTab] = useState<EvidenceTab>('plan');
  const [dialogOpen, setDialogOpen] = useState(false);
  const [addResourceOpen, setAddResourceOpen] = useState(false);
  // A create burns a server-assigned code: a double tap must send once (Review Focus).
  const createGuard = useRef(createOnceGuard());
  const [addColumnOpen, setAddColumnOpen] = useState(false);
  // The server's 409 for the last "Tilføj kolonne" (R3-D3), shown in the dialog.
  const [columnError, setColumnError] = useState<string | null>(null);
  const columnGuard = useRef(createOnceGuard());
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
  const template = templates.find((t) => t.id === templateId) ?? null;
  const columns = useMemo(() => templateColumns(template, keys), [template, keys]);
  const index = useMemo(() => resourceIndex(resources), [resources]);

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
    setKeys(data[3]);
    setTemplates(data[4]);
    setResources(() => data[5]);
    projectRef.current = projectIdentity(data[7], data[6].project.name);
    setRefreshNotice(null);
    setTemplateId((current) => pickTemplate(data[4], current ?? readStoredTemplateId(projectRef.current))?.id ?? null);
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
  }, [data, setTypes, setResources, select]);

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

  // A dialog does not hand focus back on close; the page does.
  const anyDialog = dialogOpen || addResourceOpen || addColumnOpen;
  const wasOpen = useRef(false);
  useEffect(() => {
    if (wasOpen.current && !anyDialog) tableRef.current?.focus({ preventScroll: true });
    wasOpen.current = anyDialog;
  }, [anyDialog]);

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

  // A survey PATCH changes `sys:` values, so both re-read resources (R5).
  // Approve, reject and reopen do not: review status is not a key.
  function patchType(t: SurveyType, patch: Parameters<typeof api.patchSurveyType>[1]) {
    void mutate(async () => {
      const body = await api.patchSurveyType(t.id, patch);
      setTypes((prev) => replaceType(prev, body));
      await reread(refreshResources);
    });
  }

  function patchPart(p: SurveyPart, patch: Parameters<typeof api.patchSurveyPart>[1]) {
    void mutate(async () => {
      const body = await api.patchSurveyPart(p.code, patch);
      setTypes((prev) => replacePart(prev, body));
      await reread(refreshResources);
    });
  }

  /**
   * The re-read after a write that already succeeded. A failure never undoes
   * the success (no "Kunne ikke gemme" for a value that was saved); it shows
   * the stale-view notice instead, which a later successful re-read clears.
   */
  async function reread(refresh: () => Promise<unknown>) {
    if (await refreshAfterSave(refresh, setRefreshNotice)) setRefreshNotice(null);
  }

  function chooseTemplate(id: number) {
    setTemplateId(id);
    writeStoredTemplateId(projectRef.current, id);
  }

  /** Re-read every resource (after a survey PATCH changed sys: values — plan R5). */
  async function refreshResources() {
    const list = await api.resources();
    setResources(() => list);
  }

  /** Re-read the survey (after a resource write changed type state — plan R5). */
  async function refreshSurvey() {
    const s = await api.survey();
    setTypes(() => s.types);
  }

  /**
   * An inline cell's commit: never gated on `busy` (field commits never are).
   * A refused value names the key by its label, never the server's text with
   * the raw key id. Returns the queued write, so a select or checkbox can
   * show its choice until the write settles.
   */
  function commitCell(code: string, keyId: string, value: string | null): Promise<void> {
    const label = keys.find((k) => k.id === keyId)?.label ?? 'feltet';
    return mutate(async () => {
      let body;
      try {
        body = await api.patchResource(code, { [keyId]: value });
      } catch (cause) {
        toast.show(cellErrorMessage(cause, label));
        return;
      }
      setResources((prev) => replaceResources(prev, patchedResources(body)));
      if (touchesSurvey([keyId])) await reread(refreshSurvey);
    });
  }

  /**
   * "Tilføj ressource": create a manual part, then show and select it. The
   * dialog closes as soon as the part exists — a failed re-read after that
   * must not leave it open, or a second submit would create a second part —
   * and a failed create keeps it open with its input (the queue's toast).
   */
  function addResource(body: ResourceCreate) {
    createGuard.current.run(() =>
      mutate(async () => {
        const created = await api.createResource(body);
        setAddResourceOpen(false);
        toast.show(`${created.code} tilføjet`);
        await reread(async () => {
          const [s, list] = await Promise.all([api.survey(), api.resources()]);
          const next = setTypes(() => s.types);
          setResources(() => list);
          const view = viewForNewResource(next, created.type_id, viewRef.current.tab, viewRef.current.filters);
          setTab(view.tab);
          setFilters(view.filters);
          select({ typeId: created.type_id, partCode: created.code });
        });
      }),
    );
  }

  /**
   * "Tilføj kolonne": create a user column, then append `col:<id>` as a key
   * member to the selected template — or, on request for a seed, to a fresh
   * copy of it, which is then selected. A name conflict (409) stays in the
   * dialog with the server's reason (R3-D3); any other create failure goes
   * to the queue's toast and the dialog keeps its input. A failure after the
   * column exists closes the dialog with the partial-failure copy; either
   * way the catalogue and templates are re-read so the column shows. A
   * failed re-read keeps the toast and adds the stale-view notice.
   */
  function addColumn(draft: ColumnDraft, copyInstead: boolean) {
    const base = template;
    if (!base) return;
    setColumnError(null);
    columnGuard.current.run(() =>
      mutate(async () => {
        let def;
        try {
          def = await api.createResourceColumn(columnCreateBody(draft));
        } catch (cause) {
          const conflict = columnCreateConflict(cause);
          if (conflict === null) throw cause;
          setColumnError(conflict);
          return;
        }
        try {
          const target = duplicateFirst(base, copyInstead) ? await api.duplicateTemplate(base.id) : base;
          await api.patchTemplate(target.id, {
            members: appendKeyMember(target.members, resourceColumnKeyId(def.id)),
          });
          chooseTemplate(target.id);
          toast.show(`Kolonnen »${def.name}« er tilføjet til »${target.name}«`);
        } catch (cause) {
          toast.show(columnPartialFailureMessage(def.name, errorMessage(cause)));
        }
        setAddColumnOpen(false);
        await reread(async () => {
          const [k, t] = await Promise.all([api.resourceKeys(), api.templates()]);
          setKeys(k);
          setTemplates(t);
        });
      }),
    );
  }

  /** "Slet ressource": only a manual part (an instance-backed one is a server 409). */
  function deleteResource() {
    const p = selPart;
    if (!p || !isManual(p)) return;
    void mutate(async () => {
      await api.deleteResource(p.code);
      toast.show(`${p.code} slettet`);
      await reread(async () => {
        const [s, list] = await Promise.all([api.survey(), api.resources()]);
        setTypes(() => s.types);
        setResources(() => list);
        select({ typeId: p.type_id, partCode: null });
      });
    });
  }

  function invalidValue(label: string) {
    toast.show(invalidValueMessage(label));
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
      // A write like any other: it joins the app-wide chain (R11), so it lands
      // after earlier writes and before any screen's next first load.
      await appWriteChain.enqueue(async () => {
        toast.show(syncMessage(await api.syncSurvey()));
      });
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
        <ErrorBanner error={error} onRetry={reload} context="kortlægningen" />
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

      {refreshNotice && (
        <p className={styles.notice} role="alert">
          {refreshNotice}
        </p>
      )}

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
            columns={columns}
            resources={index}
            catalogue={keys}
            templates={templates}
            templateId={templateId}
            onTemplate={chooseTemplate}
            onCellCommit={commitCell}
            onInvalid={invalidValue}
            onAddResource={() => setAddResourceOpen(true)}
            onAddColumn={() => {
              setColumnError(null);
              setAddColumnOpen(true);
            }}
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
              manual={selPart !== null && isManual(selPart)}
              onDeleteResource={deleteResource}
              resource={selPart ? (index.get(selPart.code) ?? null) : null}
              catalogue={keys}
              onCellCommit={commitCell}
              onInvalid={invalidValue}
              home={tableRef}
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

      {addResourceOpen && (
        <AddResourceDialog
          types={types}
          defaultTypeId={selType?.id ?? null}
          busy={busy}
          onCancel={() => setAddResourceOpen(false)}
          onSubmit={addResource}
        />
      )}

      {addColumnOpen && (
        <AddColumnDialog
          template={template}
          existingLabels={keys.map((k) => k.label)}
          busy={busy}
          serverError={columnError}
          onCancel={() => setAddColumnOpen(false)}
          onSubmit={addColumn}
        />
      )}

      <Toast message={toast.message} />
    </div>
  );
}
