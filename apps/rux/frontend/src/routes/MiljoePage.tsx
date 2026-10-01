// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useRef, useState } from 'react';
import { useLocation, useNavigate } from 'react-router-dom';

import { api, type ApiRequestError } from '../api/client';
import type { Sample, SampleCreate, SamplePatch, SampleResult, SurveyType } from '../api/types';
import { MILJOE_PATH, parseMiljoeQuery } from '../app/links';
import { saveErrorMessage } from '../app/saveError';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useToast } from '../app/useToast';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { NewSampleForm } from '../components/miljoe/NewSampleForm';
import { SampleCard } from '../components/miljoe/SampleCard';
import {
  addSample,
  advancePatch,
  gateChanges,
  gateMessage,
  removeSample,
  replaceSample,
  resultPatch,
  resultToast,
  toggleLink,
  UNDO_RESULT_PATCH,
} from '../miljoe/model';
import styles from './MiljoePage.module.css';

/**
 * Miljø & prøver — the environmental samples that gate approval in
 * Kortlægning. One card per sample: its stage chain, the next step or the
 * result buttons, and the survey types it covers.
 *
 * Every write runs on one serial chain. After each one, the same queued task
 * re-reads `GET /survey`, so the linked types' miljøstatus is the server's,
 * and the toast says what the change did to the approval gate. `refresh()`
 * then re-reads the summary behind both sidebar badges.
 */
export function MiljoePage() {
  const location = useLocation();
  const navigate = useNavigate();
  const query = parseMiljoeQuery(location.search);
  // Queued tasks outlive the render that queued them: they read the location
  // and the mount state through refs, never a stale closure.
  const locationRef = useRef(location);
  locationRef.current = location;
  const mountedRef = useRef(true);
  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
    };
  }, []);
  const { data, error, loading, reload } = useAsync((s) => Promise.all([api.samples(s), api.survey(s)]), []);
  const { refresh } = useSurveyCounts();
  const toast = useToast(2600);
  const { busy, mutate } = useMutationQueue({
    onError: (cause) => toast.show(saveErrorMessage(cause)),
    onSettled: refresh,
  });

  const [samples, setSamplesState] = useState<Sample[]>([]);
  const samplesRef = useRef<Sample[]>([]);
  const setSamples = useCallback((update: (prev: Sample[]) => Sample[]) => {
    samplesRef.current = update(samplesRef.current);
    setSamplesState(samplesRef.current);
  }, []);
  const [types, setTypesState] = useState<SurveyType[]>([]);
  // Mirrors `types` synchronously: a queued task's gate diff compares against
  // the snapshot the previous task produced, not a stale render's.
  const typesRef = useRef<SurveyType[]>([]);
  const setTypes = useCallback((next: SurveyType[]) => {
    typesRef.current = next;
    setTypesState(next);
  }, []);

  const [loadedOnce, setLoadedOnce] = useState(false);
  const [editingId, setEditingId] = useState<number | null>(null);
  const [creating, setCreating] = useState(query.newForType !== null);
  const [focusedId, setFocusedId] = useState<number | null>(query.sampleId);
  // Bumped on every focus request, so re-targeting the already-focused card
  // (the same `?sample=` link clicked again) still scrolls and focuses it.
  const [focusNonce, setFocusNonce] = useState(0);
  const focusCard = useCallback((id: number | null) => {
    setFocusedId(id);
    setFocusNonce((n) => n + 1);
  }, []);
  const newButton = useRef<HTMLButtonElement>(null);
  // `busy` is React state: two submits in the same tick would both see it
  // false. This ref is set synchronously, before the request is queued.
  const createInFlight = useRef(false);

  // Link toggles are field commits: each sends the full desired set at once
  // (PUT replaces the set, so the last request wins), and the checkboxes show
  // that desired set until the last queued PUT for the sample has settled.
  const [linkDrafts, setLinkDrafts] = useState<ReadonlyMap<number, readonly number[]>>(() => new Map());
  const linkDraftsRef = useRef<ReadonlyMap<number, readonly number[]>>(new Map());
  const linkSeq = useRef(new Map<number, number>());
  const setLinkDraft = useCallback((id: number, ids: readonly number[] | null) => {
    const next = new Map(linkDraftsRef.current);
    if (ids === null) next.delete(id);
    else next.set(id, ids);
    linkDraftsRef.current = next;
    setLinkDrafts(next);
  }, []);

  const cardRefs = useRef(new Map<number, HTMLElement>());

  useEffect(() => {
    if (!data) return;
    setSamples(() => data[0]);
    setTypes(data[1].types);
    setLoadedOnce(true);
  }, [data, setSamples, setTypes]);

  // A new deep link on the mounted page (e.g. the sidebar after ?sample=)
  // re-targets. Keyed on `location.key` too, so the identical link clicked
  // again re-focuses its card. The body only sets page state, never the
  // location, so it cannot loop. Clearing the query (after a `?ny=` create or
  // cancel) leaves the focus where the page put it.
  useEffect(() => {
    const q = parseMiljoeQuery(location.search);
    if (q.sampleId !== null) focusCard(q.sampleId);
    if (q.newForType !== null) setCreating(true);
  }, [location.key, location.search, focusCard]);

  // Scroll to and focus the deep-linked (or just created) card. A stale id
  // (deleted sample) simply finds no card.
  useEffect(() => {
    if (!loadedOnce || focusedId === null) return;
    const el = cardRefs.current.get(focusedId);
    el?.scrollIntoView({ block: 'center' });
    el?.focus({ preventScroll: true });
  }, [loadedOnce, focusedId, focusNonce]);

  /** Drop `?ny=` once the form it opened is done, so the next "+ Ny prøve" starts empty. */
  function clearNewQuery() {
    // Off the page (navigated away mid-create), the user's own history entry
    // must not be replaced; a `?sample=` that landed meanwhile is kept.
    const current = locationRef.current;
    if (!mountedRef.current || parseMiljoeQuery(current.search).newForType === null) return;
    const params = new URLSearchParams(current.search);
    params.delete('ny');
    navigate({ pathname: MILJOE_PATH, search: params.toString() }, { replace: true });
  }

  /** Re-read the survey and say what the change did to the approval gate. */
  async function refreshGate(code: string): Promise<string | null> {
    const before = typesRef.current;
    try {
      const survey = await api.survey();
      setTypes(survey.types);
      return gateMessage(code, gateChanges(before, survey.types));
    } catch {
      return 'Gemt — men miljøstatus kunne ikke genindlæses. Åbn Kortlægning for at se den.';
    }
  }

  function patch(
    sample: Sample,
    body: SamplePatch,
    fallback: string | null,
    onUnprocessable?: (cause: ApiRequestError) => void,
  ) {
    mutate(async () => {
      const updated = await api.patchSample(sample.id, body);
      setSamples((prev) => replaceSample(prev, updated));
      const message = (await refreshGate(updated.code)) ?? fallback;
      if (message) toast.show(message);
    }, onUnprocessable);
  }

  // Buttons: gated on `busy`, so a double click cannot stack requests.
  function advance(sample: Sample) {
    const body = advancePatch(sample);
    if (busy || !body) return;
    patch(sample, body, null);
  }

  function recordResult(sample: Sample, result: SampleResult) {
    if (busy) return;
    patch(sample, resultPatch(result), resultToast(sample.code, result), () =>
      toast.show(`${sample.code}: svaret kan først registreres, når prøven er sendt til lab.`),
    );
  }

  function undoResult(sample: Sample) {
    if (busy) return;
    patch(sample, UNDO_RESULT_PATCH, `${sample.code}: svaret er fortrudt — afventer igen svar fra lab`);
  }

  function remove(sample: Sample) {
    if (busy) return;
    mutate(async () => {
      await api.deleteSample(sample.id);
      // Hand focus to the next card, else the previous one, else "+ Ny prøve";
      // never leave it on <body> or on the deleted id.
      const list = samplesRef.current;
      const at = list.findIndex((x) => x.id === sample.id);
      const neighbour = at < 0 ? undefined : (list[at + 1] ?? list[at - 1]);
      setSamples((prev) => removeSample(prev, sample.id));
      setEditingId((id) => (id === sample.id ? null : id));
      if (neighbour) {
        focusCard(neighbour.id);
      } else {
        setFocusedId((id) => (id === sample.id ? null : id));
        newButton.current?.focus();
      }
      const message = await refreshGate(sample.code);
      toast.show(message ?? `${sample.code} slettet`);
    });
  }

  function create(body: SampleCreate) {
    if (busy || createInFlight.current) return;
    createInFlight.current = true;
    mutate(async () => {
      try {
        const created = await api.createSample(body);
        setSamples((prev) => addSample(prev, created));
        setCreating(false);
        clearNewQuery();
        focusCard(created.id);
        const message = await refreshGate(created.code);
        toast.show(message ?? `${created.code} registreret`);
      } finally {
        createInFlight.current = false;
      }
    });
  }

  function cancelCreate() {
    setCreating(false);
    clearNewQuery();
  }

  // Field commits: never gated on `busy`, never dropped — they wait their turn.
  function editText(sample: Sample, body: Pick<SamplePatch, 'title' | 'what'>) {
    patch(sample, body, null);
  }

  function onToggleLink(sample: Sample, typeId: number) {
    const base = linkDraftsRef.current.get(sample.id) ?? sample.type_ids;
    const next = toggleLink(base, typeId);
    setLinkDraft(sample.id, next);
    const seq = (linkSeq.current.get(sample.id) ?? 0) + 1;
    linkSeq.current.set(sample.id, seq);
    mutate(async () => {
      try {
        const updated = await api.setSampleLinks(sample.id, next);
        setSamples((prev) => replaceSample(prev, updated));
        const message = await refreshGate(updated.code);
        if (message) toast.show(message);
      } finally {
        // Only the last toggle hands the checkboxes back to the server's set;
        // on failure that set is whatever the last successful PUT left.
        if (linkSeq.current.get(sample.id) === seq) setLinkDraft(sample.id, null);
      }
    });
  }

  // --------------------------------------------------------------- render --

  if (error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={error} onRetry={reload} context="Miljø & prøver" />
      </div>
    );
  }
  if ((loading && !data) || (data && !loadedOnce)) {
    return (
      <div className={styles.page}>
        <Spinner label="Indlæser prøver…" />
      </div>
    );
  }

  // An unknown or rejected `?ny=` type opens an empty form.
  const preLinked =
    query.newForType !== null && types.some((t) => t.id === query.newForType && t.review_status !== 'rejected')
      ? [query.newForType]
      : [];

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h2 className={styles.title}>Miljø & prøver</h2>
        <span className={styles.sub}>Prøver styrer miljøstatus på de koblede bygningsdele</span>
        <button
          ref={newButton}
          type="button"
          className={styles.btnPrimary}
          onClick={() => setCreating(true)}
          disabled={creating}
        >
          + Ny prøve
        </button>
      </header>

      {creating && (
        <NewSampleForm
          key={query.newForType ?? 'ny'}
          types={types}
          initialTypeIds={preLinked}
          busy={busy}
          onSubmit={create}
          onCancel={cancelCreate}
        />
      )}

      {samples.length === 0 && !creating && (
        <EmptyState
          title="Ingen prøver endnu"
          detail="Registrér en miljøprøve og kobl den til de typer i kortlægningen, den dækker. Indtil svaret foreligger, kan typerne ikke godkendes."
          action={
            <button type="button" className={styles.btnPrimaryInline} onClick={() => setCreating(true)}>
              + Ny prøve
            </button>
          }
        />
      )}

      {samples.length > 0 && (
        <ul className={styles.list} aria-label="Prøver">
          {samples.map((s) => (
            <li key={s.id}>
              <SampleCard
                sample={s}
                types={types}
                linkedIds={linkDrafts.get(s.id) ?? s.type_ids}
                busy={busy}
                focused={focusedId === s.id}
                editing={editingId === s.id}
                onEditing={(open) => setEditingId(open ? s.id : null)}
                onAdvance={() => advance(s)}
                onResult={(r) => recordResult(s, r)}
                onUndoResult={() => undoResult(s)}
                onTitle={(title) => editText(s, { title })}
                onWhat={(what) => editText(s, { what })}
                onToggleLink={(typeId) => onToggleLink(s, typeId)}
                onDelete={() => remove(s)}
                cardRef={(el) => {
                  if (el) cardRefs.current.set(s.id, el);
                  else cardRefs.current.delete(s.id);
                }}
              />
            </li>
          ))}
        </ul>
      )}

      {samples.length > 0 && (
        <p className={styles.footnote}>
          Et prøvesvar opdaterer miljøstatus på alle koblede typer i kortlægningen — én prøve kan afklare mange
          bygningsdele.
        </p>
      )}

      <Toast message={toast.message} />
    </div>
  );
}
