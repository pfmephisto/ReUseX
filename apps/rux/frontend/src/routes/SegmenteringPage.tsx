// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Segmentering (spec B3): interactive SAM3 on one sensor frame, with the image
 * front and centre.
 *
 * Prompts are drawn on the image (drag = box, click = point) or typed (text
 * only); a run segments the frame with the server's managed SAM3 model —
 * downloaded and built on first use, which the status chip and
 * `useSam3().run` wait out — and saves the mask. The saved mask is read back
 * as its 16-bit label PNG, so each prompt's pixels can be counted and
 * highlighted, and one prompt's pixels can be filed as a resource ("Opret
 * ressource fra markering" → `POST /frames/{id}/segment/resource`), which
 * makes a new instance and survey part and rewrites the label clouds.
 *
 * URL: `?frame=<id>&u=<px>&v=<px>` — u,v (from the viewport's source images,
 * or a Kortlægning part's best frame) seed a point prompt.
 *
 * This view replaces the old SegmentPanel fold-out in Billeder.
 */

import { useCallback, useEffect, useMemo, useRef, useState, type CSSProperties } from 'react';
import { Link, useSearchParams } from 'react-router-dom';

import { ApiRequestError, api } from '../api/client';
import type { FrameSegmentResult, SegmentResourceRequest, SegmentResourceResult, SurveyType } from '../api/types';
import { useJobs } from '../app/JobsContext';
import { useLabelQueue } from '../app/LabelQueueContext';
import { isField } from '../app/keyTargets';
import { parseSegmentQuery, surveyTypeHref } from '../app/links';
import { useSurveyCounts } from '../app/SurveyCountsContext';
import { useAsync } from '../app/useAsync';
import { useMutationQueue } from '../app/useMutationQueue';
import { useSam3 } from '../app/useSam3';
import { useToast } from '../app/useToast';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { Sam3StatusChip } from '../components/Sam3StatusChip';
import { Filmstrip } from '../components/segmentering/Filmstrip';
import { ResourceDialog } from '../components/segmentering/ResourceDialog';
import { SegmentStage } from '../components/segmentering/SegmentStage';
import { Spinner } from '../components/Spinner';
import { Toast } from '../components/Toast';
import { decodeLabelPng, promptPixelCounts, type LabelImage } from '../data/labelPng';
import { neighborFrameIds } from '../data/labelQueue';
import { SegmentCancelled, SegmentRunError } from '../data/sam3Provisioning';
import {
  buildRequestPrompts,
  clampNeighborCount,
  NEIGHBOR_MAX,
  pointBox,
  resourceBlockReason,
  geometryHint,
  resultClasses,
  seedToImage,
  segmentKeyAction,
  stepFrame,
  type ImageBox,
  type ResultClass,
  type SegPrompt,
  type SentPrompt,
} from '../data/segmentView';
import { readLabelPalette } from '../viewport/labelColors';
import styles from './SegmenteringPage.module.css';

let nextPromptId = 0;
const newPromptId = () => `p${++nextPromptId}`;

interface RunOutcome {
  frameId: number;
  result: FrameSegmentResult;
  sent: SentPrompt[];
}

async function fetchLabelImage(frameId: number, signal: AbortSignal): Promise<LabelImage> {
  const response = await fetch(`${api.frameImageUrl(frameId, 'segmentation')}&_t=${Date.now()}`, { signal });
  if (!response.ok) throw new Error(`segmenteringen kunne ikke hentes (${response.status})`);
  return decodeLabelPng(new Uint8Array(await response.arrayBuffer()));
}

export function SegmenteringPage() {
  const [params, setParams] = useSearchParams();
  const query = useMemo(() => parseSegmentQuery(`?${params.toString()}`), [params]);

  const frames = useAsync((signal) => api.frames({}, signal), []);
  const segmentedList = useAsync((signal) => api.frames({ segmented: true }, signal), []);
  const ids = frames.data?.ids ?? [];
  const [segmentedLocal, setSegmentedLocal] = useState<number[]>([]);
  const segmented = useMemo(
    () => new Set([...(segmentedList.data?.ids ?? []), ...segmentedLocal]),
    [segmentedList.data, segmentedLocal],
  );

  // No frame in the URL: open the first one (replace, so Back leaves the view).
  const frameId = query.frameId;
  useEffect(() => {
    if (frameId === null && ids.length > 0) setParams({ frame: String(ids[0]) }, { replace: true });
  }, [frameId, ids, setParams]);

  const selectFrame = useCallback(
    (id: number) => setParams({ frame: String(id) }, { replace: true }),
    [setParams],
  );

  const frame = useAsync((signal) => (frameId === null ? Promise.resolve(undefined) : api.frame(frameId, signal)), [frameId]);

  const sam3 = useSam3();
  const toast = useToast(4000);
  const labelQueue = useLabelQueue();
  const { markCloudsChanged } = useJobs();
  const { refresh: refreshCounts } = useSurveyCounts();
  const runQueue = useMutationQueue({ scope: 'page', onError: (e) => setError(String(e)) });
  const writeQueue = useMutationQueue({ onError: (e) => setDialogError(errorText(e)), onSettled: refreshCounts });

  const [size, setSize] = useState<{ width: number; height: number } | null>(null);
  const [prompts, setPrompts] = useState<SegPrompt[]>([]);
  const [confidence, setConfidence] = useState(0.5);
  // The frame a run is in flight for (one at a time), or null.
  const [runningFrame, setRunningFrame] = useState<number | null>(null);
  const running = runningFrame !== null;
  // The frame on screen now, for a run that resolves after the user moved on.
  const currentFrame = useRef(frameId);
  currentFrame.current = frameId;
  const [touchDraw, setTouchDraw] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [outcome, setOutcome] = useState<RunOutcome | null>(null);
  const [mask, setMask] = useState<LabelImage | null>(null);
  const [maskError, setMaskError] = useState<string | null>(null);
  const [showMask, setShowMask] = useState(true);
  const [selected, setSelected] = useState<number | null>(null);
  const [dialogFor, setDialogFor] = useState<ResultClass | null>(null);
  const [dialogError, setDialogError] = useState<string | null>(null);
  const [created, setCreated] = useState<SegmentResourceResult | null>(null);
  const [before, setBefore] = useState(0);
  const [after, setAfter] = useState(0);
  const paletteSize = useMemo(() => Math.max(1, readLabelPalette().colors.length), []);

  // A new frame starts clean.
  useEffect(() => {
    setSize(null);
    setPrompts([]);
    setOutcome(null);
    setMask(null);
    setMaskError(null);
    setSelected(null);
    setError(null);
    setCreated(null);
  }, [frameId]);

  // Seed a point prompt from ?u=&v= once the image size is known.
  const seededKey = useRef<string | null>(null);
  useEffect(() => {
    if (frameId === null || !query.seed || !size || !frame.data || frame.data.id !== frameId) return;
    const key = `${frameId}:${query.seed.u}:${query.seed.v}`;
    if (seededKey.current === key) return;
    seededKey.current = key;
    const at = seedToImage(query.seed, frame.data.intrinsics, size);
    if (!at) return;
    setPrompts((current) => [
      ...current,
      { id: newPromptId(), text: '', box: pointBox(at.x, at.y, size.width, size.height), point: true, at: [at.x, at.y] },
    ]);
  }, [frameId, query.seed, size, frame.data]);

  // Prompt index (= mask label value) of each prompt that would be sent.
  const slots = useMemo(() => {
    const map = new Map<string, number>();
    buildRequestPrompts(prompts).sent.forEach((s, i) => map.set(s.promptId, i));
    return map;
  }, [prompts]);
  const slotOf = useCallback((id: string) => slots.get(id) ?? null, [slots]);

  const counts = useMemo(() => (mask ? promptPixelCounts(mask) : null), [mask]);
  const classes = useMemo(
    () => (outcome ? resultClasses(outcome.result, outcome.sent, counts) : []),
    [outcome, counts],
  );

  const addDrawn = useCallback((box: ImageBox, at: [number, number] | null) => {
    setPrompts((current) => [
      ...current,
      at ? { id: newPromptId(), text: '', box, point: true, at } : { id: newPromptId(), text: '', box, point: false },
    ]);
  }, []);

  const run = useCallback(() => {
    if (frameId === null || running) return;
    const { prompts: wire, sent } = buildRequestPrompts(prompts);
    const id = frameId;
    setRunningFrame(id);
    setError(null);
    setCreated(null);
    // A run outlives a frame switch: its result is filed (segmented dot) but
    // only shown — mask, selection, error — if its frame is still on screen.
    const stillHere = () => currentFrame.current === id;
    void runQueue.mutate(async () => {
      try {
        const result = await sam3.run(() =>
          api.segmentFrame(id, { prompts: wire.length > 0 ? wire : undefined, confidence, save: true }),
        );
        if (result.saved) setSegmentedLocal((s) => [...s, id]);
        if (!stillHere()) return;
        setOutcome({ frameId: id, result, sent });
        // One class: select it, so "Opret ressource" is one click away.
        const reported = new Set([...sent.map((_, i) => i), ...Object.keys(result.labels).map(Number)]);
        setSelected(reported.size === 1 ? [...reported][0] : null);
        setShowMask(true);
        if (result.saved) {
          setMask(null);
          setMaskError(null);
          try {
            const image = await fetchLabelImage(id, new AbortController().signal);
            if (stillHere()) setMask(image);
          } catch (e) {
            if (stillHere()) setMaskError(e instanceof Error ? e.message : String(e));
          }
        }
      } catch (e) {
        if (e instanceof SegmentCancelled || !stillHere()) return;
        setError(e instanceof SegmentRunError ? e.message : errorText(e));
      } finally {
        setRunningFrame(null);
      }
    });
  }, [frameId, running, prompts, confidence, runQueue, sam3]);

  const enqueue = useCallback(() => {
    if (frameId === null) return;
    const targets = neighborFrameIds(ids, frameId, before, after);
    labelQueue.enqueue(targets, buildRequestPrompts(prompts).prompts, confidence);
    toast.show(`${targets.length} ${targets.length === 1 ? 'billede' : 'billeder'} lagt i køen`);
  }, [frameId, ids, before, after, labelQueue, prompts, confidence, toast]);

  // Survey types and label names for the dialog, fetched when it opens.
  const [types, setTypes] = useState<SurveyType[] | null>(null);
  const [labelNames, setLabelNames] = useState<Record<string, string> | null>(null);
  const openDialog = useCallback((cls: ResultClass) => {
    setDialogFor(cls);
    setDialogError(null);
    setTypes(null);
    api.survey().then((s) => setTypes(s.types), (e: unknown) => setDialogError(errorText(e)));
    api.clouds().then(
      (clouds) => setLabelNames(clouds.find((c) => c.name === 'labels')?.labels ?? null),
      () => setLabelNames(null),
    );
  }, []);

  const createResource = useCallback(
    (request: SegmentResourceRequest) => {
      if (frameId === null) return;
      const id = frameId;
      void writeQueue.mutate(
        async () => {
          const result = await api.segmentResource(id, request);
          setCreated(result);
          setDialogFor(null);
          markCloudsChanged(result.clouds);
          toast.show(`${result.resource_code} oprettet · ${result.point_count.toLocaleString('da-DK')} punkter`);
        },
        (cause) => setDialogError(cause.message),
      );
    },
    [frameId, writeQueue, markCloudsChanged, toast],
  );

  // Keys: ←/→ frames, M mask, Ctrl/⌘+Enter run. Never while typing.
  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (dialogFor) return;
      const action = segmentKeyAction({
        key: e.key,
        inField: isField(e.target),
        metaKey: e.metaKey,
        ctrlKey: e.ctrlKey,
        altKey: e.altKey,
      });
      if (!action) return;
      e.preventDefault();
      if (action === 'run') run();
      else if (action === 'mask') setShowMask((v) => !v);
      else {
        const next = stepFrame(ids, frameId, action === 'next' ? 1 : -1);
        if (next !== null && next !== frameId) selectFrame(next);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [dialogFor, run, ids, frameId, selectFrame]);

  // ---------------------------------------------------------------- render --

  if (frames.error) {
    return (
      <div className={styles.page}>
        <ErrorBanner error={frames.error} onRetry={frames.reload} context="billederne" />
      </div>
    );
  }
  if (!frames.data) return <Spinner label="Indlæser billeder…" />;
  if (ids.length === 0) {
    return (
      <div className={styles.page}>
        <EmptyState
          title="Ingen sensorbilleder"
          detail="Projektet har ingen billeder at segmentere. Importér en scanning først (rux import)."
        />
      </div>
    );
  }

  const blockReason = resourceBlockReason(frame.data);
  const promptCount = slots.size;
  const queueCount = frameId === null ? 0 : neighborFrameIds(ids, frameId, before, after).length;
  const resultIsCurrent = outcome !== null && outcome.frameId === frameId;
  const drawnHint = geometryHint(classes, prompts);

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Segmentering</h1>
        <span className={styles.sub}>
          Billede <span className="mono">{frameId ?? '–'}</span>
          {frameId !== null && segmented.has(frameId) ? ' · har gemt segmentering' : ''}
        </span>
        {sam3.available && (sam3.view.phase === 'ready' || sam3.view.phase === 'unknown') && (
          <div className={styles.actions}>
            <Sam3StatusChip view={sam3.view} compact />
          </div>
        )}
      </header>

      <div className={styles.bench}>
        <main className={styles.main}>
          {frameId !== null && (
            <SegmentStage
              key={frameId}
              imageUrl={api.frameImageUrl(frameId, 'color')}
              alt={`Farvebillede ${frameId}`}
              size={size}
              onLoad={setSize}
              prompts={prompts}
              slotOf={slotOf}
              paletteSize={paletteSize}
              onDraw={addDrawn}
              mask={resultIsCurrent ? mask : null}
              showMask={showMask}
              selected={selected}
              disabled={running}
              touchDraw={touchDraw}
            />
          )}
          <Filmstrip ids={ids} current={frameId} segmented={segmented} onSelect={selectFrame} />
        </main>

        <aside className={styles.side} aria-label="Segmentering">
          {sam3.available && sam3.view.phase !== 'ready' && sam3.view.message && (
            <section className={styles.section}>
              <Sam3StatusChip view={sam3.view} />
            </section>
          )}

          <section className={styles.section}>
            <h2 className={styles.heading}>Prompts</h2>
            <p className={styles.hint}>
              Klik på et objekt eller træk en boks om det for at markere netop det. Skriv et klassenavn
              (fx dør) for at finde alle af slagsen; sammen med en boks findes kun dem i boksen.
            </p>
            {prompts.length > 0 && (
              <ol className={styles.prompts}>
                {prompts.map((p) => {
                  const slot = slots.get(p.id);
                  return (
                    <li key={p.id} className={styles.prompt}>
                      <span
                        className={styles.swatch}
                        data-slot={slot === undefined ? 'none' : undefined}
                        style={slot === undefined ? undefined : ({ '--c': `var(--label-${slot % paletteSize})` } as CSSProperties)}
                        title={p.box ? (p.point ? 'Punkt' : 'Boks') : 'Kun tekst'}
                      >
                        {slot === undefined ? '·' : slot + 1}
                      </span>
                      <span className={styles.kind} aria-hidden="true">
                        {p.box ? (p.point ? '•' : '▭') : 'T'}
                      </span>
                      <input
                        className={styles.input}
                        value={p.text}
                        placeholder={p.box ? 'klasse (valgfri)' : 'klasse, fx dør'}
                        aria-label={`Klassenavn for prompt ${slot === undefined ? '' : slot + 1}`}
                        onChange={(e) =>
                          setPrompts((cur) => cur.map((q) => (q.id === p.id ? { ...q, text: e.target.value } : q)))
                        }
                      />
                      <button
                        type="button"
                        className={styles.remove}
                        onClick={() => setPrompts((cur) => cur.filter((q) => q.id !== p.id))}
                        aria-label="Fjern prompt"
                      >
                        ✕
                      </button>
                    </li>
                  );
                })}
              </ol>
            )}
            <div className={styles.row}>
              <button
                type="button"
                className={styles.textBtn}
                onClick={() => setPrompts((cur) => [...cur, { id: newPromptId(), text: '', box: null, point: false }])}
              >
                + Tekstprompt
              </button>
              {prompts.length > 0 && (
                <button type="button" className={styles.textBtn} onClick={() => setPrompts([])}>
                  Ryd alle
                </button>
              )}
              <button
                type="button"
                className={styles.touchOnly}
                aria-pressed={touchDraw}
                onClick={() => setTouchDraw((v) => !v)}
                title="Med en finger: træk en boks på billedet i stedet for at rulle siden"
              >
                {touchDraw ? 'Tegn boks: til' : 'Tegn boks'}
              </button>
            </div>
            <label className={styles.slider}>
              <span>
                Konfidens <span className="mono">{confidence.toFixed(2)}</span>
              </span>
              <input
                type="range"
                min={0}
                max={1}
                step={0.05}
                value={confidence}
                onChange={(e) => setConfidence(Number(e.target.value))}
              />
            </label>
            <button
              type="button"
              className={styles.run}
              onClick={run}
              disabled={running || frameId === null}
              aria-busy={running}
              title="Ctrl/⌘ + Enter"
            >
              {running
                ? runningFrame !== frameId
                  ? `Segmenterer billede ${runningFrame}…`
                  : sam3.view.phase === 'preparing' || sam3.view.phase === 'first-run'
                    ? 'Klargør model…'
                    : 'Segmenterer…'
                : promptCount === 0
                  ? 'Kør med standardklasser'
                  : `Kør segmentering (${promptCount})`}
            </button>
            {error && (
              <div className={styles.error} role="alert">
                <span>{error}</span>
                <button type="button" className={styles.textBtn} onClick={run} disabled={running}>
                  Prøv igen
                </button>
              </div>
            )}
          </section>

          {resultIsCurrent && outcome && (
            <section className={styles.section}>
              <div className={styles.headingRow}>
                <h2 className={styles.heading}>Resultat</h2>
                <label className={styles.toggle}>
                  <input type="checkbox" className={styles.check} checked={showMask} onChange={(e) => setShowMask(e.target.checked)} />
                  Vis maske <span className={styles.key}>M</span>
                </label>
              </div>
              <p className={styles.hint}>
                {outcome.result.labeled_pixels.toLocaleString('da-DK')} px segmenteret
                {outcome.result.saved ? ' og gemt.' : ' (ikke gemt).'}
                {maskError ? ` Masken kunne ikke vises: ${maskError}` : ''}
              </p>
              {classes.length === 0 ? (
                <p className={styles.hint}>Modellen fandt intet. Prøv en lavere konfidens eller en anden prompt.</p>
              ) : (
                <ul className={styles.classes}>
                  {classes.map((c) => (
                    <li key={c.index}>
                      <button
                        type="button"
                        className={`${styles.cls} ${selected === c.index ? styles.clsOn : ''}`}
                        aria-pressed={selected === c.index}
                        onClick={() => setSelected((s) => (s === c.index ? null : c.index))}
                      >
                        <span
                          className={styles.swatch}
                          style={{ '--c': `var(--label-${c.index % paletteSize})` } as CSSProperties}
                        >
                          {c.index + 1}
                        </span>
                        <span className={styles.clsName}>{c.name}</span>
                        <span className={styles.clsCount}>
                          {c.pixels === null ? '…' : `${c.pixels.toLocaleString('da-DK')} px`}
                        </span>
                      </button>
                    </li>
                  ))}
                </ul>
              )}
              {drawnHint && <p className={styles.hint}>{drawnHint}</p>}
              {outcome.result.saved && classes.length > 0 && (
                <>
                  <button
                    type="button"
                    className={styles.primary}
                    disabled={selected === null || blockReason !== null || writeQueue.busy || classes.find((c) => c.index === selected)?.pixels === 0}
                    onClick={() => {
                      const cls = classes.find((c) => c.index === selected);
                      if (cls) openDialog(cls);
                    }}
                  >
                    Opret ressource fra markering
                  </button>
                  <p className={styles.hint}>
                    {blockReason ??
                      (selected === null
                        ? 'Vælg en klasse ovenfor for at oprette den som ressource.'
                        : classes.find((c) => c.index === selected)?.pixels === 0
                          ? 'Den valgte klasse har ingen pixels.'
                          : null)}
                  </p>
                </>
              )}
              {created && (
                <div className={styles.created} role="status">
                  <strong>{created.resource_code}</strong> oprettet · {created.point_count.toLocaleString('da-DK')}{' '}
                  punkter
                  {created.type_created ? ' · ny type' : ''}
                  <span className={styles.links}>
                    <Link to={surveyTypeHref(created.type_id)}>Vis i Kortlægning</Link>
                    <Link to="/viewport?labels=instances">Vis i Viewport</Link>
                  </span>
                </div>
              )}
            </section>
          )}

          <section className={styles.section}>
            <h2 className={styles.heading}>Tilføj til kø</h2>
            <p className={styles.hint}>
              Samme prompts på nabobillederne, kørt samlet fra køen i{' '}
              <Link to="/frames">Billeder</Link>.
            </p>
            <div className={styles.row}>
              <label className={styles.num}>
                Før
                <input
                  type="number"
                  min={0}
                  max={NEIGHBOR_MAX}
                  value={before}
                  onChange={(e) => setBefore(clampNeighborCount(e.target.value))}
                />
              </label>
              <label className={styles.num}>
                Efter
                <input
                  type="number"
                  min={0}
                  max={NEIGHBOR_MAX}
                  value={after}
                  onChange={(e) => setAfter(clampNeighborCount(e.target.value))}
                />
              </label>
              <button type="button" className={styles.ghost} onClick={enqueue} disabled={frameId === null}>
                Tilføj {queueCount} til kø
              </button>
            </div>
          </section>

          <p className={styles.footnote}>
            <span className={styles.key}>←</span> <span className={styles.key}>→</span> skift billede ·{' '}
            <span className={styles.key}>M</span> maske · <span className={styles.key}>Ctrl</span>+
            <span className={styles.key}>Enter</span> kør
          </p>
        </aside>
      </div>

      {dialogFor && (
        <ResourceDialog
          cls={dialogFor}
          types={types}
          labelNames={labelNames}
          busy={writeQueue.busy}
          error={dialogError}
          onCancel={() => setDialogFor(null)}
          onSubmit={createResource}
        />
      )}
      <Toast message={toast.message} />
    </div>
  );
}

function errorText(e: unknown): string {
  if (e instanceof ApiRequestError) return e.message;
  return e instanceof Error ? e.message : String(e);
}
