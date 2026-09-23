// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import type { DescriptorMatchResult, FrameImageKind } from '../api/types';
import {
  DESCRIPTOR_METHOD_LABELS,
  DESCRIPTOR_METHODS,
  type DescriptorMethod,
} from '../data/framePair';
import { FramePairPanel, type PanelViewMode } from './FramePairPanel';
import styles from './FramePairInspector.module.css';

export interface FramePairInspectorProps {
  frameA: number | null;
  frameB: number | null;
}

/** How the two images are presented. */
type CompareMode = 'side-by-side' | 'overlay';

interface MatchState {
  status: 'idle' | 'loading' | 'done' | 'error';
  result?: DescriptorMatchResult;
  errorMsg?: string;
}

/**
 * The side-by-side frame-pair inspector.
 *
 * Two panels sit next to each other, each independently showing a frame's
 * image or a cloud stub. When overlay mode is chosen panel B is composited
 * over panel A using CSS opacity + translate — no canvas or pixel manipulation
 * needed. This lets the user visually gauge how much two frames overlap.
 *
 * The descriptor-similarity panel at the bottom stubs the backend call: it
 * shows the method selector and calls `api.frameDescriptorMatch()`, which hits
 * an endpoint that does not yet exist. The 404 is caught and shown gracefully
 * with a note about the follow-up backend work.
 */
export function FramePairInspector({ frameA, frameB }: FramePairInspectorProps) {
  const [compareMode, setCompareMode] = useState<CompareMode>('side-by-side');
  const [kindA, setKindA] = useState<FrameImageKind>('color');
  const [kindB, setKindB] = useState<FrameImageKind>('color');
  const [viewModeA, setViewModeA] = useState<PanelViewMode>('image');
  const [viewModeB, setViewModeB] = useState<PanelViewMode>('image');

  // Overlay controls — opacity of B over A, pixel offsets.
  const [overlayOpacity, setOverlayOpacity] = useState(0.5);
  const [overlayDx, setOverlayDx] = useState(0);
  const [overlayDy, setOverlayDy] = useState(0);

  // Descriptor-similarity panel.
  const [descriptorMethod, setDescriptorMethod] = useState<DescriptorMethod>('orb');
  const [matchState, setMatchState] = useState<MatchState>({ status: 'idle' });

  const handleMatch = useCallback(async () => {
    if (frameA === null || frameB === null) return;
    setMatchState({ status: 'loading' });
    try {
      const result = await api.frameDescriptorMatch(frameA, frameB, descriptorMethod);
      setMatchState({ status: 'done', result });
    } catch (err) {
      const msg =
        err instanceof ApiRequestError
          ? err.isNotFound || err.isNotImplemented
            ? 'Backend endpoint not yet implemented. ' +
              'Expected: POST /api/v1/frames/{a}/descriptor-match/{b}. ' +
              'Track in issue #446.'
            : err.message
          : err instanceof Error
            ? err.message
            : String(err);
      setMatchState({ status: 'error', errorMsg: msg });
    }
  }, [frameA, frameB, descriptorMethod]);

  const overlayBStyle: React.CSSProperties | undefined =
    compareMode === 'overlay'
      ? {
          opacity: overlayOpacity,
          transform: `translate(${overlayDx}px, ${overlayDy}px)`,
        }
      : undefined;

  return (
    <div className={styles.root}>
      {/* Compare-mode selector */}
      <div className={styles.modeRow}>
        <span className={styles.modeLabel}>Compare</span>
        <div className={styles.modeGroup} role="group" aria-label="Compare mode">
          {(['side-by-side', 'overlay'] as const).map((mode) => (
            <button
              key={mode}
              type="button"
              className={`${styles.modeBtn} ${compareMode === mode ? styles.modeBtnActive : ''}`}
              onClick={() => setCompareMode(mode)}
            >
              {mode === 'side-by-side' ? 'Side by side' : 'Overlay'}
            </button>
          ))}
        </div>
      </div>

      {/* Main panel area */}
      <div className={`${styles.panels} ${compareMode === 'overlay' ? styles.panelsOverlay : ''}`}>
        {compareMode === 'side-by-side' ? (
          <>
            <FramePairPanel
              label="A"
              frameId={frameA}
              viewMode={viewModeA}
              imageKind={kindA}
              onViewModeChange={setViewModeA}
              onKindChange={setKindA}
            />
            <FramePairPanel
              label="B"
              frameId={frameB}
              viewMode={viewModeB}
              imageKind={kindB}
              onViewModeChange={setViewModeB}
              onKindChange={setKindB}
            />
          </>
        ) : (
          /* Overlay mode: A is the base, B floats over it. */
          <div className={styles.overlayAnchor}>
            <FramePairPanel
              label="A"
              frameId={frameA}
              viewMode={viewModeA}
              imageKind={kindA}
              onViewModeChange={setViewModeA}
              onKindChange={setKindA}
            />
            {/* In overlay mode both panels share A's kind/view state so the
                comparison is always like-for-like. B's controls write to A's
                state too, so changing the kind in either panel updates both. */}
            <FramePairPanel
              label="B"
              frameId={frameB}
              viewMode={viewModeA}
              imageKind={kindA}
              onViewModeChange={setViewModeA}
              onKindChange={setKindA}
              overlayStyle={overlayBStyle}
            />
          </div>
        )}
      </div>

      {/* Overlay controls — only shown in overlay mode */}
      {compareMode === 'overlay' && (
        <section className={styles.overlayControls} aria-label="Overlay controls">
          <OverlaySlider
            label="Opacity"
            value={overlayOpacity}
            min={0}
            max={1}
            step={0.01}
            onChange={setOverlayOpacity}
            format={(v) => `${Math.round(v * 100)}%`}
          />
          <OverlaySlider
            label="Shift X"
            value={overlayDx}
            min={-200}
            max={200}
            step={1}
            onChange={setOverlayDx}
            format={(v) => `${v}px`}
          />
          <OverlaySlider
            label="Shift Y"
            value={overlayDy}
            min={-200}
            max={200}
            step={1}
            onChange={setOverlayDy}
            format={(v) => `${v}px`}
          />
          <button
            type="button"
            className={styles.resetBtn}
            onClick={() => { setOverlayDx(0); setOverlayDy(0); setOverlayOpacity(0.5); }}
          >
            Reset
          </button>
        </section>
      )}

      {/* Descriptor-similarity panel */}
      <section className={styles.descriptorPanel} aria-label="Descriptor similarity">
        <h3 className={styles.descriptorTitle}>Feature descriptor similarity</h3>
        <div className={styles.descriptorControls}>
          <div className={styles.methodGroup} role="group" aria-label="Descriptor method">
            {DESCRIPTOR_METHODS.map((method) => (
              <button
                key={method}
                type="button"
                className={`${styles.methodBtn} ${descriptorMethod === method ? styles.methodBtnActive : ''}`}
                onClick={() => setDescriptorMethod(method)}
              >
                {DESCRIPTOR_METHOD_LABELS[method]}
              </button>
            ))}
          </div>
          <button
            type="button"
            className={styles.matchBtn}
            disabled={frameA === null || frameB === null || matchState.status === 'loading'}
            onClick={() => void handleMatch()}
          >
            {matchState.status === 'loading' ? 'Matching…' : 'Match'}
          </button>
        </div>

        <MatchResult state={matchState} />
      </section>
    </div>
  );
}

// -------------------------------------------------------------------------- //

interface OverlaySliderProps {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (value: number) => void;
  format: (value: number) => string;
}

function OverlaySlider({ label, value, min, max, step, onChange, format }: OverlaySliderProps) {
  return (
    <div className={styles.sliderRow}>
      <label className={styles.sliderLabel}>{label}</label>
      <input
        type="range"
        className={styles.slider}
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      />
      <span className={`${styles.sliderValue} mono`}>{format(value)}</span>
    </div>
  );
}

// -------------------------------------------------------------------------- //

function MatchResult({ state }: { state: MatchState }) {
  if (state.status === 'idle') {
    return (
      <p className={styles.matchHint}>
        Choose a method and click Match to compute keypoint correspondences between the two frames.
      </p>
    );
  }

  if (state.status === 'loading') {
    return <p className={styles.matchHint}>Computing…</p>;
  }

  if (state.status === 'error') {
    return (
      <div className={styles.matchError}>
        <span className={styles.matchErrorIcon} aria-hidden="true">✕</span>
        <p className={styles.matchErrorMsg}>{state.errorMsg}</p>
      </div>
    );
  }

  if (state.status === 'done' && state.result) {
    const r = state.result;
    return (
      <dl className={styles.matchResult}>
        <div className={styles.matchStat}>
          <dt>Inliers</dt>
          <dd className="mono">{r.n_inliers}</dd>
        </div>
        <div className={styles.matchStat}>
          <dt>RMS</dt>
          <dd className="mono">
            {r.rms_m !== null ? `${(r.rms_m * 1000).toFixed(1)} mm` : '—'}
          </dd>
        </div>
        {r.error && (
          <div className={`${styles.matchStat} ${styles.matchStatError}`}>
            <dt>Note</dt>
            <dd>{r.error}</dd>
          </div>
        )}
      </dl>
    );
  }

  return null;
}
