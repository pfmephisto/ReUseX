// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * SAM3 segmentation panel for one 360 panorama (#448).
 *
 * The panorama counterpart of the Segmentering view, calling POST
 * /panoramas/{id}/segment. Box regions in equirect space are not supported in
 * v1 — text-only prompts drive the tiled SAM3 pass. Like every segment call
 * it uses the server's managed SAM3 model through `useSam3().run`, which waits
 * out a first-run download/build.
 */

import { useState } from 'react';

import { api } from '../api/client';
import type { PanoramaSegmentResult } from '../api/types';
import { useSam3 } from '../app/useSam3';
import { SegmentCancelled } from '../data/sam3Provisioning';
import { Sam3StatusChip } from './Sam3StatusChip';
import styles from './PanoramaSegmentPanel.module.css';
import { useCanEdit } from '../app/CaseRoleContext';

// Module-level counter avoids key collisions across add/remove cycles.
let _nextId = 0;
const genId = () => String(++_nextId);

interface UIPrompt {
  id: string;
  text: string;
}

export interface PanoramaSegmentPanelProps {
  panoramaId: number;
  /** Called after a successful save so the parent can refresh metadata. */
  onSegmented?: () => void;
}

export function PanoramaSegmentPanel({
  panoramaId,
  onSegmented,
}: PanoramaSegmentPanelProps) {
  const canEdit = useCanEdit();
  const sam3 = useSam3();
  const [prompts, setPrompts] = useState<UIPrompt[]>([]);
  const [confidence, setConfidence] = useState(0.5);
  const [nYaw, setNYaw] = useState(8);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [result, setResult] = useState<PanoramaSegmentResult | null>(null);

  const addPrompt = () =>
    setPrompts((prev) => [...prev, { id: genId(), text: '' }]);

  const removePrompt = (id: string) =>
    setPrompts((prev) => prev.filter((p) => p.id !== id));

  const updateText = (id: string, text: string) =>
    setPrompts((prev) => prev.map((p) => (p.id === id ? { ...p, text } : p)));

  const handleRun = async () => {
    const apiPrompts = prompts
      .filter((p) => p.text.trim())
      .map((p) => ({ text: p.text.trim() }));

    setRunning(true);
    setError(null);

    try {
      const res = await sam3.run(() =>
        api.segmentPanorama(panoramaId, {
          prompts: apiPrompts.length > 0 ? apiPrompts : undefined,
          confidence,
          n_yaw: nYaw,
          save: true,
        }),
      );
      setResult(res);
      onSegmented?.();
    } catch (err) {
      if (err instanceof SegmentCancelled) return;
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setRunning(false);
    }
  };

  return (
    <div className={styles.panel}>
      {sam3.available && <Sam3StatusChip view={sam3.view} />}

      {/* ---- prompts ---- */}
      <section className={styles.section}>
        <h4 className={styles.sectionHead}>Text prompts</h4>
        <p className={styles.hint}>
          Each prompt names one object class for SAM3 to find across all
          perspective tiles. Leave empty to use the model's built-in class list.
        </p>
        {prompts.length > 0 && (
          <ul className={styles.promptList}>
            {prompts.map((p) => (
              <li key={p.id} className={styles.promptRow}>
                <span className={styles.promptKind} aria-hidden="true">
                  ⌨
                </span>
                <input
                  className={styles.promptText}
                  type="text"
                  placeholder="class name, e.g. wall"
                  value={p.text}
                  onChange={(e) => updateText(p.id, e.target.value)}
                  disabled={running}
                  aria-label="Class name for this prompt"
                />
                <button
                  type="button"
                  className={styles.removeAction}
                  onClick={() => removePrompt(p.id)}
                  disabled={running}
                  aria-label="Remove prompt"
                >
                  ✕
                </button>
              </li>
            ))}
          </ul>
        )}
        <button
          type="button"
          className={styles.addAction}
          onClick={addPrompt}
          disabled={running}
        >
          + Add text prompt
        </button>
      </section>

      {/* ---- confidence ---- */}
      <section className={styles.section}>
        <label
          className={styles.fieldLabel}
          htmlFor={`pano-seg-conf-${panoramaId}`}
        >
          Confidence:{' '}
          <span className="mono">{confidence.toFixed(2)}</span>
        </label>
        <input
          id={`pano-seg-conf-${panoramaId}`}
          className={styles.slider}
          type="range"
          min={0}
          max={1}
          step={0.05}
          value={confidence}
          onChange={(e) => setConfidence(Number(e.target.value))}
          disabled={running}
        />
      </section>

      {/* ---- tiling options ---- */}
      <section className={styles.section}>
        <label
          className={styles.fieldLabel}
          htmlFor={`pano-seg-nyaw-${panoramaId}`}
        >
          Equator tiles (n_yaw)
        </label>
        <input
          id={`pano-seg-nyaw-${panoramaId}`}
          className={styles.textInput}
          type="number"
          min={4}
          max={24}
          value={nYaw}
          onChange={(e) => setNYaw(Math.max(4, Number(e.target.value) | 0))}
          disabled={running}
          aria-label="Number of equator tiles around the sphere"
        />
        <p className={styles.hint}>
          More tiles = finer coverage; fewer tiles = faster inference. Default
          8.
        </p>
      </section>

      {/* ---- error ---- */}
      {error && (
        <p className={styles.errorMsg} role="alert">
          {error}
        </p>
      )}

      {/* ---- run button ---- */}
      {canEdit && (
      <button
        type="button"
        className={styles.runBtn}
        onClick={handleRun}
        disabled={running}
        aria-busy={running}
      >
        {running
          ? sam3.view.phase === 'preparing' || sam3.view.phase === 'first-run'
            ? 'Klargør model…'
            : 'Segmenterer…'
          : 'Kør segmentering'}
      </button>
      )}

      {/* ---- result ---- */}
      {result && !running && (
        <section className={styles.resultSection}>
          <h4 className={styles.sectionHead}>Result</h4>
          <dl className={styles.resultFacts}>
            <div className={styles.resultRow}>
              <dt>Labeled pixels</dt>
              <dd className="mono">{result.labeled_pixels.toLocaleString()}</dd>
            </div>
            {Object.keys(result.labels).length > 0 && (
              <div className={styles.resultRow}>
                <dt>Classes</dt>
                <dd>{Object.values(result.labels).join(', ')}</dd>
              </div>
            )}
            <div className={styles.resultRow}>
              <dt>Saved</dt>
              <dd>{result.saved ? 'yes' : 'no'}</dd>
            </div>
          </dl>

          {result.saved && (
            <p className={styles.hint}>
              The equirect label map is stored. To process every panorama in
              one batch, run <code className={styles.code}>rux create annotate-360</code>.
            </p>
          )}
        </section>
      )}
    </div>
  );
}
