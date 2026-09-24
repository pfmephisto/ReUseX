// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * SAM3 segmentation panel for one 360 panorama (#448).
 *
 * Mirrors SegmentPanel for sensor frames but calls POST
 * /panoramas/{id}/segment instead.  Box regions in equirect space are not
 * supported in v1 — text-only prompts drive the tiled SAM3 pass.
 */

import { useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import type { PanoramaSegmentResult } from '../api/types';
import styles from './SegmentPanel.module.css';

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
  const [modelPath, setModelPath] = useState('');
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
    if (!modelPath.trim()) {
      setError('Model path is required.');
      return;
    }

    const apiPrompts = prompts
      .filter((p) => p.text.trim())
      .map((p) => ({ text: p.text.trim() }));

    setRunning(true);
    setError(null);

    try {
      const res = await api.segmentPanorama(panoramaId, {
        model_path: modelPath.trim(),
        prompts: apiPrompts.length > 0 ? apiPrompts : undefined,
        confidence,
        n_yaw: nYaw,
        save: true,
      });
      setResult(res);
      onSegmented?.();
    } catch (err) {
      if (err instanceof ApiRequestError) {
        if (err.status === 503) {
          setError(
            'No SAM3 panorama segmenter is registered on this server. ' +
              'The server must be started via `rux gui` and a SAM3 model must be accessible at the given path.',
          );
        } else if (err.status === 409) {
          setError(
            'A pipeline job is currently running and holds the write lock. ' +
              'Wait for it to finish, then try again.',
          );
        } else {
          setError(err.message);
        }
      } else {
        setError(String(err));
      }
    } finally {
      setRunning(false);
    }
  };

  return (
    <div className={styles.panel}>
      {/* ---- model path ---- */}
      <section className={styles.section}>
        <label
          className={styles.fieldLabel}
          htmlFor={`pano-seg-model-${panoramaId}`}
        >
          SAM3 model path
          <span className={styles.required} aria-hidden="true">
            {' '}
            *
          </span>
        </label>
        <input
          id={`pano-seg-model-${panoramaId}`}
          className={styles.textInput}
          type="text"
          placeholder="/path/to/sam3-engines  (server-side)"
          value={modelPath}
          onChange={(e) => setModelPath(e.target.value)}
          disabled={running}
          autoComplete="off"
          spellCheck={false}
        />
        <p className={styles.hint}>
          Server-side path to a TensorRT engine directory or{' '}
          <code className={styles.code}>.onnx</code> file.
        </p>
      </section>

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
      <button
        type="button"
        className={styles.runBtn}
        onClick={handleRun}
        disabled={running || !modelPath.trim()}
        aria-busy={running}
      >
        {running ? 'Running…' : 'Run segmentation'}
      </button>

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
            <div className={styles.nextSteps}>
              <p className={styles.nextStepsHead}>Next steps</p>
              <p className={styles.hint}>
                The equirect label map is stored. Use{' '}
                <code className={styles.code}>rux create annotate-360</code> to
                process all panoramas in batch, or run the annotate-360 stage
                from the Pipeline page.
              </p>
            </div>
          )}
        </section>
      )}
    </div>
  );
}
