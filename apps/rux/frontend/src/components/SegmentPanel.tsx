// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Interactive SAM3 segmentation panel for one sensor frame (#409 / #466).
 *
 * The user draws bounding boxes on the colour image to create prompts, then
 * fires the POST /frames/{id}/segment endpoint.  A successful run stores the
 * label mask in the project and shows a colourised overlay.
 *
 * Point-click emulation: SAM3 has no native point prompts, so a click that
 * doesn't drag is converted to a 16 × 16 box centred on the click.
 *
 * After a successful save the panel suggests the two follow-up pipeline stages
 * (`create project` and `create instances`) that turn the 2D mask into 3D
 * instance clouds.  Actual per-frame projection is a follow-up to #409.
 */

import { useCallback, useEffect, useRef, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import type { FrameSegmentResult } from '../api/types';
import {
  buildPrompt,
  canvasToImageBox,
  clickToDisplayBox,
  normalizeDisplayBox,
  type DisplayBox,
} from '../data/segmentPrompts';
import styles from './SegmentPanel.module.css';

// Module-level counter avoids React key collisions across add/remove cycles.
let _nextId = 0;
const genId = () => String(++_nextId);

interface UIPrompt {
  id: string;
  /** Class name typed by the user; may be empty for unlabelled box regions. */
  text: string;
  /** Drawn region in canvas display pixels, null for text-only prompts. */
  displayBox: DisplayBox | null;
}

export interface SegmentPanelProps {
  frameId: number;
  /** From FrameInfo.intrinsics — used for coordinate conversion. */
  imageWidth?: number;
  imageHeight?: number;
  /** Called after a successful save so the parent can refresh frame metadata. */
  onSegmented?: () => void;
}

export function SegmentPanel({
  frameId,
  imageWidth,
  imageHeight,
  onSegmented,
}: SegmentPanelProps) {
  const [modelPath, setModelPath] = useState('');
  const [prompts, setPrompts] = useState<UIPrompt[]>([]);
  const [confidence, setConfidence] = useState(0.5);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [result, setResult] = useState<FrameSegmentResult | null>(null);
  const [maskVersion, setMaskVersion] = useState(0);
  const [showMask, setShowMask] = useState(false);

  const imgRef = useRef<HTMLImageElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Drag state lives in a ref so mouse handlers can read it without stale
  // closures; drawing is triggered via the draw() call, not state updates.
  const dragRef = useRef<{
    start: { x: number; y: number };
    current: { x: number; y: number };
  } | null>(null);

  // The latest prompts are needed inside draw() which is called from
  // syncCanvasSize (a stable callback). Using a ref avoids re-creating
  // syncCanvasSize on every prompt change.
  const promptsRef = useRef<UIPrompt[]>(prompts);
  promptsRef.current = prompts;

  // ---------------------------------------------------------------- canvas --

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || canvas.width === 0) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    for (const p of promptsRef.current) {
      if (!p.displayBox) continue;
      const { x1, y1, x2, y2 } = p.displayBox;
      ctx.strokeStyle = 'rgba(110, 168, 254, 0.9)';
      ctx.lineWidth = 2;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      if (p.text) {
        ctx.fillStyle = 'rgba(110, 168, 254, 0.9)';
        ctx.font = '11px monospace';
        // Keep label inside the canvas top edge.
        ctx.fillText(p.text, x1 + 3, Math.max(y1 - 3, 12));
      }
    }

    const drag = dragRef.current;
    if (drag) {
      const { x1, y1, x2, y2 } = normalizeDisplayBox({
        x1: drag.start.x,
        y1: drag.start.y,
        x2: drag.current.x,
        y2: drag.current.y,
      });
      ctx.strokeStyle = 'rgba(255, 200, 0, 0.9)';
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      ctx.setLineDash([]);
    }
  }, []); // stable — reads state via refs

  // Match the canvas resolution (attributes) to the image's displayed size.
  const syncCanvasSize = useCallback(() => {
    const img = imgRef.current;
    const canvas = canvasRef.current;
    if (!img || !canvas || img.clientWidth === 0) return;
    canvas.width = img.clientWidth;
    canvas.height = img.clientHeight;
    draw();
  }, [draw]);

  // Keep canvas in sync with layout changes (responsive resize).
  useEffect(() => {
    const img = imgRef.current;
    if (!img) return;
    const observer = new ResizeObserver(syncCanvasSize);
    observer.observe(img);
    return () => observer.disconnect();
  }, [syncCanvasSize]);

  // Redraw whenever the prompt list changes.
  useEffect(() => {
    draw();
  }, [prompts, draw]);

  // ----------------------------------------------------------- mouse events --

  const getPos = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const rect = canvasRef.current!.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  };

  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const pos = getPos(e);
    dragRef.current = { start: pos, current: pos };
    draw();
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current) return;
    dragRef.current.current = getPos(e);
    draw();
  };

  const handleMouseUp = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const drag = dragRef.current;
    if (!drag) return;
    dragRef.current = null;

    const end = getPos(e);
    const dx = Math.abs(end.x - drag.start.x);
    const dy = Math.abs(end.y - drag.start.y);

    const displayBox =
      dx < 4 && dy < 4
        ? clickToDisplayBox(drag.start.x, drag.start.y)
        : normalizeDisplayBox({
            x1: drag.start.x,
            y1: drag.start.y,
            x2: end.x,
            y2: end.y,
          });

    setPrompts((prev) => [...prev, { id: genId(), text: '', displayBox }]);
  };

  const handleMouseLeave = () => {
    if (!dragRef.current) return;
    dragRef.current = null;
    draw();
  };

  // --------------------------------------------------------- prompt actions --

  const removePrompt = (id: string) =>
    setPrompts((prev) => prev.filter((p) => p.id !== id));

  const updateText = (id: string, text: string) =>
    setPrompts((prev) => prev.map((p) => (p.id === id ? { ...p, text } : p)));

  const addTextPrompt = () =>
    setPrompts((prev) => [...prev, { id: genId(), text: '', displayBox: null }]);

  // ------------------------------------------------------------------- run --

  const handleRun = async () => {
    if (!modelPath.trim()) {
      setError('Model path is required.');
      return;
    }

    const img = imgRef.current;
    const displayW = img?.clientWidth ?? 1;
    const displayH = img?.clientHeight ?? 1;
    const imgW = imageWidth ?? img?.naturalWidth ?? 1;
    const imgH = imageHeight ?? img?.naturalHeight ?? 1;

    const apiPrompts = prompts
      .filter((p) => p.text.trim() || p.displayBox)
      .map((p) => {
        const boxes: [number, number, number, number][] = [];
        if (p.displayBox) {
          boxes.push(canvasToImageBox(p.displayBox, displayW, displayH, imgW, imgH));
        }
        return buildPrompt(p.text.trim(), boxes);
      });

    setRunning(true);
    setError(null);

    try {
      const res = await api.segmentFrame(frameId, {
        model_path: modelPath.trim(),
        prompts: apiPrompts.length > 0 ? apiPrompts : undefined,
        confidence,
        save: true,
      });
      setResult(res);
      if (res.saved) {
        setMaskVersion((v) => v + 1);
        setShowMask(true);
      }
      onSegmented?.();
    } catch (err) {
      if (err instanceof ApiRequestError) {
        if (err.status === 503) {
          setError(
            'No SAM3 segmenter is registered on this server. ' +
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

  // Cache-bust the mask URL on every successful run so the browser fetches the
  // new mask rather than reusing the cached (possibly stale) segmentation image.
  const maskUrl = `${api.frameImageUrl(frameId, 'segmentation', { normalize: true })}&_v=${maskVersion}`;

  // ---------------------------------------------------------------- render --

  return (
    <div className={styles.panel}>
      {/* ---- model path ---- */}
      <section className={styles.section}>
        <label className={styles.fieldLabel} htmlFor={`seg-model-${frameId}`}>
          SAM3 model path
          <span className={styles.required} aria-hidden="true"> *</span>
        </label>
        <input
          id={`seg-model-${frameId}`}
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

      {/* ---- interactive image + canvas ---- */}
      <section className={styles.section}>
        <h4 className={styles.sectionHead}>Draw regions</h4>
        <p className={styles.hint}>
          Drag to draw a box · Click to place a point (16 × 16 px). Each region
          becomes one segmentation prompt.
        </p>
        <div className={styles.imageWrap}>
          <img
            ref={imgRef}
            className={styles.frameImage}
            src={api.frameImageUrl(frameId, 'color')}
            alt={`Colour image of frame ${frameId}`}
            onLoad={syncCanvasSize}
            draggable={false}
          />
          {showMask && result?.saved && (
            <img
              className={styles.maskOverlay}
              src={maskUrl}
              alt="Segmentation mask overlay"
              draggable={false}
            />
          )}
          <canvas
            ref={canvasRef}
            className={styles.canvas}
            aria-label="Draw bounding-box regions to guide segmentation"
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={handleMouseLeave}
          />
        </div>
        {result && (
          <label className={styles.toggle}>
            <input
              type="checkbox"
              checked={showMask}
              onChange={(e) => setShowMask(e.target.checked)}
              disabled={!result.saved}
            />
            <span>Show segmentation mask</span>
          </label>
        )}
      </section>

      {/* ---- prompt list ---- */}
      <section className={styles.section}>
        <h4 className={styles.sectionHead}>Prompts</h4>
        {prompts.length === 0 ? (
          <p className={styles.hint}>
            No prompts yet — draw a region above or{' '}
            <button
              type="button"
              className={styles.inlineAction}
              onClick={addTextPrompt}
              disabled={running}
            >
              add a text prompt
            </button>
            . Leave empty to use the model's built-in class list.
          </p>
        ) : (
          <>
            <ul className={styles.promptList}>
              {prompts.map((p) => (
                <li key={p.id} className={styles.promptRow}>
                  <span
                    className={styles.promptKind}
                    title={p.displayBox ? 'Box region' : 'Text only'}
                    aria-hidden="true"
                  >
                    {p.displayBox ? '▭' : '⌨'}
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
            <button
              type="button"
              className={styles.addAction}
              onClick={addTextPrompt}
              disabled={running}
            >
              + Add text-only prompt
            </button>
          </>
        )}
      </section>

      {/* ---- confidence slider ---- */}
      <section className={styles.section}>
        <label className={styles.fieldLabel} htmlFor={`seg-conf-${frameId}`}>
          Confidence:{' '}
          <span className="mono">{confidence.toFixed(2)}</span>
        </label>
        <input
          id={`seg-conf-${frameId}`}
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
                The mask is stored. Project it into 3D and create instances:
              </p>
              <pre className={styles.cmdBlock}>
                {'rux create project\nrux create instances'}
              </pre>
              <p className={styles.hint}>
                Or use the{' '}
                <a href="/pipeline" className={styles.pipelineLink}>
                  Pipeline page
                </a>{' '}
                to run those stages from the GUI.
              </p>
            </div>
          )}
        </section>
      )}
    </div>
  );
}
