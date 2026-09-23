// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';
import { useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import { parseFrameId } from '../data/framePair';
import { FramePairInspector } from '../components/FramePairInspector';
import styles from './GraphViewPage.module.css';

/**
 * The frame-pair inspector page (issue #446, sub-issue of #407).
 *
 * ## Layout
 *
 * A compact picker bar at the top lets the user select two frame IDs (`?a=`
 * and `?b=`). The main area is the `FramePairInspector`, which shows both
 * frames side by side (or overlaid) with image-kind selection, overlay
 * controls, and the descriptor-similarity panel.
 *
 * ## Selection model
 *
 * Frame IDs live in URL search params so every comparison state is a
 * shareable, reloadable link — the same convention the frames page uses for
 * `?frame=` and the viewport uses for `?cloud=`.
 *
 * ## Pose-graph integration (deferred)
 *
 * The long-term UX (from #407) is to click a pose-graph edge in the viewport
 * and have that edge's two endpoint frames load here automatically.  That
 * requires 3D picking plumbing in `PoseGraphScene.ts` and a cross-route
 * communication channel (event, context, or shared URL segment) that are
 * out of scope for this issue.  For now the user types or pastes frame IDs
 * directly.
 */
export function GraphViewPage() {
  const [params, setParams] = useSearchParams();

  const frameA = parseFrameId(params.get('a'));
  const frameB = parseFrameId(params.get('b'));

  const setParam = useCallback(
    (key: string, value: string | null) => {
      const next = new URLSearchParams(params);
      if (value === null) next.delete(key);
      else next.set(key, value);
      setParams(next, { replace: true });
    },
    [params, setParams],
  );

  const handleSwap = useCallback(() => {
    const next = new URLSearchParams(params);
    const a = params.get('a');
    const b = params.get('b');
    if (a !== null) next.set('b', a); else next.delete('b');
    if (b !== null) next.set('a', b); else next.delete('a');
    setParams(next, { replace: true });
  }, [params, setParams]);

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <h1 className={styles.title}>Graph View</h1>
        <div className={styles.pickerRow}>
          {/* key=frameId remounts the picker when the URL changes externally
              (e.g. Swap), resetting the draft to the new value. */}
          <FramePicker
            key={`a-${frameA ?? 'none'}`}
            label="Frame A"
            frameId={frameA}
            onSet={(v) => setParam('a', v)}
          />
          <button
            type="button"
            className={styles.swapBtn}
            onClick={handleSwap}
            title="Swap A and B"
            aria-label="Swap frame A and frame B"
          >
            ⇌
          </button>
          <FramePicker
            key={`b-${frameB ?? 'none'}`}
            label="Frame B"
            frameId={frameB}
            onSet={(v) => setParam('b', v)}
          />
        </div>
      </header>

      <div className={styles.body}>
        <FramePairInspector frameA={frameA} frameB={frameB} />
      </div>
    </div>
  );
}

// -------------------------------------------------------------------------- //

interface FramePickerProps {
  label: string;
  frameId: number | null;
  onSet: (value: string | null) => void;
}

/**
 * A compact frame picker: a text input + live thumbnail preview.
 *
 * The draft state is initialized from `frameId` at mount time. The parent
 * remounts this component (via `key`) whenever `frameId` changes externally
 * (e.g. the Swap button), resetting the draft. This avoids reading
 * `document.activeElement` during render or calling a setState mid-render.
 *
 * The thumbnail loads directly from the API for the draft value (once it
 * parses as a non-negative integer). A 404 is silent — the missing thumbnail
 * already communicates that the frame doesn't exist.
 */
function FramePicker({ label, frameId, onSet }: FramePickerProps) {
  const [draft, setDraft] = useState<string>(frameId !== null ? String(frameId) : '');
  const [thumbFailed, setThumbFailed] = useState(false);

  const commit = useCallback(
    (raw: string) => {
      const trimmed = raw.trim();
      const id = parseFrameId(trimmed);
      onSet(id !== null ? String(id) : null);
    },
    [onSet],
  );

  const previewId = parseFrameId(draft);

  return (
    <div className={styles.picker}>
      <label className={styles.pickerLabel} htmlFor={`picker-${label}`}>
        {label}
      </label>
      <div className={styles.pickerInputRow}>
        <input
          id={`picker-${label}`}
          type="text"
          inputMode="numeric"
          className={`${styles.pickerInput} mono`}
          value={draft}
          placeholder="Frame ID"
          onChange={(e) => {
            setDraft(e.target.value);
            setThumbFailed(false);
          }}
          onBlur={(e) => commit(e.target.value)}
          onKeyDown={(e) => {
            if (e.key === 'Enter') commit((e.target as HTMLInputElement).value);
          }}
        />
        {previewId !== null && (
          <div className={styles.pickerThumb}>
            {thumbFailed ? (
              <span className={styles.pickerThumbMissing}>?</span>
            ) : (
              <img
                className={styles.pickerThumbImg}
                src={api.frameImageUrl(previewId, 'color', { maxSize: 80 })}
                alt={`Frame ${previewId}`}
                decoding="async"
                onError={() => setThumbFailed(true)}
              />
            )}
          </div>
        )}
      </div>
    </div>
  );
}
