// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Combined label library + annotation queue panel (#447).
 *
 * Two sections:
 *  1. Label library — a persistent, session-scoped list of named labels that
 *     the user curates once and reuses across frames.
 *  2. Labelling queue — the staged (frame, prompts) jobs; a manual "Run" button
 *     processes them sequentially against `POST /frames/{id}/segment`.
 *
 * The run must NOT start automatically — it only fires on explicit user action,
 * because a full scan can have hundreds of frames and inference is slow.
 * A 503 response means the server has no segmenter; the run stops immediately
 * with a clear message rather than failing every remaining item.
 *
 * Follow-up: for hundreds of frames a server-side batch job (`rux create
 * annotate` already does this) would be materially better — the client-
 * orchestrated loop here is a convenience for small targeted batches.
 */

import { useCallback, useRef, useState } from 'react';

import { ApiRequestError, api } from '../api/client';
import { useLabelQueue } from '../app/LabelQueueContext';
import type { QueueItem } from '../data/labelQueue';
import styles from './LabelQueuePanel.module.css';

// ------------------------------------------------------------------ panel --

export function LabelQueuePanel() {
  return (
    <aside className={styles.panel} aria-label="Label queue">
      <LabelLibrarySection />
      <hr className={styles.divider} />
      <QueueSection />
    </aside>
  );
}

// ------------------------------------------------------ label library UI --

function LabelLibrarySection() {
  const { labels, addLabel, removeLabel, updateLabel } = useLabelQueue();
  const [newText, setNewText] = useState('');
  const [newConf, setNewConf] = useState('');
  const [editingId, setEditingId] = useState<string | null>(null);
  const [editText, setEditText] = useState('');
  const [editConf, setEditConf] = useState('');

  const handleAdd = () => {
    const text = newText.trim();
    if (!text) return;
    const conf = newConf !== '' ? parseFloat(newConf) : undefined;
    addLabel(text, conf !== undefined && !isNaN(conf) ? conf : undefined);
    setNewText('');
    setNewConf('');
  };

  const startEdit = (label: ReturnType<typeof useLabelQueue>['labels'][number]) => {
    setEditingId(label.id);
    setEditText(label.text);
    setEditConf(label.confidence !== undefined ? String(label.confidence) : '');
  };

  const commitEdit = (id: string) => {
    const text = editText.trim();
    if (!text) return;
    const conf = editConf !== '' ? parseFloat(editConf) : undefined;
    updateLabel(id, text, conf !== undefined && !isNaN(conf) ? conf : undefined);
    setEditingId(null);
  };

  const cancelEdit = () => setEditingId(null);

  return (
    <section className={styles.section}>
      <h3 className={styles.sectionHead}>Label library</h3>
      <p className={styles.hint}>
        Named labels that persist in this browser session. Use them as prompts
        when queuing frames for annotation.
      </p>

      {labels.length === 0 ? (
        <p className={styles.empty}>No labels yet — add one below.</p>
      ) : (
        <ul className={styles.labelList}>
          {labels.map((label) => (
            <li key={label.id} className={styles.labelRow}>
              {editingId === label.id ? (
                <>
                  <input
                    className={styles.labelInput}
                    type="text"
                    value={editText}
                    onChange={(e) => setEditText(e.target.value)}
                    onKeyDown={(e) => {
                      if (e.key === 'Enter') commitEdit(label.id);
                      if (e.key === 'Escape') cancelEdit();
                    }}
                    autoFocus
                    aria-label="Label text"
                  />
                  <input
                    className={styles.confInput}
                    type="number"
                    min={0}
                    max={1}
                    step={0.05}
                    placeholder="conf"
                    value={editConf}
                    onChange={(e) => setEditConf(e.target.value)}
                    aria-label="Confidence override"
                  />
                  <button
                    type="button"
                    className={styles.actionBtn}
                    onClick={() => commitEdit(label.id)}
                  >
                    Save
                  </button>
                  <button type="button" className={styles.removeBtn} onClick={cancelEdit}>
                    ✕
                  </button>
                </>
              ) : (
                <>
                  <span className={styles.labelText}>{label.text}</span>
                  {label.confidence !== undefined && (
                    <span className={styles.confBadge} title="Confidence override">
                      {label.confidence.toFixed(2)}
                    </span>
                  )}
                  <button
                    type="button"
                    className={styles.actionBtn}
                    onClick={() => startEdit(label)}
                  >
                    Edit
                  </button>
                  <button
                    type="button"
                    className={styles.removeBtn}
                    onClick={() => removeLabel(label.id)}
                    aria-label={`Remove label "${label.text}"`}
                  >
                    ✕
                  </button>
                </>
              )}
            </li>
          ))}
        </ul>
      )}

      <div className={styles.addRow}>
        <input
          className={styles.labelInput}
          type="text"
          placeholder="Label name, e.g. wall"
          value={newText}
          onChange={(e) => setNewText(e.target.value)}
          onKeyDown={(e) => { if (e.key === 'Enter') handleAdd(); }}
          aria-label="New label name"
        />
        <input
          className={styles.confInput}
          type="number"
          min={0}
          max={1}
          step={0.05}
          placeholder="conf"
          value={newConf}
          onChange={(e) => setNewConf(e.target.value)}
          aria-label="New label confidence (optional)"
        />
        <button
          type="button"
          className={styles.actionBtn}
          onClick={handleAdd}
          disabled={!newText.trim()}
        >
          Add
        </button>
      </div>
    </section>
  );
}

// ------------------------------------------------------------ queue UI --

function QueueSection() {
  const { items, dequeue, clearFinished, setItemStatus, pendingCount } = useLabelQueue();

  const [isRunning, setIsRunning] = useState(false);
  const [runError, setRunError] = useState<string | null>(null);
  const [progress, setProgress] = useState<{ done: number; total: number } | null>(null);
  const cancelRef = useRef(false);

  const handleRun = useCallback(async () => {
    if (isRunning) return;
    cancelRef.current = false;
    setIsRunning(true);
    setRunError(null);

    const pending = items.filter((i) => i.status === 'pending');
    setProgress({ done: 0, total: pending.length });

    let done = 0;
    for (const item of pending) {
      if (cancelRef.current) break;

      setItemStatus(item.id, 'running');

      try {
        const result = await api.segmentFrame(item.frameId, {
          model_path: item.modelPath,
          prompts: item.prompts.length > 0 ? item.prompts : undefined,
          confidence: item.confidence,
          save: true,
        });
        setItemStatus(item.id, 'done', { labeledPixels: result.labeled_pixels });
        done++;
        setProgress({ done, total: pending.length });
      } catch (err) {
        if (err instanceof ApiRequestError) {
          if (err.status === 503) {
            setItemStatus(item.id, 'failed', {
              error: 'No SAM3 segmenter registered.',
            });
            setRunError(
              'No SAM3 segmenter registered on this server. ' +
                'Start `rux gui` with the SAM3 model accessible at the path recorded in each queued item.',
            );
            break;
          } else if (err.status === 409) {
            setItemStatus(item.id, 'failed', {
              error: 'Pipeline job holds the write lock.',
            });
            setRunError(
              'A pipeline job is currently running and holds the write lock. ' +
                'Wait for it to finish, then run the queue again.',
            );
            break;
          } else {
            setItemStatus(item.id, 'failed', { error: err.message });
            done++;
            setProgress({ done, total: pending.length });
            // Continue to the next item for other errors.
          }
        } else {
          setItemStatus(item.id, 'failed', { error: String(err) });
          done++;
          setProgress({ done, total: pending.length });
        }
      }
    }

    setIsRunning(false);
    setProgress(null);
  }, [isRunning, items, setItemStatus]);

  const handleCancel = () => {
    cancelRef.current = true;
  };

  const doneCount = items.filter((i) => i.status === 'done').length;
  const failedCount = items.filter((i) => i.status === 'failed').length;

  return (
    <section className={styles.section}>
      <div className={styles.queueHead}>
        <h3 className={styles.sectionHead}>
          Queue
          {items.length > 0 && (
            <span className={styles.badge}>
              {pendingCount} pending · {items.length} total
            </span>
          )}
        </h3>

        {doneCount > 0 && (
          <button
            type="button"
            className={styles.clearBtn}
            onClick={clearFinished}
            disabled={isRunning}
          >
            Clear done ({doneCount})
          </button>
        )}
      </div>

      {/* ---- run controls ---- */}
      {items.length > 0 && (
        <div className={styles.runBar}>
          {isRunning ? (
            <>
              {progress && (
                <span className={styles.progressText} aria-live="polite">
                  Processing {progress.done} / {progress.total}…
                </span>
              )}
              <button
                type="button"
                className={styles.cancelBtn}
                onClick={handleCancel}
              >
                Cancel
              </button>
            </>
          ) : (
            <button
              type="button"
              className={styles.runBtn}
              onClick={handleRun}
              disabled={pendingCount === 0}
              aria-busy={isRunning}
            >
              {pendingCount === 0
                ? 'Nothing to run'
                : `Run queue (${pendingCount} pending)`}
            </button>
          )}
        </div>
      )}

      {/* ---- run error ---- */}
      {runError && !isRunning && (
        <p className={styles.runError} role="alert">
          {runError}
        </p>
      )}

      {/* ---- items ---- */}
      {items.length === 0 ? (
        <div className={styles.emptyQueue}>
          <p className={styles.empty}>No frames queued.</p>
          <p className={styles.hint}>
            Select a frame, draw prompts in the Segment tab, and click{' '}
            <strong>Add to queue</strong>.
          </p>
        </div>
      ) : (
        <ul className={styles.itemList}>
          {items.map((item) => (
            <QueueItemRow
              key={item.id}
              item={item}
              onRemove={dequeue}
              disabled={isRunning}
            />
          ))}
        </ul>
      )}

      {/* ---- summary ---- */}
      {items.length > 0 && !isRunning && (doneCount > 0 || failedCount > 0) && (
        <p className={styles.summary}>
          {doneCount > 0 && `${doneCount} completed`}
          {doneCount > 0 && failedCount > 0 && ' · '}
          {failedCount > 0 && `${failedCount} failed`}
        </p>
      )}
    </section>
  );
}

// -------------------------------------------------------- queue item row --

interface QueueItemRowProps {
  item: QueueItem;
  onRemove: (id: string) => void;
  disabled: boolean;
}

function QueueItemRow({ item, onRemove, disabled }: QueueItemRowProps) {
  const promptNames = item.prompts
    .map((p) => p.text.trim())
    .filter(Boolean)
    .join(', ');

  return (
    <li className={`${styles.itemRow} ${styles[`item_${item.status}`]}`}>
      <div className={styles.itemMain}>
        <span className={styles.itemFrameId} title={`Frame ${item.frameId}`}>
          #{item.frameId}
        </span>
        <span className={styles.itemPrompts} title={promptNames || '(model defaults)'}>
          {promptNames || <em>model defaults</em>}
        </span>
        <StatusBadge status={item.status} />
      </div>

      {item.status === 'done' && item.labeledPixels !== undefined && (
        <span className={styles.itemDetail}>
          {item.labeledPixels.toLocaleString()} px labeled
        </span>
      )}

      {item.status === 'failed' && item.error && (
        <span className={styles.itemError} title={item.error}>
          {item.error}
        </span>
      )}

      <button
        type="button"
        className={styles.removeBtn}
        onClick={() => onRemove(item.id)}
        disabled={disabled || item.status === 'running'}
        aria-label={`Remove frame ${item.frameId} from queue`}
      >
        ✕
      </button>
    </li>
  );
}

function StatusBadge({ status }: { status: QueueItem['status'] }) {
  const labels: Record<QueueItem['status'], string> = {
    pending: 'pending',
    running: 'running…',
    done: 'done',
    failed: 'failed',
  };
  return (
    <span
      className={`${styles.statusBadge} ${styles[`status_${status}`]}`}
      aria-label={`Status: ${labels[status]}`}
    >
      {labels[status]}
    </span>
  );
}
