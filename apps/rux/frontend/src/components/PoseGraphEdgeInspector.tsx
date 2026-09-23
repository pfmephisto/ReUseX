// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pose-graph edge inspector panel (#407).
 *
 * Shows the properties of the currently selected edge (from, to, type,
 * residual, weight) with frame thumbnails and Delete / Add-edge actions.
 * The "re-run optimizer" prompt is surfaced here too — it directs the user
 * to the CLI because `optimize` is not yet a GUI job stage.
 */

import { useState, useCallback } from 'react';
import { api } from '../api/client';
import type { PoseGraphEdgeType } from '../api/types';
import type { SelectedEdge } from '../viewport/PoseGraphScene';
import styles from './PoseGraphEdgeInspector.module.css';

export interface PoseGraphEdgeInspectorProps {
  edge: SelectedEdge | null;
  /** Called after a successful delete so the parent can refresh the graph. */
  onDeleted: () => void;
  /** Called after a successful add so the parent can refresh the graph. */
  onAdded: () => void;
  /** Called to clear the selection. */
  onDeselect: () => void;
}

const EDGE_TYPE_LABELS: Record<PoseGraphEdgeType, string> = {
  odometry: 'Odometry',
  loop_closure: 'Loop closure',
  panorama: 'Panorama',
};

export function PoseGraphEdgeInspector({
  edge,
  onDeleted,
  onAdded,
  onDeselect,
}: PoseGraphEdgeInspectorProps) {
  const [deleting, setDeleting] = useState(false);
  const [deleteError, setDeleteError] = useState<string | null>(null);
  const [addOpen, setAddOpen] = useState(false);

  const handleDelete = useCallback(async () => {
    if (!edge) return;
    setDeleting(true);
    setDeleteError(null);
    try {
      await api.deletePoseGraphEdge(edge.from, edge.to, edge.type);
      onDeleted();
      onDeselect();
    } catch (err) {
      setDeleteError(err instanceof Error ? err.message : String(err));
    } finally {
      setDeleting(false);
    }
  }, [edge, onDeleted, onDeselect]);

  if (!edge) {
    return (
      <div className={styles.empty}>
        <p className={styles.emptyText}>Click an edge to inspect it.</p>
        <button
          type="button"
          className={styles.addBtn}
          onClick={() => setAddOpen(true)}
        >
          + Add manual edge
        </button>
        {addOpen && (
          <AddEdgeForm
            onAdded={() => {
              onAdded();
              setAddOpen(false);
            }}
            onCancel={() => setAddOpen(false)}
          />
        )}
      </div>
    );
  }

  return (
    <div className={styles.inspector}>
      <div className={styles.header}>
        <span className={styles.typeTag}>{EDGE_TYPE_LABELS[edge.type]}</span>
        <button
          type="button"
          className={styles.deselectBtn}
          onClick={onDeselect}
          aria-label="Clear selection"
        >
          ✕
        </button>
      </div>

      <div className={styles.fields}>
        <div className={styles.fieldRow}>
          <span className={styles.fieldKey}>From</span>
          <span className={`${styles.fieldVal} mono`}>{edge.from}</span>
        </div>
        <div className={styles.fieldRow}>
          <span className={styles.fieldKey}>To</span>
          <span className={`${styles.fieldVal} mono`}>{edge.to}</span>
        </div>
        <div className={styles.fieldRow}>
          <span className={styles.fieldKey}>Residual</span>
          <span className={`${styles.fieldVal} mono`}>{edge.residual.toFixed(4)}</span>
        </div>
        {edge.weight !== undefined && (
          <div className={styles.fieldRow}>
            <span className={styles.fieldKey}>Weight</span>
            <span className={`${styles.fieldVal} mono`}>{edge.weight.toFixed(3)}</span>
          </div>
        )}
      </div>

      <div className={styles.thumbs}>
        <FrameThumb id={edge.from} label={`Frame ${edge.from}`} />
        <FrameThumb id={edge.to} label={`Frame ${edge.to}`} />
      </div>

      {deleteError && <p className={styles.error}>{deleteError}</p>}

      <div className={styles.actions}>
        <button
          type="button"
          className={styles.deleteBtn}
          onClick={handleDelete}
          disabled={deleting}
        >
          {deleting ? 'Deleting…' : 'Delete edge'}
        </button>
        <button
          type="button"
          className={styles.addBtn}
          onClick={() => setAddOpen(true)}
        >
          + Add manual edge
        </button>
      </div>

      {addOpen && (
        <AddEdgeForm
          defaultFrom={edge.from}
          defaultTo={edge.to}
          onAdded={() => {
            onAdded();
            setAddOpen(false);
          }}
          onCancel={() => setAddOpen(false)}
        />
      )}

      <div className={styles.optimizeHint}>
        <p className={styles.hintText}>
          After editing edges, re-run the optimizer to recalculate residuals
          and update frame poses:
        </p>
        <code className={styles.cliCmd}>rux optimize</code>
      </div>
    </div>
  );
}

// -------------------------------------------------------------------------- //

interface FrameThumbProps {
  id: number;
  label: string;
}

function FrameThumb({ id, label }: FrameThumbProps) {
  const [failed, setFailed] = useState(false);
  const src = api.frameImageUrl(id, 'color', { maxSize: 80 });
  return (
    <div className={styles.thumb}>
      {failed ? (
        <span className={styles.thumbMissing}>?</span>
      ) : (
        <img
          className={styles.thumbImg}
          src={src}
          alt={label}
          decoding="async"
          onError={() => setFailed(true)}
        />
      )}
      <span className={`${styles.thumbLabel} mono`}>{id}</span>
    </div>
  );
}

// -------------------------------------------------------------------------- //

const EDGE_TYPES: PoseGraphEdgeType[] = ['loop_closure', 'odometry', 'panorama'];

interface AddEdgeFormProps {
  defaultFrom?: number;
  defaultTo?: number;
  onAdded: () => void;
  onCancel: () => void;
}

function AddEdgeForm({ defaultFrom, defaultTo, onAdded, onCancel }: AddEdgeFormProps) {
  const [from, setFrom] = useState<string>(defaultFrom !== undefined ? String(defaultFrom) : '');
  const [to, setTo] = useState<string>(defaultTo !== undefined ? String(defaultTo) : '');
  const [type, setType] = useState<PoseGraphEdgeType>('loop_closure');
  const [weight, setWeight] = useState('1.0');
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = useCallback(async () => {
    const fromId = parseInt(from, 10);
    const toId = parseInt(to, 10);
    if (isNaN(fromId) || isNaN(toId)) {
      setError('Frame IDs must be integers.');
      return;
    }
    const w = parseFloat(weight);
    if (isNaN(w) || w <= 0) {
      setError('Weight must be a positive number.');
      return;
    }
    setSubmitting(true);
    setError(null);
    try {
      await api.addPoseGraphEdge({ from: fromId, to: toId, type, weight: w });
      onAdded();
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err));
    } finally {
      setSubmitting(false);
    }
  }, [from, to, type, weight, onAdded]);

  return (
    <div className={styles.addForm}>
      <div className={styles.fieldRow}>
        <label className={styles.fieldKey} htmlFor="add-edge-from">From</label>
        <input
          id="add-edge-from"
          type="text"
          inputMode="numeric"
          className={`${styles.addInput} mono`}
          value={from}
          placeholder="Frame ID"
          onChange={(e) => setFrom(e.target.value)}
        />
      </div>
      <div className={styles.fieldRow}>
        <label className={styles.fieldKey} htmlFor="add-edge-to">To</label>
        <input
          id="add-edge-to"
          type="text"
          inputMode="numeric"
          className={`${styles.addInput} mono`}
          value={to}
          placeholder="Frame ID"
          onChange={(e) => setTo(e.target.value)}
        />
      </div>
      <div className={styles.fieldRow}>
        <label className={styles.fieldKey} htmlFor="add-edge-type">Type</label>
        <select
          id="add-edge-type"
          className={styles.addSelect}
          value={type}
          onChange={(e) => setType(e.target.value as PoseGraphEdgeType)}
        >
          {EDGE_TYPES.map((t) => (
            <option key={t} value={t}>{t}</option>
          ))}
        </select>
      </div>
      <div className={styles.fieldRow}>
        <label className={styles.fieldKey} htmlFor="add-edge-weight">Weight</label>
        <input
          id="add-edge-weight"
          type="text"
          inputMode="decimal"
          className={`${styles.addInput} mono`}
          value={weight}
          onChange={(e) => setWeight(e.target.value)}
        />
      </div>
      {error && <p className={styles.error}>{error}</p>}
      <div className={styles.formActions}>
        <button
          type="button"
          className={styles.submitBtn}
          onClick={handleSubmit}
          disabled={submitting}
        >
          {submitting ? 'Adding…' : 'Add edge'}
        </button>
        <button type="button" className={styles.cancelBtn} onClick={onCancel}>
          Cancel
        </button>
      </div>
    </div>
  );
}
