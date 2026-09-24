// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pose-graph edge inspector panel (#407, #464).
 *
 * Shows the properties of the currently selected edge (from, to, type,
 * residual, weight) with frame thumbnails and Delete / Add-edge actions.
 * Includes a "Re-run optimize" button that submits an optimize job and
 * reflects progress via JobsContext.
 */

import { useState, useCallback } from 'react';
import { api } from '../api/client';
import { useJobs } from '../app/JobsContext';
import type { IcpRefineResult, PoseGraphEdgeType } from '../api/types';
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
  const [optimizing, setOptimizing] = useState(false);
  const [optimizeError, setOptimizeError] = useState<string | null>(null);
  const { active } = useJobs();

  const isJobActive = active.some((j) => j.stage === 'optimize');

  const handleOptimize = useCallback(async () => {
    setOptimizing(true);
    setOptimizeError(null);
    try {
      await api.submitJob({ stage: 'optimize' });
    } catch (err) {
      setOptimizeError(err instanceof Error ? err.message : String(err));
    } finally {
      setOptimizing(false);
    }
  }, []);

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
        <OptimizeSection
          optimizing={optimizing}
          isJobActive={isJobActive}
          optimizeError={optimizeError}
          onOptimize={handleOptimize}
        />
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

      <OptimizeSection
        optimizing={optimizing}
        isJobActive={isJobActive}
        optimizeError={optimizeError}
        onOptimize={handleOptimize}
      />
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
  const [refining, setRefining] = useState(false);
  const [icpResult, setIcpResult] = useState<IcpRefineResult | null>(null);
  const [icpError, setIcpError] = useState<string | null>(null);

  const handleIcpRefine = useCallback(async () => {
    const fromId = parseInt(from, 10);
    const toId = parseInt(to, 10);
    if (isNaN(fromId) || isNaN(toId)) {
      setIcpError('Enter valid integer Frame IDs before refining.');
      return;
    }
    if (fromId === toId) {
      setIcpError("'From' and 'To' must be different frames.");
      return;
    }
    setRefining(true);
    setIcpError(null);
    setIcpResult(null);
    try {
      const result = await api.refinePoseGraphIcp({ from: fromId, to: toId });
      setIcpResult(result);
    } catch (err) {
      setIcpError(err instanceof Error ? err.message : String(err));
    } finally {
      setRefining(false);
    }
  }, [from, to]);

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
          onChange={(e) => { setFrom(e.target.value); setIcpResult(null); }}
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
          onChange={(e) => { setTo(e.target.value); setIcpResult(null); }}
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

      <IcpRefineSection
        refining={refining}
        result={icpResult}
        error={icpError}
        onRefine={handleIcpRefine}
      />

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

// -------------------------------------------------------------------------- //

interface IcpRefineSectionProps {
  refining: boolean;
  result: IcpRefineResult | null;
  error: string | null;
  onRefine: () => void;
}

function IcpRefineSection({ refining, result, error, onRefine }: IcpRefineSectionProps) {
  return (
    <div className={styles.icpSection}>
      <button
        type="button"
        className={styles.icpBtn}
        onClick={onRefine}
        disabled={refining}
        title="Run point-cloud ICP to estimate the relative pose and fitness before adding the edge"
      >
        {refining ? 'Refining…' : 'ICP refine'}
      </button>
      {error && <p className={styles.error}>{error}</p>}
      {result && (
        <div className={styles.icpResult}>
          <div className={styles.fieldRow}>
            <span className={styles.fieldKey}>Fitness</span>
            <span className={`${styles.fieldVal} mono`}>
              {(result.fitness * 100).toFixed(1)} cm RMS
            </span>
          </div>
          <div className={styles.fieldRow}>
            <span className={styles.fieldKey}>Inliers</span>
            <span className={`${styles.fieldVal} mono`}>
              {(result.inlier_fraction * 100).toFixed(0)} %
            </span>
          </div>
          <div className={styles.fieldRow}>
            <span className={styles.fieldKey}>Converged</span>
            <span className={`${styles.fieldVal} mono`}>
              {result.converged ? 'yes' : 'no'}
            </span>
          </div>
        </div>
      )}
    </div>
  );
}

// -------------------------------------------------------------------------- //

interface OptimizeSectionProps {
  optimizing: boolean;
  isJobActive: boolean;
  optimizeError: string | null;
  onOptimize: () => void;
}

function OptimizeSection({
  optimizing,
  isJobActive,
  optimizeError,
  onOptimize,
}: OptimizeSectionProps) {
  const busy = optimizing || isJobActive;
  const label = isJobActive ? 'Optimizing…' : optimizing ? 'Submitting…' : 'Re-run optimize';

  return (
    <div className={styles.optimizeHint}>
      <p className={styles.hintText}>
        After editing edges, re-run the optimizer to recalculate residuals and
        update frame poses.
      </p>
      {optimizeError && <p className={styles.error}>{optimizeError}</p>}
      <button
        type="button"
        className={styles.optimizeBtn}
        onClick={onOptimize}
        disabled={busy}
      >
        {label}
      </button>
    </div>
  );
}
