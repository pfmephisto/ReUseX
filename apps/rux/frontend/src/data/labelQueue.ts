// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Pure types and functions for the global label library and annotation queue
 * (#447).
 *
 * No React, no side effects — only input → output. The context in
 * `LabelQueueContext.tsx` owns React state and localStorage persistence; these
 * functions are the pure core that vitest exercises without a DOM.
 */

import type { FrameSegmentPrompt } from '../api/types';

// ------------------------------------------------------------------ types --

export interface LabelEntry {
  id: string;
  /** Open-vocabulary class name, e.g. "wall". */
  text: string;
  /** Per-label confidence override; absent ⟹ use the queue-level default. */
  confidence?: number;
}

export type QueueStatus = 'pending' | 'running' | 'done' | 'failed';

export interface QueueItem {
  id: string;
  frameId: number;
  prompts: FrameSegmentPrompt[];
  modelPath: string;
  confidence: number;
  status: QueueStatus;
  /** Server error message when status is 'failed'. */
  error?: string;
  /** Labeled pixel count reported by the server on success. */
  labeledPixels?: number;
}

// ---------------------------------------------------------- id generators --

// Module-level counters ensure uniqueness within a session. They do NOT need to
// be stable across page reloads — restored IDs come from localStorage as
// strings and are never compared to freshly generated ones.
let _nextLabelId = 0;
let _nextItemId = 0;

/** Reset counters. Exported for use in tests only — not part of the public API. */
export function _resetIds(): void {
  _nextLabelId = 0;
  _nextItemId = 0;
}

// --------------------------------------------------------- label library --

export function makeLabel(text: string, confidence?: number): LabelEntry {
  return { id: `lbl-${++_nextLabelId}`, text: text.trim(), confidence };
}

export function updateLabel(
  labels: LabelEntry[],
  id: string,
  text: string,
  confidence?: number,
): LabelEntry[] {
  return labels.map((l) => (l.id === id ? { ...l, text: text.trim(), confidence } : l));
}

export function removeLabel(labels: LabelEntry[], id: string): LabelEntry[] {
  return labels.filter((l) => l.id !== id);
}

// -------------------------------------------------------------- queue ops --

/**
 * Create one pending queue item per frame ID, sharing the same prompt list and
 * model config. This is what propagating one annotation decision to a range of
 * frames looks like: N items, one per frame, all with the same prompts.
 */
export function makeQueueItems(
  frameIds: number[],
  prompts: FrameSegmentPrompt[],
  modelPath: string,
  confidence: number,
): QueueItem[] {
  return frameIds.map((frameId) => ({
    id: `qi-${++_nextItemId}`,
    frameId,
    prompts: [...prompts],
    modelPath,
    confidence,
    status: 'pending',
  }));
}

export function setItemStatus(
  items: QueueItem[],
  id: string,
  status: QueueStatus,
  extra?: { error?: string; labeledPixels?: number },
): QueueItem[] {
  return items.map((item) =>
    item.id === id
      ? { ...item, status, error: undefined, labeledPixels: undefined, ...extra }
      : item,
  );
}

export function removeItem(items: QueueItem[], id: string): QueueItem[] {
  return items.filter((item) => item.id !== id);
}

/**
 * Remove 'done' items; keep 'pending', 'running', and 'failed'.
 *
 * 'running' items are rare (only if the panel unmounts mid-run) but dropping
 * them would silently lose the user's in-progress context.
 */
export function clearFinished(items: QueueItem[]): QueueItem[] {
  return items.filter((item) => item.status !== 'done');
}

// --------------------------------------------------------- frame windowing --

/**
 * Return the contiguous slice of `allIds` centred on `centerId`, extending
 * `before` frames backwards and `after` frames forwards in the list order.
 *
 * Returns `[centerId]` when the centre is not found in `allIds`.
 * Clamps to the array bounds — asking for 100 frames before the first frame
 * returns only what exists.
 */
export function neighborFrameIds(
  allIds: number[],
  centerId: number,
  before: number,
  after: number,
): number[] {
  const idx = allIds.indexOf(centerId);
  if (idx === -1) return [centerId];
  const start = Math.max(0, idx - Math.max(0, before));
  const end = Math.min(allIds.length - 1, idx + Math.max(0, after));
  return allIds.slice(start, end + 1);
}

// --------------------------------------------------------- localStorage I/O --

const STORAGE_KEY = 'reusex:label-queue-v1';

interface StoredState {
  labels: LabelEntry[];
  items: QueueItem[];
}

/**
 * Load and repair persisted state.
 *
 * 'running' items that never finished (page closed mid-run) are reset to
 * 'pending' so they can be retried.
 */
export function loadStoredState(): { labels: LabelEntry[]; items: QueueItem[] } {
  try {
    const store = typeof localStorage !== 'undefined' ? localStorage : null;
    const raw = store?.getItem(STORAGE_KEY) ?? null;
    if (!raw) return { labels: [], items: [] };
    const parsed = JSON.parse(raw) as Partial<StoredState>;
    const items = ((parsed.items ?? []) as QueueItem[]).map((item) =>
      item.status === 'running' ? { ...item, status: 'pending' as QueueStatus } : item,
    );
    return { labels: parsed.labels ?? [], items };
  } catch {
    return { labels: [], items: [] };
  }
}

export function saveState(labels: LabelEntry[], items: QueueItem[]): void {
  try {
    const store = typeof localStorage !== 'undefined' ? localStorage : null;
    store?.setItem(STORAGE_KEY, JSON.stringify({ labels, items } satisfies StoredState));
  } catch {
    // Non-fatal: storage quota exceeded or private browsing.
  }
}
