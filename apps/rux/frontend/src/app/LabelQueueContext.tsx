// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Global label library + annotation queue context (#447).
 *
 * Wraps the pure functions in `data/labelQueue.ts` in React state, persisting
 * every write to localStorage so the library and queue survive page refreshes.
 * Mount `<LabelQueueProvider>` once at the application root (App.tsx).
 */

import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from 'react';

import type { FrameSegmentPrompt } from '../api/types';
import {
  clearFinished as clearFinishedFn,
  loadStoredState,
  makeLabel,
  makeQueueItems,
  removeItem as removeItemFn,
  removeLabel as removeLabelFn,
  saveState,
  setItemStatus as setItemStatusFn,
  updateLabel as updateLabelFn,
  type LabelEntry,
  type QueueItem,
  type QueueStatus,
} from '../data/labelQueue';

export interface LabelQueueContextValue {
  // --------------------------------------------------------- label library --
  labels: LabelEntry[];
  addLabel: (text: string, confidence?: number) => void;
  removeLabel: (id: string) => void;
  updateLabel: (id: string, text: string, confidence?: number) => void;

  // -------------------------------------------------------------- queue ops --
  items: QueueItem[];
  /** Add `frameIds × prompts` as pending items. */
  enqueue: (
    frameIds: number[],
    prompts: FrameSegmentPrompt[],
    modelPath: string,
    confidence: number,
  ) => void;
  dequeue: (id: string) => void;
  clearFinished: () => void;
  setItemStatus: (
    id: string,
    status: QueueStatus,
    extra?: { error?: string; labeledPixels?: number },
  ) => void;

  /** Number of items currently in 'pending' state. */
  pendingCount: number;
}

const LabelQueueContext = createContext<LabelQueueContextValue | null>(null);

export function LabelQueueProvider({ children }: { children: ReactNode }) {
  // Lazy init from localStorage — runs once on mount.
  const [labels, setLabels] = useState<LabelEntry[]>(() => loadStoredState().labels);
  const [items, setItems] = useState<QueueItem[]>(() => loadStoredState().items);

  // Persist on every change. Both slices are stable references from useState,
  // so this only fires when the user actually mutates the library or queue.
  useEffect(() => {
    saveState(labels, items);
  }, [labels, items]);

  const addLabel = useCallback((text: string, confidence?: number) => {
    setLabels((prev) => [...prev, makeLabel(text, confidence)]);
  }, []);

  const removeLabel = useCallback((id: string) => {
    setLabels((prev) => removeLabelFn(prev, id));
  }, []);

  const updateLabel = useCallback((id: string, text: string, confidence?: number) => {
    setLabels((prev) => updateLabelFn(prev, id, text, confidence));
  }, []);

  const enqueue = useCallback(
    (frameIds: number[], prompts: FrameSegmentPrompt[], modelPath: string, confidence: number) => {
      setItems((prev) => [...prev, ...makeQueueItems(frameIds, prompts, modelPath, confidence)]);
    },
    [],
  );

  const dequeue = useCallback((id: string) => {
    setItems((prev) => removeItemFn(prev, id));
  }, []);

  const clearFinished = useCallback(() => {
    setItems((prev) => clearFinishedFn(prev));
  }, []);

  const setItemStatus = useCallback(
    (id: string, status: QueueStatus, extra?: { error?: string; labeledPixels?: number }) => {
      setItems((prev) => setItemStatusFn(prev, id, status, extra));
    },
    [],
  );

  const pendingCount = useMemo(
    () => items.filter((i) => i.status === 'pending').length,
    [items],
  );

  const value = useMemo<LabelQueueContextValue>(
    () => ({
      labels,
      addLabel,
      removeLabel,
      updateLabel,
      items,
      enqueue,
      dequeue,
      clearFinished,
      setItemStatus,
      pendingCount,
    }),
    [
      labels,
      addLabel,
      removeLabel,
      updateLabel,
      items,
      enqueue,
      dequeue,
      clearFinished,
      setItemStatus,
      pendingCount,
    ],
  );

  return <LabelQueueContext.Provider value={value}>{children}</LabelQueueContext.Provider>;
}

export function useLabelQueue(): LabelQueueContextValue {
  const ctx = useContext(LabelQueueContext);
  if (!ctx) throw new Error('useLabelQueue must be used inside <LabelQueueProvider>');
  return ctx;
}
