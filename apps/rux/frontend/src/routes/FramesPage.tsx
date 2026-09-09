// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback } from 'react';
import { useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import { useAsync } from '../app/useAsync';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { FrameDetail } from '../components/FrameDetail';
import { FrameGrid } from '../components/FrameGrid';
import { Spinner } from '../components/Spinner';
import {
  FRAME_FILTERS,
  FRAME_FILTER_LABELS,
  describeFrameCounts,
  parseFrameFilter,
  parseSelectedFrame,
  segmentedParam,
} from '../data/framesModel';
import styles from './FramesPage.module.css';

/**
 * The sensor-frame browser.
 *
 * Both pieces of state — the filter and the selection — live in the URL, the
 * same convention the viewport uses for `?cloud=`. That makes "the segmented
 * frames of this scan" and "frame 412" links, which is what a screen someone
 * uses to point at a problem needs to be.
 *
 * The filter is a **request**, not a predicate. `GET /frames?segmented=` is
 * refetched when it changes rather than the full id list being narrowed on the
 * client, because the only client-side way to know whether a frame carries a
 * mask is `GET /frames/{id}` — which decodes that frame's depth and confidence
 * blobs to answer. Doing that several hundred times to draw a filtered list
 * would be the most expensive way to compute something the server already
 * knows.
 */
export function FramesPage() {
  const [params, setParams] = useSearchParams();
  const filter = parseFrameFilter(params.get('filter'));

  const frames = useAsync(
    (signal) => api.frames({ segmented: segmentedParam(filter) }, signal),
    [filter],
  );

  const ids = frames.data?.ids ?? [];
  const selected = parseSelectedFrame(params.get('frame'), ids);

  const setParam = useCallback(
    (key: string, value: string | null) => {
      const next = new URLSearchParams(params);
      if (value === null) next.delete(key);
      else next.set(key, value);
      setParams(next, { replace: true });
    },
    [params, setParams],
  );

  const handleSelect = useCallback(
    (id: number) => setParam('frame', String(id)),
    [setParam],
  );

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <div className={styles.filters} role="group" aria-label="Frame filter">
          {FRAME_FILTERS.map((option) => (
            <button
              key={option}
              type="button"
              aria-pressed={option === filter}
              className={`${styles.filter} ${option === filter ? styles.active : ''}`}
              onClick={() => setParam('filter', option === 'all' ? null : option)}
            >
              {FRAME_FILTER_LABELS[option]}
            </button>
          ))}
        </div>
        <p className={styles.counts}>
          {frames.data ? describeFrameCounts(frames.data, filter) : ' '}
        </p>
      </header>

      {frames.error ? (
        <ErrorBanner
          error={frames.error}
          onRetry={frames.reload}
          context="the frame inventory"
        />
      ) : !frames.data ? (
        <Spinner label="Reading frames…" />
      ) : ids.length === 0 ? (
        <EmptyState
          title={filter === 'all' ? 'No sensor frames' : `No ${filter} frames`}
          detail={
            filter === 'all'
              ? '`rux import rtabmap` (or `mushroom`, `arkitscenes`) brings captured frames into the project.'
              : filter === 'segmented'
                ? 'None of this scan\'s frames carry a segmentation mask yet. `rux create annotate` produces them.'
                : 'Every frame in this scan already carries a segmentation mask.'
          }
        />
      ) : (
        <div className={styles.body}>
          <FrameGrid ids={ids} selected={selected} onSelect={handleSelect} />
          {selected !== null && (
            <FrameDetail id={selected} onClose={() => setParam('frame', null)} />
          )}
        </div>
      )}
    </div>
  );
}
