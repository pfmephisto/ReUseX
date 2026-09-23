// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useState } from 'react';
import { useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import type { ScanGroup } from '../api/types';
import { useAsync } from '../app/useAsync';
import { EmptyState } from '../components/EmptyState';
import { ErrorBanner } from '../components/ErrorBanner';
import { FrameDetail } from '../components/FrameDetail';
import { FrameGrid } from '../components/FrameGrid';
import { PanoramaGrid } from '../components/PanoramaGrid';
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
 * Frames are presented in collapsible groups — one per import scan plus the
 * 360 Images group — so the view does not overwhelm the user with a single
 * flat grid of hundreds of thumbnails. Each group is lazy: its thumbnails are
 * only loaded when the group is expanded.
 *
 * When the server returns a single scan (or a pre-v17 project with no `scans`
 * field), the view falls back to a single "Sensor Frames" group exactly as
 * before. When there are multiple scans, each gets its own collapsible section
 * labelled by the basename of the import path.
 *
 * Both pieces of URL state — the filter and the selection — follow the same
 * convention the viewport uses for `?cloud=`. The filter is a **request** to
 * the server, not a client-side predicate: `GET /frames?segmented=` is cheaper
 * than one `/frames/{id}` per frame to read a single boolean.
 */
export function FramesPage() {
  const [params, setParams] = useSearchParams();
  const filter = parseFrameFilter(params.get('filter'));

  // Frame IDs are fetched eagerly — they are integers and cheap, and the
  // selection URL state (`?frame=`) needs them to validate on every render,
  // including when a Sensor Frames group is collapsed.
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

  const scanGroups = frames.data?.scans;
  const isMultiScan = scanGroups !== undefined && scanGroups.length > 1;

  return (
    <div className={styles.page}>
      <header className={styles.head}>
        <div className={styles.filters} role="group" aria-label="Sensor frame filter">
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
          {frames.data ? describeFrameCounts(frames.data, filter) : ' '}
        </p>
      </header>

      <div className={styles.body}>
        <div className={styles.groupsColumn}>
          {isMultiScan ? (
            // One collapsible group per import scan (#462).
            scanGroups!.map((scan, index) => (
              <FrameGroupSection
                key={scan.scan_id}
                label={scanGroupLabel(scan)}
                count={scan.ids.length}
                defaultExpanded={index === 0}
                grow
              >
                {scan.ids.length === 0 ? (
                  <EmptyState
                    title={filter === 'all' ? 'No sensor frames' : `No ${filter} frames`}
                    detail={
                      filter === 'segmented'
                        ? "None of this scan's frames carry a segmentation mask yet."
                        : filter === 'unsegmented'
                          ? 'Every frame in this scan already carries a segmentation mask.'
                          : 'This scan has no frames.'
                    }
                  />
                ) : (
                  <FrameGrid ids={scan.ids} selected={selected} onSelect={handleSelect} />
                )}
              </FrameGroupSection>
            ))
          ) : (
            // Single scan or pre-v17 project: one "Sensor Frames" group.
            <FrameGroupSection
              label={scanGroups?.length === 1 ? scanGroupLabel(scanGroups[0]) : 'Sensor Frames'}
              count={frames.data?.total_count}
              defaultExpanded
              grow
            >
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
                        ? "None of this scan's frames carry a segmentation mask yet. `rux create annotate` produces them."
                        : 'Every frame in this scan already carries a segmentation mask.'
                  }
                />
              ) : (
                <FrameGrid ids={ids} selected={selected} onSelect={handleSelect} />
              )}
            </FrameGroupSection>
          )}

          <FrameGroupSection label="360 Images">
            <PanoramaGroupContent />
          </FrameGroupSection>
        </div>

        {selected !== null && (
          <FrameDetail id={selected} onClose={() => setParam('frame', null)} />
        )}
      </div>
    </div>
  );
}

/** Display label for a scan group: the final path segment, or "Scan N" fallback. */
function scanGroupLabel(scan: ScanGroup): string {
  const basename = scan.source_path.split(/[/\\]/).at(-1) ?? '';
  return basename || `Scan ${scan.scan_id}`;
}

// ---------------------------------------------------------------------------
// Group section
// ---------------------------------------------------------------------------

interface FrameGroupSectionProps {
  label: string;
  /**
   * Badge count shown in the header. Absent while loading (the panoramas group
   * does not know its count until the data is fetched).
   */
  count?: number;
  /**
   * Whether the section fills the remaining vertical space. True for Sensor
   * Frames (contains a virtualised grid that must be tall to be useful), false
   * for 360 Images (wraps to a few rows of thumbnails).
   */
  grow?: boolean;
  defaultExpanded?: boolean;
  children: React.ReactNode;
}

/**
 * A collapsible group in the Frames view.
 *
 * Children are only mounted when the section is expanded — this is what makes
 * loading lazy: an unmounted child issues no fetch and creates no `<img>`
 * elements. The group remembers whether it was ever opened, so re-collapsing
 * and re-expanding does not re-trigger a fetch (the child stays mounted once
 * it is first shown).
 *
 * `grow` controls whether the section stretches to fill remaining vertical
 * space in the column or wraps to its content height.
 */
function FrameGroupSection({
  label,
  count,
  grow = false,
  defaultExpanded = false,
  children,
}: FrameGroupSectionProps) {
  const [expanded, setExpanded] = useState(defaultExpanded);
  // Keep the child mounted once it has been shown so re-expanding does not
  // re-trigger a fetch. The initial load is still lazy: mounting happens only
  // on the first expand.
  const [everExpanded, setEverExpanded] = useState(defaultExpanded);

  const toggle = useCallback(() => {
    setExpanded((e) => {
      if (!e) setEverExpanded(true);
      return !e;
    });
  }, []);

  return (
    <section className={`${styles.group} ${grow ? styles.groupGrow : ''}`}>
      <button
        type="button"
        className={styles.groupHeader}
        onClick={toggle}
        aria-expanded={expanded}
      >
        <span className={styles.groupChevron} aria-hidden="true">
          {expanded ? '▾' : '▸'}
        </span>
        <span className={styles.groupLabel}>{label}</span>
        {count !== undefined && (
          <span className={`${styles.groupCount} mono`}>{count}</span>
        )}
      </button>
      {everExpanded && (
        <div className={`${styles.groupBody} ${expanded ? '' : styles.groupBodyHidden}`}>
          {children}
        </div>
      )}
    </section>
  );
}

// ---------------------------------------------------------------------------
// Panorama group content (lazy)
// ---------------------------------------------------------------------------

/**
 * The body of the 360 Images group.
 *
 * Mounted only when the group is first expanded, so the panorama fetch starts
 * lazily. Panorama selection is local to this component: there is no
 * panorama-detail pane in this view, and the selection does not need to
 * survive the group being collapsed and re-opened.
 */
function PanoramaGroupContent() {
  const [selected, setSelected] = useState<number | null>(null);
  const panoramas = useAsync((signal) => api.panoramas(signal), []);

  if (panoramas.error) {
    return (
      <ErrorBanner
        error={panoramas.error}
        onRetry={panoramas.reload}
        context="the panorama list"
      />
    );
  }

  if (!panoramas.data) {
    return <Spinner label="Reading panoramas…" />;
  }

  if (panoramas.data.length === 0) {
    return (
      <EmptyState
        title="No 360° panoramas"
        detail="`rux import 360` brings equirectangular images into the project."
      />
    );
  }

  return (
    <PanoramaGrid panoramas={panoramas.data} selected={selected} onSelect={setSelected} />
  );
}
