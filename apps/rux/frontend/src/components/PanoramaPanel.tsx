// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState } from 'react';

import { api } from '../api/client';
import {
  describePlacement,
  panoramaNote,
  resolvePlacement,
  type PanoramaPlacement,
} from '../viewport/panorama';
import type { PanoramaPanelState } from './LayerPanel';
import { PanoramaSegmentPanel } from './PanoramaSegmentPanel';
import styles from './PanoramaPanel.module.css';

export interface PanoramaPanelProps {
  panorama: PanoramaPanelState;
  /** Close the panel and return to the compact trigger in the layer panel. */
  onClose: () => void;
}

/**
 * The 360-panorama browser, split out of `LayerPanel` (#442).
 *
 * The per-image list used to live inline in the layer panel, where a project
 * with twenty panoramas pushed the Colour and Display controls off the bottom.
 * It is its own surface now: a sibling `<aside>` with an internally scrolling
 * list, so the number of panoramas never changes the height of anything else.
 *
 * The rows are the same buttons the layer panel drew before — not toggles:
 * entering a panorama is a *place to stand*, and only one can be occupied at a
 * time, so a checkbox would promise a combination the viewport cannot show. A
 * panorama the project cannot place (no aligned pose, no matched frame pose) is
 * listed and disabled rather than hidden, so "this panorama exists but nothing
 * knows where it was taken" is visible instead of looking like a missing import.
 *
 * The thumbnail is fetched at `max_size=128`. At full resolution a strip of
 * twenty of these is tens of megabytes, which is exactly the reason that
 * parameter was added to the contract for this phase.
 */
export function PanoramaPanel({ panorama, onClose }: PanoramaPanelProps) {
  const { items, error, activeId, markersVisible, onMarkersVisibleChange, onEnter } = panorama;
  const note = panoramaNote(items, error);

  const [segmentOpen, setSegmentOpen] = useState(false);

  return (
    <aside className={styles.panel} aria-label="360 panoramas">
      <div className={styles.header}>
        <h2 className={styles.heading}>360 panoramas</h2>
        <button
          type="button"
          className={styles.close}
          onClick={onClose}
          title="Close"
          aria-label="Close 360 panorama panel"
        >
          ×
        </button>
      </div>

      {note && <p className={styles.note}>{note}</p>}

      {(items ?? []).length > 0 && (
        <label className={styles.markerToggle}>
          <input
            type="checkbox"
            checked={markersVisible}
            onChange={(event) => onMarkersVisibleChange(event.target.checked)}
            className={styles.checkbox}
          />
          <span className={styles.markerLabel}>Show capture positions</span>
        </label>
      )}

      <div className={styles.list}>
        {(items ?? []).map((info) => {
          const placement: PanoramaPlacement | null = resolvePlacement(info);
          const active = activeId === info.id;
          return (
            <button
              key={info.id}
              type="button"
              className={`${styles.panorama} ${active ? styles.panoramaActive : ''}`}
              disabled={placement === null}
              aria-pressed={active}
              onClick={() => onEnter(active ? null : info.id)}
            >
              <img
                className={styles.thumbnail}
                src={api.panoramaImageUrl(info.id, { maxSize: 128 })}
                alt=""
                loading="lazy"
              />
              <span className={styles.panoramaText}>
                <span className={styles.panoramaName} title={info.filename}>
                  {info.filename}
                </span>
                <span className={styles.note}>{describePlacement(info, placement)}</span>
              </span>
            </button>
          );
        })}
      </div>

      {/* ---- SAM3 segmentation for the active panorama (#448) ---- */}
      {activeId !== null && (
        <div className={styles.segmentSection}>
          <button
            type="button"
            className={styles.segmentToggle}
            onClick={() => setSegmentOpen((o) => !o)}
            aria-expanded={segmentOpen}
          >
            {segmentOpen ? '▾ Segmentation' : '▸ Segmentation'}
          </button>
          {segmentOpen && (
            <PanoramaSegmentPanel
              panoramaId={activeId}
              onSegmented={() => {}}
            />
          )}
        </div>
      )}
    </aside>
  );
}
