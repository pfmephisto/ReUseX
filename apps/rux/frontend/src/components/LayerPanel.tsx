// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { api } from '../api/client';
import type { CloudInfo, GsplatInfo, PanoramaInfo } from '../api/types';
import type { ColorMode } from '../viewport/PointCloudScene';
import type { CloudStreamState } from '../viewport/useCloudStream';
import type { SplatLayerState } from '../viewport/Viewport';
import { describeGsplat, gsplatNote } from '../viewport/gsplatLayer';
import {
  describePlacement,
  panoramaNote,
  resolvePlacement,
  type PanoramaPlacement,
} from '../viewport/panorama';
import { EmptyState } from './EmptyState';
import { LabelLegend } from './LabelLegend';
import { LayerRow } from './LayerRow';
import styles from './LayerPanel.module.css';

/** Everything the panel needs about the Gaussian-splat layers (#322). */
export interface SplatPanelState {
  /** Splats stored in the project, or null while the list is being fetched. */
  items: GsplatInfo[] | null;
  /** A failure listing them, as opposed to a project that has none. */
  error: Error | null;
  visible: Record<string, boolean>;
  /** Load progress per splat; absent until one has been switched on. */
  loading: Record<string, SplatLayerState>;
  onToggle: (name: string, visible: boolean) => void;
}

/** Everything the panel needs about the 360 panoramas (#265, Phase 5). */
export interface PanoramaPanelState {
  /** Panoramas in the project, or null while the list is being fetched. */
  items: PanoramaInfo[] | null;
  /** A failure listing them, as opposed to a project that has none. */
  error: Error | null;
  /** The panorama currently being looked through, if any. */
  activeId: number | null;
  /** Whether the capture-position markers are drawn in the orbit view. */
  markersVisible: boolean;
  onMarkersVisibleChange: (visible: boolean) => void;
  /** Enter a panorama, or leave the current one with null. */
  onEnter: (id: number | null) => void;
}

export interface LayerPanelProps {
  /** Renderable geometry clouds (`PointXYZRGB` / `PointXYZ`). */
  clouds: CloudInfo[];
  visible: Record<string, boolean>;
  progress: Record<string, CloudStreamState>;
  onToggleLayer: (name: string, visible: boolean) => void;

  /**
   * `Label` clouds that may legitimately colour the current selection.
   *
   * The caller filters these by point count — see `ViewportPage`.
   */
  labelSources: CloudInfo[];
  labelCloud: string | null;
  onLabelCloudChange: (name: string | null) => void;
  /** Why no label source is offered, when none is. */
  labelSourceNote?: string;

  /**
   * The Gaussian-splat layers, when the page models them.
   *
   * One optional object rather than five loose props, so a page with no splat
   * support says so by omitting one thing.
   */
  splat?: SplatPanelState;

  /**
   * The 360 panoramas, when the page models them.
   *
   * Optional for the same reason `splat` is: a page with no panorama support
   * says so by omitting one thing rather than by passing five empty props.
   */
  panorama?: PanoramaPanelState;

  colorMode: ColorMode;
  onColorModeChange: (mode: ColorMode) => void;

  pointSize: number;
  onPointSizeChange: (size: number) => void;

  onFrame: () => void;
}

/**
 * Viewport side panel: what is shown, and how it is coloured.
 *
 * This is the discoverable replacement for `rux view`'s keyboard vocabulary —
 * the design brief's "interaction seed". Every control here corresponds to a
 * keypress an architect would otherwise have had to be told about.
 */
export function LayerPanel({
  clouds,
  visible,
  progress,
  onToggleLayer,
  labelSources,
  labelCloud,
  onLabelCloudChange,
  labelSourceNote,
  splat,
  panorama,
  colorMode,
  onColorModeChange,
  pointSize,
  onPointSizeChange,
  onFrame,
}: LayerPanelProps) {
  const activeLabelCloud = labelSources.find((cloud) => cloud.name === labelCloud);

  return (
    <aside className={styles.panel} aria-label="Layers">
      <section className={styles.section}>
        <h2 className={styles.heading}>Clouds</h2>
        {clouds.length === 0 ? (
          <EmptyState
            title="No renderable clouds"
            detail="Run `rux create clouds` to back-project the sensor frames into a fused cloud."
          />
        ) : (
          <div className={styles.layers}>
            {clouds.map((cloud) => (
              <LayerRow
                key={cloud.name}
                cloud={cloud}
                visible={visible[cloud.name] ?? false}
                progress={progress[cloud.name]}
                onToggle={(next) => onToggleLayer(cloud.name, next)}
              />
            ))}
          </div>
        )}
      </section>

      {splat && <SplatSection splat={splat} />}

      {panorama && <PanoramaSection panorama={panorama} />}

      <section className={styles.section}>
        <h2 className={styles.heading}>Colour</h2>
        <div className={styles.modes} role="group" aria-label="Colour mode">
          <button
            type="button"
            className={`${styles.mode} ${colorMode === 'rgb' ? styles.modeActive : ''}`}
            onClick={() => onColorModeChange('rgb')}
          >
            Sensor RGB
          </button>
          <button
            type="button"
            className={`${styles.mode} ${colorMode === 'label' ? styles.modeActive : ''}`}
            onClick={() => onColorModeChange('label')}
            disabled={labelCloud === null}
            title={labelCloud === null ? 'Select a label source first' : undefined}
          >
            Labels
          </button>
        </div>

        <label className={styles.field}>
          <span className={styles.fieldLabel}>Label source</span>
          <select
            className={styles.select}
            value={labelCloud ?? ''}
            onChange={(event) => onLabelCloudChange(event.target.value || null)}
            disabled={labelSources.length === 0}
          >
            <option value="">None</option>
            {labelSources.map((cloud) => (
              <option key={cloud.name} value={cloud.name}>
                {cloud.name}
              </option>
            ))}
          </select>
        </label>

        {labelSources.length === 0 && labelSourceNote && (
          <p className={styles.note}>{labelSourceNote}</p>
        )}

        {colorMode === 'label' && activeLabelCloud?.labels && (
          <LabelLegend labels={activeLabelCloud.labels} />
        )}
      </section>

      <section className={styles.section}>
        <h2 className={styles.heading}>Display</h2>
        <label className={styles.field}>
          <span className={styles.fieldLabel}>
            Point size <span className="mono">{pointSize.toFixed(3)} m</span>
          </span>
          <input
            type="range"
            min={0.002}
            max={0.12}
            step={0.002}
            value={pointSize}
            onChange={(event) => onPointSizeChange(Number(event.target.value))}
            className={styles.range}
          />
        </label>
        <button type="button" className={styles.action} onClick={onFrame}>
          Frame all
        </button>
      </section>
    </aside>
  );
}

/**
 * One row per Gaussian splat stored in the project, or the reason there are
 * none.
 *
 * A project without a splat still gets the heading and a sentence. Hiding the
 * section entirely would leave a user who has just run `rux create gsplat` no
 * way to tell "this viewer cannot draw splats" apart from "this project has
 * none" — and those want completely different next actions.
 *
 * Toggled independently of the point cloud, and of each other: comparing a
 * splat against the LiDAR cloud it was seeded from is the reason to have both
 * in one viewport at all.
 */
function SplatSection({ splat }: { splat: SplatPanelState }) {
  const { items, error, visible, loading, onToggle } = splat;
  const note = gsplatNote(items, error);

  return (
    <section className={styles.section}>
      <h2 className={styles.heading}>Gaussian splats</h2>

      {note && <p className={styles.note}>{note}</p>}

      {(items ?? []).map((info) => {
        const state = loading[info.name];
        return (
          <div key={info.name} className={styles.splat}>
            <label className={styles.splatToggle}>
              <input
                type="checkbox"
                checked={visible[info.name] ?? false}
                onChange={(event) => onToggle(info.name, event.target.checked)}
                className={styles.checkbox}
              />
              <span className={styles.splatName} title={info.name}>
                {info.name}
              </span>
            </label>

            <p className={styles.note}>{describeGsplat(info)}</p>

            {state?.loading && (
              <div className={styles.track}>
                <div
                  className={`${styles.fill} ${
                    state.fraction === null ? styles.indeterminate : ''
                  }`}
                  style={
                    state.fraction === null ? undefined : { width: `${state.fraction * 100}%` }
                  }
                />
              </div>
            )}

            {state?.error && (
              <p className={styles.error}>
                The splat could not be loaded: {state.error.message}
              </p>
            )}
          </div>
        );
      })}
    </section>
  );
}

/**
 * One row per 360 panorama, and the reason there are none.
 *
 * The rows are buttons, not toggles: entering a panorama is a *place to
 * stand*, and only one can be occupied at a time — a checkbox would promise a
 * combination the viewport cannot show. A panorama the project cannot place
 * (no aligned pose, no matched frame pose) is listed and disabled rather than
 * hidden, so "this panorama exists but nothing knows where it was taken" is
 * visible instead of looking like a missing import.
 *
 * The thumbnail is fetched at `max_size=128`. At full resolution a strip of
 * twenty of these is tens of megabytes, which is exactly the reason that
 * parameter was added to the contract for this phase.
 */
function PanoramaSection({ panorama }: { panorama: PanoramaPanelState }) {
  const { items, error, activeId, markersVisible, onMarkersVisibleChange, onEnter } = panorama;
  const note = panoramaNote(items, error);

  return (
    <section className={styles.section}>
      <h2 className={styles.heading}>360 panoramas</h2>

      {note && <p className={styles.note}>{note}</p>}

      {(items ?? []).length > 0 && (
        <label className={styles.splatToggle}>
          <input
            type="checkbox"
            checked={markersVisible}
            onChange={(event) => onMarkersVisibleChange(event.target.checked)}
            className={styles.checkbox}
          />
          <span className={styles.splatName}>Show capture positions</span>
        </label>
      )}

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
    </section>
  );
}
