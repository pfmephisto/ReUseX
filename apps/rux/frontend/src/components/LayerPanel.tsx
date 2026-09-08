// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { CloudInfo } from '../api/types';
import type { ColorMode } from '../viewport/PointCloudScene';
import type { CloudStreamState } from '../viewport/useCloudStream';
import { EmptyState } from './EmptyState';
import { LabelLegend } from './LabelLegend';
import { LayerRow } from './LayerRow';
import styles from './LayerPanel.module.css';

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
