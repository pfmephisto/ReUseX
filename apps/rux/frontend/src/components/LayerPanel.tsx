// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useState, type ReactNode } from 'react';

import type { CloudInfo, GsplatInfo, MeshInfo, PanoramaInfo, PoseGraph, PoseGraphEdgeType } from '../api/types';
import type { ColorMode } from '../viewport/PointCloudScene';
import {
  VIEW_PRESETS,
  type CameraProjection,
  type LightingState,
  type ViewPreset,
} from '../viewport/cameraViews';
import type { CloudStreamState } from '../viewport/useCloudStream';
import type { MeshLayerState, SplatLayerState } from '../viewport/Viewport';
import { describeGsplat, gsplatNote } from '../viewport/gsplatLayer';
import { describeMesh, meshNote } from '../viewport/meshLayer';
import { panoramaNote } from '../viewport/panorama';
import { EDGE_BASE_COLORS, posegraphNote, type ProjectionPlane } from '../viewport/posegraphLayer';
import { EmptyState } from './EmptyState';
import { LabelLegend } from './LabelLegend';
import { LayerRow } from './LayerRow';
import { PanoramaPanel } from './PanoramaPanel';
import styles from './LayerPanel.module.css';

/** Everything the panel needs about the PLY mesh layers (#265, review pt 2). */
export interface MeshPanelState {
  /** Meshes stored in the project, or null while the list is being fetched. */
  items: MeshInfo[] | null;
  error: Error | null;
  visible: Record<string, boolean>;
  wireframe: Record<string, boolean>;
  /** Load progress per mesh; absent until one has been switched on. */
  loading: Record<string, MeshLayerState>;
  onToggle: (name: string, visible: boolean) => void;
  onWireframeToggle: (name: string, wireframe: boolean) => void;
}

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

/** Pose-graph panel state, passed from the viewport page. */
export interface PoseGraphPanelState {
  graph: PoseGraph | null;
  error: Error | null;
  visible: boolean;
  onToggle: (visible: boolean) => void;
  // Inspection controls added in #445:
  projectionPlane: ProjectionPlane;
  onProjectionPlaneChange: (plane: ProjectionPlane) => void;
  edgeTypeVisible: Record<PoseGraphEdgeType, boolean>;
  onEdgeTypeChange: (type: PoseGraphEdgeType, visible: boolean) => void;
  /** 0 = disabled; edges above this residual value are highlighted in amber. */
  residualThreshold: number;
  onResidualThresholdChange: (threshold: number) => void;
  nodeColorMode: 'default' | 'degree';
  onNodeColorModeChange: (mode: 'default' | 'degree') => void;
}

/** Serialisable bounding-box corners, one per axis. */
export interface BoxCorner {
  x: number;
  y: number;
  z: number;
}

/** Clipping-box panel state (#444). */
export interface ClippingPanelState {
  enabled: boolean;
  /** Current box corners, or null before the first enable. */
  min: BoxCorner | null;
  max: BoxCorner | null;
  onEnabledChange: (enabled: boolean) => void;
  onBoxChange: (min: BoxCorner, max: BoxCorner) => void;
  onReset: () => void;
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
   * PLY mesh layers, when the page models them (#265, review pt 2).
   *
   * One optional object rather than six loose props, so a page with no mesh
   * support says so by omitting one thing.
   */
  mesh?: MeshPanelState;

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

  /** Pose-graph overlay (#265, review pt 4). */
  posegraph?: PoseGraphPanelState;

  colorMode: ColorMode;
  onColorModeChange: (mode: ColorMode) => void;

  pointSize: number;
  onPointSizeChange: (size: number) => void;

  onFrame: () => void;

  /** Orbit-camera projection and its toggle (#443). */
  projection: CameraProjection;
  onProjectionChange: (projection: CameraProjection) => void;
  /** Reorient to a named axis view (Top / Front / …) (#443). */
  onView: (preset: ViewPreset) => void;
  /** Key + fill light settings and their controls (#443). */
  lighting: LightingState;
  onLightingChange: (next: Partial<LightingState>) => void;
  /** Interactive clipping box (#444). */
  clipping?: ClippingPanelState;
}

/**
 * Viewport side panel: what is shown, and how it is coloured.
 *
 * This is the discoverable replacement for `rux view`'s keyboard vocabulary —
 * the design brief's "interaction seed". Every control here corresponds to a
 * keypress an architect would otherwise have had to be told about.
 *
 * Sections are collapsible and the 360 panorama list lives in its own surface
 * (`PanoramaPanel`, #442), so the panel stays short regardless of how many
 * clouds, meshes, splats or panoramas a project has.
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
  mesh,
  splat,
  panorama,
  posegraph,
  colorMode,
  onColorModeChange,
  pointSize,
  onPointSizeChange,
  onFrame,
  projection,
  onProjectionChange,
  onView,
  lighting,
  onLightingChange,
  clipping,
}: LayerPanelProps) {
  const activeLabelCloud = labelSources.find((cloud) => cloud.name === labelCloud);

  // The 360 list is its own surface, opened on demand from the compact trigger
  // in the panorama section. Closed by default so the page opens compact.
  const [panoramaOpen, setPanoramaOpen] = useState(false);

  return (
    <>
      {panorama && panoramaOpen && (
        <PanoramaPanel panorama={panorama} onClose={() => setPanoramaOpen(false)} />
      )}

      <aside className={styles.panel} aria-label="Layers">
        <Section title="Clouds">
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
        </Section>

        {mesh && <MeshSection mesh={mesh} />}

        {splat && <SplatSection splat={splat} />}

        {panorama && (
          <PanoramaTrigger
            panorama={panorama}
            open={panoramaOpen}
            onToggle={() => setPanoramaOpen((open) => !open)}
          />
        )}

        {posegraph && <PoseGraphSection posegraph={posegraph} />}

        <Section title="Colour">
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
        </Section>

        <Section title="Display">
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
        </Section>

        <ViewSection
          projection={projection}
          onProjectionChange={onProjectionChange}
          onView={onView}
          lighting={lighting}
          onLightingChange={onLightingChange}
        />

        {clipping && <ClippingSection clipping={clipping} />}
      </aside>
    </>
  );
}

/**
 * Camera and lighting controls (#443).
 *
 * The discoverable replacement for the camera keys `rux view` binds and the
 * `--view` presets `rux render` takes: a projection toggle, the axis-aligned
 * preset views, and the key/fill lights the reconstructed mesh is lit by. The
 * preset orientations match the CLI's so a shot framed here is the shot the
 * headless renderer produces.
 */
function ViewSection({
  projection,
  onProjectionChange,
  onView,
  lighting,
  onLightingChange,
}: {
  projection: CameraProjection;
  onProjectionChange: (projection: CameraProjection) => void;
  onView: (preset: ViewPreset) => void;
  lighting: LightingState;
  onLightingChange: (next: Partial<LightingState>) => void;
}) {
  return (
    <Section title="View">
      <div className={styles.modes} role="group" aria-label="Camera projection">
        <button
          type="button"
          className={`${styles.mode} ${projection === 'perspective' ? styles.modeActive : ''}`}
          onClick={() => onProjectionChange('perspective')}
        >
          Perspective
        </button>
        <button
          type="button"
          className={`${styles.mode} ${projection === 'orthographic' ? styles.modeActive : ''}`}
          onClick={() => onProjectionChange('orthographic')}
        >
          Orthographic
        </button>
      </div>

      <div className={styles.presets} role="group" aria-label="Preset views">
        {VIEW_PRESETS.map(({ preset, label }) => (
          <button
            key={preset}
            type="button"
            className={styles.action}
            onClick={() => onView(preset)}
          >
            {label}
          </button>
        ))}
      </div>

      <label className={styles.field}>
        <span className={styles.fieldLabel}>
          Key light <span className="mono">{lighting.keyIntensity.toFixed(1)}</span>
        </span>
        <input
          type="range"
          min={0}
          max={5}
          step={0.1}
          value={lighting.keyIntensity}
          onChange={(event) => onLightingChange({ keyIntensity: Number(event.target.value) })}
          className={styles.range}
        />
      </label>

      <label className={styles.field}>
        <span className={styles.fieldLabel}>
          Fill light <span className="mono">{lighting.ambientIntensity.toFixed(1)}</span>
        </span>
        <input
          type="range"
          min={0}
          max={3}
          step={0.1}
          value={lighting.ambientIntensity}
          onChange={(event) =>
            onLightingChange({ ambientIntensity: Number(event.target.value) })
          }
          className={styles.range}
        />
      </label>

      <label className={styles.field}>
        <span className={styles.fieldLabel}>
          Light bearing <span className="mono">{Math.round(lighting.azimuth)}°</span>
        </span>
        <input
          type="range"
          min={-180}
          max={180}
          step={5}
          value={lighting.azimuth}
          onChange={(event) => onLightingChange({ azimuth: Number(event.target.value) })}
          className={styles.range}
        />
      </label>

      <label className={styles.field}>
        <span className={styles.fieldLabel}>
          Light height <span className="mono">{Math.round(lighting.elevation)}°</span>
        </span>
        <input
          type="range"
          min={0}
          max={90}
          step={5}
          value={lighting.elevation}
          onChange={(event) => onLightingChange({ elevation: Number(event.target.value) })}
          className={styles.range}
        />
      </label>
    </Section>
  );
}

/**
 * Clipping box section (#444).
 *
 * A toggle enables/disables the six renderer clipping planes that crop the
 * loaded clouds, mesh and splats. When on, six range sliders (min/max per axis)
 * let the user fine-tune the cut region, and the 3D viewport shows spherical
 * face handles for direct dragging. A Reset button snaps the box back to the
 * full scene bounds.
 *
 * The sliders are disabled when the box corners are not yet known (first enable
 * auto-initialises from scene bounds; the 3D drag then populates the values).
 */
function ClippingSection({ clipping }: { clipping: ClippingPanelState }) {
  const { enabled, min, max, onEnabledChange, onBoxChange, onReset } = clipping;

  const handleAxis = (
    axis: 'x' | 'y' | 'z',
    side: 'min' | 'max',
    value: number,
  ) => {
    if (!min || !max) return;
    const nextMin = { ...min };
    const nextMax = { ...max };
    if (side === 'min') nextMin[axis] = value;
    else nextMax[axis] = value;
    onBoxChange(nextMin, nextMax);
  };

  const axisLabel: Record<'x' | 'y' | 'z', string> = { x: 'X', y: 'Y', z: 'Z' };
  const hasBox = min !== null && max !== null;

  // Determine slider range: extend 10 % beyond current box so the user can push
  // the face outward without first resetting. Fallback for first-enable.
  const range = (axis: 'x' | 'y' | 'z') => {
    if (!min || !max) return { lo: -50, hi: 50 };
    const span = max[axis] - min[axis];
    const pad = Math.max(span * 0.5, 1);
    return { lo: min[axis] - pad, hi: max[axis] + pad };
  };

  return (
    <Section title="Clipping" defaultOpen={false}>
      <label className={styles.splatToggle}>
        <input
          type="checkbox"
          checked={enabled}
          onChange={(event) => onEnabledChange(event.target.checked)}
          className={styles.checkbox}
        />
        <span className={styles.splatName}>Enable clipping box</span>
      </label>

      {enabled && (
        <>
          {(['x', 'y', 'z'] as const).map((axis) => {
            const r = range(axis);
            return (
              <div key={axis}>
                <label className={styles.field}>
                  <span className={styles.fieldLabel}>
                    {axisLabel[axis]} min{' '}
                    <span className="mono">{hasBox ? min![axis].toFixed(2) : '—'} m</span>
                  </span>
                  <input
                    type="range"
                    min={r.lo}
                    max={hasBox ? max![axis] - 0.01 : r.hi}
                    step={0.01}
                    value={hasBox ? min![axis] : 0}
                    disabled={!hasBox}
                    onChange={(event) => handleAxis(axis, 'min', Number(event.target.value))}
                    className={styles.range}
                  />
                </label>
                <label className={styles.field}>
                  <span className={styles.fieldLabel}>
                    {axisLabel[axis]} max{' '}
                    <span className="mono">{hasBox ? max![axis].toFixed(2) : '—'} m</span>
                  </span>
                  <input
                    type="range"
                    min={hasBox ? min![axis] + 0.01 : r.lo}
                    max={r.hi}
                    step={0.01}
                    value={hasBox ? max![axis] : 0}
                    disabled={!hasBox}
                    onChange={(event) => handleAxis(axis, 'max', Number(event.target.value))}
                    className={styles.range}
                  />
                </label>
              </div>
            );
          })}
          <button type="button" className={styles.action} onClick={onReset}>
            Reset to scene bounds
          </button>
          {!hasBox && (
            <p className={styles.note}>Enable and load a scan to see clipping controls.</p>
          )}
        </>
      )}
    </Section>
  );
}

/**
 * A collapsible panel section (#442).
 *
 * `<details>`/`<summary>` rather than hand-rolled open state: it is keyboard-
 * and screen-reader-accessible for free, and collapsing a section is what keeps
 * the panel short no matter how many meshes, splats or clouds a project has.
 * Open by default so nothing an architect relies on is hidden until they choose
 * to hide it.
 */
function Section({
  title,
  children,
  defaultOpen = true,
}: {
  title: string;
  children: ReactNode;
  defaultOpen?: boolean;
}) {
  return (
    <details className={styles.section} open={defaultOpen}>
      <summary className={styles.summary}>
        <span className={styles.chevron} aria-hidden="true">
          ▸
        </span>
        {title}
      </summary>
      {children}
    </details>
  );
}

/**
 * One row per PLY mesh stored in the project, or the reason there are none.
 *
 * A project without a mesh still gets the heading and a sentence, for the same
 * reason `SplatSection` does: "cannot draw meshes" and "has no mesh" want
 * completely different next actions from the user.
 *
 * Toggled independently of the point cloud — comparing the reconstructed mesh
 * against the raw cloud is one of the most useful checks an architect can do.
 * Wireframe mode reveals the cell-complex triangle structure (room boundaries).
 */
function MeshSection({ mesh }: { mesh: MeshPanelState }) {
  const { items, error, visible, wireframe, loading, onToggle, onWireframeToggle } = mesh;
  const note = meshNote(items, error);

  return (
    <Section title="Mesh">
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

            <p className={styles.note}>{describeMesh(info)}</p>

            {(visible[info.name] ?? false) && (
              <label className={styles.splatToggle}>
                <input
                  type="checkbox"
                  checked={wireframe[info.name] ?? false}
                  onChange={(event) => onWireframeToggle(info.name, event.target.checked)}
                  className={styles.checkbox}
                />
                <span className={styles.note}>Wireframe</span>
              </label>
            )}

            {state?.loading && (
              <div className={styles.track}>
                <div className={`${styles.fill} ${styles.indeterminate}`} />
              </div>
            )}

            {state?.error && (
              <p className={styles.error}>
                The mesh could not be loaded: {state.error.message}
              </p>
            )}
          </div>
        );
      })}
    </Section>
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
    <Section title="Gaussian splats">
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
    </Section>
  );
}

/**
 * The compact 360-panorama entry in the layer panel (#442).
 *
 * The per-image list itself lives in `PanoramaPanel`, a surface of its own, so
 * a project with twenty panoramas no longer buries the Colour and Display
 * controls. What stays here is the reason there are none (when there are none)
 * and a single button to open that surface — the "interaction seed" for 360
 * mode, discoverable without listing every image inline.
 */
function PanoramaTrigger({
  panorama,
  open,
  onToggle,
}: {
  panorama: PanoramaPanelState;
  open: boolean;
  onToggle: () => void;
}) {
  const { items, error } = panorama;
  const note = panoramaNote(items, error);
  const count = (items ?? []).length;

  return (
    <Section title="360 panoramas">
      {note && <p className={styles.note}>{note}</p>}

      {count > 0 && (
        <button
          type="button"
          className={`${styles.action} ${open ? styles.actionActive : ''}`}
          onClick={onToggle}
          aria-expanded={open}
        >
          {open ? 'Hide panoramas' : `Browse panoramas (${count})`}
        </button>
      )}
    </Section>
  );
}

const PROJECTION_PLANES: ProjectionPlane[] = ['3D', 'XY', 'XZ', 'YZ'];

const EDGE_TYPE_LABELS: Record<PoseGraphEdgeType, string> = {
  odometry: 'Odometry',
  loop_closure: 'Loop closure',
  panorama: 'Panorama',
};

const EDGE_TYPES: PoseGraphEdgeType[] = ['odometry', 'loop_closure', 'panorama'];

/**
 * Pose-graph inspection section (#265, review pt 4; #445).
 *
 * Controls:
 * - On/off toggle for the whole layer.
 * - Projection-plane switch (3D / XY / XZ / YZ) for 2D graph inspection.
 * - Per-type edge visibility (Odometry / Loop closure / Panorama).
 * - Residual threshold slider: highlights strained edges in amber.
 * - Node colour mode: uniform grey or connectivity-degree heat ramp.
 *
 * TODO: relaxation under different constraints (#445 follow-up) — visualising
 * which constraints are active in a hypothetical graph requires a new backend
 * endpoint that re-solves with a subset of edges and returns the updated poses.
 * Document as a separate issue once the endpoint contract is agreed.
 */
function PoseGraphSection({ posegraph }: { posegraph: PoseGraphPanelState }) {
  const {
    graph,
    error,
    visible,
    onToggle,
    projectionPlane,
    onProjectionPlaneChange,
    edgeTypeVisible,
    onEdgeTypeChange,
    residualThreshold,
    onResidualThresholdChange,
    nodeColorMode,
    onNodeColorModeChange,
  } = posegraph;
  const note = posegraphNote(graph, error);
  const hasNodes = (graph?.nodes.length ?? 0) > 0;
  const hasEdges = (graph?.edges.length ?? 0) > 0;

  return (
    <Section title="Pose graph">
      {note && <p className={styles.note}>{note}</p>}

      {hasNodes && (
        <>
          <label className={styles.splatToggle}>
            <input
              type="checkbox"
              checked={visible}
              onChange={(event) => onToggle(event.target.checked)}
              className={styles.checkbox}
            />
            <span className={styles.splatName}>Show pose graph</span>
          </label>

          <div className={styles.fieldLabel}>Projection</div>
          <div className={styles.modes} role="group" aria-label="Projection plane">
            {PROJECTION_PLANES.map((plane) => (
              <button
                key={plane}
                type="button"
                className={`${styles.mode} ${projectionPlane === plane ? styles.modeActive : ''}`}
                onClick={() => onProjectionPlaneChange(plane)}
              >
                {plane}
              </button>
            ))}
          </div>

          <div className={styles.fieldLabel}>Node colour</div>
          <div className={styles.modes} role="group" aria-label="Node colour mode">
            <button
              type="button"
              className={`${styles.mode} ${nodeColorMode === 'default' ? styles.modeActive : ''}`}
              onClick={() => onNodeColorModeChange('default')}
            >
              Default
            </button>
            <button
              type="button"
              className={`${styles.mode} ${nodeColorMode === 'degree' ? styles.modeActive : ''}`}
              onClick={() => onNodeColorModeChange('degree')}
            >
              By degree
            </button>
          </div>

          {hasEdges && (
            <>
              <div className={styles.fieldLabel}>Edge types</div>
              {EDGE_TYPES.map((type) => {
                const [r, g, b] = EDGE_BASE_COLORS[type];
                return (
                  <label key={type} className={styles.splatToggle}>
                    <input
                      type="checkbox"
                      checked={edgeTypeVisible[type] ?? true}
                      onChange={(event) => onEdgeTypeChange(type, event.target.checked)}
                      className={styles.checkbox}
                    />
                    <span
                      className={styles.edgeSwatch}
                      style={{
                        backgroundColor: `rgb(${Math.round(r * 255)},${Math.round(g * 255)},${Math.round(b * 255)})`,
                      }}
                      aria-hidden="true"
                    />
                    <span className={styles.splatName}>{EDGE_TYPE_LABELS[type]}</span>
                  </label>
                );
              })}

              <label className={styles.field}>
                <span className={styles.fieldLabel}>
                  Residual threshold
                  <span className="mono">
                    {residualThreshold === 0 ? 'Off' : residualThreshold.toFixed(2)}
                  </span>
                </span>
                <input
                  type="range"
                  min={0}
                  max={10}
                  step={0.05}
                  value={residualThreshold}
                  onChange={(event) =>
                    onResidualThresholdChange(Number(event.target.value))
                  }
                  className={styles.range}
                />
              </label>
            </>
          )}
        </>
      )}
    </Section>
  );
}
