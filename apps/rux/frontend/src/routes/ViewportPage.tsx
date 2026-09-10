// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useMemo, useState } from 'react';
import { useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import type { CloudInfo, GsplatInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { EmptyState } from '../components/EmptyState';
import { LayerPanel } from '../components/LayerPanel';
import { Spinner } from '../components/Spinner';
import { Viewport, type SplatLayerState, type ViewportLayer } from '../viewport/Viewport';
import type { ColorMode } from '../viewport/PointCloudScene';
import type { CloudStreamState } from '../viewport/useCloudStream';
import styles from './ViewportPage.module.css';

/** Cloud types this viewport can draw. */
const RENDERABLE = new Set(['PointXYZRGB', 'PointXYZ']);

/**
 * The 3D view: canvas plus layer panel.
 *
 * Owns which clouds are on, which label cloud tints them, and the camera-framing
 * trigger. Everything below is presentational or imperative; nothing here knows
 * about three.js.
 *
 * ## Which label clouds are offered, and why it is a filter
 *
 * `docs/CONTRACTS.md` guarantees that the sibling clouds of **one scan**
 * (`cloud` / `normals` / `planes` / `rooms` / `instances` / `labels`) are
 * index-aligned, which is what lets this zip a label page onto a geometry page
 * by position. It guarantees nothing between a geometry cloud and an unrelated
 * label cloud — a downsampled or filtered cloud has neither the same length nor
 * the same ordering.
 *
 * Matching point counts is the strongest check available from `/clouds` alone.
 * It is necessary, not sufficient (two clouds of equal length could still be
 * ordered differently), so it is used to *exclude* the obviously-wrong sources
 * rather than to bless the remainder. Silently colouring a cloud with labels
 * belonging to different points would produce a plausible, entirely fictional
 * segmentation — the worst possible failure for a tool an architect is meant to
 * make decisions from.
 */
export function ViewportPage() {
  const [params, setParams] = useSearchParams();

  const {
    data: clouds,
    error,
    loading,
    reload,
  } = useAsync<CloudInfo[]>((signal) => api.clouds(signal), []);

  // Asked for once alongside the cloud inventory. Metadata only — the blobs
  // stay in the project until a layer is switched on.
  const { data: gsplats, error: gsplatError } = useAsync<GsplatInfo[]>(
    (signal) => api.gsplats(signal),
    [],
  );

  const [visible, setVisible] = useState<Record<string, boolean>>({});
  const [progress, setProgress] = useState<Record<string, CloudStreamState>>({});
  // `null` means "follow the label source". Set only when the user picks a mode
  // explicitly, so that arriving on ?labels=planes shows labels — a deep link
  // that selects a label source but renders sensor RGB is telling the user two
  // different things at once — while still letting them flip back to RGB to
  // compare without the selection being yanked out from under them.
  const [explicitColorMode, setExplicitColorMode] = useState<ColorMode | null>(null);
  const [pointSize, setPointSize] = useState(0.02);
  const [frameToken, setFrameToken] = useState(0);
  const [initialised, setInitialised] = useState(false);

  // `?splat=<name>` deep-links one on. Otherwise every splat starts off: the
  // blob is hundreds of megabytes where a cloud streams in pages, so loading
  // one has to be a decision the user made — which is why the panel shows the
  // size beside the toggle.
  const deepLinkedSplat = params.get('splat');
  const [splatVisible, setSplatVisible] = useState<Record<string, boolean>>(() =>
    deepLinkedSplat ? { [deepLinkedSplat]: true } : {},
  );
  // Sticky: once a download has started, hiding the layer must not throw it
  // away. This is what keeps the toggle instant on the second click, the same
  // rule `CloudLayerLoader` follows for a cloud's pages.
  const [splatRequested, setSplatRequested] = useState<Record<string, boolean>>(() =>
    deepLinkedSplat ? { [deepLinkedSplat]: true } : {},
  );
  const [splatLoading, setSplatLoading] = useState<Record<string, SplatLayerState>>({});
  const handleSplatProgress = useCallback((name: string, state: SplatLayerState) => {
    setSplatLoading((current) => ({ ...current, [name]: state }));
  }, []);
  const handleToggleSplat = useCallback((name: string, next: boolean) => {
    setSplatVisible((current) => ({ ...current, [name]: next }));
    if (next) setSplatRequested((current) => ({ ...current, [name]: true }));
  }, []);

  const renderable = useMemo(
    () => (clouds ?? []).filter((cloud) => RENDERABLE.has(cloud.type)),
    [clouds],
  );
  const labelClouds = useMemo(
    () => (clouds ?? []).filter((cloud) => cloud.type === 'Label'),
    [clouds],
  );

  // Turn on the cloud named in ?cloud=, else the first renderable one. Done as
  // a render-time fold rather than an effect so the first paint already has the
  // layer switched on and the download has started.
  const requested = params.get('cloud');
  if (!initialised && renderable.length > 0) {
    const pick =
      renderable.find((cloud) => cloud.name === requested) ??
      renderable.find((cloud) => cloud.name === 'cloud') ??
      renderable[0];
    setVisible({ [pick.name]: true });
    setInitialised(true);
  }

  const primary = useMemo(
    () => renderable.find((cloud) => visible[cloud.name]),
    [renderable, visible],
  );

  const labelSources = useMemo(
    () =>
      primary
        ? labelClouds.filter((cloud) => cloud.point_count === primary.point_count)
        : [],
    [labelClouds, primary],
  );

  const labelCloud = params.get('labels');
  const activeLabelCloud =
    labelCloud && labelSources.some((cloud) => cloud.name === labelCloud) ? labelCloud : null;

  const colorMode: ColorMode = explicitColorMode ?? (activeLabelCloud ? 'label' : 'rgb');

  const layers = useMemo<ViewportLayer[]>(
    () =>
      renderable
        .filter((cloud) => visible[cloud.name] !== undefined)
        .map((cloud) => ({
          cloud: cloud.name,
          visible: visible[cloud.name] ?? false,
          // Only the layer the label cloud is length-compatible with gets it.
          labelCloud:
            activeLabelCloud && primary?.name === cloud.name ? activeLabelCloud : null,
        })),
    [renderable, visible, activeLabelCloud, primary],
  );

  const handleProgress = useCallback((cloud: string, state: CloudStreamState) => {
    setProgress((current) => ({ ...current, [cloud]: state }));
  }, []);

  const handleLabelCloudChange = useCallback(
    (name: string | null) => {
      const next = new URLSearchParams(params);
      if (name) next.set('labels', name);
      else next.delete('labels');
      setParams(next, { replace: true });
      // Drop any explicit override so the mode follows the new source: picking
      // a label source and seeing nothing change is a dead end.
      setExplicitColorMode(null);
    },
    [params, setParams],
  );

  if (loading && !clouds) return <Spinner label="Loading clouds…" />;
  if (error) return <ErrorBanner error={error} onRetry={reload} context="cloud inventory" />;

  // A splat alone is something to render, so the empty state is only honest
  // when there is neither. A project whose splat was imported, or whose seed
  // cloud was since deleted, is the case this covers.
  if (renderable.length === 0 && (gsplats ?? []).length === 0) {
    return (
      <EmptyState
        title="Nothing to render yet"
        detail="This project has no PointXYZRGB or PointXYZ cloud. Run `rux create clouds` to back-project the sensor frames into one."
      />
    );
  }

  return (
    <div className={styles.page}>
      <Viewport
        layers={layers}
        colorMode={colorMode}
        pointSize={pointSize}
        frameToken={frameToken}
        onLayerProgress={handleProgress}
        splats={(gsplats ?? [])
          .filter((info) => splatRequested[info.name])
          .map((info) => ({
            name: info.name,
            url: api.gsplatDataUrl(info.name),
            visible: splatVisible[info.name] ?? false,
          }))}
        onSplatProgress={handleSplatProgress}
      />
      <LayerPanel
        clouds={renderable}
        splat={{
          items: gsplats ?? null,
          error: gsplatError ?? null,
          visible: splatVisible,
          loading: splatLoading,
          onToggle: handleToggleSplat,
        }}
        visible={visible}
        progress={progress}
        onToggleLayer={(name, next) =>
          setVisible((current) => ({ ...current, [name]: next }))
        }
        labelSources={labelSources}
        labelCloud={activeLabelCloud}
        onLabelCloudChange={handleLabelCloudChange}
        labelSourceNote={
          labelClouds.length === 0
            ? 'No label clouds in this project. `rux create planes`, `rooms` or `instances` produce them.'
            : 'No label cloud matches the visible cloud point-for-point, so none can be zipped onto it safely.'
        }
        colorMode={colorMode}
        onColorModeChange={setExplicitColorMode}
        pointSize={pointSize}
        onPointSizeChange={setPointSize}
        onFrame={() => setFrameToken((token) => token + 1)}
      />
    </div>
  );
}
