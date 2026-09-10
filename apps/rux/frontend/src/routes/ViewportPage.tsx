// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useState } from 'react';
import { useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import type { CloudInfo, GsplatInfo, PanoramaInfo } from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { EmptyState } from '../components/EmptyState';
import { LayerPanel } from '../components/LayerPanel';
import { PanoramaBar } from '../components/PanoramaBar';
import { Spinner } from '../components/Spinner';
import {
  Viewport,
  type PanoramaLayerState,
  type SplatLayerState,
  type ViewportLayer,
} from '../viewport/Viewport';
import type { PanoramaMarker } from '../viewport/PanoramaScene';
import type { ColorMode } from '../viewport/PointCloudScene';
import { resolvePlacement, stepPanorama } from '../viewport/panorama';
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

  // --- 360 panoramas (#265, Phase 5) --------------------------------------

  const { data: panoramas, error: panoramaError } = useAsync<PanoramaInfo[]>(
    (signal) => api.panoramas(signal),
    [],
  );

  const [markersVisible, setMarkersVisible] = useState(true);
  const [panoramaState, setPanoramaState] = useState<PanoramaLayerState>({
    loading: false,
    error: null,
  });
  // Off by default: the panorama is what the user asked to look at. See
  // `PanoramaBar` for why it is offered at all.
  const [overlayGeometry, setOverlayGeometry] = useState(false);

  // Placeable panoramas only. One with neither an aligned pose nor a matched
  // frame pose has no position, and drawing it at the origin would put a
  // photograph somewhere the building is not — see `resolvePlacement`.
  const markers = useMemo<PanoramaMarker[]>(
    () =>
      (panoramas ?? []).flatMap((pano) => {
        const placement = resolvePlacement(pano);
        return placement ? [{ id: pano.id, placement }] : [];
      }),
    [panoramas],
  );

  // `?pano=<id>` is the deep link, and it is also where the active panorama
  // lives: the URL is then shareable and survives a reload, which a piece of
  // component state would not.
  const panoParam = params.get('pano');
  const requestedPano = panoParam === null ? null : Number(panoParam);
  const activePano = useMemo(
    () =>
      requestedPano === null
        ? null
        : ((panoramas ?? []).find((pano) => pano.id === requestedPano) ?? null),
    [panoramas, requestedPano],
  );
  const activeMarker = useMemo(
    () => markers.find((marker) => marker.id === activePano?.id) ?? null,
    [markers, activePano],
  );
  // Immersive only once a placement exists: a panorama named in the URL but
  // unplaceable must not leave the page in a mode it cannot render.
  const immersive = activeMarker !== null;

  const enterPanorama = useCallback(
    (id: number | null) => {
      const next = new URLSearchParams(params);
      if (id === null) next.delete('pano');
      else next.set('pano', String(id));
      setParams(next, { replace: true });
    },
    [params, setParams],
  );

  const stepBy = useCallback(
    (delta: number) => {
      const id = stepPanorama(
        markers.map((marker) => marker.id),
        activeMarker?.id ?? null,
        delta,
      );
      if (id !== null) enterPanorama(id);
    },
    [markers, activeMarker, enterPanorama],
  );

  // `Esc` to leave and `[` / `]` to step, the keys `rux view` already uses.
  // Bound only while immersive so they stay available to the rest of the app.
  useEffect(() => {
    if (!immersive) return;
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.metaKey || event.ctrlKey || event.altKey) return;
      if (event.key === 'Escape') enterPanorama(null);
      else if (event.key === '[') stepBy(-1);
      else if (event.key === ']') stepBy(1);
      else return;
      event.preventDefault();
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [immersive, enterPanorama, stepBy]);

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

  // Inside a panorama the geometry is hidden unless the user asks for it —
  // `rux view` hides every other prop on entering, and this keeps that
  // default. Hidden, not unmounted: the pages stay resident, so leaving the
  // panorama does not re-download the cloud.
  const geometryHidden = immersive && !overlayGeometry;

  const layers = useMemo<ViewportLayer[]>(
    () =>
      renderable
        .filter((cloud) => visible[cloud.name] !== undefined)
        .map((cloud) => ({
          cloud: cloud.name,
          visible: geometryHidden ? false : (visible[cloud.name] ?? false),
          // Only the layer the label cloud is length-compatible with gets it.
          labelCloud:
            activeLabelCloud && primary?.name === cloud.name ? activeLabelCloud : null,
        })),
    [renderable, visible, activeLabelCloud, primary, geometryHidden],
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
  // A splat or a placeable panorama is also something to render, so the empty
  // state is only honest when there is none of the three.
  if (renderable.length === 0 && (gsplats ?? []).length === 0 && markers.length === 0) {
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
        panoramas={markers}
        showPanoramaMarkers={markersVisible && !immersive}
        activePanorama={
          activeMarker
            ? {
                marker: activeMarker,
                // Capped rather than native. A stored equirect can be
                // 12000 px wide; as an RGBA texture with mipmaps that is
                // hundreds of megabytes of GPU memory for detail no viewport
                // can show, and the upload stalls the first frames.
                imageUrl: api.panoramaImageUrl(activeMarker.id, { maxSize: 4096 }),
              }
            : null
        }
        onPanoramaState={setPanoramaState}
        onPickPanorama={enterPanorama}
        overlay={
          immersive && activePano ? (
            <PanoramaBar
              panorama={activePano}
              placement={activeMarker?.placement ?? null}
              index={markers.findIndex((marker) => marker.id === activePano.id) + 1}
              total={markers.length}
              loading={panoramaState.loading}
              error={panoramaState.error}
              showGeometry={overlayGeometry}
              onShowGeometryChange={setOverlayGeometry}
              onStep={stepBy}
              onExit={() => enterPanorama(null)}
            />
          ) : null
        }
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
        panorama={{
          items: panoramas ?? null,
          error: panoramaError ?? null,
          activeId: activeMarker?.id ?? null,
          markersVisible,
          onMarkersVisibleChange: setMarkersVisible,
          onEnter: enterPanorama,
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
        onFrame={() => {
          // Framing the whole scan from inside a panorama is a request to
          // stop being inside it; leaving the backdrop up while the camera
          // flies out would look like the viewer had broken.
          if (immersive) enterPanorama(null);
          setFrameToken((token) => token + 1);
        }}
      />
    </div>
  );
}
