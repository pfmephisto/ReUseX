// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useNavigate, useSearchParams } from 'react-router-dom';

import { api } from '../api/client';
import type { CloudInfo, FrameVisibilityList, GsplatInfo, MeshInfo, PanoramaInfo, PoseGraph, PoseGraphEdgeType } from '../api/types';
import { useAsync } from '../app/useAsync';
import { ErrorBanner } from '../components/ErrorBanner';
import { EmptyState } from '../components/EmptyState';
import { LayerPanel, type BoxCorner } from '../components/LayerPanel';
import { PanoramaBar } from '../components/PanoramaBar';
import { SourceImagePanel } from '../components/SourceImagePanel';
import { Spinner } from '../components/Spinner';
import {
  Viewport,
  type MeshLayerState,
  type PanoramaLayerState,
  type SelectedEdge,
  type SplatLayerState,
  type ViewportLayer,
  type ViewportMesh,
} from '../viewport/Viewport';
import type { PanoramaMarker } from '../viewport/PanoramaScene';
import type { ColorMode } from '../viewport/PointCloudScene';
import {
  DEFAULT_LIGHTING,
  type CameraProjection,
  type LightingState,
  type ViewPreset,
} from '../viewport/cameraViews';
import { resolvePlacement, stepPanorama } from '../viewport/panorama';
import type { ProjectionPlane } from '../viewport/posegraphLayer';
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
  const navigate = useNavigate();

  // Thumbnail-capture mode: entered from a material passport's thumbnail cell
  // via `/viewport?captureFor=<guid>`. When set, a floating button grabs the
  // Three.js canvas and PUTs it as that passport's thumbnail, then returns to
  // the materials table.
  const captureFor = params.get('captureFor');
  const [capturing, setCapturing] = useState(false);
  const [captureError, setCaptureError] = useState<string | null>(null);

  const captureThumbnail = useCallback(() => {
    if (!captureFor || capturing) return;
    const canvas = document.querySelector('canvas') as HTMLCanvasElement | null;
    if (!canvas) {
      setCaptureError('No 3D viewport found');
      return;
    }
    setCapturing(true);
    setCaptureError(null);
    // Firefox-compatible: canvas.toBlob rather than any Chrome-only capture API.
    canvas.toBlob(
      (blob) => {
        if (!blob) {
          setCapturing(false);
          setCaptureError('Could not read the viewport image');
          return;
        }
        const file = new File([blob], 'thumbnail.png', { type: 'image/png' });
        api
          .uploadThumbnail(captureFor, file)
          .then(() => navigate('/data?tab=materials'))
          .catch((error: unknown) => {
            setCaptureError(error instanceof Error ? error.message : String(error));
            setCapturing(false);
          });
      },
      'image/png',
    );
  }, [captureFor, capturing, navigate]);

  const {
    data: clouds,
    error,
    loading,
    reload,
  } = useAsync<CloudInfo[]>((signal) => api.clouds(signal), []);

  // Mesh list — metadata only. Blobs stay in the project until switched on.
  const { data: meshes, error: meshError } = useAsync<MeshInfo[]>(
    (signal) => api.meshes(signal),
    [],
  );

  const [meshVisible, setMeshVisible] = useState<Record<string, boolean>>({});
  const [meshWireframe, setMeshWireframe] = useState<Record<string, boolean>>({});
  // Sticky: once a download has started, hiding the layer must not throw it away.
  const [meshRequested, setMeshRequested] = useState<Record<string, boolean>>({});
  const [meshLoading, setMeshLoading] = useState<Record<string, MeshLayerState>>({});

  const handleMeshProgress = useCallback((name: string, state: MeshLayerState) => {
    setMeshLoading((current) => ({ ...current, [name]: state }));
  }, []);
  const handleToggleMesh = useCallback((name: string, next: boolean) => {
    setMeshVisible((current) => ({ ...current, [name]: next }));
    if (next) setMeshRequested((current) => ({ ...current, [name]: true }));
  }, []);
  const handleWireframeMesh = useCallback((name: string, next: boolean) => {
    setMeshWireframe((current) => ({ ...current, [name]: next }));
  }, []);

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

  // Camera and lighting controls (#443). The projection and light rig are
  // ordinary state; a preset view is a one-shot command, so it carries a nonce
  // the viewport effect keys on — asking for the same view twice re-frames.
  const [projection, setProjection] = useState<CameraProjection>('perspective');
  const [lighting, setLighting] = useState<LightingState>(DEFAULT_LIGHTING);
  const [viewRequest, setViewRequest] = useState<{ preset: ViewPreset; nonce: number } | null>(
    null,
  );
  const handleLightingChange = useCallback((next: Partial<LightingState>) => {
    setLighting((current) => ({ ...current, ...next }));
  }, []);

  // --- clipping box (#444) ------------------------------------------------

  const [clippingEnabled, setClippingEnabled] = useState(false);
  const [clippingMin, setClippingMin] = useState<BoxCorner | null>(null);
  const [clippingMax, setClippingMax] = useState<BoxCorner | null>(null);

  const handleClippingBoxChange = useCallback((min: BoxCorner, max: BoxCorner) => {
    setClippingMin(min);
    setClippingMax(max);
  }, []);

  const handleClippingReset = useCallback(() => {
    setClippingMin(null);
    setClippingMax(null);
  }, []);

  // --- source-image cross-reference (#454) -----------------------------------

  const [pickMode, setPickMode] = useState(false);
  const [pickedPoint, setPickedPoint] = useState<{ x: number; y: number; z: number } | null>(null);
  const [visibility, setVisibility] = useState<FrameVisibilityList | null | undefined>(undefined);
  const [visibilityError, setVisibilityError] = useState<Error | null>(null);
  // Abort the in-flight fetch when the user picks a new point.
  const visibilityAbortRef = useRef<AbortController | null>(null);

  const handlePickPoint = useCallback((point: { x: number; y: number; z: number }) => {
    visibilityAbortRef.current?.abort();
    const controller = new AbortController();
    visibilityAbortRef.current = controller;
    setPickedPoint(point);
    setPickMode(false);
    setVisibility(null);
    setVisibilityError(null);
    api
      .pointVisibility(point.x, point.y, point.z, { limit: 12 }, controller.signal)
      .then((result) => {
        if (!controller.signal.aborted) setVisibility(result);
      })
      .catch((error: unknown) => {
        if (controller.signal.aborted) return;
        setVisibilityError(error instanceof Error ? error : new Error(String(error)));
      });
  }, []);

  const handleCloseSourcePanel = useCallback(() => {
    visibilityAbortRef.current?.abort();
    setPickedPoint(null);
    setVisibility(undefined);
    setVisibilityError(null);
  }, []);

  // Abort any in-flight visibility fetch on unmount (e.g. the user navigates away).
  useEffect(() => () => { visibilityAbortRef.current?.abort(); }, []);

  const handleOpenFrame = useCallback(
    (frameId: number) => {
      navigate(`/frames?frame=${frameId}`);
    },
    [navigate],
  );

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

  // --- pose graph (#265, review pt 4; #445; #407) ---------------------------

  const { data: poseGraphData, error: poseGraphError, reload: reloadPoseGraph } =
    useAsync<PoseGraph>((signal) => api.posegraph(signal), []);
  const [poseGraphVisible, setPoseGraphVisible] = useState(false);
  const [pgProjectionPlane, setPgProjectionPlane] = useState<ProjectionPlane>('3D');
  const [pgEdgeTypeVisible, setPgEdgeTypeVisible] = useState<Record<PoseGraphEdgeType, boolean>>({
    odometry: true,
    loop_closure: true,
    panorama: true,
  });
  const [pgResidualThreshold, setPgResidualThreshold] = useState(0);
  const [pgNodeColorMode, setPgNodeColorMode] = useState<'default' | 'degree'>('default');
  const [pgSelectedEdge, setPgSelectedEdge] = useState<SelectedEdge | null>(null);

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

  const handleView = useCallback(
    (preset: ViewPreset) => {
      // Framing a preset from inside a panorama is a request to stop being
      // inside it — the same reading `onFrame` gives a Frame-all press.
      if (immersive) enterPanorama(null);
      setViewRequest((prev) => ({ preset, nonce: (prev?.nonce ?? 0) + 1 }));
    },
    [immersive, enterPanorama],
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
        projection={projection}
        lighting={lighting}
        view={viewRequest}
        onLayerProgress={handleProgress}
        meshes={(meshes ?? [])
          .filter((info) => meshRequested[info.name])
          .map<ViewportMesh>((info) => ({
            name: info.name,
            url: api.meshDataUrl(info.name),
            visible: meshVisible[info.name] ?? false,
            wireframe: meshWireframe[info.name] ?? false,
          }))}
        onMeshProgress={handleMeshProgress}
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
        poseGraph={poseGraphData ?? null}
        poseGraphVisible={poseGraphVisible}
        poseGraphProjectionPlane={pgProjectionPlane}
        poseGraphEdgeTypeVisible={pgEdgeTypeVisible}
        poseGraphResidualThreshold={pgResidualThreshold}
        poseGraphNodeColorMode={pgNodeColorMode}
        poseGraphSelectedEdge={pgSelectedEdge}
        onPickPoseGraphEdge={setPgSelectedEdge}
        clipping={{
          enabled: clippingEnabled,
          min: clippingMin,
          max: clippingMax,
          onBoxChange: handleClippingBoxChange,
        }}
        pickMode={pickMode}
        onPickPoint={handlePickPoint}
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
          ) : (
            <>
              {!immersive && (
                <PickButton active={pickMode} onToggle={() => setPickMode((m) => !m)} />
              )}
              {pickedPoint !== null && (
                <SourceImagePanel
                  visibility={visibility}
                  error={visibilityError}
                  onClose={handleCloseSourcePanel}
                  onOpenFrame={handleOpenFrame}
                />
              )}
            </>
          )
        }
      />
      <LayerPanel
        clouds={renderable}
        mesh={{
          items: meshes ?? null,
          error: meshError ?? null,
          visible: meshVisible,
          wireframe: meshWireframe,
          loading: meshLoading,
          onToggle: handleToggleMesh,
          onWireframeToggle: handleWireframeMesh,
        }}
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
        posegraph={{
          graph: poseGraphData ?? null,
          error: poseGraphError ?? null,
          visible: poseGraphVisible,
          onToggle: setPoseGraphVisible,
          projectionPlane: pgProjectionPlane,
          onProjectionPlaneChange: setPgProjectionPlane,
          edgeTypeVisible: pgEdgeTypeVisible,
          onEdgeTypeChange: (type, vis) =>
            setPgEdgeTypeVisible((current) => ({ ...current, [type]: vis })),
          residualThreshold: pgResidualThreshold,
          onResidualThresholdChange: setPgResidualThreshold,
          nodeColorMode: pgNodeColorMode,
          onNodeColorModeChange: setPgNodeColorMode,
          selectedEdge: pgSelectedEdge,
          onEdgeSelect: setPgSelectedEdge,
          onGraphChanged: reloadPoseGraph,
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
        projection={projection}
        onProjectionChange={setProjection}
        onView={handleView}
        lighting={lighting}
        onLightingChange={handleLightingChange}
        clipping={{
          enabled: clippingEnabled,
          min: clippingMin,
          max: clippingMax,
          onEnabledChange: setClippingEnabled,
          onBoxChange: handleClippingBoxChange,
          onReset: handleClippingReset,
        }}
        onFrame={() => {
          // Framing the whole scan from inside a panorama is a request to
          // stop being inside it; leaving the backdrop up while the camera
          // flies out would look like the viewer had broken.
          if (immersive) enterPanorama(null);
          setFrameToken((token) => token + 1);
        }}
      />
      {captureFor && (
        <div className={styles.captureBar}>
          {captureError && <span className={styles.captureError}>{captureError}</span>}
          <button
            type="button"
            className={styles.captureButton}
            onClick={captureThumbnail}
            disabled={capturing}
          >
            {capturing ? 'Saving…' : 'Set as thumbnail'}
          </button>
        </div>
      )}
    </div>
  );
}

/**
 * Small overlay toggle for point-pick mode (#454).
 *
 * Positioned at the top-left of the viewport canvas so it does not overlap the
 * SourceImagePanel (top-right) or the PanoramaBar (top-center). Uses inline
 * styles built from design tokens so no extra CSS module is needed for a
 * two-state icon button.
 */
function PickButton({ active, onToggle }: { active: boolean; onToggle: () => void }) {
  return (
    <button
      type="button"
      onClick={onToggle}
      title={active ? 'Cancel pick (click a point in the cloud to inspect source images)' : 'Pick a point to find source images'}
      aria-pressed={active}
      style={{
        position: 'absolute',
        top: 'var(--space-3)',
        left: 'var(--space-3)',
        zIndex: 10,
        display: 'inline-flex',
        alignItems: 'center',
        gap: 'var(--space-1)',
        padding: 'var(--space-1) var(--space-2)',
        border: `1px solid ${active ? 'var(--color-accent)' : 'var(--color-border-strong)'}`,
        borderRadius: 'var(--radius-sm)',
        background: active ? 'var(--color-accent-muted)' : 'var(--color-surface-raised)',
        color: active ? 'var(--color-accent)' : 'var(--color-text-muted)',
        fontFamily: 'var(--font-sans)',
        fontSize: 'var(--font-size-xs)',
        cursor: 'pointer',
      }}
    >
      ⊕ {active ? 'Cancel pick' : 'Pick point'}
    </button>
  );
}
