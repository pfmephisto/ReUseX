// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import {
  type PointerEvent as ReactPointerEvent,
  type ReactNode,
  useEffect,
  useRef,
  useState,
} from 'react';
import * as THREE from 'three';

import { PointCloudScene, type ColorMode } from './PointCloudScene';
import type { CameraProjection, LightingState, ViewPreset } from './cameraViews';
import { ClippingBoxLayer } from './ClippingBoxLayer';
import { MeshScene } from './MeshScene';
import { PanoramaScene, type PanoramaMarker } from './PanoramaScene';
import { PoseGraphScene } from './PoseGraphScene';
import { SplatScene } from './SplatScene';
import { useCloudStream, type CloudStreamState } from './useCloudStream';
import styles from './Viewport.module.css';
import type { PoseGraph, PoseGraphEdgeType } from '../api/types';
import type { ProjectionPlane } from './posegraphLayer';

/** Progress of the PLY mesh layer, as the panel reports it (#265, review pt 2). */
export interface MeshLayerState {
  loading: boolean;
  error: Error | null;
}

/** One mesh the viewport should render. */
export interface ViewportMesh {
  /** Mesh name — also the scene layer id. */
  name: string;
  /** Where the renderer fetches the PLY blob. */
  url: string;
  visible: boolean;
  wireframe: boolean;
}

/** Progress of the Gaussian-splat layer, as the panel reports it. */
export interface SplatLayerState {
  loading: boolean;
  /** 0..1 while downloading, null before the first progress callback. */
  fraction: number | null;
  loaded: boolean;
  error: Error | null;
}

export interface ViewportSplat {
  /** The splat's name in the project — also the scene layer id. */
  name: string;
  /** Where the renderer fetches the INRIA `.ply`. */
  url: string;
  visible: boolean;
}

export interface ViewportLayer {
  /** Cloud name — also the scene layer id. */
  cloud: string;
  visible: boolean;
  /**
   * `Label` cloud to colour this layer by, or null.
   *
   * The caller is responsible for only supplying one that is genuinely
   * index-aligned with this layer; see `ViewportPage`.
   */
  labelCloud: string | null;
}

export interface ViewportProps {
  layers: ViewportLayer[];
  colorMode: ColorMode;
  pointSize: number;
  /** Bump to re-frame the camera on the loaded content. */
  frameToken: number;

  /** Orbit-camera projection (#443). Defaults to perspective when omitted. */
  projection?: CameraProjection;
  /** Key + fill light settings (#443). Defaults to the scene's own when omitted. */
  lighting?: LightingState;
  /**
   * A pending preset-view request (#443).
   *
   * A `{ preset, nonce }` rather than a bare preset so that asking for the same
   * view twice still re-frames: the effect keys on the object identity, which
   * the page changes by bumping the nonce on every button press.
   */
  view?: { preset: ViewPreset; nonce: number } | null;

  onLayerProgress?: (cloud: string, state: CloudStreamState) => void;
  /**
   * PLY mesh layers (#265, review pt 2).
   *
   * Only meshes the user has switched on belong here — the blob can be large
   * and the loader starts fetching immediately on mount.
   */
  meshes?: ViewportMesh[];
  onMeshProgress?: (name: string, state: MeshLayerState) => void;

  /**
   * The Gaussian-splat layers the page has asked to be loaded (#322).
   *
   * A separate array rather than more `ViewportLayer`s: a splat is not a
   * cloud, it does not stream in pages, and it is not coloured by a label
   * source — folding it into that array would mean every field of
   * `ViewportLayer` becoming optional to describe something it does not model.
   *
   * Only splats that should actually be downloaded belong here. A splat the
   * user has never switched on must be absent, not present-and-hidden: the
   * blob is hundreds of megabytes and mounting the loader starts fetching it.
   */
  splats?: ViewportSplat[];
  onSplatProgress?: (name: string, state: SplatLayerState) => void;

  /**
   * Capture positions to mark in the orbit view (#265, Phase 5).
   *
   * Only placeable panoramas belong here — one with neither an aligned pose
   * nor a matched frame pose has no position to mark, and marking it at the
   * origin would put it somewhere the scan never was.
   */
  panoramas?: PanoramaMarker[];
  showPanoramaMarkers?: boolean;
  /**
   * The panorama to stand inside, or null for the orbit view.
   *
   * A marker plus a URL rather than an id, so this component never has to know
   * how a panorama is placed or where its image comes from.
   */
  activePanorama?: { marker: PanoramaMarker; imageUrl: string } | null;
  onPanoramaState?: (state: PanoramaLayerState) => void;
  /** A marker was clicked. */
  onPickPanorama?: (id: number) => void;

  /**
   * Pose-graph overlay (#265, review pt 4; #445).
   *
   * null while the fetch is in flight; an empty object when the project has
   * never been optimised (no edges, nodes from sensor-frame poses).
   */
  poseGraph?: PoseGraph | null;
  poseGraphVisible?: boolean;
  /** Flatten graph positions to a 2D plane for inspection (#445). */
  poseGraphProjectionPlane?: ProjectionPlane;
  /** Independent edge-type visibility (#445). */
  poseGraphEdgeTypeVisible?: Record<PoseGraphEdgeType, boolean>;
  /** Highlight edges above this residual value in amber; 0 = disabled (#445). */
  poseGraphResidualThreshold?: number;
  /** Colour nodes by connectivity degree instead of uniform grey (#445). */
  poseGraphNodeColorMode?: 'default' | 'degree';

  /**
   * Clipping box state (#444).
   *
   * When present, a `ClippingBoxLayer` is mounted. The layer applies six
   * renderer-level clipping planes when enabled, and calls `onBoxChange` on
   * drag-end so the panel sliders stay in sync.
   */
  clipping?: {
    enabled: boolean;
    /** Scene-local min corner, or null to initialise from scene bounds. */
    min: { x: number; y: number; z: number } | null;
    /** Scene-local max corner, or null to initialise from scene bounds. */
    max: { x: number; y: number; z: number } | null;
    onBoxChange: (min: { x: number; y: number; z: number }, max: { x: number; y: number; z: number }) => void;
  };

  /**
   * Chrome drawn over the canvas.
   *
   * A slot rather than a component, because the host element is what makes
   * `position: absolute` mean "over the viewport" — an overlay rendered beside
   * `<Viewport>` would position itself against the page instead.
   */
  overlay?: ReactNode;

  /**
   * Point-pick mode (#454).
   *
   * When true the orbit controls are suspended; the next pointer-up that did not
   * move more than 4 px picks the nearest point-cloud point and calls
   * `onPickPoint` with its world-space coordinates.  The host is responsible for
   * toggling this back off (e.g. after the panel opens) if it only wants a single
   * pick per mode-entry.
   */
  pickMode?: boolean;
  /** Called with the world-space point the user just picked (#454). */
  onPickPoint?: (point: { x: number; y: number; z: number }) => void;
}

/** Load state of the panorama backdrop, as the page reports it. */
export interface PanoramaLayerState {
  loading: boolean;
  error: Error | null;
}

/**
 * Canvas host.
 *
 * React owns the element and the scene's lifetime; the scene owns everything
 * inside it. Nothing about the GPU state lives in component state, so a page of
 * points landing does not cause a render — see the note on `PointCloudScene`.
 */
export function Viewport({
  layers,
  colorMode,
  pointSize,
  frameToken,
  projection,
  lighting,
  view,
  onLayerProgress,
  meshes,
  onMeshProgress,
  splats,
  onSplatProgress,
  panoramas,
  showPanoramaMarkers = true,
  activePanorama,
  onPanoramaState,
  onPickPanorama,
  poseGraph,
  poseGraphVisible = false,
  poseGraphProjectionPlane,
  poseGraphEdgeTypeVisible,
  poseGraphResidualThreshold,
  poseGraphNodeColorMode,
  clipping,
  overlay,
  pickMode = false,
  onPickPoint,
}: ViewportProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [scene, setScene] = useState<PointCloudScene | null>(null);
  const framedOnce = useRef(false);
  const panoramaRef = useRef<PanoramaScene | null>(null);
  const poseGraphRef = useRef<PoseGraphScene | null>(null);

  useEffect(() => {
    if (!canvasRef.current) return;
    const instance = new PointCloudScene(canvasRef.current);
    setScene(instance);
    return () => {
      instance.dispose();
      setScene(null);
      framedOnce.current = false;
    };
  }, []);

  useEffect(() => {
    scene?.setColorMode(colorMode);
  }, [scene, colorMode]);

  useEffect(() => {
    scene?.setPointSize(pointSize);
  }, [scene, pointSize]);

  useEffect(() => {
    if (projection) scene?.setProjection(projection);
  }, [scene, projection]);

  useEffect(() => {
    if (lighting) scene?.setLighting(lighting);
  }, [scene, lighting]);

  useEffect(() => {
    if (view) scene?.setView(view.preset);
  }, [scene, view]);

  useEffect(() => {
    for (const layer of layers) {
      scene?.setLayerVisible(layer.cloud, layer.visible);
      // The coarse overview is part of the same layer as far as the user is
      // concerned, so it has to follow the same toggle. Missing this would
      // leave a ghost of a hidden layer on screen until its stream finished.
      scene?.setLayerVisible(overviewLayerId(layer.cloud), layer.visible);
    }
  }, [scene, layers]);

  useEffect(() => {
    // frameToken 0 is the initial value and must not steal the automatic
    // first-content framing below.
    if (frameToken > 0) scene?.frameAll();
  }, [scene, frameToken]);

  // --- panorama layer ------------------------------------------------------

  useEffect(() => {
    if (!scene) return;
    const layer = new PanoramaScene(scene);
    panoramaRef.current = layer;
    return () => {
      panoramaRef.current = null;
      layer.dispose();
    };
  }, [scene]);

  // Pose-graph layer (#265 review pt 4): mount once per scene, load/clear when
  // the data changes, show/hide when the toggle changes.
  useEffect(() => {
    if (!scene) return;
    const pg = new PoseGraphScene(scene.sceneRoot());
    poseGraphRef.current = pg;
    return () => {
      poseGraphRef.current = null;
      pg.dispose();
    };
  }, [scene]);

  useEffect(() => {
    poseGraphRef.current?.load(poseGraph ?? null);
  }, [scene, poseGraph]);

  useEffect(() => {
    poseGraphRef.current?.setVisible(poseGraphVisible);
  }, [scene, poseGraphVisible]);

  useEffect(() => {
    poseGraphRef.current?.setProjectionPlane(poseGraphProjectionPlane ?? '3D');
  }, [scene, poseGraphProjectionPlane]);

  useEffect(() => {
    if (!poseGraphEdgeTypeVisible) return;
    for (const [type, vis] of Object.entries(poseGraphEdgeTypeVisible) as [PoseGraphEdgeType, boolean][]) {
      poseGraphRef.current?.setEdgeTypeVisible(type, vis);
    }
  }, [scene, poseGraphEdgeTypeVisible]);

  useEffect(() => {
    poseGraphRef.current?.setResidualThreshold(poseGraphResidualThreshold ?? 0);
  }, [scene, poseGraphResidualThreshold]);

  useEffect(() => {
    poseGraphRef.current?.setNodeColorMode(poseGraphNodeColorMode ?? 'default');
  }, [scene, poseGraphNodeColorMode]);

  // Dispatch 'rux-camera-move' whenever the orbit camera changes so that
  // tile-streaming layers can re-evaluate their visible tile sets.
  useEffect(() => {
    if (!scene) return;
    return scene.onCameraChange(() => {
      if (typeof window !== 'undefined')
        window.dispatchEvent(new Event('rux-camera-move'));
    });
  }, [scene]);

  useEffect(() => {
    panoramaRef.current?.setMarkers(panoramas ?? []);
  }, [scene, panoramas]);

  useEffect(() => {
    panoramaRef.current?.setMarkersVisible(showPanoramaMarkers);
  }, [scene, panoramas, showPanoramaMarkers]);

  // Reported through a ref for the same reason the splat loader does it: both
  // callbacks are inline closures at the call site, and depending on them
  // would tear the backdrop down and re-download it on every parent render.
  const panoramaStateRef = useRef(onPanoramaState);
  panoramaStateRef.current = onPanoramaState;

  const activeId = activePanorama?.marker.id ?? null;
  const activeUrl = activePanorama?.imageUrl ?? null;
  useEffect(() => {
    const layer = panoramaRef.current;
    if (!layer) return;
    if (activeId === null || !activeUrl || !activePanorama) {
      layer.hide();
      panoramaStateRef.current?.({ loading: false, error: null });
      return;
    }

    let cancelled = false;
    panoramaStateRef.current?.({ loading: true, error: null });
    layer
      .show(activePanorama.marker.placement, activeUrl)
      .then(() => {
        if (!cancelled) panoramaStateRef.current?.({ loading: false, error: null });
      })
      .catch((error: unknown) => {
        if (cancelled) return;
        panoramaStateRef.current?.({
          loading: false,
          error: error instanceof Error ? error : new Error(String(error)),
        });
      });

    return () => {
      cancelled = true;
    };
    // Keyed by id and URL, not by the object: the page rebuilds the marker
    // array on every render, and depending on it would reload the texture.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scene, activeId, activeUrl]);

  // Suspend orbit controls while pick mode is active so a click does not also
  // start an orbit drag.
  useEffect(() => {
    scene?.setControlsEnabled(!pickMode);
  }, [scene, pickMode]);

  /**
   * Click-to-enter / click-to-pick.
   *
   * `pointerup` with a movement threshold rather than `click`, because a drag
   * that happens to end over a marker is an orbit, not a request to teleport
   * into it — and `click` cannot tell the two apart.
   */
  const pressedAt = useRef<{ x: number; y: number } | null>(null);
  const handlePointerDown = (event: ReactPointerEvent<HTMLDivElement>) => {
    pressedAt.current = { x: event.clientX, y: event.clientY };
  };
  const handlePointerUp = (event: ReactPointerEvent<HTMLDivElement>) => {
    const start = pressedAt.current;
    pressedAt.current = null;
    if (!start || !scene) return;
    if (Math.hypot(event.clientX - start.x, event.clientY - start.y) > 4) return;

    const bounds = event.currentTarget.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
      -((event.clientY - bounds.top) / bounds.height) * 2 + 1,
    );

    // Point-pick mode: raycasts the cloud geometry and calls onPickPoint (#454).
    if (pickMode && onPickPoint) {
      const world = scene.pickCloudPoint(ndc);
      if (world) onPickPoint({ x: world.x, y: world.y, z: world.z });
      return;
    }

    // Panorama-marker picking (original behaviour).
    const layer = panoramaRef.current;
    if (!layer || !onPickPanorama || !showPanoramaMarkers) return;
    for (const hit of scene.pick(ndc, layer.markerObjects())) {
      const id = layer.panoramaIdOf(hit.object);
      if (id !== null) {
        onPickPanorama(id);
        return;
      }
    }
  };

  return (
    <div
      className={`${styles.host}${pickMode ? ` ${styles.pickMode}` : ''}`}
      onPointerDown={handlePointerDown}
      onPointerUp={handlePointerUp}
    >
      <canvas ref={canvasRef} className={styles.canvas} />
      {scene &&
        layers.map((layer) => (
          <CloudLayerLoader
            key={layer.cloud}
            scene={scene}
            layer={layer}
            onFirstContent={() => {
              if (framedOnce.current) return;
              framedOnce.current = true;
              scene.frameAll();
            }}
            onProgress={onLayerProgress}
          />
        ))}
      {scene &&
        (meshes ?? []).map((mesh) => (
          <MeshLayerLoader
            key={mesh.name}
            scene={scene}
            mesh={mesh}
            onProgress={(state) => onMeshProgress?.(mesh.name, state)}
          />
        ))}
      {scene &&
        (splats ?? []).map((splat) => (
          <SplatLayerLoader
            key={splat.name}
            scene={scene}
            url={splat.url}
            visible={splat.visible}
            onFirstContent={() => {
              if (framedOnce.current) return;
              framedOnce.current = true;
              scene.frameAll();
            }}
            onProgress={(state) => onSplatProgress?.(splat.name, state)}
          />
        ))}
      {scene && clipping && (
        <ClippingBoxLayerLoader scene={scene} clipping={clipping} />
      )}
      {overlay}
    </div>
  );
}

/**
 * Loads one PLY mesh into the scene. Renders nothing.
 *
 * Mounting starts the download; unmounting disposes the geometry and removes it
 * from the scene. Hiding is handled by `MeshScene.setVisible`, not by
 * unmounting — the blob is too large to re-download on every toggle.
 */
function MeshLayerLoader({
  scene,
  mesh,
  onProgress,
}: {
  scene: PointCloudScene;
  mesh: ViewportMesh;
  onProgress?: (state: MeshLayerState) => void;
}) {
  const meshRef = useRef<MeshScene | null>(null);
  const progressRef = useRef(onProgress);
  progressRef.current = onProgress;

  useEffect(() => {
    let cancelled = false;
    const instance = new MeshScene(scene);
    meshRef.current = instance;
    progressRef.current?.({ loading: true, error: null });

    instance
      .load(mesh.url)
      .then(() => {
        if (!cancelled) progressRef.current?.({ loading: false, error: null });
      })
      .catch((error: unknown) => {
        if (cancelled) return;
        progressRef.current?.({
          loading: false,
          error: error instanceof Error ? error : new Error(String(error)),
        });
      });

    return () => {
      cancelled = true;
      meshRef.current = null;
      instance.dispose();
    };
  // Re-mount when the URL changes (different mesh); visibility is a separate effect.
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [scene, mesh.url]);

  useEffect(() => {
    meshRef.current?.setVisible(mesh.visible);
  }, [mesh.visible]);

  useEffect(() => {
    meshRef.current?.setWireframe(mesh.wireframe);
  }, [mesh.wireframe]);

  return null;
}

/**
 * Streams one layer into the scene. Renders nothing.
 *
 * A component rather than a loop inside `Viewport` so that React's own keying
 * handles the lifecycle: mounting starts a download, unmounting aborts it and
 * drops the layer's buffers. Hiding a layer does **not** unmount it — the
 * points stay resident and the toggle is instant, which is the whole reason a
 * user toggles a layer in the first place.
 */
function CloudLayerLoader({
  scene,
  layer,
  onFirstContent,
  onProgress,
}: {
  scene: PointCloudScene;
  layer: ViewportLayer;
  onFirstContent: () => void;
  onProgress?: (cloud: string, state: CloudStreamState) => void;
}) {
  const state = useCloudStream({
    cloud: layer.cloud,
    labelCloud: layer.labelCloud,
    getCameraState: () => scene.getCameraState(),
    onPage: (buffers, kind) => {
      // The overview goes into a layer of its own so it can be dropped whole
      // once the full-resolution pages have covered the same ground. Its
      // points are a subset of theirs at identical coordinates, so while both
      // are resident they coincide rather than fight — no z-fighting, just a
      // scene that is complete from the first request instead of the last.
      const id = kind.overview ? overviewLayerId(layer.cloud) : layer.cloud;
      scene.addPage(id, buffers);
      scene.setLayerVisible(id, layer.visible);
      onFirstContent();
    },
    onOverviewSuperseded: () => scene.removeLayer(overviewLayerId(layer.cloud)),
  });

  useEffect(() => {
    onProgress?.(layer.cloud, state);
    // `onProgress` is an inline closure at the call site; including it here
    // would fire this effect every render of the parent.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [layer.cloud, state]);

  useEffect(() => {
    return () => {
      scene.removeLayer(layer.cloud);
      scene.removeLayer(overviewLayerId(layer.cloud));
    };
  }, [scene, layer.cloud]);

  return null;
}

/**
 * Scene layer id for a cloud's coarse overview (#320).
 *
 * `::` cannot appear in a cloud name (they are SQL identifiers written by the
 * pipeline), so this can never collide with the layer of a real cloud.
 */
function overviewLayerId(cloud: string): string {
  return `${cloud}::overview`;
}

/**
 * Mounts the clipping box layer and drives it from React props (#444).
 *
 * Mirrors the pattern of `MeshLayerLoader`: mount once per scene, update
 * imperatively from props via effects, never re-mount for property changes.
 * The layer handles its own pointer events and calls `onBoxChange` after
 * each drag to sync the React sliders.
 */
function ClippingBoxLayerLoader({
  scene,
  clipping,
}: {
  scene: PointCloudScene;
  clipping: NonNullable<ViewportProps['clipping']>;
}) {
  const layerRef = useRef<ClippingBoxLayer | null>(null);
  const onBoxChangeRef = useRef(clipping.onBoxChange);
  onBoxChangeRef.current = clipping.onBoxChange;

  useEffect(() => {
    const layer = new ClippingBoxLayer(scene, (box) => {
      onBoxChangeRef.current({ x: box.min.x, y: box.min.y, z: box.min.z }, { x: box.max.x, y: box.max.y, z: box.max.z });
    });
    layerRef.current = layer;
    return () => {
      layerRef.current = null;
      layer.dispose();
    };
  }, [scene]);

  useEffect(() => {
    layerRef.current?.setEnabled(clipping.enabled);
  }, [clipping.enabled]);

  useEffect(() => {
    if (clipping.min && clipping.max) {
      const box = new THREE.Box3(
        new THREE.Vector3(clipping.min.x, clipping.min.y, clipping.min.z),
        new THREE.Vector3(clipping.max.x, clipping.max.y, clipping.max.z),
      );
      layerRef.current?.setBox(box);
    }
    // When min/max are null the layer auto-initialises from scene bounds on enable.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [clipping.min?.x, clipping.min?.y, clipping.min?.z, clipping.max?.x, clipping.max?.y, clipping.max?.z]);

  return null;
}

/**
 * Streams the Gaussian splat into the scene. Renders nothing.
 *
 * Mirrors `CloudLayerLoader`, including the rule that hiding is not unloading:
 * the splat is one large file and re-downloading it on every toggle would make
 * the toggle useless. Unlike a cloud it arrives as a single response, so the
 * download is started once, on mount, and visibility is a separate effect.
 *
 * Mounted only for splats the page has asked for and keyed by name, so React's
 * own lifecycle handles the download and its teardown.
 */
function SplatLayerLoader({
  scene,
  url,
  visible,
  onFirstContent,
  onProgress,
}: {
  scene: PointCloudScene;
  url: string;
  visible: boolean;
  onFirstContent: () => void;
  onProgress?: (state: SplatLayerState) => void;
}) {
  const splatRef = useRef<SplatScene | null>(null);
  // Effects below read the latest callbacks without listing them as deps: both
  // are inline closures at the call site, and depending on them would tear the
  // splat down and re-download it on every render of the parent.
  const progressRef = useRef(onProgress);
  progressRef.current = onProgress;
  const firstContentRef = useRef(onFirstContent);
  firstContentRef.current = onFirstContent;

  useEffect(() => {
    let cancelled = false;
    const instance = new SplatScene(scene, {
      onProgress: (fraction) =>
        progressRef.current?.({ loading: true, fraction, loaded: false, error: null }),
    });
    splatRef.current = instance;
    progressRef.current?.({ loading: true, fraction: null, loaded: false, error: null });

    instance
      .load(url)
      .then(() => {
        if (cancelled) return;
        progressRef.current?.({ loading: false, fraction: 1, loaded: true, error: null });
        firstContentRef.current();
      })
      .catch((error: unknown) => {
        if (cancelled) return;
        progressRef.current?.({
          loading: false,
          fraction: null,
          loaded: false,
          error: error instanceof Error ? error : new Error(String(error)),
        });
      });

    return () => {
      cancelled = true;
      splatRef.current = null;
      // Fire-and-forget: `dispose` is async because the library's is, but
      // React's cleanup is not, and nothing here waits on the teardown.
      void instance.dispose();
    };
  }, [scene, url]);

  useEffect(() => {
    splatRef.current?.setVisible(visible);
  }, [visible]);

  return null;
}
