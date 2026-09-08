// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { useEffect, useRef, useState } from 'react';

import { PointCloudScene, type ColorMode } from './PointCloudScene';
import { useCloudStream, type CloudStreamState } from './useCloudStream';
import styles from './Viewport.module.css';

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
  onLayerProgress?: (cloud: string, state: CloudStreamState) => void;
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
  onLayerProgress,
}: ViewportProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [scene, setScene] = useState<PointCloudScene | null>(null);
  const framedOnce = useRef(false);

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
    for (const layer of layers) scene?.setLayerVisible(layer.cloud, layer.visible);
  }, [scene, layers]);

  useEffect(() => {
    // frameToken 0 is the initial value and must not steal the automatic
    // first-content framing below.
    if (frameToken > 0) scene?.frameAll();
  }, [scene, frameToken]);

  return (
    <div className={styles.host}>
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
    </div>
  );
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
    onPage: (buffers) => {
      scene.addPage(layer.cloud, buffers);
      scene.setLayerVisible(layer.cloud, layer.visible);
      onFirstContent();
    },
  });

  useEffect(() => {
    onProgress?.(layer.cloud, state);
    // `onProgress` is an inline closure at the call site; including it here
    // would fire this effect every render of the parent.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [layer.cloud, state]);

  useEffect(() => {
    return () => scene.removeLayer(layer.cloud);
  }, [scene, layer.cloud]);

  return null;
}
