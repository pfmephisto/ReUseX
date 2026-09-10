// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import * as THREE from 'three';

import { PANORAMA_RADIUS, type PointCloudScene } from './PointCloudScene';
import { equirectSphere, type PanoramaPlacement } from './panorama';

/**
 * The 360-panorama layer of the viewport (#265, Phase 5).
 *
 * Two things at once, because they are two views of one fact — where a
 * panorama was taken:
 *
 *  * **Markers.** A screen-space dot at every placeable panorama, drawn over
 *    the scan and pickable. This is the discoverability that `rux view`'s
 *    numbered `1`–`9` keys never had: the capture positions are *in* the scan,
 *    so the way to enter one is to click the place it was taken.
 *  * **The backdrop.** The equirect mapped onto the inside of a sphere at the
 *    panorama's pose, with the camera standing at its centre.
 *
 * Geometry is deliberately *not* hidden by this class. `rux view` hides every
 * other prop on entering panorama mode, and the page can still ask for that —
 * but a point cloud drawn inside the sphere is the cheapest visual check there
 * is on whether an alignment is right, so the choice belongs to the page.
 *
 * ## Coordinates
 *
 * Placements arrive in world coordinates; `PointCloudScene` recentres
 * everything it draws (see its class comment), so both the markers and the
 * sphere go through `toSceneLocal`. The sphere's own vertices are generated in
 * the panorama's optical frame by `equirectSphere`, which is what lets the
 * stored pose be applied to the mesh unmodified — see `panorama.ts` for the
 * two conventions involved.
 */

/**
 * Marker size, as a fraction of the viewport.
 *
 * Screen-space rather than metres, because a marker is an affordance and not
 * scan data: a 15 cm sphere in a 30 m building is one pixel from the framing
 * distance, which is precisely the view a user picks a capture position from.
 */
const MARKER_SCALE = 0.018;

/** The dot a marker is drawn as: filled disc, darker rim. */
function markerTexture(): THREE.Texture {
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const context = canvas.getContext('2d');
  if (context) {
    context.beginPath();
    context.arc(size / 2, size / 2, size / 2 - 4, 0, Math.PI * 2);
    context.fillStyle = '#ffffff';
    context.fill();
    // Drawn white and tinted by the material's colour, so one texture serves
    // both provenance colours.
    context.lineWidth = 6;
    context.strokeStyle = 'rgba(0, 0, 0, 0.55)';
    context.stroke();
  }
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/** A panorama the scene can draw a marker for. */
export interface PanoramaMarker {
  id: number;
  placement: PanoramaPlacement;
}

/** Read a design token, so the 3D view stays inside the design system. */
function token(name: string, fallback: string): string {
  const value = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return value || fallback;
}

export class PanoramaScene {
  private readonly markerGroup = new THREE.Group();
  private readonly markerIds = new Map<THREE.Object3D, number>();
  private readonly markerTexture = markerTexture();
  private readonly resectedMaterial: THREE.SpriteMaterial;
  private readonly levelledMaterial: THREE.SpriteMaterial;

  private sphere: THREE.Mesh | null = null;
  private texture: THREE.Texture | null = null;
  private disposed = false;

  constructor(private readonly host: PointCloudScene) {
    this.host.sceneRoot().add(this.markerGroup);

    const material = (color: string) =>
      new THREE.SpriteMaterial({
        map: this.markerTexture,
        color: new THREE.Color(color),
        sizeAttenuation: false,
        // Markers are chrome, not geometry. Depth-testing them would hide
        // every capture position inside the building behind the ceiling —
        // which is exactly the view the scan is framed from.
        depthTest: false,
        depthWrite: false,
      });

    this.resectedMaterial = material(token('--color-accent', '#6ea8fe'));
    // A different colour, not a different size: the two say different things
    // about how much the position is worth, and a size difference would read
    // as importance instead of as provenance.
    this.levelledMaterial = material(token('--color-text-faint', '#6b7382'));
  }

  /** Replace the marker set. Cheap enough to call whenever the list changes. */
  setMarkers(markers: PanoramaMarker[]): void {
    if (this.disposed) return;
    this.clearMarkers();

    for (const marker of markers) {
      const sprite = new THREE.Sprite(
        marker.placement.heading === 'resected' ? this.resectedMaterial : this.levelledMaterial,
      );
      sprite.scale.set(MARKER_SCALE, MARKER_SCALE, 1);
      sprite.renderOrder = 1;
      sprite.position.copy(this.host.toSceneLocal(worldVector(marker.placement.position)));
      this.markerGroup.add(sprite);
      this.markerIds.set(sprite, marker.id);
    }
  }

  setMarkersVisible(visible: boolean): void {
    this.markerGroup.visible = visible;
  }

  /** The pickable marker meshes, for {@link PointCloudScene.pick}. */
  markerObjects(): THREE.Object3D[] {
    return this.markerGroup.children;
  }

  /** Which panorama a picked object belongs to, if any. */
  panoramaIdOf(object: THREE.Object3D): number | null {
    return this.markerIds.get(object) ?? null;
  }

  /**
   * Show @p url as the backdrop of @p placement and stand the camera in it.
   *
   * Rejects with the loader's error rather than leaving an empty sphere: a
   * panorama that silently fails to load is indistinguishable from one that is
   * uniformly black, and one of those is a bug the user should hear about.
   */
  async show(placement: PanoramaPlacement, url: string): Promise<void> {
    if (this.disposed) return;
    this.hideSphere();

    const texture = await new THREE.TextureLoader().loadAsync(url);
    if (this.disposed) {
      texture.dispose();
      return;
    }
    // The stored JPEG is sRGB. Without this the renderer treats it as linear
    // and the whole panorama comes out washed out next to the point cloud.
    texture.colorSpace = THREE.SRGBColorSpace;
    texture.minFilter = THREE.LinearMipmapLinearFilter;
    texture.magFilter = THREE.LinearFilter;
    // Longitude wraps; without this the seam column clamps and smears.
    texture.wrapS = THREE.RepeatWrapping;
    this.texture = texture;

    const { positions, uvs, indices } = equirectSphere();
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(scaled(positions, PANORAMA_RADIUS), 3),
    );
    geometry.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));

    const sphere = new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({
        map: texture,
        // Seen from the inside. The alternative — a negatively scaled sphere —
        // mirrors the geometry, and then the vertex positions no longer mean
        // what `equirectSphere` says they mean.
        side: THREE.BackSide,
        // A backdrop, not a surface: writing depth would let it occlude the
        // scan's own far wall, which is exactly what the user is comparing it
        // against.
        depthWrite: false,
      }),
    );
    sphere.renderOrder = -1;

    const local = this.host.toSceneLocal(worldVector(placement.position));
    sphere.position.copy(local);
    sphere.quaternion.setFromRotationMatrix(basisMatrix(placement.basis));

    this.host.sceneRoot().add(sphere);
    this.sphere = sphere;

    // Look along the panorama's own forward axis — its +z in world. For a
    // resected panorama that is where the camera was pointed; for a levelled
    // one it is the arbitrary heading, which is the honest thing to open on.
    const forward = new THREE.Vector3(
      placement.basis[6],
      placement.basis[7],
      placement.basis[8],
    );
    this.host.enterFirstPerson(local, forward);
  }

  /** Drop the backdrop and give the orbit camera back. */
  hide(): void {
    this.hideSphere();
    this.host.exitFirstPerson();
  }

  /** True while a backdrop is on screen. */
  hasBackdrop(): boolean {
    return this.sphere !== null;
  }

  dispose(): void {
    this.disposed = true;
    this.hideSphere();
    this.clearMarkers();
    this.markerGroup.removeFromParent();
    this.markerTexture.dispose();
    this.resectedMaterial.dispose();
    this.levelledMaterial.dispose();
  }

  private hideSphere(): void {
    if (this.sphere) {
      this.sphere.removeFromParent();
      this.sphere.geometry.dispose();
      (this.sphere.material as THREE.Material).dispose();
      this.sphere = null;
    }
    // The texture is several megabytes of GPU memory and is not shared, so it
    // goes with the sphere rather than waiting for the page to unmount.
    this.texture?.dispose();
    this.texture = null;
  }

  private clearMarkers(): void {
    for (const child of [...this.markerGroup.children]) child.removeFromParent();
    this.markerIds.clear();
  }
}

function worldVector(position: [number, number, number]): THREE.Vector3 {
  return new THREE.Vector3(position[0], position[1], position[2]);
}

/** A column-major 3x3 basis as a `Matrix4` rotation. */
function basisMatrix(basis: number[]): THREE.Matrix4 {
  return new THREE.Matrix4().set(
    basis[0], basis[3], basis[6], 0,
    basis[1], basis[4], basis[7], 0,
    basis[2], basis[5], basis[8], 0,
    0, 0, 0, 1,
  );
}

function scaled(values: Float32Array, factor: number): Float32Array {
  const out = new Float32Array(values.length);
  for (let i = 0; i < values.length; i += 1) out[i] = values[i] * factor;
  return out;
}
