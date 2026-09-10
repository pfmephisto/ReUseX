// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PanoramaInfo } from '../api/types';
import {
  describePlacement,
  headingCaveat,
  type PanoramaPlacement,
} from '../viewport/panorama';
import styles from './PanoramaBar.module.css';

export interface PanoramaBarProps {
  panorama: PanoramaInfo;
  placement: PanoramaPlacement | null;
  /** Position in the walk, 1-based, for the `n of m` readout. */
  index: number;
  total: number;
  loading: boolean;
  error: Error | null;
  /** Whether the point cloud stays drawn inside the sphere. */
  showGeometry: boolean;
  onShowGeometryChange: (show: boolean) => void;
  onStep: (delta: number) => void;
  onExit: () => void;
}

/**
 * The heads-up bar shown while standing inside a panorama.
 *
 * It exists to make `rux view`'s panorama mode discoverable rather than
 * memorised: the same `[` / `]` stepping and `Esc` exit, with the keys named
 * on the controls that do the same thing (the design brief's "interaction
 * seed"). `rux view` gave no indication that panorama mode was even entered
 * beyond the picture changing.
 *
 * The caveat line is the reason this is a bar and not a caption. A panorama
 * that has not been through `rux align 360` is drawn with an arbitrary
 * heading, and an architect comparing it against the scan has to be told
 * that — a wrong heading in a photorealistic backdrop is invisible and
 * completely misleading.
 */
export function PanoramaBar({
  panorama,
  placement,
  index,
  total,
  loading,
  error,
  showGeometry,
  onShowGeometryChange,
  onStep,
  onExit,
}: PanoramaBarProps) {
  const caveat = headingCaveat(placement);

  return (
    <div className={styles.bar} role="group" aria-label="Panorama">
      <div className={styles.controls}>
        <button
          type="button"
          className={styles.step}
          onClick={() => onStep(-1)}
          disabled={total < 2}
          title="Previous panorama ( [ )"
          aria-label="Previous panorama"
        >
          [
        </button>
        <button
          type="button"
          className={styles.step}
          onClick={() => onStep(1)}
          disabled={total < 2}
          title="Next panorama ( ] )"
          aria-label="Next panorama"
        >
          ]
        </button>
      </div>

      <div className={styles.text}>
        <span className={styles.name} title={panorama.filename}>
          {panorama.filename}
        </span>
        <span className={styles.detail}>
          {index} of {total} · {describePlacement(panorama, placement)}
          {loading ? ' · loading…' : ''}
        </span>
      </div>

      {caveat && <p className={styles.caveat}>{caveat}</p>}
      {error && <p className={styles.error}>Could not load the image: {error.message}</p>}

      {/*
        Off by default, on by choice: the backdrop is what the user came to
        look at. But overlaying the cloud on the photograph is the cheapest
        check there is on whether a pose is right, so it is one click away
        rather than a rebuild of the viewer.
      */}
      <label className={styles.toggle}>
        <input
          type="checkbox"
          className={styles.checkbox}
          checked={showGeometry}
          onChange={(event) => onShowGeometryChange(event.target.checked)}
        />
        Overlay geometry
      </label>

      <button type="button" className={styles.exit} onClick={onExit} title="Exit (Esc)">
        Exit
      </button>
    </div>
  );
}
