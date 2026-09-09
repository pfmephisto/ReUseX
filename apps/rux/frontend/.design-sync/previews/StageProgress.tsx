// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { StageProgress } from 'reusex-gui';

/** Early in a determinate phase: current/total and a matching percent. */
export const EarlyDeterminate = () => (
  <StageProgress
    progress={{
      stage: 'region_growing',
      stage_label: 'Growing planar regions',
      current: 3,
      total: 40,
    }}
  />
);

/** Nearly done, `fraction` supplied explicitly rather than derived. */
export const NearlyComplete = () => (
  <StageProgress
    progress={{
      stage: 'cloud_reconstruction',
      stage_label: 'Assembling cloud',
      current: 37,
      total: 40,
      fraction: 0.925,
    }}
  />
);

/** `total: 0` — indeterminate by contract: sweeping bar, no number, no percent. */
export const Indeterminate = () => (
  <StageProgress
    progress={{
      stage: 'mip_solve',
      stage_label: 'Solving cell complex (MIP)',
      current: 0,
      total: 0,
    }}
  />
);

/** Overshoot: `current` exceeds `total`, fraction must clamp rather than overflow the track. */
export const Overshoot = () => (
  <StageProgress
    progress={{
      stage: 'texture_bake',
      stage_label: 'Baking textures',
      current: 52,
      total: 48,
    }}
  />
);
