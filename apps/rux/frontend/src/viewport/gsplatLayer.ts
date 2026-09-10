// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Everything the splat layer decides that does not need a GPU (#322).
 *
 * Kept apart from `SplatScene.ts` on purpose: that module imports three.js and
 * a WebGL renderer and so can only run in a browser, while *this* is where the
 * questions a user actually notices are answered — what a splat costs to load,
 * and what to say when a project has none. Those are unit-testable, and
 * `src/test/gsplat.test.ts` tests them.
 */

import type { GsplatInfo } from '../api/types';

const COUNT = new Intl.NumberFormat();

/**
 * A byte size at the granularity a download decision is made at.
 *
 * A splat is one blob of hundreds of megabytes — unlike the point cloud, which
 * streams in pages and shows progress as it goes, switching this layer on is a
 * single commitment. The number is there so the commitment is informed, which
 * is why it rounds to whole MB rather than pretending to know bytes.
 */
export function formatSplatSize(bytes: number | undefined): string | null {
  if (bytes === undefined || !Number.isFinite(bytes) || bytes < 0) return null;
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${Math.round(bytes / 1024)} kB`;
  if (bytes < 1024 * 1024 * 1024) return `${Math.round(bytes / (1024 * 1024))} MB`;
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(1)} GB`;
}

/**
 * The one-line description under a splat's toggle: count, size, colour model.
 *
 * Tolerant of a missing field even though the contract marks all three
 * required: a row reading `undefined Gaussians` would be worse than a shorter
 * row, and this renders against whatever the server actually sent.
 */
export function describeGsplat(info: GsplatInfo): string {
  const parts: string[] = [];
  if (Number.isFinite(info.gaussian_count))
    parts.push(`${COUNT.format(info.gaussian_count)} Gaussians`);
  const size = formatSplatSize(info.byte_size);
  if (size) parts.push(size);
  if (Number.isFinite(info.sh_degree)) {
    // Degree 0 is not "spherical harmonics missing" — it is a deliberate
    // view-independent fit, and saying so is more use than "SH 0".
    parts.push(info.sh_degree === 0 ? 'flat colour' : `SH degree ${info.sh_degree}`);
  }
  return parts.join(' · ');
}

/**
 * What to say when the panel has no splat rows to show.
 *
 * Order matters. A transport failure is about the *server*, and reporting "no
 * splat in this project" on top of a request that never arrived would be a
 * fabrication — so the error wins. Returns null when there is something to
 * list, because then the rows speak for themselves.
 */
export function gsplatNote(
  splats: GsplatInfo[] | null | undefined,
  error?: Error | null,
): string | null {
  if (error) return `Could not list this project's Gaussian splats: ${error.message}`;
  if (!splats) return null;
  if (splats.length > 0) return null;
  return (
    'No Gaussian splat is stored in this project. Train one with ' +
    '`rux create gsplat`, or bring an existing .ply in with `rux import gsplat`.'
  );
}
