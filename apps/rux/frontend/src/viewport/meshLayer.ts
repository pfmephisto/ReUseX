// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Mesh-layer helpers that do not need a GPU or a WebGL context.
 *
 * Kept apart from `MeshScene.ts` for the same reason `gsplatLayer.ts` is kept
 * apart from `SplatScene.ts`: the panel descriptions and empty-state logic are
 * plain string logic, unit-testable without a renderer.
 */

import type { MeshInfo } from '../api/types';

const COUNT = new Intl.NumberFormat();

/**
 * One-line description under a mesh's toggle: vertex count, face count, format.
 *
 * Tolerant of missing fields — the contract marks them required, but a row
 * reading `NaN vertices` would be worse than a shorter row.
 */
export function describeMesh(info: MeshInfo): string {
  const parts: string[] = [];
  if (Number.isFinite(info.vertex_count) && info.vertex_count >= 0)
    parts.push(`${COUNT.format(info.vertex_count)} vertices`);
  if (Number.isFinite(info.face_count) && info.face_count >= 0)
    parts.push(`${COUNT.format(info.face_count)} faces`);
  if (info.format) parts.push(info.format.toUpperCase());
  return parts.join(' · ');
}

/**
 * What to say when the mesh section has no rows.
 *
 * Same ordering rule as `gsplatNote`: a transport failure is about the server,
 * and reporting "this project has no mesh" on top of a request that never
 * arrived would state something the client does not know.
 */
export function meshNote(
  meshes: MeshInfo[] | null | undefined,
  error?: Error | null,
): string | null {
  if (error) return `Could not list this project's meshes: ${error.message}`;
  if (!meshes) return null;
  if (meshes.length > 0) return null;
  return 'No mesh in this project. Run `rux create mesh` to compute one.';
}
