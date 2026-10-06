// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The technical inventory that closes Overblik (Kortlægning fixes spec A2),
 * formerly the Værktøjer › Projektdata screen. Pure, so it is testable in Node;
 * the tables' column definitions live in `components/overblik/projectDataColumns.tsx`.
 */

import type { ProjectSummary } from '../api/types';

const COUNT = new Intl.NumberFormat('da-DK');

/** A count with Danish thousands grouping — `12.345.678`, never `12345678`. */
export function countText(n: number): string {
  return COUNT.format(n);
}

function counted(n: number, one: string, many: string): string {
  return `${countText(n)} ${n === 1 ? one : many}`;
}

/** The one-line summary under the closed disclosure's heading. */
export function projectDataSummary(s: ProjectSummary): string {
  const points = s.clouds.reduce((sum, c) => sum + c.point_count, 0);
  return [
    `${counted(s.clouds.length, 'punktsky', 'punktskyer')} (${counted(points, 'punkt', 'punkter')})`,
    counted(s.meshes.length, 'mesh', 'meshes'),
    `${counted(s.sensor_frames.total_count, 'sensorbillede', 'sensorbilleder')} ` +
      `(${counted(s.sensor_frames.segmented_count, 'segmenteret', 'segmenterede')})`,
    `${counted(s.panoramic_images.total_count, 'panorama', 'panoramaer')} ` +
      `(${counted(s.panoramic_images.matched_count, 'matchet', 'matchede')})`,
    counted(s.components.total_count, 'komponent', 'komponenter'),
    `skema v${s.schema_version}`,
    s.path,
  ].join(' · ');
}

export interface TypeCount {
  type: string;
  count: number;
}

/** Building components per type, the most common first, ties by name. */
export function componentTypeRows(s: ProjectSummary): TypeCount[] {
  return Object.entries(s.components.count_by_type)
    .map(([type, count]) => ({ type, count }))
    .sort((a, b) => b.count - a.count || a.type.localeCompare(b.type));
}
