// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { describe, expect, it } from 'vitest';

import type { CloudInfo, ProjectSummary } from '../api/types';
import { componentTypeRows, countText, projectDataSummary } from '../overblik/projectData';

function summary(over: Partial<ProjectSummary> = {}): ProjectSummary {
  return {
    path: 'newoffice.rux',
    schema_version: 25,
    projects: [],
    clouds: [
      { name: 'cloud', point_count: 12_000_000 } as CloudInfo,
      { name: 'labels', point_count: 345_678 } as CloudInfo,
    ],
    meshes: [],
    sensor_frames: { total_count: 3876, segmented_count: 120 },
    panoramic_images: { total_count: 40, matched_count: 38 },
    components: { total_count: 1, count_by_type: {} },
    materials: [],
    ...over,
  };
}

describe('countText', () => {
  it('groups thousands the Danish way', () => {
    expect(countText(12_345_678)).toBe('12.345.678');
    expect(countText(0)).toBe('0');
  });
});

describe('projectDataSummary (A2)', () => {
  it('reads as one line of figures, the path last', () => {
    expect(projectDataSummary(summary())).toBe(
      '2 punktskyer (12.345.678 punkter) · 0 meshes · 3.876 sensorbilleder (120 segmenterede) · ' +
        '40 panoramaer (38 matchede) · 1 komponent · skema v25 · newoffice.rux',
    );
  });

  it('uses the singular for one', () => {
    const s = summary({
      clouds: [{ name: 'cloud', point_count: 1 } as CloudInfo],
      meshes: [{ name: 'm', vertex_count: 1, face_count: 1 }],
      sensor_frames: { total_count: 1, segmented_count: 1 },
      panoramic_images: { total_count: 1, matched_count: 1 },
      components: { total_count: 2, count_by_type: {} },
    });
    expect(projectDataSummary(s)).toBe(
      '1 punktsky (1 punkt) · 1 mesh · 1 sensorbillede (1 segmenteret) · 1 panorama (1 matchet) · ' +
        '2 komponenter · skema v25 · newoffice.rux',
    );
  });
});

describe('componentTypeRows', () => {
  it('lists the types most common first, ties by name', () => {
    const s = summary({ components: { total_count: 9, count_by_type: { wall: 2, door: 5, beam: 2 } } });
    expect(componentTypeRows(s)).toEqual([
      { type: 'door', count: 5 },
      { type: 'beam', count: 2 },
      { type: 'wall', count: 2 },
    ]);
  });

  it('is empty with no components', () => {
    expect(componentTypeRows(summary())).toEqual([]);
  });
});
