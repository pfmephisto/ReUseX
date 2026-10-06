// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Column definitions for Overblik's Projektdata tables (spec A2), extracted
 * from the retired Projektdata screen so they live in one place.
 */

import { Link } from 'react-router-dom';

import type { CloudInfo, MeshInfo } from '../../api/types';
import { countText, type TypeCount } from '../../overblik/projectData';
import type { Column } from '../DataTable';

/** A stored value the contract represents as empty-string-means-absent. */
function present(value?: string): string | null {
  const trimmed = value?.trim();
  return trimmed ? trimmed : null;
}

export const CLOUD_COLUMNS: Column<CloudInfo>[] = [
  {
    key: 'name',
    header: 'Punktsky',
    // The viewport route reads `?cloud=`; linking from the name keeps the row
    // itself inert, so selecting text in a wide table does not navigate.
    render: (cloud) => <Link to={`/viewport?cloud=${encodeURIComponent(cloud.name)}`}>{cloud.name}</Link>,
  },
  { key: 'type', header: 'Type', render: (cloud) => cloud.type },
  { key: 'point_count', header: 'Punkter', numeric: true, render: (cloud) => countText(cloud.point_count) },
  {
    key: 'dims',
    header: 'B × H',
    numeric: true,
    render: (cloud) => `${countText(cloud.width)} × ${countText(cloud.height)}`,
  },
  { key: 'organized', header: 'Organiseret', render: (cloud) => (cloud.organized ? 'ja' : 'nej') },
];

export const MESH_COLUMNS: Column<MeshInfo>[] = [
  { key: 'name', header: 'Mesh', render: (mesh) => mesh.name },
  { key: 'format', header: 'Format', render: (mesh) => mesh.format ?? '—' },
  { key: 'vertex_count', header: 'Hjørner', numeric: true, render: (mesh) => countText(mesh.vertex_count) },
  { key: 'face_count', header: 'Flader', numeric: true, render: (mesh) => countText(mesh.face_count) },
  {
    key: 'texture_count',
    header: 'Teksturer',
    numeric: true,
    render: (mesh) => countText(mesh.texture_count ?? 0),
  },
  {
    // Shown as stored. The contract types it as a bare string with no zone, so
    // re-rendering it in the viewer's local time would shift it by an unknown
    // offset — see the same reasoning in PipelineLogList.
    key: 'created_at',
    header: 'Oprettet',
    render: (mesh) => present(mesh.created_at) ?? '—',
  },
];

export const TYPE_COLUMNS: Column<TypeCount>[] = [
  { key: 'type', header: 'Komponenttype', render: (row) => row.type },
  { key: 'count', header: 'Antal', numeric: true, render: (row) => countText(row.count) },
];
