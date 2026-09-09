// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { DataTable, EmptyState } from 'reusex-gui';

interface CloudRow {
  name: string;
  type: string;
  points: number;
  organized: boolean;
}

const clouds: CloudRow[] = [
  { name: 'cloud', type: 'PointXYZRGB', points: 10500, organized: false },
  { name: 'normals', type: 'Normal', points: 10500, organized: false },
  { name: 'labels', type: 'Label', points: 10500, organized: false },
  { name: 'planes', type: 'Label', points: 10500, organized: false },
  { name: 'plane_centroids', type: 'PointXYZ', points: 2, organized: false },
];

const columns = [
  { key: 'name', header: 'Cloud', render: (r: CloudRow) => r.name },
  { key: 'type', header: 'Type', render: (r: CloudRow) => r.type },
  {
    key: 'points',
    header: 'Points',
    numeric: true,
    render: (r: CloudRow) => r.points.toLocaleString('en-US'),
  },
  { key: 'organized', header: 'Organized', render: (r: CloudRow) => (r.organized ? 'yes' : 'no') },
];

/** The project inventory: dense rows, numeric column right-aligned in mono. */
export const CloudInventory = () => (
  <DataTable columns={columns} rows={clouds} rowKey={(r: CloudRow) => r.name} />
);

/** Clickable rows (cloud name links into the viewport in the app). */
export const ClickableRows = () => (
  <DataTable
    columns={columns}
    rows={clouds.slice(0, 3)}
    rowKey={(r: CloudRow) => r.name}
    onRowClick={() => {}}
  />
);

/** The empty slot points at the pipeline stage that fills it. */
export const Empty = () => (
  <DataTable
    columns={columns}
    rows={[]}
    rowKey={(r: CloudRow) => r.name}
    empty={<EmptyState title="No meshes" detail="`rux create mesh` solves the cell complex into a watertight surface." />}
  />
);
