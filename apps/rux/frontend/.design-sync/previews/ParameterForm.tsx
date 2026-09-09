// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { ParameterForm } from 'reusex-gui';

const noopChange = () => {};
const noopReset = () => {};

const parameters = [
  {
    key: 'resolution',
    type: 'number' as const,
    label: 'Voxel size',
    description: 'Grid resolution in metres for the fused cloud.',
    default: 0.05,
    minimum: 0.001,
    maximum: 1,
    presence_sensitive: false,
  },
  {
    key: 'plane_dist_threshold',
    type: 'number' as const,
    label: 'Plane distance threshold',
    description:
      'Distance in metres a point may deviate from a fitted plane. Derived from measured sensor noise unless pinned.',
    default: null,
    minimum: 0,
    maximum: null,
    presence_sensitive: true,
  },
  {
    key: 'min_inliers',
    type: 'integer' as const,
    label: 'Minimum inliers',
    description:
      'Fewest points a region may have to be accepted as a plane. Derived from measured sensor noise unless pinned.',
    default: null,
    minimum: 1,
    maximum: null,
    presence_sensitive: true,
  },
  {
    key: 'loop_closure',
    type: 'boolean' as const,
    label: 'Loop closure',
    description: 'Enable wide-baseline ORB + depth RANSAC loop-closure edges (--loop-closure).',
    default: false,
    minimum: null,
    maximum: null,
    presence_sensitive: false,
  },
  {
    key: 'label_filter',
    type: 'integer_list' as const,
    label: 'Label filter',
    description: 'Restrict instance extraction to these semantic label ids.',
    default: null,
    minimum: null,
    maximum: null,
    presence_sensitive: false,
  },
  {
    key: 'solver',
    type: 'string' as const,
    label: 'Solver',
    description: 'MIP backend for cell selection.',
    default: 'auto',
    minimum: null,
    maximum: null,
    presence_sensitive: false,
  },
];

/** Every field at its library default — nothing pinned, Reset disabled. */
export const Defaults = () => (
  <ParameterForm
    parameters={parameters}
    state={{
      resolution: '0.05',
      plane_dist_threshold: '',
      min_inliers: '',
      loop_closure: false,
      label_filter: '',
      solver: 'auto',
    }}
    errors={{}}
    onChange={noopChange}
    onReset={noopReset}
  />
);

/** Voxel size and solver changed, both adaptive thresholds pinned — "4 changed". */
export const Touched = () => (
  <ParameterForm
    parameters={parameters}
    state={{
      resolution: '0.08',
      plane_dist_threshold: '0.02',
      min_inliers: '150',
      loop_closure: true,
      label_filter: '3,5,6',
      solver: 'cuopt',
    }}
    errors={{}}
    onChange={noopChange}
    onReset={noopReset}
  />
);

/** Four fields fail to parse at once — per-field errors, invalid input styling. */
export const WithErrors = () => (
  <ParameterForm
    parameters={parameters}
    state={{
      resolution: '-1',
      plane_dist_threshold: 'abc',
      min_inliers: '0',
      loop_closure: false,
      label_filter: 'x',
      solver: '',
    }}
    errors={{
      resolution: 'Must be between 0.001 and 1',
      plane_dist_threshold: 'Not a number',
      min_inliers: 'Must be at least 1',
      label_filter: 'Not a valid list of label ids',
    }}
    onChange={noopChange}
    onReset={noopReset}
  />
);

/** Locked while the stage is running — inputs and Reset both disabled. */
export const Disabled = () => (
  <ParameterForm
    parameters={parameters}
    state={{
      resolution: '0.05',
      plane_dist_threshold: '',
      min_inliers: '',
      loop_closure: false,
      label_filter: '',
      solver: 'auto',
    }}
    errors={{}}
    disabled
    onChange={noopChange}
    onReset={noopReset}
  />
);
