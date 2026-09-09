// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { LayerPanel } from 'reusex-gui';

const noop = () => {};
const noopToggle = () => {};
const noopString = () => {};
const noopNumber = () => {};

const cloud = {
  name: 'cloud',
  type: 'PointXYZRGB' as const,
  point_count: 2450213,
  width: 2450213,
  height: 1,
  organized: false,
};
const normals = {
  name: 'normals',
  type: 'Normal' as const,
  point_count: 2450213,
  width: 2450213,
  height: 1,
  organized: false,
};
const labels = {
  name: 'labels',
  type: 'Label' as const,
  point_count: 2450213,
  width: 2450213,
  height: 1,
  organized: false,
  labels: {
    '1': 'wall',
    '2': 'floor',
    '3': 'ceiling',
    '4': 'window',
    '5': 'door',
    '6': 'radiator',
  },
};

/** Sensor RGB, only the fused cloud switched on and fully loaded. */
export const Default = () => (
  <LayerPanel
    clouds={[cloud, normals, labels]}
    visible={{ cloud: true, normals: false, labels: false }}
    progress={{ cloud: { loaded: 2450213, total: 2450213, fraction: 1, done: true } }}
    onToggleLayer={noopToggle}
    labelSources={[labels]}
    labelCloud={null}
    onLabelCloudChange={noopString}
    colorMode="rgb"
    onColorModeChange={noopString}
    pointSize={0.02}
    onPointSizeChange={noopNumber}
    onFrame={noop}
  />
);

/** Coloured by the `labels` cloud — the legend composes in below the picker. */
export const LabelColoring = () => (
  <LayerPanel
    clouds={[cloud, normals, labels]}
    visible={{ cloud: true, normals: false, labels: true }}
    progress={{
      cloud: { loaded: 2450213, total: 2450213, fraction: 1, done: true },
      labels: { loaded: 2450213, total: 2450213, fraction: 1, done: true },
    }}
    onToggleLayer={noopToggle}
    labelSources={[labels]}
    labelCloud="labels"
    onLabelCloudChange={noopString}
    colorMode="label"
    onColorModeChange={noopString}
    pointSize={0.035}
    onPointSizeChange={noopNumber}
    onFrame={noop}
  />
);

/** Before `create project`: no Label cloud is large enough to offer yet. */
export const NoLabelSource = () => (
  <LayerPanel
    clouds={[cloud, normals]}
    visible={{ cloud: true, normals: false }}
    progress={{ cloud: { loaded: 2450213, total: 2450213, fraction: 1, done: true } }}
    onToggleLayer={noopToggle}
    labelSources={[]}
    labelCloud={null}
    onLabelCloudChange={noopString}
    labelSourceNote="No label cloud has as many points as `cloud` yet — run `rux create project` to generate one."
    colorMode="rgb"
    onColorModeChange={noopString}
    pointSize={0.02}
    onPointSizeChange={noopNumber}
    onFrame={noop}
  />
);

/** Before `create clouds`: nothing to render, nothing to colour by. */
export const Empty = () => (
  <LayerPanel
    clouds={[]}
    visible={{}}
    progress={{}}
    onToggleLayer={noopToggle}
    labelSources={[]}
    labelCloud={null}
    onLabelCloudChange={noopString}
    colorMode="rgb"
    onColorModeChange={noopString}
    pointSize={0.02}
    onPointSizeChange={noopNumber}
    onFrame={noop}
  />
);
