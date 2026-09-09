// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { LayerRow } from 'reusex-gui';

const noop = () => {};

/** Fully downloaded and visible: solid bar is gone, count is final. */
export const Loaded = () => (
  <LayerRow
    cloud={{
      name: 'cloud',
      type: 'PointXYZRGB',
      point_count: 2450213,
      width: 2450213,
      height: 1,
      organized: false,
    }}
    visible
    progress={{ loaded: 2450213, total: 2450213, fraction: 1, done: true }}
    onToggle={noop}
  />
);

/** Mid-stream: a determinate fill at 33%, one page at a time. */
export const Streaming = () => (
  <LayerRow
    cloud={{
      name: 'normals',
      type: 'Normal',
      point_count: 2450213,
      width: 2450213,
      height: 1,
      organized: false,
    }}
    visible
    progress={{ loaded: 812000, total: 2450213, fraction: 0.331, done: false }}
    onToggle={noop}
  />
);

/** First page just landed — total is still unknown, so the bar is indeterminate. */
export const Indeterminate = () => (
  <LayerRow
    cloud={{
      name: 'labels',
      type: 'Label',
      point_count: 2450213,
      width: 2450213,
      height: 1,
      organized: false,
      labels: { '1': 'wall', '2': 'floor', '3': 'ceiling' },
    }}
    visible
    progress={{ loaded: 45000, total: undefined, fraction: null, done: false }}
    onToggle={noop}
  />
);

/** A page request failed partway through — the bar is replaced by the error line. */
export const ErrorState = () => (
  <LayerRow
    cloud={{
      name: 'rooms',
      type: 'Label',
      point_count: 2450213,
      width: 2450213,
      height: 1,
      organized: false,
    }}
    visible
    progress={{
      loaded: 120000,
      total: 2450213,
      fraction: 0.049,
      done: false,
      error: new Error('503 Service Unavailable — a job holds the database, retrying'),
    }}
    onToggle={noop}
  />
);

/** Never switched on: unchecked, no progress readout has ever been attached. */
export const NotLoaded = () => (
  <LayerRow
    cloud={{
      name: 'plane_centroids',
      type: 'PointXYZ',
      point_count: 24,
      width: 24,
      height: 1,
      organized: false,
    }}
    visible={false}
    onToggle={noop}
  />
);
