// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { StatCard } from 'reusex-gui';

/** The dashboard's headline figure: value + label + optional breakdown hint. */
export const Default = () => (
  <StatCard label="Point clouds" value={6} hint="42,004 points total" />
);

/** Pre-formatted large value — thousands separators are the caller's job. */
export const LargeValue = () => (
  <StatCard label="Sensor frames" value="2,317" hint="1,904 segmented" />
);

/** `muted` tone: the tile is present but carries nothing yet. */
export const MutedEmpty = () => (
  <StatCard label="Meshes" value={0} tone="muted" />
);
