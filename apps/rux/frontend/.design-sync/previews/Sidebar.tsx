// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { Sidebar } from 'reusex-gui';

/**
 * The navy case sidebar: PROJEKT eyebrow, the case workflow (Overblik active,
 * the rest inert with their phase reasons), Værktøjer, and "← Alle sager".
 * The preview harness's MemoryRouter starts at "/", so Overblik is always the
 * active entry here.
 */
export const Default = () => (
  <div style={{ display: 'flex', height: 560 }}>
    <Sidebar projectName="Måløv Byvej 229" />
  </div>
);

/** With the review-queue and pending-sample counts wired up. */
export const WithBadges = () => (
  <div style={{ display: 'flex', height: 560 }}>
    <Sidebar projectName="Måløv Byvej 229" badges={{ reviewQueue: 7, pendingSamples: 2 }} />
  </div>
);
