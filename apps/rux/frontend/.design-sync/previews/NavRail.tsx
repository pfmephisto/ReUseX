// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { NavRail } from 'reusex-gui';

/**
 * The whole nav model: Overview / Viewport / Pipeline, plus Data shown inert
 * with its Phase-4 reason. The preview harness's MemoryRouter starts at "/",
 * so Overview is always the active entry here — there is no prop or route
 * this preview file can vary (NavRail takes none and reads router context
 * the harness owns), so both cards below render identically on purpose.
 */
export const Default = () => <NavRail />;

/** Same rail inside a taller shell, showing the sunken background and border run full height. */
export const InAppShell = () => (
  <div style={{ display: 'flex', height: 420 }}>
    <NavRail />
  </div>
);
