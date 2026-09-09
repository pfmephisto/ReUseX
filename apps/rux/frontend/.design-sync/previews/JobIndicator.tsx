// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { JobIndicator } from 'reusex-gui';

/** Idle, event channel live — nothing running, socket connected. */
export const IdleConnected = () => <JobIndicator connection="open" activeJobCount={0} />;

/** One job running while connected — the common "busy" state. */
export const OneJobRunning = () => <JobIndicator connection="open" activeJobCount={1} />;

/** Several jobs queued/running at once, e.g. annotate + project + instances. */
export const MultipleJobsRunning = () => <JobIndicator connection="open" activeJobCount={3} />;

/** Socket still negotiating on first load. */
export const Connecting = () => <JobIndicator connection="connecting" activeJobCount={0} />;

/**
 * Disconnected while a 20-minute `create clouds` job is still running server-side.
 * Must not read as "nothing is happening" — that's the whole point of this component.
 */
export const DisconnectedWhileBusy = () => <JobIndicator connection="closed" activeJobCount={1} />;
