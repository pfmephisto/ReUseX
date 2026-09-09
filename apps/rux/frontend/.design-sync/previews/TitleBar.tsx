// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import { TitleBar } from 'reusex-gui';

/** Healthy: project open, live event channel, one job running. */
export const Healthy = () => (
  <TitleBar
    projectName="office.rux"
    projectOpen
    schemaVersion={11}
    version="0.14.2"
    implementation="rux-gui"
    connection="open"
    activeJobCount={1}
  />
);

/** Server up, but the database could not be opened — wrong `-p`, or a corrupt file. */
export const ProjectNotOpen = () => (
  <TitleBar
    projectName="warehouse.rux"
    projectOpen={false}
    version="0.14.2"
    implementation="rux-gui"
    connection="open"
    activeJobCount={0}
  />
);

/** REST is fine but the event channel dropped mid-`create clouds` — results are current, progress is not. */
export const EventChannelDropped = () => (
  <TitleBar
    projectName="newoffice.rux"
    projectOpen
    schemaVersion={11}
    version="0.14.2"
    implementation="rux-gui"
    connection="closed"
    activeJobCount={1}
  />
);

/** `GET /health` itself failed — restart the server, not the request. */
export const ServerUnreachable = () => (
  <TitleBar connection="closed" activeJobCount={0} unreachable />
);

/** First paint before `/health` has resolved: no project name yet, socket still connecting. */
export const Loading = () => <TitleBar connection="connecting" activeJobCount={0} />;
