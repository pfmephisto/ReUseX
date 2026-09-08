// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { ReactNode } from 'react';

import { api } from '../api/client';
import type { Health } from '../api/types';
import { TitleBar } from '../components/TitleBar';
import { NavRail } from '../components/NavRail';
import { JobToaster } from '../components/JobToaster';
import { useAsync } from './useAsync';
import { useJobs } from './JobsContext';
import styles from './AppShell.module.css';

/**
 * Title bar + nav rail + content region.
 *
 * The shell resolves the project identity once, from `GET /health`, rather than
 * from `/project`: health is the cheap call, it is the one the contract
 * designates for the version handshake, and it reports `project.open === false`
 * when the database could not be opened — which is exactly the state a title
 * bar must not render as if everything were fine.
 */
export function AppShell({ children }: { children: ReactNode }) {
  const { data: health, error } = useAsync<Health>((signal) => api.health(signal), []);
  const { active, status } = useJobs();

  return (
    <div className={styles.shell}>
      <TitleBar
        projectName={health?.project.name}
        projectOpen={health?.project.open}
        schemaVersion={health?.project.schema_version}
        version={health?.version}
        implementation={health?.implementation}
        connection={status}
        activeJobCount={active.length}
        unreachable={Boolean(error)}
      />
      <div className={styles.body}>
        <NavRail />
        <main className={styles.content}>{children}</main>
      </div>
      <JobToaster />
    </div>
  );
}
