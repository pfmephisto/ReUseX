// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import {
  createContext,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from 'react';

import { api } from '../api/client';
import {
  EventStream,
  activeJobs,
  emptyJobState,
  jobsNewestFirst,
  type ConnectionStatus,
  type JobState,
} from '../api/events';
import type { Job } from '../api/types';

interface JobsContextValue {
  state: JobState;
  status: ConnectionStatus;
  /** Newest first. */
  jobs: Job[];
  /** Jobs that are `queued` or `running`. */
  active: Job[];
}

const JobsContext = createContext<JobsContextValue | null>(null);

/**
 * Owns the single WebSocket connection to `/api/v1/events`.
 *
 * One connection for the whole app, mounted at the root: the server pushes the
 * full `Job` object in every envelope, so there is nothing a second connection
 * could learn that this one does not already have. Ordering, reconnection and
 * the `hello` snapshot are all handled inside `EventStream`; this component
 * only bridges its listeners into React state.
 *
 * Phase 3 (pipeline runner) consumes this same context — the runner UI is the
 * missing piece, not the plumbing.
 */
export function JobsProvider({ children }: { children: ReactNode }) {
  const [state, setState] = useState<JobState>(emptyJobState);
  const [status, setStatus] = useState<ConnectionStatus>('closed');

  useEffect(() => {
    const stream = new EventStream({ url: api.eventsUrl() });
    const offState = stream.onState(setState);
    const offStatus = stream.onStatus(setStatus);
    stream.start();
    return () => {
      offState();
      offStatus();
      stream.close();
    };
  }, []);

  const value = useMemo<JobsContextValue>(
    () => ({
      state,
      status,
      jobs: jobsNewestFirst(state),
      active: activeJobs(state),
    }),
    [state, status],
  );

  return <JobsContext.Provider value={value}>{children}</JobsContext.Provider>;
}

export function useJobs(): JobsContextValue {
  const value = useContext(JobsContext);
  if (!value) throw new Error('useJobs must be used inside <JobsProvider>');
  return value;
}
