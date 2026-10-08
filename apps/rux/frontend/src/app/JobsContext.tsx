// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import {
  createContext,
  useCallback,
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
  bumpCloudRevisions,
  type CloudRevisions,
  emptyJobState,
  jobsNewestFirst,
  type ConnectionStatus,
  type JobState,
} from '../api/events';
import type { Job } from '../api/types';
import { ALL_CASES_PATH } from './navigation';

interface JobsContextValue {
  state: JobState;
  status: ConnectionStatus;
  /** Newest first. */
  jobs: Job[];
  /** Jobs that are `queued` or `running`. */
  active: Job[];
  /**
   * Per-cloud revisions, bumped by the `clouds.changed` event. The viewport
   * restarts a cloud's stream and reloads the cloud list when they move.
   */
  cloudRevisions: CloudRevisions;
  /**
   * A write this client made changed `names`. The server broadcasts
   * `clouds.changed` for it; this bumps locally only when the socket is not
   * open to deliver that, so an edit is never left stale on screen.
   */
  markCloudsChanged: (names: readonly string[]) => void;
}

const JobsContext = createContext<JobsContextValue | null>(null);

/**
 * Owns the single WebSocket connection to the open case's
 * `/api/v1/cases/{cid}/events` (the URL comes from the case-scoped `api`).
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
  const [cloudRevisions, setCloudRevisions] = useState<CloudRevisions>({});

  useEffect(() => {
    const stream = new EventStream({ url: api.eventsUrl() });
    const offState = stream.onState(setState);
    const offStatus = stream.onStatus(setStatus);
    const offClouds = stream.onCloudsChanged((names) =>
      setCloudRevisions((revs) => bumpCloudRevisions(revs, names)),
    );
    // The case was deleted under us: nothing here is valid any more.
    const offClosed = stream.onCaseClosed(() => window.location.assign(ALL_CASES_PATH));
    stream.start();
    return () => {
      offState();
      offStatus();
      offClouds();
      offClosed();
      stream.close();
    };
  }, []);

  const markCloudsChanged = useCallback(
    (names: readonly string[]) => {
      if (status === 'open') return; // the broadcast will arrive
      setCloudRevisions((revs) => bumpCloudRevisions(revs, names));
    },
    [status],
  );

  const value = useMemo<JobsContextValue>(
    () => ({
      state,
      status,
      jobs: jobsNewestFirst(state),
      active: activeJobs(state),
      cloudRevisions,
      markCloudsChanged,
    }),
    [state, status, cloudRevisions, markCloudsChanged],
  );

  return <JobsContext.Provider value={value}>{children}</JobsContext.Provider>;
}

export function useJobs(): JobsContextValue {
  const value = useContext(JobsContext);
  if (!value) throw new Error('useJobs must be used inside <JobsProvider>');
  return value;
}
