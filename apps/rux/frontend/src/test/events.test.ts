// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Contract tests for the `/api/v1/events` reducer and transport.
 *
 * The load-bearing test in this file is the out-of-order one. `docs/gui/
 * websocket-events.md` warns that events are published without the server's job
 * lock held, so a `job.submitted` raised on the HTTP thread can arrive *after*
 * the `job.started` the worker raises microseconds later; a client that applies
 * by arrival order "will occasionally show a job snapping back from running to
 * queued". `seq` is the authoritative ordering. Everything else here is
 * scaffolding around keeping that true.
 */

import { describe, expect, it } from 'vitest';
import {
  activeJobs,
  applyEvent,
  emptyJobState,
  EventStream,
  jobsNewestFirst,
  type JobState,
  type SocketLike,
} from '../api/events';
import type { HelloEvent, Job, JobEvent, ServerEvent } from '../api/types';
import { RECORDED_JOB_ID, WS_EVENTS } from './fixtures';

/** Fold a whole sequence, in the order given. */
function applyAll(events: ServerEvent[], initial: JobState = emptyJobState): JobState {
  return events.reduce(applyEvent, initial);
}

function job(overrides: Partial<Job> & Pick<Job, 'id'>): Job {
  return {
    project: 'e2e.rux',
    stage: 'planes',
    status: 'queued',
    submitted_at: '2026-09-08T14:34:05Z',
    ...overrides,
  };
}

describe('applyEvent — the recorded sequence', () => {
  it('ends with the job succeeded and fully progressed', () => {
    const state = applyAll(WS_EVENTS);

    expect(state.project).toBe('e2e.rux');
    expect(state.implementation).toBe('rux-gui');
    expect(state.apiVersion).toBe('1.0.0');

    const finished = state.jobs[RECORDED_JOB_ID];
    expect(finished.status).toBe('succeeded');
    expect(finished.progress?.current).toBe(10500);
    expect(finished.progress?.total).toBe(10500);
    expect(state.seqByJob[RECORDED_JOB_ID]).toBe(5);
  });

  it('reaches the same end state whatever order the frames arrive in', () => {
    const forwards = applyAll(WS_EVENTS);
    // The handshake still has to come first (it resets everything), but the
    // five job events may be shuffled arbitrarily by the network.
    const shuffled = applyAll([
      WS_EVENTS[0],
      WS_EVENTS[5],
      WS_EVENTS[1],
      WS_EVENTS[3],
      WS_EVENTS[2],
      WS_EVENTS[4],
    ]);
    expect(shuffled.jobs[RECORDED_JOB_ID]).toEqual(forwards.jobs[RECORDED_JOB_ID]);
    expect(shuffled.seqByJob[RECORDED_JOB_ID]).toBe(5);
  });
});

describe('applyEvent — ordering by seq', () => {
  it('drops a lower seq: a late job.submitted must not un-start a running job', () => {
    const submitted = WS_EVENTS[1]; // seq 1, status queued
    const started = WS_EVENTS[2]; // seq 2, status running

    // Arrival order is reversed relative to the server's ordering.
    const afterStart = applyEvent(emptyJobState, started);
    expect(afterStart.jobs[RECORDED_JOB_ID].status).toBe('running');

    const afterLateSubmit = applyEvent(afterStart, submitted);
    // This is the regression the contract warns about: applying by arrival
    // would snap the job back to `queued`.
    expect(afterLateSubmit.jobs[RECORDED_JOB_ID].status).toBe('running');
    expect(afterLateSubmit.seqByJob[RECORDED_JOB_ID]).toBe(2);
    expect(afterLateSubmit).toBe(afterStart);
  });

  it('drops a duplicate seq (equal, not merely lower)', () => {
    const started = WS_EVENTS[2];
    const once = applyEvent(emptyJobState, started);
    const twice = applyEvent(once, started);
    expect(twice).toBe(once);
    expect(twice.seqByJob[RECORDED_JOB_ID]).toBe(2);
  });

  it('tracks the watermark per job, not globally', () => {
    const other: JobEvent = {
      ...WS_EVENTS[1],
      seq: 1,
      job: job({ id: 'other-job', status: 'queued' }),
    };
    // Job A reaches seq 5; job B's own seq 1 must still be accepted.
    const afterA = applyAll(WS_EVENTS);
    const afterB = applyEvent(afterA, other);
    expect(afterB.jobs['other-job'].status).toBe('queued');
    expect(afterB.seqByJob['other-job']).toBe(1);
    expect(afterB.seqByJob[RECORDED_JOB_ID]).toBe(5);
  });
});

describe('applyEvent — referential stability', () => {
  it('returns the same object when a dropped event changes nothing', () => {
    const state = applyAll(WS_EVENTS);
    expect(applyEvent(state, WS_EVENTS[1])).toBe(state);
    expect(applyEvent(state, WS_EVENTS[4])).toBe(state);
  });

  it('returns the same object for an unknown event type', () => {
    // Forward compatibility: a Phase 6 ruxd may add event types, and a frontend
    // built today must ignore them rather than throw.
    const state = applyAll(WS_EVENTS);
    expect(applyEvent(state, { type: 'job.paused' })).toBe(state);
    expect(applyEvent(state, { type: 'ruxd.worker.joined' })).toBe(state);
    expect(() => applyEvent(state, { type: 'totally.new' })).not.toThrow();
  });

  it('returns the same object for a malformed envelope', () => {
    const state = applyAll(WS_EVENTS);
    // No `job` at all.
    expect(applyEvent(state, { type: 'job.progress' } as ServerEvent)).toBe(state);
    // A `job` but no numeric `seq`.
    expect(
      applyEvent(state, { type: 'job.progress', job: job({ id: 'x' }) } as unknown as ServerEvent),
    ).toBe(state);
    // A non-numeric `seq`.
    expect(
      applyEvent(state, {
        type: 'job.progress',
        seq: 'nine',
        job: job({ id: 'x' }),
      } as unknown as ServerEvent),
    ).toBe(state);
    // A `job` with no id.
    expect(
      applyEvent(state, { type: 'job.progress', seq: 9, job: {} } as unknown as ServerEvent),
    ).toBe(state);
    // A null `job`.
    expect(
      applyEvent(state, { type: 'job.progress', seq: 9, job: null } as unknown as ServerEvent),
    ).toBe(state);
  });
});

describe('applyEvent — hello', () => {
  it('replaces the job map wholesale and resets the seq watermarks', () => {
    const before = applyAll(WS_EVENTS);
    expect(before.jobs[RECORDED_JOB_ID]).toBeDefined();
    expect(before.seqByJob[RECORDED_JOB_ID]).toBe(5);

    // A reconnect: the server restarted, so it knows about a different job and
    // its `seq` counter started over at 1.
    const hello: HelloEvent = {
      type: 'hello',
      timestamp: '2026-09-08T15:00:00Z',
      api_version: '1.0.0',
      implementation: 'rux-gui',
      project: 'other.rux',
      jobs: [job({ id: 'fresh-job', status: 'running' })],
    };
    const after = applyEvent(before, hello);

    // Half one: the snapshot is truth. The old job is gone, not merged in.
    expect(Object.keys(after.jobs)).toEqual(['fresh-job']);
    expect(after.jobs[RECORDED_JOB_ID]).toBeUndefined();
    expect(after.project).toBe('other.rux');
    expect(after.seqByJob).toEqual({});

    // Half two: because the watermarks were reset, a low `seq` from the
    // restarted server IS applied. Keeping the old watermark of 5 would have
    // silently blocked the first four events of every new job.
    const restarted: JobEvent = {
      ...WS_EVENTS[2],
      seq: 1,
      job: job({ id: 'fresh-job', status: 'succeeded' }),
    };
    const applied = applyEvent(after, restarted);
    expect(applied).not.toBe(after);
    expect(applied.jobs['fresh-job'].status).toBe('succeeded');
    expect(applied.seqByJob['fresh-job']).toBe(1);
  });

  it('clears a previous lastError', () => {
    const errored = applyEvent(emptyJobState, {
      type: 'error',
      timestamp: '2026-09-08T14:00:00Z',
      error: 'unknown message type',
    });
    expect(errored.lastError).toBe('unknown message type');
    expect(applyEvent(errored, WS_EVENTS[0]).lastError).toBeUndefined();
  });
});

describe('applyEvent — error envelopes', () => {
  it('sets lastError without disturbing the jobs', () => {
    const before = applyAll(WS_EVENTS);
    const after = applyEvent(before, {
      type: 'error',
      timestamp: '2026-09-08T14:35:00Z',
      error: "unknown message type 'subscrib'",
    });

    expect(after.lastError).toBe("unknown message type 'subscrib'");
    expect(after.jobs).toBe(before.jobs);
    expect(after.seqByJob).toBe(before.seqByJob);
    expect(after.project).toBe(before.project);
  });

  it('substitutes a message when the server sends none', () => {
    const after = applyEvent(emptyJobState, { type: 'error' } as ServerEvent);
    expect(after.lastError).toBe('unknown server error');
  });
});

describe('job selectors', () => {
  const state: JobState = {
    seqByJob: {},
    jobs: {
      oldest: job({ id: 'oldest', status: 'succeeded', submitted_at: '2026-09-08T10:00:00Z' }),
      tie_a: job({ id: 'tie_a', status: 'running', submitted_at: '2026-09-08T12:00:00Z' }),
      tie_b: job({ id: 'tie_b', status: 'queued', submitted_at: '2026-09-08T12:00:00Z' }),
      newest: job({ id: 'newest', status: 'failed', submitted_at: '2026-09-08T14:00:00Z' }),
    },
  };

  it('sorts newest-first by submitted_at', () => {
    const sorted = jobsNewestFirst(state);
    expect(sorted[0].id).toBe('newest');
    expect(sorted[3].id).toBe('oldest');
  });

  it('breaks a submitted_at tie deterministically on the id', () => {
    const sorted = jobsNewestFirst(state).map((entry) => entry.id);
    expect(sorted).toEqual(['newest', 'tie_b', 'tie_a', 'oldest']);
    // Insertion order of the map must not leak into the result.
    const reinserted: JobState = {
      seqByJob: {},
      jobs: {
        tie_b: state.jobs.tie_b,
        newest: state.jobs.newest,
        oldest: state.jobs.oldest,
        tie_a: state.jobs.tie_a,
      },
    };
    expect(jobsNewestFirst(reinserted).map((entry) => entry.id)).toEqual(sorted);
  });

  it('counts only queued and running jobs as active', () => {
    expect(activeJobs(state).map((entry) => entry.id)).toEqual(['tie_b', 'tie_a']);
    expect(activeJobs(applyAll(WS_EVENTS))).toEqual([]);
  });

  it('returns an empty list for an empty state', () => {
    expect(jobsNewestFirst(emptyJobState)).toEqual([]);
    expect(activeJobs(emptyJobState)).toEqual([]);
  });
});

// ------------------------------------------------------------ transport ----

class FakeSocket implements SocketLike {
  closed = false;
  sent: string[] = [];
  onopen: ((event: unknown) => void) | null = null;
  onclose: ((event: unknown) => void) | null = null;
  onerror: ((event: unknown) => void) | null = null;
  onmessage: ((event: { data: unknown }) => void) | null = null;

  close(): void {
    this.closed = true;
  }

  send(data: string): void {
    this.sent.push(data);
  }

  /** Deliver a frame exactly as a browser WebSocket would. */
  deliver(data: unknown): void {
    this.onmessage?.({ data });
  }
}

interface ScheduledTimer {
  fn: () => void;
  ms: number;
  cleared: boolean;
}

/** A stream wired to fake sockets and a fake clock. */
function harness(options: { minReconnectDelayMs?: number; maxReconnectDelayMs?: number } = {}) {
  const sockets: FakeSocket[] = [];
  const timers: ScheduledTimer[] = [];

  const stream = new EventStream({
    url: 'ws://localhost:8420/api/v1/events',
    createSocket: () => {
      const socket = new FakeSocket();
      sockets.push(socket);
      return socket;
    },
    setTimer: (fn, ms) => {
      const timer: ScheduledTimer = { fn, ms, cleared: false };
      timers.push(timer);
      return timer;
    },
    clearTimer: (handle) => {
      (handle as ScheduledTimer).cleared = true;
    },
    minReconnectDelayMs: options.minReconnectDelayMs ?? 100,
    maxReconnectDelayMs: options.maxReconnectDelayMs ?? 400,
  });

  /** Fire the most recently scheduled, uncleared timer. */
  const runPendingTimer = () => {
    const timer = timers.filter((entry) => !entry.cleared).at(-1);
    timer?.fn();
  };

  return { stream, sockets, timers, runPendingTimer, latest: () => sockets.at(-1)! };
}

describe('EventStream', () => {
  it('delivers a hello frame to an onState listener', () => {
    const { stream, latest } = harness();
    const states: JobState[] = [];
    stream.onState((state) => states.push(state));
    stream.start();

    latest().onopen?.({});
    expect(stream.currentStatus()).toBe('open');

    latest().deliver(JSON.stringify(WS_EVENTS[0]));
    latest().deliver(JSON.stringify(WS_EVENTS[1]));

    expect(states).toHaveLength(2);
    expect(states[1].jobs[RECORDED_JOB_ID].status).toBe('queued');
    expect(stream.currentState()).toBe(states[1]);
  });

  it('does not notify listeners for a frame the reducer drops', () => {
    const { stream, latest } = harness();
    let notifications = 0;
    stream.onState(() => {
      notifications += 1;
    });
    stream.start();

    latest().deliver(JSON.stringify(WS_EVENTS[2])); // seq 2 — applied
    latest().deliver(JSON.stringify(WS_EVENTS[1])); // seq 1 — dropped
    expect(notifications).toBe(1);
  });

  it('ignores a binary frame', () => {
    const { stream, latest } = harness();
    let notifications = 0;
    stream.onState(() => {
      notifications += 1;
    });
    stream.start();

    // Binary frames are reserved for the #283 transport; today they carry
    // nothing this reducer understands.
    latest().deliver(new Uint8Array([1, 2, 3]));
    latest().deliver(new ArrayBuffer(8));
    expect(notifications).toBe(0);
    expect(stream.currentState()).toBe(emptyJobState);
  });

  it('survives malformed JSON without throwing or closing the connection', () => {
    const { stream, latest } = harness();
    stream.start();
    const socket = latest();

    expect(() => socket.deliver('{"type":"hello"')).not.toThrow();
    expect(() => socket.deliver('not json at all')).not.toThrow();
    expect(socket.closed).toBe(false);
    expect(stream.currentState()).toBe(emptyJobState);

    // And a good frame after a bad one still lands.
    socket.deliver(JSON.stringify(WS_EVENTS[0]));
    expect(stream.currentState().project).toBe('e2e.rux');
  });

  it('forwards a subscribe over the open socket', () => {
    const { stream, latest } = harness();
    stream.start();
    stream.subscribe(RECORDED_JOB_ID);
    stream.subscribe(null);
    expect(latest().sent).toEqual([
      JSON.stringify({ type: 'subscribe', job_id: RECORDED_JOB_ID }),
      JSON.stringify({ type: 'subscribe', job_id: null }),
    ]);
  });

  it('reconnects with exponential backoff, capped at maxReconnectDelayMs', () => {
    const { stream, sockets, timers, runPendingTimer, latest } = harness({
      minReconnectDelayMs: 100,
      maxReconnectDelayMs: 400,
    });
    stream.start();

    for (let attempt = 0; attempt < 4; attempt += 1) {
      latest().onclose?.({});
      runPendingTimer();
    }

    // 100 → 200 → 400 → capped at 400, not 800.
    expect(timers.map((timer) => timer.ms)).toEqual([100, 200, 400, 400]);
    expect(sockets).toHaveLength(5); // the original plus one per reconnect
  });

  it('resets the backoff once a reconnect succeeds', () => {
    const { stream, timers, runPendingTimer, latest } = harness({
      minReconnectDelayMs: 100,
      maxReconnectDelayMs: 400,
    });
    stream.start();

    latest().onclose?.({});
    runPendingTimer();
    latest().onclose?.({});
    runPendingTimer();
    expect(timers.map((timer) => timer.ms)).toEqual([100, 200]);

    // A socket that actually opens clears the penalty.
    latest().onopen?.({});
    latest().onclose?.({});
    expect(timers.map((timer) => timer.ms)).toEqual([100, 200, 100]);
  });

  it('schedules only one reconnect per close', () => {
    const { stream, timers, latest } = harness();
    stream.start();
    const socket = latest();
    socket.onclose?.({});
    socket.onclose?.({});
    expect(timers).toHaveLength(1);
  });

  it('close() stops the stream for good', () => {
    const { stream, sockets, timers, latest } = harness();
    stream.start();
    const socket = latest();

    stream.close();
    expect(socket.closed).toBe(true);
    expect(stream.currentStatus()).toBe('closed');

    // A close event arriving after the fact must not resurrect the stream…
    socket.onclose?.({});
    expect(timers).toHaveLength(0);
    // …and neither must an explicit restart.
    stream.start();
    expect(sockets).toHaveLength(1);
  });

  it('close() cancels a reconnect already in flight', () => {
    const { stream, sockets, timers, runPendingTimer, latest } = harness();
    stream.start();
    latest().onclose?.({});
    expect(timers).toHaveLength(1);

    stream.close();
    expect(timers[0].cleared).toBe(true);

    // Even if the underlying timer fires anyway, no socket is opened.
    timers[0].fn();
    runPendingTimer();
    expect(sockets).toHaveLength(1);
  });

  it('reports connection status transitions', () => {
    const { stream, runPendingTimer, latest } = harness();
    const statuses: string[] = [];
    stream.onStatus((status) => statuses.push(status));

    stream.start();
    latest().onopen?.({});
    latest().onclose?.({});
    runPendingTimer();
    latest().onopen?.({});

    expect(statuses).toEqual(['connecting', 'open', 'closed', 'connecting', 'open']);
  });

  it('starts from the empty state and does not open a socket until start()', () => {
    const { stream, sockets } = harness();
    expect(sockets).toHaveLength(0);
    expect(stream.currentState()).toBe(emptyJobState);
    expect(stream.currentStatus()).toBe('closed');
  });
});
