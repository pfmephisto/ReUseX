// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * The `/api/v1/events` WebSocket client (`docs/gui/websocket-events.md`).
 *
 * The interesting part is not the socket — it is `applyEvent`, a pure reducer
 * that enforces the contract's ordering rule:
 *
 * > Events are published without the server's job lock held, so two events can
 * > reach a client out of the order in which they happened. `seq` is assigned
 * > under the lock at the moment the state actually changed and is the
 * > authoritative ordering.
 *
 * Concretely: a `job.submitted` raised on the HTTP thread can lose the race
 * with the `job.started` the worker raises microseconds later. Applying by
 * arrival makes a job visibly snap back from "running" to "queued". So the
 * reducer keeps the highest `seq` seen **per job** and drops anything at or
 * below it.
 *
 * The reducer is separated from the transport precisely so that rule can be
 * tested without a server, a socket or a clock.
 */

import { isHelloEvent, isJobEvent, type Job, type ServerEvent } from './types';

/** Everything the UI knows about jobs, derived purely from the event stream. */
export interface JobState {
  /** Jobs by id. Replaced wholesale on each accepted event, never merged. */
  jobs: Record<string, Job>;
  /** Highest `seq` applied per job id. */
  seqByJob: Record<string, number>;
  /** Set by the `hello` handshake. */
  project?: string;
  implementation?: string;
  apiVersion?: string;
  /** The most recent server-side `error` message, if any. */
  lastError?: string;
}

export const emptyJobState: JobState = { jobs: {}, seqByJob: {} };

/**
 * Fold one server event into the job state.
 *
 * Returns the *same* object when the event changed nothing, so a React caller
 * can use referential equality to skip a render. Three things are dropped
 * silently and on purpose:
 *
 *  - an event whose `seq` is not newer than what has already been applied for
 *    that job (the out-of-order case described above);
 *  - an unknown `type` — the contract requires clients to treat those as
 *    ignorable, so that a Phase 6 ruxd can add event types without breaking a
 *    frontend built today;
 *  - a malformed envelope (no `job.id`, no numeric `seq`).
 */
export function applyEvent(state: JobState, event: ServerEvent): JobState {
  if (isHelloEvent(event)) {
    // The snapshot is truth on connect: a reconnecting client must not carry
    // stale jobs across, and `seq` resets when the server restarts, so the
    // per-job watermarks are reset too rather than blocking the new stream.
    const jobs: Record<string, Job> = {};
    for (const job of event.jobs ?? []) jobs[job.id] = job;
    return {
      jobs,
      seqByJob: {},
      project: event.project,
      implementation: event.implementation,
      apiVersion: event.api_version,
      lastError: undefined,
    };
  }

  if (event.type === 'error') {
    const message = (event as { error?: string }).error ?? 'unknown server error';
    return { ...state, lastError: message };
  }

  if (!isJobEvent(event)) return state;

  const previous = state.seqByJob[event.job.id];
  if (previous !== undefined && event.seq <= previous) return state;

  return {
    ...state,
    jobs: { ...state.jobs, [event.job.id]: event.job },
    seqByJob: { ...state.seqByJob, [event.job.id]: event.seq },
  };
}

/** Jobs newest-first by `submitted_at`, with the id as a stable tiebreak. */
export function jobsNewestFirst(state: JobState): Job[] {
  return Object.values(state.jobs).sort((a, b) => {
    if (a.submitted_at === b.submitted_at) return a.id < b.id ? 1 : -1;
    return a.submitted_at < b.submitted_at ? 1 : -1;
  });
}

/** Jobs that are neither terminal nor cancelled — what the indicator counts. */
export function activeJobs(state: JobState): Job[] {
  return jobsNewestFirst(state).filter(
    (job) => job.status === 'queued' || job.status === 'running',
  );
}

export type ConnectionStatus = 'connecting' | 'open' | 'closed';

/** Minimal structural type for a WebSocket, so tests can supply a fake. */
export interface SocketLike {
  close(): void;
  send(data: string): void;
  onopen: ((event: unknown) => void) | null;
  onclose: ((event: unknown) => void) | null;
  onerror: ((event: unknown) => void) | null;
  onmessage: ((event: { data: unknown }) => void) | null;
}

export interface EventStreamOptions {
  url: string;
  /** Injectable socket factory. Defaults to the global `WebSocket`. */
  createSocket?: (url: string) => SocketLike;
  /** Injectable timer, so a test need not wait a real second. */
  setTimer?: (fn: () => void, ms: number) => unknown;
  clearTimer?: (handle: unknown) => void;
  /** Reconnect backoff bounds, in milliseconds. */
  minReconnectDelayMs?: number;
  maxReconnectDelayMs?: number;
}

/**
 * A reconnecting client for the event channel.
 *
 * There is no replay buffer on the server, so reconnection is not resumption:
 * the fresh `hello` snapshot is taken as truth and `applyEvent` resets the
 * per-job watermarks accordingly.
 */
export class EventStream {
  private socket: SocketLike | null = null;
  private reconnectHandle: unknown = null;
  private delayMs: number;
  private stopped = false;

  private readonly options: Required<
    Pick<
      EventStreamOptions,
      'url' | 'createSocket' | 'setTimer' | 'clearTimer' | 'minReconnectDelayMs' | 'maxReconnectDelayMs'
    >
  >;

  private stateListeners = new Set<(state: JobState) => void>();
  private statusListeners = new Set<(status: ConnectionStatus) => void>();

  private state: JobState = emptyJobState;
  private status: ConnectionStatus = 'closed';

  constructor(options: EventStreamOptions) {
    this.options = {
      url: options.url,
      createSocket:
        options.createSocket ?? ((url: string) => new WebSocket(url) as unknown as SocketLike),
      setTimer: options.setTimer ?? ((fn, ms) => setTimeout(fn, ms)),
      clearTimer: options.clearTimer ?? ((handle) => clearTimeout(handle as never)),
      minReconnectDelayMs: options.minReconnectDelayMs ?? 500,
      maxReconnectDelayMs: options.maxReconnectDelayMs ?? 10_000,
    };
    this.delayMs = this.options.minReconnectDelayMs;
  }

  currentState(): JobState {
    return this.state;
  }

  currentStatus(): ConnectionStatus {
    return this.status;
  }

  onState(listener: (state: JobState) => void): () => void {
    this.stateListeners.add(listener);
    return () => this.stateListeners.delete(listener);
  }

  onStatus(listener: (status: ConnectionStatus) => void): () => void {
    this.statusListeners.add(listener);
    return () => this.statusListeners.delete(listener);
  }

  /** Open the socket. Safe to call once; `close()` makes it permanent. */
  start(): void {
    if (this.stopped || this.socket) return;
    this.setStatus('connecting');

    const socket = this.options.createSocket(this.options.url);
    this.socket = socket;

    socket.onopen = () => {
      this.delayMs = this.options.minReconnectDelayMs;
      this.setStatus('open');
    };
    socket.onmessage = (message) => this.handleMessage(message.data);
    socket.onerror = () => {
      /* onclose always follows; reconnect is driven from there alone. */
    };
    socket.onclose = () => {
      this.socket = null;
      this.setStatus('closed');
      this.scheduleReconnect();
    };
  }

  /** Stop for good: no further reconnects. */
  close(): void {
    this.stopped = true;
    if (this.reconnectHandle !== null) {
      this.options.clearTimer(this.reconnectHandle);
      this.reconnectHandle = null;
    }
    const socket = this.socket;
    this.socket = null;
    socket?.close();
    this.setStatus('closed');
  }

  /** Narrow this connection to one job, or pass `null` to widen it again. */
  subscribe(jobId: string | null): void {
    this.socket?.send(JSON.stringify({ type: 'subscribe', job_id: jobId }));
  }

  private handleMessage(data: unknown): void {
    if (typeof data !== 'string') return; // binary frames are reserved (#283)
    let event: ServerEvent;
    try {
      event = JSON.parse(data) as ServerEvent;
    } catch {
      return; // a malformed frame is not worth tearing the connection down for
    }
    const next = applyEvent(this.state, event);
    if (next === this.state) return;
    this.state = next;
    for (const listener of this.stateListeners) listener(next);
  }

  private setStatus(status: ConnectionStatus): void {
    if (this.status === status) return;
    this.status = status;
    for (const listener of this.statusListeners) listener(status);
  }

  private scheduleReconnect(): void {
    if (this.stopped || this.reconnectHandle !== null) return;
    const delay = this.delayMs;
    this.delayMs = Math.min(this.delayMs * 2, this.options.maxReconnectDelayMs);
    this.reconnectHandle = this.options.setTimer(() => {
      this.reconnectHandle = null;
      this.start();
    }, delay);
  }
}
