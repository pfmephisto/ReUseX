// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

import type { PipelineLogEntry } from '../api/types';

/**
 * Parse a stored timestamp for arithmetic only.
 *
 * `pipeline_log.started_at` defaults to sqlite's `datetime('now')`, which is
 * UTC written as `YYYY-MM-DD HH:MM:SS` with no zone marker — and the contract
 * declares the field as a bare string, so a client cannot assume more than
 * that. `Date.parse` would read it as *local* time. That does not matter for a
 * difference between two such stamps, which is all this is used for, as long
 * as both are pinned to the same offset; appending `Z` does that and also
 * stops a DST boundary between the two from inventing an hour of runtime.
 *
 * The displayed value stays the raw stored string. Re-rendering it in local
 * time would silently shift every entry by the viewer's offset and claim a
 * precision the contract does not grant.
 */
export function epochMs(stored: string): number | null {
  const normalized = /^\d{4}-\d{2}-\d{2}[ T]\d{2}:\d{2}:\d{2}$/.test(stored)
    ? `${stored.replace(' ', 'T')}Z`
    : stored;
  const parsed = Date.parse(normalized);
  return Number.isNaN(parsed) ? null : parsed;
}

/** Human duration of a finished entry, or null while it runs / cannot be read. */
export function formatDuration(entry: PipelineLogEntry): string | null {
  if (!entry.finished_at) return null; // empty means "still running"
  const from = epochMs(entry.started_at);
  const to = epochMs(entry.finished_at);
  if (from === null || to === null || to < from) return null;

  const seconds = Math.round((to - from) / 1000);
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ${seconds % 60}s`;
  return `${Math.floor(minutes / 60)}h ${minutes % 60}m`;
}

/**
 * Was this run started from the GUI, and by which job?
 *
 * The contract says a `POST /jobs` run stores its `job_id` inside the stage
 * parameters, which is how durable history is joined back to a job after a
 * server restart. `parameters` is an opaque *string* though — written by
 * whatever ran the stage, including a hand-run `rux` — so it is parsed
 * defensively. An unparseable blob means "no marker", never a crash: a
 * malformed parameters column must not take out the whole history panel.
 */
export function jobIdOf(parameters?: string): string | null {
  if (!parameters) return null;
  try {
    const parsed: unknown = JSON.parse(parameters);
    if (typeof parsed !== 'object' || parsed === null) return null;
    const jobId = (parsed as Record<string, unknown>).job_id;
    return typeof jobId === 'string' && jobId.length > 0 ? jobId : null;
  } catch {
    return null;
  }
}

/**
 * The parameters a run was given, minus the plumbing.
 *
 * `job_id` is caller metadata the runner injects, not something the user
 * chose, so showing it among the knobs would misrepresent the run.
 */
export function chosenParameters(parameters?: string): Record<string, unknown> {
  if (!parameters) return {};
  try {
    const parsed: unknown = JSON.parse(parameters);
    if (typeof parsed !== 'object' || parsed === null || Array.isArray(parsed)) return {};
    const { job_id: _ignored, ...rest } = parsed as Record<string, unknown>;
    return rest;
  } catch {
    return {};
  }
}

/** One timeline row, with everything the view needs precomputed. */
export interface HistoryRow {
  entry: PipelineLogEntry;
  duration: string | null;
  jobId: string | null;
  parameters: Record<string, unknown>;
}

/**
 * Newest first.
 *
 * The server already returns the log in descending id order, but the contract
 * documents neither the order nor a guarantee that it will stay that way, and
 * a history view that silently reverses when the backend changes is a bug a
 * reader would blame on their own project. So the order is asserted here.
 * `started_at` leads because that is what the reader is scanning; `id`
 * disambiguates two runs recorded within the same second.
 */
export function historyRows(entries: PipelineLogEntry[]): HistoryRow[] {
  return [...entries]
    .sort((a, b) => {
      const left = epochMs(a.started_at);
      const right = epochMs(b.started_at);
      if (left !== null && right !== null && left !== right) return right - left;
      return b.id - a.id;
    })
    .map((entry) => ({
      entry,
      duration: formatDuration(entry),
      jobId: jobIdOf(entry.parameters),
      parameters: chosenParameters(entry.parameters),
    }));
}

/** Rows whose stage matches @p stage, or every row when @p stage is null. */
export function filterByStage(rows: HistoryRow[], stage: string | null): HistoryRow[] {
  if (!stage) return rows;
  return rows.filter((row) => row.entry.stage === stage);
}

/** Rows whose status matches @p status, or every row when @p status is null. */
export function filterByStatus(
  rows: HistoryRow[],
  status: PipelineLogEntry['status'] | null,
): HistoryRow[] {
  if (!status) return rows;
  return rows.filter((row) => row.entry.status === status);
}

/**
 * The lowercased text a free-text search matches against.
 *
 * Everything the reader can actually see in the row is searchable — the stage,
 * the status word, the error, the job marker, and each `key=value` knob — so a
 * query for "planes", "failed", "job 3f", or "voxel_size" all land where the
 * reader expects. Values are stringified the same way the row renders them, so
 * what you see is what you can search. Timestamps are included so a date
 * fragment like "2026-09-08" narrows to a day without a dedicated picker.
 */
export function rowHaystack(row: HistoryRow): string {
  const parts: string[] = [
    row.entry.stage,
    row.entry.status,
    row.entry.started_at,
    row.entry.finished_at ?? '',
    row.entry.error_msg ?? '',
    row.jobId ?? '',
  ];
  for (const [key, value] of Object.entries(row.parameters)) {
    parts.push(`${key}=${JSON.stringify(value)}`);
  }
  return parts.join(' ').toLowerCase();
}

/**
 * Rows matching a free-text @p query, or every row when it is blank.
 *
 * Case-insensitive substring match over {@link rowHaystack}. A whitespace-only
 * query is treated as no query rather than as a filter that hides everything.
 */
export function searchRows(rows: HistoryRow[], query: string): HistoryRow[] {
  const needle = query.trim().toLowerCase();
  if (needle === '') return rows;
  return rows.filter((row) => rowHaystack(row).includes(needle));
}

/** The filters the log view applies together. A null/blank field is inactive. */
export interface HistoryFilter {
  stage: string | null;
  status: PipelineLogEntry['status'] | null;
  query: string;
}

/**
 * Apply stage, status and free-text filters in one pass, newest-first order
 * preserved. Composed from the single-axis helpers so each stays independently
 * testable; the order is irrelevant because every filter only removes rows.
 */
export function applyHistoryFilter(rows: HistoryRow[], filter: HistoryFilter): HistoryRow[] {
  return searchRows(filterByStatus(filterByStage(rows, filter.stage), filter.status), filter.query);
}

/** Every distinct stage name present in the history, in first-seen order. */
export function stagesInHistory(rows: HistoryRow[]): string[] {
  const seen: string[] = [];
  for (const row of rows) if (!seen.includes(row.entry.stage)) seen.push(row.entry.stage);
  return seen;
}
