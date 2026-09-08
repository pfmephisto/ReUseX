<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# WebSocket event channel — `/api/v1/events`

The progress channel of the GUI contract (issue #265). OpenAPI cannot describe
a bidirectional channel, so this file is normative for it;
[`events.schema.json`](events.schema.json) is the machine-readable form and is
what clients and tests should validate against.

```
ws://localhost:8420/api/v1/events
```

No subprotocol is negotiated. All frames are UTF-8 **text** frames containing a
single JSON object. Binary frames are reserved for the Phase 2/5 point-cloud
transport and are not sent today.

## Why this shape

The library reports progress through the process-global
`core::IProgressObserver` singleton
(`libs/reusex/include/core/processing_observer.hpp`), whose three callbacks are:

| Callback | Meaning |
|---|---|
| `on_process_started(Stage, total)` | a phase began; `total` is the work unit count, 0 = indeterminate |
| `on_process_updated(Stage, n)` | advance the current phase by `n` units (an **increment**, not an absolute) |
| `on_process_finished(Stage)` | the phase ended |

The server registers its own observer for the duration of a job, chains it to
whatever observer was already installed (so the CLI progress bar keeps working),
accumulates increments into an absolute counter, and republishes them attributed
to a job id. `core::Stage` is a *phase within* a job (`region_growing`,
`ray_tracing`, …) — it is not the job's own stage, which is the pipeline stage
that was submitted. Both are present in every event.

## Envelope

Every server message:

```jsonc
{
  "type": "job.progress",          // discriminator, see below
  "timestamp": "2026-09-08T11:22:33Z",  // ISO-8601 UTC, when the server emitted
  "job": { /* the full Job object from openapi.yaml */ }
}
```

The full `Job` object is embedded in **every** event rather than a delta. It is
small, it makes each message self-contained, and it means a client that
reconnects mid-run is fully caught up by the first event it receives. Clients
should key on `job.id` and replace their local copy wholesale.

## Event types

| `type` | When | Notes |
|---|---|---|
| `job.submitted` | a job was accepted onto the queue | `job.status` is `queued` |
| `job.started` | the worker picked the job up | `job.status` is `running` |
| `job.progress` | progress counters changed | **throttled to at most one per 100 ms per job** |
| `job.finished` | the job reached a terminal status | `job.status` is `succeeded`, `failed` or `cancelled` |
| `hello` | sent once, immediately on connect | server/project handshake, no `job` |
| `error` | the server rejected a client message | carries `error`, no `job` |

Throttling matters: the stages call `update()` per point or per voxel, which is
millions of calls. Only `job.progress` is throttled — the lifecycle transitions
are never dropped, so a client that only tracks `submitted`/`started`/`finished`
sees an exact record.

### `hello`

```json
{
  "type": "hello",
  "timestamp": "2026-09-08T11:22:33Z",
  "api_version": "1.0.0",
  "implementation": "rux-gui",
  "project": "scan.rux",
  "jobs": [ /* every currently known Job, most recent first */ ]
}
```

`jobs` lets a client render the correct state on connect without a separate
`GET /jobs` round trip.

### `job.progress`

```json
{
  "type": "job.progress",
  "timestamp": "2026-09-08T11:22:34Z",
  "job": {
    "id": "0a5b...",
    "stage": "planes",
    "status": "running",
    "parameters": { "radius": 0.5 },
    "error": "",
    "submitted_at": "2026-09-08T11:22:30Z",
    "started_at": "2026-09-08T11:22:30Z",
    "finished_at": "",
    "cancel_requested": false,
    "progress": {
      "stage": "region_growing",
      "stage_label": "Region Growing",
      "current": 412350,
      "total": 1830112,
      "fraction": 0.2253
    }
  }
}
```

`progress.total` is `0` when the phase did not declare a work count. Render an
indeterminate indicator in that case; `fraction` is `null`.

### `job.finished`

```json
{
  "type": "job.finished",
  "timestamp": "2026-09-08T11:24:02Z",
  "job": {
    "id": "0a5b...",
    "stage": "planes",
    "status": "failed",
    "error": "stage inputs not satisfied — missing_cloud: cloud 'normals' not found",
    "finished_at": "2026-09-08T11:24:02Z",
    "...": "..."
  }
}
```

A cancelled job also arrives as `job.finished`, with `status: "cancelled"` and
`error` carrying the reason (`"cancelled before execution"` when it never ran).

## Client messages

The channel is primarily server-push. Two client messages are accepted; both are
optional and a read-only client can ignore them entirely.

| Message | Effect |
|---|---|
| `{"type":"ping"}` | server replies `{"type":"pong","timestamp":"..."}` |
| `{"type":"subscribe","job_id":"<id>"}` | filter this connection to one job |

`subscribe` narrows the connection to a single job id; send
`{"type":"subscribe","job_id":null}` to go back to receiving everything. This
exists so a job-detail view does not have to filter a firehose client-side. It
is a *filter*, never a guarantee of delivery order across jobs.

Any other message is answered with:

```json
{ "type": "error", "timestamp": "...", "error": "unknown message type 'foo'" }
```

Malformed JSON is answered the same way. The server never closes the connection
because of a bad client message.

## Reconnection

There is no replay buffer and no sequence number. A client that drops should
reconnect, take the `hello` snapshot as truth, and additionally `GET
/api/v1/pipeline-log` if it needs history from before the server started —
`/jobs` and this channel only cover the current server process, while
`pipeline_log` is durable.

## Forward compatibility (Phase 6, ruxd)

Nothing here assumes in-process execution. A queued/remote implementation emits
exactly the same envelopes; `job.submitted` to `job.started` simply widens, and
`progress` may be absent for a worker that reports coarser progress. Clients
must therefore:

- treat unknown `type` values as ignorable rather than an error,
- tolerate `progress` being absent or all-zero,
- never assume `job.started` follows `job.submitted` immediately, or that jobs
  finish in submission order.
