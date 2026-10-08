<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# WebSocket event channel — `/api/v1/cases/{cid}/events`

The progress channel of the GUI contract (issue #265). OpenAPI cannot describe
a bidirectional channel, so this file is normative for it;
[`events.schema.json`](events.schema.json) is the machine-readable form and is
what clients and tests should validate against.

```
ws://localhost:8420/api/v1/cases/{cid}/events
```

One channel **per case** (ruxd multi-case spec, phase S2): a connection only
ever hears about the case its URL names. An unknown case refuses the upgrade
(`404` instead of `101`). An open connection also keeps its case open on the
server (cases otherwise close when idle).

No subprotocol is negotiated. All frames are UTF-8 **text** frames containing a
single JSON object. Binary frames are reserved for the Phase 2/5 point-cloud
transport and are not sent today.

## Why this shape

The library reports progress through `core::IProgressObserver`
(`libs/reusex/include/core/processing_observer.hpp`), whose three callbacks are:

| Callback | Meaning |
|---|---|
| `on_process_started(Stage, total)` | a phase began; `total` is the work unit count, 0 = indeterminate |
| `on_process_updated(Stage, n)` | advance the current phase by `n` units (an **increment**, not an absolute) |
| `on_process_finished(Stage)` | the phase ended |

The server installs its own observer for the duration of a job — per job, on
the worker thread running it (`core::ScopedProgressObserver`), so two cases'
jobs running at once never mix their progress — chains it to whatever observer
was already installed (so the CLI progress bar keeps working),
accumulates increments into an absolute counter, and republishes them attributed
to a job id. `core::Stage` is a *phase within* a job (`region_growing`,
`ray_tracing`, …) — it is not the job's own stage, which is the pipeline stage
that was submitted. Both are present in every event.

## Envelope

Every server message:

```jsonc
{
  "type": "job.progress",          // discriminator, see below
  "seq": 17,                       // monotonic emission order (see below)
  "timestamp": "2026-09-08T11:22:33Z",  // ISO-8601 UTC, when the server emitted
  "case": "scan",                  // the case this concerns (its {cid})
  "project": "scan.rux",           // that case's project file name
  "job": { /* the full Job object from openapi.yaml */ }
}
```

### `seq` — order by this, not by arrival

Events are *published* without the server's job lock held, because a listener
must never run under it. That means two events can reach a client out of the
order in which they happened: a `job.submitted` raised on the HTTP thread that
accepted the submission can lose the race with the `job.started` the worker
raises microseconds later.

`seq` is assigned **under the lock, at the moment the state actually changed**,
and increases by one per event for the life of the server. It is the
authoritative ordering. A client that renders state transitions must sort by
`seq` (or simply ignore an event whose `seq` is lower than the highest it has
already applied for that job); using arrival order will occasionally show a job
snapping back from "running" to "queued".

`seq` is per case, not per job, and resets when the case is reopened (cases
close when idle) or the server restarts — pair it with the `hello` snapshot
rather than persisting it.

### `case` and `project`

Every envelope names the case it concerns (`case`, the `{cid}`) and that
case's project file (`project`); the embedded `Job` carries `project` too.
Since a connection is per case both are redundant with its URL; they are there
so a message logged or forwarded out of context still says where it belongs.

The full `Job` object is embedded in **every** event rather than a delta. It is
small, it makes each message self-contained, and it means a client that
reconnects mid-run is fully caught up by the first event it receives. Clients
should key on `job.id` and replace their local copy wholesale.

## Event types

| `type` | When | Notes |
|---|---|---|
| `job.submitted` | a job was accepted onto the queue | `job.status` is `queued`; may arrive after `job.started` — see `seq` |
| `job.started` | the worker picked the job up | `job.status` is `running` |
| `job.progress` | progress counters changed | **throttled to at most one per 100 ms per job** |
| `job.finished` | the job reached a terminal status | `job.status` is `succeeded`, `failed` or `cancelled` |
| `hello` | sent once, immediately on connect | server/project handshake, no `job` |
| `clouds.changed` | an editor endpoint rewrote named clouds | carries `names`, no `job`, no `seq` |
| `case.closed` | the case was deleted while connected | no `job`, no `seq`; nothing more arrives on this connection |
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
  "implementation": "ruxd",
  "case": "scan",
  "project": "scan.rux",
  "jobs": [ /* every currently known Job, most recent first */ ]
}
```

`jobs` lets a client render the correct state on connect without a separate
`GET /jobs` round trip.

### `clouds.changed`

```json
{
  "type": "clouds.changed",
  "timestamp": "2026-10-06T11:22:33Z",
  "case": "scan",
  "project": "scan.rux",
  "names": ["labels", "instances"]
}
```

Sent after an editor endpoint rewrote point clouds outside a job — today
`POST /frames/{id}/segment/resource`, which rewrites `labels` and `instances`
(and may create them). A client showing any of `names` should refetch those
clouds' points and the cloud list (`GET /clouds`, for new label definitions).
Pipeline jobs do not send it: their clouds change under `job.finished`.

It is not about a job, so it carries no `job` and no `seq`, and a connection's
`subscribe` filter does not apply to it: every connection receives it. Like any
event it is not replayed; a client that reconnects should reload what it shows.

### `case.closed`

```json
{ "type": "case.closed", "case": "scan" }
```

The case was deleted (`DELETE /api/v1/cases/{cid}`) while this connection was
open. Nothing more arrives; a reconnect is refused. A client should leave the
case (go to the case list).

### `job.progress`

```json
{
  "type": "job.progress",
  "seq": 17,
  "timestamp": "2026-09-08T11:22:34Z",
  "case": "scan",
  "project": "scan.rux",
  "job": {
    "id": "0a5b...",
    "project": "scan.rux",
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
  "seq": 31,
  "timestamp": "2026-09-08T11:24:02Z",
  "project": "scan.rux",
  "job": {
    "id": "0a5b...",
    "project": "scan.rux",
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

## Origin checking

CORS does **not** apply to WebSockets — a browser will open one cross-origin
and hand the frames to the page's script without asking the server's
permission. The handshake is therefore the only place this can be enforced, and
`ruxd --local` does enforce it: an upgrade carrying an `Origin` header that is
not loopback and not named with `--allow-origin` is refused at the handshake. A
request with no `Origin` (curl, a CLI client, a test) is allowed. When the
server runs with `--auth-token`, the upgrade must also present the token
(`ruxd_token_<port>` cookie, `Authorization: Bearer`, or `?token=`). The
upgrade's Host header must name the server too (see docs/gui/README.md).

## Reconnection

There is no replay buffer. A client that drops should reconnect and take the
`hello` snapshot as truth; `seq` orders what arrives afterwards but does not let
you recover what was missed.

For history from before the server started, `GET /api/v1/cases/{cid}/pipeline-log`
is the durable record — `/jobs` and this channel only cover the current server
process (a case's job history does survive the case closing and reopening). Log rows written by a job carry that job's id under
`parameters.job_id`, so a reconnecting client can join its old job ids back to
what actually happened to them.

## Forward compatibility (multi-user ruxd)

Nothing here assumes in-process execution. A queued/remote implementation emits
exactly the same envelopes; `job.submitted` to `job.started` simply widens, and
`progress` may be absent for a worker that reports coarser progress. Clients
must therefore:

- treat unknown `type` values as ignorable rather than an error,
- tolerate `progress` being absent or all-zero,
- never assume `job.started` follows `job.submitted` immediately, or that jobs
  finish in submission order.
