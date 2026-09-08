<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX GUI API contract

This directory holds the **shared HTTP + WebSocket contract** between the
ReUseX web frontend and whatever serves it. It is the normative artefact for
issue [#265](https://github.com/pfmephisto/ReUseX/issues/265); the frontend is a
pure client of this contract and must not depend on which implementation it is
talking to.

| File | What it is |
|---|---|
| [`openapi.yaml`](openapi.yaml) | OpenAPI 3.1 description of every REST endpoint |
| [`websocket-events.md`](websocket-events.md) | The `/api/v1/events` channel: envelope, event types, client messages |
| [`events.schema.json`](events.schema.json) | JSON Schema for the WebSocket envelope, for client/test validation |

> `docs/api/` is Doxygen output and is **not** related to this directory.

## Two implementations, one contract

```
                    ┌──── browser (React SPA) ────┐
                    │  one client, one contract   │
                    └──────┬───────────────┬──────┘
                  localhost│               │LAN / cloud (Phase 6)
             ┌─────────────┴────┐   ┌──────┴──────────────┐
             │ rux gui          │   │ ruxd                │
             │ in-process jobs  │   │ queued jobs,        │
             │ one ProjectDB    │   │ PG/Redis/S3 workers │
             └──────────────────┘   └─────────────────────┘
```

`rux gui` implements the contract today (Phase 1). `ruxd` will implement the
same paths in Phase 6. Nothing in the contract assumes the client and server
share a filesystem, and job identifiers are opaque server-generated strings, so
a queued/remote implementation slots in without frontend changes.

## Versioning

Everything is served under `/api/v1`. Breaking changes take a new prefix; the
`GET /api/v1/health` response carries `api_version` and the `rux` build version
so a client can detect a mismatch.

## Implementation status (Phase 1)

Read endpoints, the job endpoints and the WebSocket channel are implemented by
`rux gui` (`apps/rux/src/gui/`). Two documented gaps, both deliberate:

- **`GET /api/v1/clouds/{name}/points` returns paged JSON only.** The contract
  reserves `format=binary` for the Phase 2/5 chunked-binary + LOD + Draco
  transport; the server rejects it with `501` today.
- **Runnable stages are `clouds`, `planes`, `rooms`, `instances`.** `mesh`,
  `texture` and the ML `annotate` stages are described by
  `GET /api/v1/stages` as `runnable: false` until their runners land.

## Running it

```bash
rux -p scan.rux gui --port 8420 --no-browser
curl -s localhost:8420/api/v1/project | jq
```

See `rux gui --help` for the asset directory, bind address and browser flags.
