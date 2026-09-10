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

## Security model

`rux gui` has **no authentication** and its `POST /jobs` endpoint executes
pipeline stages. Two controls keep that from being reachable by any page the
user happens to have open:

1. **Loopback bind by default.** `--bind` changes it; that is a deliberate act.
2. **A server-side origin allowlist.** A request carrying an `Origin` that is
   not loopback and not named with `--allow-origin` is refused with `403`
   before any handler runs. This is enforcement, not a CORS hint — CORS alone
   would not help, because a `text/plain` POST is a *simple* request that the
   browser dispatches before it reads any response header. Mutating routes
   additionally require `Content-Type: application/json`, which a simple
   request cannot set.

WebSocket upgrades are checked the same way, in the handshake: CORS does not
apply to WebSockets at all, so the server has to do it itself.

The contract nevertheless **declares** a `bearerAuth` security scheme, while
leaving the document-level requirement as `security: []`. That pair is a
deliberate statement rather than an oversight: the empty list says every
operation here needs no credential, and a generated client honours it by
sending none. `ruxd` already takes `--auth-token` and will serve this same
contract in Phase 6; declaring the scheme now means that deployment overrides
one document-level field instead of forcing every client to be regenerated
against a differently-shaped spec.

### The frontend must be same-origin

Preflighted cross-origin requests (i.e. anything with a JSON body) are **not
supported**. Crow 1.3 answers `OPTIONS` from `Router::handle_initial()`, before
the request headers are parsed, so the server cannot see the `Origin` at
preflight time and cannot emit a correct preflight response. There is no hook
and no opt-out.

This costs nothing in practice, because the frontend is same-origin in both
supported setups:

- **Production** — `rux gui` serves the bundle itself, from the same origin as
  the API.
- **Development** — point the Vite dev server's proxy at it, which is the
  normal arrangement anyway:

  ```js
  // vite.config.ts
  export default {
    server: {
      proxy: {
        '/api': { target: 'http://localhost:8420', ws: true },
      },
    },
  };
  ```

  With the proxy in place the browser only ever talks to the Vite origin, and
  CORS never enters into it.

`--allow-origin` remains useful for simple cross-origin `GET`s and for
non-browser clients.

**This is a settled limitation, not an open task.** Lifting it needs either an
upstream Crow change or a different HTTP layer — decorating Crow's own `204`
after the fact was tried and rejected, because the first `OPTIONS` on a fresh
connection loses the added headers, which is worse than a clean refusal. Since
both supported deployments are same-origin, the limitation costs nothing today,
and it is worth revisiting only if a genuine cross-origin deployment appears
(#285).

## Versioning

Everything is served under `/api/v1`. Breaking changes take a new prefix; the
`GET /api/v1/health` response carries `api_version` and the `rux` build version
so a client can detect a mismatch.

## One paging idiom

Every collection that can grow with the size of a scan is paged the same way:
`offset` and `limit` query parameters in, and an `offset` / `count` / `total`
envelope out, alongside the items. `limit=0` means "the server maximum", which
is a real number per resource and never "unbounded" — so no request can ask the
server to materialise an arbitrarily large response, whatever a client sends.
A page past the end is a `200` with `count: 0`, never a `404`.

The defaults differ where the cost differs, and only there. `/pipeline-log`
keeps a default of 100 because a history view wants a screenful, not all of it.
`/frames` allows up to 10 000 ids, because an id is four bytes and forcing a
scan of ordinary size to page would be ceremony. `/clouds/{name}/points` keeps
its own much larger page, because its unit is a point. What is uniform is the
*shape*: one set of parameter names, one set of response fields, so a client
writes the pagination logic once.

Three collections are deliberately **not** paged — `/endpoints`, `/stages`, and
`/meshes/{name}/textures`. Their size is fixed by the server build or bounded
by one parent object, so a page would add a round trip to answer a question
that has no second page.

## Jobs are live state, `pipeline_log` is the record

`/jobs` is in-memory and scoped to one server process, and it is **bounded**:
queued and running jobs are always listed, but finished ones are retained only
up to a limit, oldest dropped first. That keeps a server left open all day from
accumulating job records forever.

The consequence is that `GET /jobs/{id}` can answer `404` for a job that really
ran — because it was evicted, or because the server restarted. The contract
does not try to distinguish those cases, because the recovery is identical:
look the id up in `/pipeline-log`, where `parameters.job_id` carries the job id
that caused each run. **Anything that must still be findable tomorrow keys on
`pipeline_log`, not on a job id.**

Durable job identity — one store rather than two views — is the shape Phase 6
needs anyway, where `ruxd` keeps jobs in PostgreSQL and they outlive any single
worker. Building it into the in-process runner first would be inventing a
second answer to a question Phase 6 has to answer properly (#286).

A finished job that succeeded also carries `result`: the stage's own summary
plus the artifacts it wrote, each with the name it is fetched under. That is
there so a UI refreshes what changed instead of re-fetching every collection on
the chance that one of them did — a successful `planes` run names `planes`,
`plane_centroids` and `plane_normals`, and the rest of the screen is left
alone. `result.outputs` is what this run actually persisted; `StageInfo.outputs`
is what the stage writes in general. They are not the same question.

## Describing a stage, not just naming it

`GET /api/v1/stages` (and `GET /api/v1/stages/{stage}/validation`, which
re-checks one of them) answers three separate questions per stage, which a
runner UI needs to keep apart:

- **Can it run at all here?** `runnable` — whether a library job runner exists.
- **Can it run *now*?** `ready`, with `blockers` as printable strings and
  `issues` as the structured form. Each error-severity issue carries the
  artifact it is about and a `hint`: the resolution command derived from the
  stage-contract table (#295) by walking back to the first prerequisite that is
  actually satisfied. That is what lets a card say "inputs missing: cloud — run
  `rux create clouds` first" without re-deriving the pipeline order client-side.
- **What can I set?** `parameters` — a descriptor per knob, with the default
  read off the library option struct rather than re-typed
  (`pipeline::stage_parameters()`). A front end that builds its form from this
  cannot drift from the library the way a hard-coded one does (STANDARDS §4).

One descriptor field deserves attention: `presence_sensitive`. For
`planes.plane_dist_threshold` and `planes.min_inliers`, sending the key **at
all** pins the parameter and switches off adaptive derivation for it (#214),
whatever its value. A form that helpfully echoed every default back would
silently disable adaptive thresholding on every GUI-started run. Omit untouched
keys.

## Implementation status (Phase 1)

Read endpoints, the job endpoints and the WebSocket channel are implemented by
`rux gui` (`apps/rux/src/gui/`). One documented gap, deliberate:

- **Runnable stages are `clouds`, `planes`, `rooms`, `instances`.** `mesh`,
  `texture` and the ML `annotate` stages are described by
  `GET /api/v1/stages` as `runnable: false` until their runners land.

`GET /api/v1/clouds/{name}/points` serves both the RUXP binary transport
(`format=binary`, [`binary-points.md`](binary-points.md)) and voxel LOD
(`max_points`, #320) — "all of it, coarsely" rather than a prefix. What remains
open on #320 is the *cheap* version of LOD: a precomputed progressive ordering
in the `.rux`, which would make every level a prefix and cost the server
nothing. Draco and quantised positions are open there too.

## The pipeline runner

![The pipeline runner mid-run](images/pipeline-running.png)

A stage card is driven entirely by `/stages`: the summary and the `rux`
command come from the contract table, the knobs from the parameter
descriptors, the progress bar from the WebSocket `job.progress` stream, and the
"N more runs queued behind this one" from the job list. Nothing about any
individual stage is hard-coded in the frontend.

On an empty project every card explains itself instead of simply refusing:

![Every card names its missing inputs](images/pipeline-blocked.png)

The history pane below the cards reads `pipeline_log`, not `/jobs` — durable,
survives a restart, and includes runs made from the command line, which a
job-derived view would silently omit:

![The pipeline_log timeline](images/pipeline-history.png)

## The editors

![The sensor-frame browser](images/frames-browser.png)

The frame browser virtualises its grid, so a scan of several hundred frames
costs a screenful of `<img>` elements rather than all of them. Two contract
additions make it possible at all:

- **`GET /frames?segmented=`** filters server-side. The alternative is fetching
  `/frames/{id}` for every frame to read one boolean — and that endpoint
  *decodes the depth and confidence blobs* to answer it.
- **`?normalize=true`** on the image route renders a picture instead of a
  measurement. Depth is stored as 16-bit millimetres, and a browser decoding
  that PNG keeps the high byte, so a 3 m room arrives at 3000 of 65535 and
  paints near-black. The normalised rendering carries **no metric scale** and
  is labelled as such; omit the flag to get the stored values.
  `?max_size=` downscales for thumbnails, nearest-neighbour for label images so
  no interpolated class id is ever invented.

![The building-component table](images/components-table.png)

`area` and `source_instance_guid` are computed on read — the first by Newell
from the stored boundary, the second lifted out of the opaque `metadata` JSON.
Neither is a stored column, so neither can drift from the geometry it describes.
There is deliberately **no room column**: ReUseX does not associate a building
component with a room, and a table that showed one would be inventing it.

### Writing

![The material-passport editor](images/passport-editor.png)
![The label-legend editor](images/label-legend-editor.png)

`PATCH /materials/{guid}` and `PATCH /clouds/{name}/labels` are the first
mutating endpoints that are not job submissions. Both take a **sparse** map and
leave everything they are not told about alone, and the client sends only what
changed. That is not an optimisation: a passport imported from MaterialEPAS
carries dozens of fields no GUI form models, so a full-document PUT from an
editor that had not loaded them would be indistinguishable from a deliberate
request to delete them. For the same reason `null` deletes a property and `""`
stores an empty one — a form that conflated them would make clearing a field
unreachable.

Two edits are refused rather than attempted:

- **The `instances` legend.** Those names are generated as `SM<class>-<id>
  (<n>p)` and parsed back by the v10 migration and `io/export_scene.cpp` to
  recover the semantic class. A free-text rename corrupts a record while looking
  like a caption edit, so the server answers `409` and the UI disables the
  fields — learning the rule by breaking it is a worse experience.
- **Creating a class.** `PATCH …/labels` renames existing ids; an id not in the
  legend is a `400`, because naming an id no point carries reads as data loss.

#### One writer, enforced

`rux gui` has two writers — the pipeline job worker and these endpoints — and
they exclude each other through a real lock, not through hope.
`pipeline::JobRunner` owns the project's writer lock and **holds it for the
whole of every stage**; an editor request takes the same lock or does not run:

| Situation | Answer | Why |
|---|---|---|
| A job is running or queued | `409` | A stage holds the lock for minutes. Blocking the request that long is indistinguishable from a hung UI. |
| Another edit holds the lock | `503` after 250 ms | Editor writes take milliseconds, so anything past that is worth reporting rather than waiting on. |

Nothing is written when either check fails, so both are safe to retry — and the
two are kept distinct in the UI because their advice is opposite: a `409` means
"wait for the stage", a `503` means "send it again now".

This matters beyond tidiness. Both edits are read-modify-writes: renaming one
label class rewrites the cloud's entire map, because
`ProjectDB::save_label_definitions` replaces it wholesale. Two unsynchronised
writers would lose one of the renames with no error anywhere. SQLite would
serialise the two connections, but only per statement and only by returning
`SQLITE_BUSY` — which cannot protect a read-modify-write.

Mutating routes are gated on `Content-Type: application/json`, and that gate
keys on *whether the method mutates* rather than on `POST` specifically, so a
future route cannot opt out of CSRF protection by choosing a different verb.

## Running it

```bash
rux -p scan.rux gui --port 8420 --no-browser
curl -s localhost:8420/api/v1/project | jq
```

See `rux gui --help` for the asset directory, bind address and browser flags.
