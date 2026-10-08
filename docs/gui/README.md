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
| [`websocket-events.md`](websocket-events.md) | The per-case `/api/v1/cases/{cid}/events` channel: envelope, event types, client messages |
| [`events.schema.json`](events.schema.json) | JSON Schema for the WebSocket envelope, for client/test validation |

> `docs/api/` is Doxygen output and is **not** related to this directory.

## One server, one contract

```
                    ┌──── browser (React SPA) ────┐
                    │  one client, one contract   │
                    └──────┬───────────────┬──────┘
                  localhost│               │LAN / cloud (next phases)
             ┌─────────────┴────┐   ┌──────┴──────────────┐
             │ ruxd --local     │   │ ruxd (server mode)  │
             │ in-process jobs  │   │ cases, users,       │
             │ cases = .rux dir │   │ Postgres            │
             └──────────────────┘   └─────────────────────┘
```

`ruxd --local <file.rux | dir>` implements the contract today; it replaced
`rux gui` (2026-10-08). The API code is `ruxd_api_lib` (`apps/ruxd/src/api/`).
Since phase S2 of spec
`docs/superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md`
every `.rux` it is given is a **case**, and every project route lives under
`/api/v1/cases/{cid}/…` — the events WebSocket too. Server-level routes are the
case list and uploads (`/cases`, `/uploads`), `/health`, `/endpoints` and
`/models/sam3/status`. Cases open lazily and close when idle
(`ProjectRegistry`); jobs of every case share one worker pool
(`--job-workers`, default 1, one running job per case). Without `--local`,
ruxd is the multi-user server (phase S3): the same routes, plus users,
sessions, API tokens and case membership in Postgres — they change who may
call a route, not the routes. See *Server mode* below.
Nothing in the contract assumes the client and server share a filesystem, and
job identifiers are opaque server-generated strings, so a queued/remote
implementation slots in without frontend changes.

## Server mode: users, roles and deployment

`ruxd` without `--local` serves the same frontend and API to many people
(spec `docs/superpowers/specs/2026-10-08-ruxd-multiuser-and-qt-client-design.md`,
phase S3):

- **Postgres** (`--pg-url` / `DATABASE_URL`) holds users, sessions, API
  tokens, cases, case members, jobs and an audit log. The schema lives in
  `apps/ruxd/migrations/NNN_*.sql`, is embedded in the binary, and is applied
  at start (and by `ruxd admin`) in a `schema_migrations` table, each
  migration in its own transaction, under an advisory lock.
- **Case files** live in `--data-dir`, one directory per case
  (`<data-dir>/<slug>/project.rux`); the `cases` table points at them.
  Uploads and created cases land there; deleted ones move to
  `<data-dir>/.ruxd/trash/`. `ruxd admin register-case --path <file.rux>`
  serves an existing file where it is (never moved or deleted by the server).
  Local mode's `.ruxd/cases.json` is not used.
- **Roles per case**: *viewer* (GET only), *editor* (everything but deleting
  the case and managing members), *owner* (everything). An administrator may
  do everything everywhere and manages users. A case you are not a member of
  answers **404**, exactly like one that does not exist. The creator of a case
  is its owner; a case always keeps one owner. Every successful mutation is
  written to `audit_log`.
- **Sessions**: `POST /api/v1/auth/login` sets the `ruxd_session_<port>`
  cookie — a random 32-byte token, stored server-side only as its SHA-256 —
  HttpOnly, SameSite=Strict, and `Secure` unless the server binds loopback
  (`--cookie-secure auto|always|never`; `auto` also sets it when a proxy sends
  `X-Forwarded-Proto: https`). It expires 12 h after its last use (renewed at
  most every 5 min) and 14 days after login regardless; logout deletes it.
  When `Secure`, the cookie is named `__Host-ruxd_session_<port>`, so no
  other service on the host can plant or shadow it.
  Passwords are argon2id (OpenSSL 3 `EVP_KDF`, RFC 9106 parameters, in PHC
  form). Failed logins back off: after 5 failures an account (from any
  address), and after 20 a client address (an IPv6 address counts as its
  /64), waits 1 s, doubling per further failure up to 15 min; failures are
  forgotten after 30 quiet minutes and a successful login clears its
  account's. Wrong Bearer tokens back off per address the same way.
- **Events sockets** follow access: logout, a disabled account, a changed
  password or removal from the case closes them, and the registry's sweep
  re-checks every open socket every 30 s.
- **Scripts** use `Authorization: Bearer rxt_…` API tokens, hashed like
  sessions, expiring after 90 days by default (`--expires-days`, 0 = never),
  optionally limited to one case. Create, list and revoke them in the user
  menu's *Adgangstokens*, over `/api/v1/auth/tokens`, or with `ruxd admin
  create-token | list-tokens | revoke-token`.
  `--auth-token` is a superuser Bearer token for bootstrap and operations; it
  must be at least 32 characters. Pass secrets as `--auth-token-file` /
  `--pg-url-file` (or their environment variables), not on the command line,
  where every local user can read them (ruxd warns).
- **Adding members**: an owner can add only people they already share a case
  with; an administrator adds anyone. Otherwise, and for an unknown email,
  the answer is the same 404, so the form is no oracle for which accounts
  exist.
- **Case files are untrusted input.** Every ProjectDB connection ruxd opens
  is hardened (`ProjectDB::OpenOptions::hardened`, set process-wide at
  start): `SQLITE_DBCONFIG_DEFENSIVE`, `trusted_schema=OFF`, and triggers and
  views disabled — a ReUseX project has neither. An upload, at
  `uploads/{id}/complete`, and a registered file are first checked with
  `PRAGMA quick_check` and refused (422, a Danish message) when it fails or
  when the schema holds a trigger or a view. The check reads the whole file
  on the request's worker thread: minutes for tens of GB, so keep
  `--threads` above the number of concurrent large uploads you expect. The
  `rux` CLI opens files as before.
- **SAM3 in server mode** uses only the server's configured or managed model;
  a request's `model_path` is refused (400).
- **Audit retention**: `--audit-retention-days` (default 365), pruned
  hourly.
- **The access decision runs before the body is read**: Crow is patched
  (`overlays/patches/crow-header-check.patch`) with a header-phase hook, so an
  unauthenticated or forbidden upload is answered — and its connection closed
  — without its body being buffered. A signed-in browser's mutation must also
  carry an allowed `Origin`.
- **Redis and S3 are reserved.** `--redis-url` and `--s3-*` (and the NixOS
  module's `redisUrl` / `s3.*`) are accepted for the deferred S3-snapshot
  phase but nothing uses them; setting one logs a warning at start.
- `GET /api/v1/readyz` answers 200 when Postgres is reachable (503 otherwise),
  for a load balancer or container health check.

### First run

```bash
export DATABASE_URL=postgresql://ruxd@db.internal/ruxd
ruxd admin create-user --email anna@firma.dk --name "Anna" --admin   # prompts for the password
ruxd --data-dir /srv/ruxd --bind 127.0.0.1 --port 8080
# then, as Anna, add people in the case's Indstillinger → Medlemmer;
# accounts are made by an administrator:
ruxd admin create-user --email bo@firma.dk --name "Bo"               # or: … < password.txt
ruxd admin create-token --email ci@firma.dk --name nightly --case kontor   # prints rxt_… once
```

### Container image

`nix build .#ruxd-container` (CUDA) or `.#ruxd-container-cpu` builds an OCI
image whose entrypoint is `ruxd` in server mode, configured through the
environment: `RUXD_PORT=8080`, `RUXD_BIND=0.0.0.0` (ruxd binds loopback by
default, which nothing outside the container can reach) and
`RUXD_DATA_DIR=/data`, a volume. Supply Postgres yourself:

```bash
docker load < result
docker run -p 8080:8080 -v ruxd-cases:/data \
  -e DATABASE_URL_FILE=/run/secrets/pg-url -v ./pg-url:/run/secrets/pg-url:ro \
  ruxd:latest
```

Every flag has its environment variable in `ruxd --help` (`--bind`
`RUXD_BIND`, `--data-dir` `RUXD_DATA_DIR`, `--pg-url` `DATABASE_URL`,
`--pg-url-file` `DATABASE_URL_FILE`, `--auth-token-file`
`RUXD_AUTH_TOKEN_FILE`); a flag wins over its variable. Put the TLS proxy
below in front of the published port.

### TLS via a reverse proxy

ruxd speaks plain HTTP; put TLS in front of it. With the proxy on the same
host, bind ruxd to loopback and name the public origin, so its Host and the
browser's `Origin` are accepted:

```nginx
server {
  listen 443 ssl;
  server_name ruxd.firma.dk;
  client_max_body_size 80m;                 # upload chunks are ≤ 64 MiB
  location / {
    proxy_pass http://127.0.0.1:8080;
    proxy_set_header Host $host;
    proxy_set_header X-Forwarded-Proto https;
    proxy_http_version 1.1;                 # the events WebSocket
    proxy_set_header Upgrade $http_upgrade;
    proxy_set_header Connection "upgrade";
  }
}
```

```bash
ruxd --data-dir /srv/ruxd --bind 127.0.0.1 --port 8080 \
     --allow-origin https://ruxd.firma.dk --trusted-proxy 127.0.0.1
```

The session cookie is then `Secure` (`X-Forwarded-Proto: https`). Name the
proxy with `--trusted-proxy 127.0.0.1` so the login back-off sees each
client's own address (`X-Forwarded-For`, honoured only from trusted peers);
without it every login seems to come from the proxy, and the per-address
back-off is shared by everyone. Serving a
non-loopback bind over plain HTTP is not supported: the cookie is `Secure`
and a browser will not send it back (`--cookie-secure never` only for a
closed test network).

## Security model (local mode)

On loopback, `ruxd --local` has **no authentication**, and its `POST /jobs`
endpoint executes pipeline stages. Three controls keep that from being
reachable by any page or host it should not be:

1. **Loopback bind by default.** A `--bind` beyond loopback is refused unless
   `--auth-token` is set. The token is then required on every HTTP request and
   on the WebSocket upgrade, as `Authorization: Bearer <token>`, as the
   `ruxd_token_<port>` cookie, or as `?token=<token>`. Any presented credential
   that matches is accepted. A GET with a valid `?token=` answers `303` to the
   same URL without the token and sets the cookie (HttpOnly, SameSite=Strict,
   named per port so two servers on one host do not shadow each other), so a
   browser opened at `http://<host>:8420/?token=<token>` stays signed in for
   its `<img>`, `fetch` and WebSocket traffic — none of which can carry a
   header — and the token leaves the address bar and history. Every response
   carries `Referrer-Policy: no-referrer`, and the request log (spdlog, `-vv`)
   redacts query strings.
2. **A Host allowlist** (DNS rebinding). A page on `evil.example` that resolves
   to 127.0.0.1 sends same-origin requests with no `Origin` header; only its
   `Host` gives it away. On a loopback bind only `localhost`, `127.0.0.1` and
   `[::1]` are served; on a specific address, that address or an
   `--allow-origin` host; on a wildcard bind (`0.0.0.0`), any host, since the
   token it requires cannot be presented by a rebinding page.
3. **A server-side origin allowlist.** A request carrying an `Origin` that is
   not loopback, not named with `--allow-origin` and — with a token — not the
   request's own origin (`<scheme>://<Host>`, i.e. the page this server served
   to a browser on another machine) is refused with `403` before any handler
   runs. `--allow-origin` is therefore only needed for a frontend served from
   somewhere else. This is enforcement, not a CORS hint — CORS alone
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
sending none. `ruxd --local --auth-token` accepts that scheme, and so does the
multi-user server (API tokens and the superuser token), which additionally
accepts the `sessionCookie` scheme; which operations need which is described
per tag rather than by per-operation `security` blocks.

### The frontend must be same-origin

Preflighted cross-origin requests (i.e. anything with a JSON body) are **not
supported**. Crow 1.3 answers `OPTIONS` from `Router::handle_initial()`, before
the request headers are parsed, so the server cannot see the `Origin` at
preflight time and cannot emit a correct preflight response. There is no hook
and no opt-out.

This costs nothing in practice, because the frontend is same-origin in both
supported setups:

- **Production** — `ruxd --local` serves the bundle itself, from the same origin as
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
`ruxd --local` (`apps/ruxd/src/api/`). One documented gap, deliberate:

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

`ruxd --local` has two writers — the pipeline job worker and these endpoints — and
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

## Gaussian splats

`rux create gsplat` stores the model it trains **in the project** (schema v12),
exactly as `rux create mesh` stores a mesh, so `/gsplats` is an ordinary
project read and no filesystem path appears anywhere in the contract. A `.ply`
from a run made before v12, or from another 3DGS implementation, comes in with
`rux import gsplat`.

The header is validated on the way into the project rather than on the way
out. That check matters more than it sounds: `rux export ply` writes a
perfectly valid PLY with no `f_dc_*`/`opacity`/`scale_*`/`rot_*` properties,
and without it, pointing the importer at one would produce an empty viewport
with no explanation. It is also where `gaussian_count` and `sh_degree` come
from, so a row can never describe a different model than the one stored.

`/gsplats` is unpaged, unlike `/clouds` and `/meshes`. A project holds a
handful of splats — one per training run somebody chose to keep — not a number
that grows with the size of the scan.

In the frontend the splats are layers beside the point cloud, sharing one
canvas, one camera and one `OrbitControls`
(`apps/rux/frontend/src/viewport/SplatScene.ts`), each toggled independently
from the layer panel. Nothing is **downloaded** until a toggle is switched on:
unlike a cloud, which streams in pages, a splat is a single response of
hundreds of megabytes, so the panel shows the size and the Gaussian count and
lets the user decide. `?splat=<name>` deep-links one on.

## 360 panoramas

![Capture positions marked in the scan](images/panorama-markers.png)

The viewport draws a marker at every capture position and lets the user stand
inside one — `rux view`'s panorama skybox, made discoverable. Clicking a marker
enters; `[` / `]` step and `Esc` leaves, the keys `rux view` already uses, named
on the controls that do the same thing. `?pano=<id>` deep-links one.

Two things about the contract are worth stating, because both are places where
an easier implementation would have produced a plausible, wrong picture.

**A panorama's placement has two possible sources, and they are separate
fields.** `pose` is the pose `rux align 360` resects; it is **identity until
that command has run**, which is the state of every panorama in a
freshly-imported project. `frame_pose` is the pose of the timestamp-matched
sensor frame (`node_id`), derived on read — without it a client must issue one
`GET /frames/{id}` per panorama, and that route decodes each frame's depth and
confidence blobs to answer its availability flags.

They are not merged into one `pose` with a fallback. A borrowed frame pose
carries the 360 camera's mounting offset and the timestamp-match error, and
that is a different claim from a resection — which is what `has_pose` and
`pose_source` exist to let a client say. The frontend consumes them
accordingly: a resected panorama is oriented by its own pose, while an
unaligned one is drawn at the frame's *position* with a **level horizon and an
arbitrary heading**, labelled as such. Adopting the phone's rotation instead
would tilt a horizon that was level, and a tilted photorealistic backdrop reads
as a broken viewer rather than as missing information.

A panorama with neither pose is listed, disabled, and **not drawn** — an
unplaceable photograph rendered at the origin asserts it was taken somewhere
the building is not.

![Standing inside an unaligned panorama](images/panorama-immersive.png)

**The equirect convention is the library's, not three.js's.** `u` spans
longitude `[-pi, pi]` left to right, `v` runs down from the north pole, and a
pixel's bearing in the panorama's own optical frame (x right, y **down**, z
forward) is `(sin θ cos φ, −sin φ, cos θ cos φ)` —
`geometry/EquirectProjection.hpp`, restated under `GET /panoramas/{id}/image`.
The frontend generates its own sphere from that formula
(`apps/rux/frontend/src/viewport/panorama.ts`) rather than re-mapping
`THREE.SphereGeometry`, for two reasons that both show up on screen: the
geometry's local frame is then the panorama frame, so the stored pose applies
to the mesh unmodified; and deriving `u` from the grid column rather than from
the vertex position gives the duplicated seam vertices `u = 0` and `u = 1`
instead of the same value, which is what closes the seam at longitude ±π.

The sphere is wound **outward** and drawn with `THREE.BackSide`. The opposite
winding culls exactly the faces a viewer standing at the centre can see: the
panorama is then visible from outside the sphere, where nobody stands, and the
immersive view is empty with no error anywhere. `src/test/panorama.test.ts`
asserts the winding for that reason.

`?max_size=` on the image route is what makes the picker affordable: a stored
equirect is routinely 8192x4096 and several megabytes, so a strip of thumbnails
at full resolution is tens of megabytes for a row of 128-pixel images.

Inside a panorama the geometry is hidden by default, as `rux view` hides every
other prop — but *Overlay geometry* puts the point cloud back inside the
sphere, which is the cheapest visual check there is on whether an alignment is
right.

## Running it

```bash
ruxd --local scan.rux            # 127.0.0.1:8420
curl -s localhost:8420/api/v1/cases | jq           # the cases
curl -s localhost:8420/api/v1/cases/<cid>/project | jq

# What Gaussian splats does this project hold, and what do they cost to load?
curl -s localhost:8420/api/v1/gsplats | jq
```

See `ruxd --help` ("Web GUI") for the asset directory, bind address, token
and browser flags, and *Server mode* above for the multi-user server.
