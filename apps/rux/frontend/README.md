<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX GUI frontend

The Phase 2 web frontend for `rux gui`
([issue #265](https://github.com/pfmephisto/ReUseX/issues/265)) — a Vite +
React + TypeScript single-page app that renders a `.rux` project: its summary,
pipeline log, stages, jobs, and a three.js point-cloud viewport.

It is a **pure client of the shared API contract** in
[`docs/gui/openapi.yaml`](../../../docs/gui/openapi.yaml) and
[`docs/gui/websocket-events.md`](../../../docs/gui/websocket-events.md), never
of a particular server. `rux gui` implements that contract today; `ruxd` will
implement the same paths in Phase 6, and the frontend is not supposed to be
able to tell the difference.

## Development

```bash
nix develop                 # provides nodejs_22
npm install
npm run dev                 # http://localhost:5173  (or: gui-dev)
```

In another terminal, start the server the dev app talks to:

```bash
rux -p scan.rux gui --port 8420 --no-browser
```

The Vite dev server proxies `/api` — REST **and** the `/api/v1/events`
WebSocket — to `http://localhost:8420`. Override the target with `RUX_GUI_URL`:

```bash
RUX_GUI_URL=http://localhost:9000 npm run dev
```

**The proxy is mandatory, not a convenience.** `rux gui` cannot answer a CORS
preflight — Crow 1.3 replies to `OPTIONS` from `Router::handle_initial()`,
before the request headers are parsed, so the server never sees the `Origin`
and cannot emit a correct preflight response — which means any JSON-bodied
cross-origin call from a bare `vite dev` would fail. Proxying makes the browser
talk only to the Vite origin, so CORS never enters into it. See
[`docs/gui/README.md`](../../../docs/gui/README.md) § "The frontend must be
same-origin".

## Production

```bash
npm run build               # tsc --noEmit && vite build  ->  dist/
rux -p scan.rux gui --assets ./dist
```

`rux gui` resolves its asset directory as `--assets`, then `$RUX_GUI_ASSETS`,
then `<install prefix>/share/reusex/gui` (`apps/rux/src/gui/assets.cpp`). The
Nix build takes the third path: `pkgs/reusex-gui-frontend/package.nix` builds
this bundle as its own derivation and `default.nix` copies it into
`$out/share/reusex/gui`, so `nix run .#default -- -p scan.rux gui` serves a UI
with no flags. Keeping it a separate derivation is deliberate — a lockfile
change must not invalidate the multi-hour C++ build, and a `.cpp` edit must not
re-run npm.

## Testing

```bash
npm test                    # vitest run
npm run test:watch
```

Frontend tests run under **npm only — they are not part of `ctest`**, and that
is intentional: wiring vitest into CTest would put a Node toolchain on the
critical path of the C++ test suite, which must stay buildable and runnable
from a plain `nix develop` + `cmake` + `ctest` with no npm anywhere in it. CI
runs them as a separate `frontend` job in `.github/workflows/ci.yml`.

Tests cover the pure-logic modules (API client with an injected `fetch`, the
event reducer, the chunk/pagination state machine) against the JSON fixtures in
`src/test/fixtures/`, which are copied from the contract's examples. No DOM
environment is configured, so no jsdom dependency is carried.

## Design tokens

`src/tokens.css` is **owned by the Claude Design project "ReUseX GUI"**
(project id `19f0cf0f-1f7c-43a7-8311-1fd67e34bbb7`). Its values are
placeholders until the first `/design-sync`. Never hand-tune them — re-sync
instead. This repo owns the token *names* and their roles; the design project
owns every value on the right-hand side.

The corollary is a hard rule for everything else under `src/`: no colour,
radius, spacing or type size is ever written literally in a component, only
`var(--...)`. That is what makes the first real sync a value swap rather than a
refactor.

## Layout

```
src/
├── api/          Contract-facing layer: types.ts (generated-by-hand mirrors of
│                 openapi.yaml), client.ts (typed fetch wrapper), events.ts
│                 (WebSocket channel + reconnect)
├── app/          App shell, routing, cross-cutting state (JobsContext),
│                 useAsync
├── components/   Presentational, contract-agnostic building blocks
│                 (DataTable, StatCard, NavRail, JobToaster, ...)
├── routes/       Page-level compositions (Dashboard, ViewportPage)
├── viewport/     three.js point-cloud rendering: PointCloudScene, the paged
│                 cloud stream (useCloudStream, pagination), point decoding
│                 (decode.ts) and label colour mapping (labelColors.ts)
├── test/         Fixtures shared by the vitest suites
├── tokens.css    Design tokens — see above, do not edit by hand
└── base.css      Global reset / element defaults, built on the tokens
```
