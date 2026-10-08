# ruxd multi-user server + native Qt client — design (2026-10-08)

Source: the user's request of 2026-10-08:
1. Move the GUI backend out of `rux` into `ruxd`.
2. Support multiple users and projects.
3. Build a native Qt GUI for `rux`, launched by running `rux` with no arguments and inspired by RTABMap's two GUIs: the 3D viewer and, mainly, the database editor.
4. Before designing the Qt GUI, update the design-studio skill so that design changes get fast feedback.
5. Work in two parallel branches.
6. Implement any wacky improvements.

The user did not review this document. It records the controller's rulings. The investigation notes (`explore-ruxd.md`, `explore-rtabmap-gui.md`, `explore-qt-tooling.md`) are copied into each plan workspace.

Two streams, each on its own worktree and branch, merged locally. Phases inside each stream are sequential.

- **Stream S: ruxd server** (branch `feat-ruxd-server`)
- **Stream Q: Qt client** (branch `feat-qt-client`)

Global rules (both streams):
- The STANDARDS and CONTRACTS documents apply.
- Business logic stays library-first; handlers are thin.
- Catch2 tests use `temp_path.hpp`.
- Run `ctest --parallel`.
- New files carry SPDX headers.
- Danish copy on case screens.
- Tokens only, with no literal colours or sizes in UI code or stylesheets.
- `docs/DIRECTION.md` gets a dated changelog entry.
- CLAUDE.md is updated wherever this work changes its statements (the `gui` row, the ruxd paragraph, the "rux gui provisions SAM3" section).

---

## Stream S — the GUI backend moves into ruxd, multi-user and multi-case

### Rulings
- **`rux gui` is removed.** The web GUI is served only by `ruxd`. Plain `rux` now means the Qt client (Stream Q).
  - Local single-user use: `ruxd --local <file.rux | dir>` serves the bundled frontend and the API for one project file, or for every `.rux` in a directory.
  - Local mode needs no Postgres, Redis or S3: it uses in-memory stores.
  - Local mode binds to loopback by default and has no login. Its implicit user is `local`, with role owner.
  - `--bind` beyond loopback in local mode requires `--auth-token`.
- **"Project" names.** The server-level collection is called **case** (Danish UI: "sag"), because `/api/v1/projects` already means rows inside a `.rux` file. All project routes are re-rooted to `/api/v1/cases/{cid}/...`, including the events WebSocket, `/api/v1/cases/{cid}/events`. Server-level routes:
  - `/api/v1/cases` (list / create / update / delete);
  - `/api/v1/auth/*`;
  - `/api/v1/users/*` (admin);
  - `/api/v1/models/sam3/status` (global).
- **Code layout.** `apps/rux/src/gui/*` and `apps/rux/include/gui/*` move to `apps/ruxd/` as a light library `ruxd_api_lib`.
  - `ruxd_api_lib` links `reusex_core` and `reusex_pipeline` only; heavy pieces (SAM3, renderer, ICP, optimize) are still injected by `ruxd`'s `main`.
  - Its tests move from `tests/unit/rux_gui/` to `tests/unit/ruxd_api/` and stay in the light test binary.
  - Use `git mv` so history follows the files.
  - The legacy ruxd routes (`/materials`, `/material-columns`, `/export-templates`, `/reports`) share one `ProjectDB` across threads, which is unsafe. They are deleted in favour of the moved API. Their behaviour is now covered by `/resources`, `/resources/columns`, `/templates` and the report routes. Keep any capability the moved API lacks, and say so.
- **Per-case state.** `Server::Impl` splits into a `ProjectContext`, one per case, and a `ProjectRegistry`.
  - `ProjectContext` holds the WAL anchor connection, the writer lock, the `PhotoEvidenceCache`, the WebSocket subscribers and the job queue.
  - `ProjectRegistry` opens cases lazily, closes them after an idle timeout, and caps how many are open.
- **Jobs.** One process-wide scheduler with a bounded worker pool (`--job-workers`, default 1, because the GPU is shared). At most one running job per case.
  - The process-global progress observer becomes per-job: thread-local, or passed explicitly.
  - Jobs are persisted in Postgres in server mode and in memory in local mode.
- **Storage.** The `.rux` files live in a server data directory (`--data-dir`), one directory per case.
  - Creating a case means uploading a `.rux` (streamed to disk) or creating an empty project. An admin can also register an existing path.
  - S3 snapshots and multi-instance advisory locks are out of scope; they are noted as follow-ups in DIRECTION.
- **Postgres schema** (server mode). Versioned SQL migrations live in `apps/ruxd/migrations/NNN_*.sql`, applied at start inside a `schema_migrations` table.
  - `users`: id, email unique, display_name, password_hash (argon2id via OpenSSL 3 KDF), is_admin, created_at, disabled.
  - `sessions`: a token hash stores sha256 of a random 32-byte token. The table also holds user_id, expires_at, created_at and last_seen.
  - `api_tokens`: hashed like sessions, plus a name and an optional case scope (for CI and scripts).
  - `cases`: id, slug, name, storage_path, created_by, created_at, archived.
  - `case_members`: case_id, user_id, role ∈ owner | editor | viewer.
  - `jobs`: id, case_id, user_id, stage, params json, state, timestamps, error.
  - `audit_log`: user, case, action, at. Writes only.
- **Authentication.** Cookie sessions, because `<img src>`, raw `fetch` and the WebSocket cannot send headers.
  - Cookie flags: HttpOnly, SameSite=Strict, Secure unless serving on loopback.
  - Mutating requests are also checked against `Origin`.
  - `Authorization: Bearer <api token>` is accepted for scripts. The old global `--auth-token` remains, as a superuser token for local mode and bootstrap.
  - First run: `ruxd admin create-user --email … --admin` (a subcommand). The password is read from stdin or prompted, never taken from argv.
  - Login rate limit: 5 attempts per minute per IP and email, kept in memory.
- **Authorization.**
  - viewer: GET only.
  - editor: everything in the case except deleting it and managing members.
  - owner: everything.
  - admin: all cases and user management.
  - Every case route checks membership. A missing case and a case you are not a member of both return **404**, so cases cannot be enumerated.
- **Frontend.**
  - The API client is case-scoped (`baseUrl = /api/v1/cases/{cid}`).
  - The router gets a case prefix `/sager/:cid/...`. The old unprefixed paths redirect to the last-used case, or to `/sager`.
  - The `/sager` page lists the cases you can access, with create and upload.
  - A login page, plus a user menu showing your name and a logout action.
  - Members management lives in a case "Indstillinger" panel.
  - In local mode the login page never shows (`GET /api/v1/auth/me` returns the implicit user).
- **Tooling.**
  - `dev_env.sh` runs `ruxd --local <copy>`.
  - The Vite proxy is retargeted.
  - `default.nix` installs the frontend bundle for ruxd.
  - Update the fish completions, CLAUDE.md, the frontend README and the design-studio reference.
- **Tests.**
  - Unit tests for registry, scheduler, authorization and migrations run against in-memory stores.
  - Postgres integration tests use an ephemeral `initdb` fixture, are tagged `[postgres]`, and SKIP when `initdb` is absent.
  - Postgres is added to the devshell if it is not already there, so the tests run locally.

### Phases
- **S1. Move.** Move the code into ruxd and add `ruxd --local`. Remove `rux gui`. Delete the legacy routes. Retarget the tooling. One case; routes unchanged.
- **S2. Cases.** Add the registry and context, re-root the routes under `/cases/{cid}`, make the scheduler and progress observer per-job, and update the frontend case prefix and `/sager` page.
- **S3. Users.** Postgres migrations, users, sessions, tokens, roles, login UI, `ruxd admin`, and the Postgres test fixture.

---

## Stream Q — native Qt client (`rux` with no arguments)

### Rulings
- **Q0 comes first: the design-studio skill becomes Qt-capable.**
  - **Gallery executable.** A `rux_qt_lib` library, plus a small `rux-qt-gallery` executable that renders any page with fixture data to PNG headless: `--page --theme dark|light --size WxH --scale 2 --project <copy> --screenshot out.png`.
  - **Rendering.** It uses `QT_QPA_PLATFORM=offscreen`. 3D panes render through VTK EGL offscreen into an image, because `QVTKOpenGLNativeWidget` is blank under offscreen. `--gl` runs the real widget under xvfb-run.
  - **Scripts.** `scripts/qt_shot.sh` drives the gallery and fails if any token is missing.
  - **Theme.**
    - Read at runtime from `apps/rux/frontend/src/tokens.css`, the single source of design values. The repo still owns names and the Claude Design project owns values.
    - The values feed a QSS template with `var(--x)` placeholders and a generated `QPalette`.
    - The stylesheet hot-reloads through `QFileSystemWatcher` in debug or `--dev` mode.
    - Release builds embed a snapshot through the Qt resource system.
  - **Fonts.** Bundle Archivo, Oswald and a mono font as TTF in Qt resources. They are OFL-licensed: add the license under `LICENSES/` and keep REUSE compliant. Source them from nixpkgs or upstream, not from fontsource woff2.
  - **Docs and lint.**
    - `references/qt-client.md` covers the loop, the token mapping and the limits of QSS (shadows, letter-spacing and uppercase are done in code).
    - `token_lint.py` is extended to `.qss` and Qt C++ literals.
    - The critique checklist gains a Qt section.
    - SKILL.md's "Qt later" text becomes "Qt in progress".
  - **Gotchas to record.** `&` is a mnemonic marker in Qt widgets (write `&&`), and the incremental gallery rebuild is fast.
- **Launch.** Plain `rux` with no subcommand, or `rux -p x.rux` with no subcommand, starts the Qt app. It opens `-p`'s project if one was given; otherwise it shows a start page with Open and Recent.
  - If there is no display (no DISPLAY or WAYLAND_DISPLAY), it prints help and exits 0 instead.
  - The Qt code lives in its own target. `default.nix` wraps the binary for the Qt platform plugins (`dontWrapQtApps` handling).
- **Information architecture.** It merges RTABMap's MainWindow and DatabaseViewer into one pleasant window: a left nav rail plus workspaces, and a context inspector on the right.
  - **Database**, the main focus:
    - a project tree of tables, clouds, meshes, frames, panoramas and survey data, with counts;
    - a frame browser with A/B sliders, each side showing colour, depth, confidence and the label overlay, plus a filmstrip;
    - a pair strip for the A/B link: edge info, ICP refine (reusing `gui_icp`), and add/delete edge, with pending edits that are saved explicitly, as in RTABMap;
    - a generic table viewer for every sqlite table, read-only, with typed views where they exist;
    - a pipeline log;
    - an inspector or property editor.
  - **3D**: one VTK widget (`QVTKOpenGLNativeWidget`), using a scene builder shared with `rux render`. Extract `populate_scene()` from `visualize/render_view.cpp` so both use it.
    - Panels: layers, "colour by" a label cloud with a legend using `--label-*` colours, frame frustums, panoramas, view presets and a cut plane.
    - The canvas stays near-black.
  - **Pose graph**: nodes and edges drawn in 2D (QGraphicsView). Clicking a node or edge drives the Database A/B selection.
  - **Pipeline**: a parameter form generated from `stage_parameters()`, a run button using `pipeline::JobRunner` in-process, with progress and log.
  - **Log**: the pipeline log.
- **Look.** Dark-first and calm, with the same tokens as the web app. Typography and spacing follow the token scale. The visual direction is "pleasant engineering tool", not a stock Qt look and not RTABMap's dock clutter. Every page is screenshotted in both themes before hand-off.
- **Wacky extras** (implement and list separately):
  - (a) a **command palette** (Ctrl+K) over every action and page;
  - (b) **"Kopiér som rux-kommando"**: each pipeline run and export shows and copies the equivalent CLI command, so the GUI teaches the CLI.

### Phases
- **Q0.** The design-studio skill for Qt, plus the minimal `rux_qt_lib`, gallery and theme loader the loop needs.
- **Q1.** The app shell: launch from plain `rux`, the start page, Open and Recent, the nav rail and workspace frame, the inspector, and the command palette.
- **Q2.** The Database workspace: tree, frame browser A/B, pair strip with edits, table viewer, log.
- **Q3.** 3D, pose graph and Pipeline workspaces, including the `populate_scene` extraction and "Kopiér som rux-kommando".
