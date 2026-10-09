# Repo split: ruxd and the web frontend move out — design (2026-10-09)

Source: the user's request of 2026-10-09.
- The local repo's default remote becomes `ReUse-X/ReUseX`.
- ruxd (the backend) and the web frontend move into their own private repos in the `ReUse-X` org.
- Each new repo gets an optional nix setup ("don't force nix on collaborators"), relevant skills, a README and a CLAUDE.md.
- The moved code is then removed here, committed, and pushed.
- Agent cost is kept low.

These are the controller's rulings; the user has not reviewed this document.

## Repos
| Repo | Contents | Visibility |
|---|---|---|
| `ReUse-X/ReUseX` (this) | library, `rux` CLI, Qt client | unchanged |
| `ReUse-X/ruxd` | the HTTP server (`apps/ruxd`), its tests, the API contract (`docs/gui/openapi.yaml`, `websocket-events.md`, schemas), Postgres migrations, NixOS module, OCI image, fish completions | private |
| `ReUse-X/rux-frontend` | the React/Vite web GUI (`apps/rux/frontend`), its nix package, the web half of design-studio, `dev_env.sh` | private |

History is preserved with `git filter-repo`, which extracts only the moved paths. The result is a clean, rooted history.

## How the pieces depend on each other
- **ReUseX installs a CMake package.** Add `install(EXPORT ReUseXTargets NAMESPACE ReUseX::)` plus `ReUseXConfig.cmake`, which runs `find_dependency` for the public dependencies. Headers install under `include/reusex/`.
  - This is the only supported way to consume the library from outside: `find_package(ReUseX CONFIG REQUIRED)`.
  - An in-tree consumer test (`tests/package/`) builds a tiny project against the installed tree, so the export cannot rot.
  - The nix `default` package installs headers, libraries and the config, in a `dev` output or alongside.
- **ruxd builds against ReUseX** with `find_package(ReUseX)`.
  - Non-nix: build and `cmake --install` ReUseX into a prefix, then point `CMAKE_PREFIX_PATH` at it. CMakePresets cover the common case. The README lists system packages.
  - Nix (optional): a flake with input `reusex.url = "github:ReUse-X/ReUseX"`, plus packages `ruxd`, `ruxd-container(-cpu)`, `nixosModules.ruxd` and a devShell.
- **ruxd serves the frontend** from `--assets <dir>` or `$RUX_GUI_ASSETS`.
  - Non-nix: run `npm run build` in rux-frontend and point at `dist/`.
  - Nix: flake input `rux-frontend` (`git+ssh`, private), and its built bundle is copied into the package.
- **The frontend is plain npm.** Node is pinned via `.nvmrc` and `package.json` `engines`.
  - The vite proxy targets `$RUXD_URL` (default `http://127.0.0.1:8420`).
  - `dev_env.sh` uses `$RUXD_BIN` or `ruxd` on PATH.
  - CI uses `actions/setup-node`, not nix.
  - The optional flake provides a devShell plus the `rux-frontend` bundle package.
- **The API contract** lives in ruxd. The frontend's `src/api/types.ts` mirrors it. The frontend README names the contract's location, and an `npm run contract:check` script fetches the contract from ruxd when `RUXD_CONTRACT` is set.
- **Design tokens.** `tokens.css` is owned by rux-frontend, where Claude Design `/design-sync` writes it.
  - The ReUseX Qt client vendors a copy at `apps/rux/qt/theme/tokens.css`, together with `scripts/sync-tokens.sh <path-or-url>`.
  - A ctest checks that every token the Qt client uses exists in the vendored file.
  - The design-studio skill is split: the web half (and dev_env/shot scripts) goes to rux-frontend, the Qt half stays here. `token_lint.py` goes to both.
- **Skills.**
  - `todo-comments` → all three.
  - `review-tagged-issues` → stays here, and is also copied to ruxd if it is repo-agnostic.
  - `design-studio` → split as above.
- **CI.**
  - ruxd: GitHub Actions with nix + cachix (the reusex closure). Pure-unit jobs need nothing beyond the ReUseX package.
  - rux-frontend: node-only lint + vitest + build.
  - ReUseX: drop the frontend and docs/gui jobs from `lint.yml`.
- **Docs.** Each new repo gets a README (what it is, quick start without nix, optional nix, contributing) and a CLAUDE.md (orientation, commands, conventions, links back to ReUseX STANDARDS).
  - ReUseX CLAUDE.md / README / ARCHITECTURE / DIRECTION / STANDARDS are updated to point at the new repos.
  - The DIRECTION changelog gets an entry.

## Execution
- **P1 (ReUseX):** the CMake package export plus the consumer test, and the nix package installing it.
- **P2 (rux-frontend):** extract, make it standalone, add nix and CI. Independent of P1.
- **P3 (ruxd):** extract, make it standalone against the P1 export, add nix and CI. Starts after P1.
- **P4 (ReUseX):** remove the moved paths and update the build, nix, docs and skills.
- **Controller:**
  - creates the private repos with `gh`;
  - pushes the extracted histories;
  - merges P1+P4 locally;
  - pushes main to `origin` (ReUse-X) and to `pfmephisto` (old remote, per the request).
