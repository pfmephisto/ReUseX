# CI binary cache (`reusex.cachix.org`)

`.github/workflows/ci.yml` builds `.#checks.x86_64-linux.tests` — the CPU
variant of ReUseX with `BUILD_TESTS=ON`, plus `ctest` — on stock GitHub-hosted
runners. That is only possible because the parts of the dependency closure
that `cache.nixos.org` cannot serve come pre-built from our own Cachix cache:

- **Cache**: <https://reusex.cachix.org>
- **Visibility**: public. Reads need no token; `nix` fetches from it as an
  ordinary substituter. `flake.nix`'s `nixConfig` already declares it, so a
  local `nix build` offers to use it the first time you accept the flake
  config; CI opts in with `accept-flake-config = true`.
- **Public key**:
  `reusex.cachix.org-1:0+y68O2+rBxXytqIepgqOSRsWa+5Ccpst9weS0sUBak=`
- **Plan**: Cachix free tier, **5 GB** of (compressed) storage. Everything
  below exists to stay inside that number.
- **Write credential**: `CACHIX_AUTH_TOKEN`, an **environment** secret on the
  `CI` environment of the repo — *not* a repo-level secret. A job that does
  not declare `environment: CI` sees `secrets.CACHIX_AUTH_TOKEN` as the empty
  string and silently degrades to a read-only, push-skipping run.

## What is in the cache, and why only that

The build closure of `.#checks.x86_64-linux.tests` is **1666 store paths /
9.62 GiB** of unpacked NARs. Almost all of it — compilers, Qt, VTK, boost,
ffmpeg, and even our overlaid PCL, which happens to hash-match nixpkgs — is
already on `cache.nixos.org`: when the closure was first pushed, `cachix`
reported `Pushing 18 paths (1648 are already present)`. Those 18 are the only
ones that cost us anything, and they compress to **0.53 GiB — roughly 11% of
the 5 GB tier**:

| Path | In cache (compressed) | NAR size | Why cache.nixos.org cannot serve it |
|---|---:|---:|---|
| `reusex-gui-frontend-…-npm-deps` | 338.5 MiB | 338.9 MiB | fixed-output npm dep tarball (already compressed) |
| `libtorch` | 108.6 MiB | 592.2 MiB | `pkgs/libtorch` — pinned upstream binary release |
| `opencv-4.13.0` | 29.2 MiB | 91.2 MiB | `overlays/` — non-default feature set |
| `openmvs-2.4.0` | 19.4 MiB | — | `pkgs/openmvs` — not in nixpkgs |
| `opennurbs-…` | 12.7 MiB | 44.5 MiB | `pkgs/opennurbs` — not in nixpkgs |
| `rtabmap-0.23.2` | 9.4 MiB | 30.1 MiB | `pkgs/rtabmap` — not in nixpkgs |
| `tokenizers-cpp-0.1.1` | 6.0 MiB | 29.4 MiB | `pkgs/tokenizers-cpp` — not in nixpkgs |
| `gtsam-4.2.1` | 3.8 MiB | 12.4 MiB | `overlays/` — pinned version |
| `highs-1.14.0` | ~2 MiB | 7.5 MiB | `overlays/highs.nix` — `CUPDLP_GPU=OFF` |
| `aws-sdk-cpp` (+ `-dev`) | 2.3 MiB | 10.7 MiB | pulled in by OpenMVS |
| `reusex-gui-frontend`, `zed-open-capture`, patches, sources | 8.5 MiB | ~11.5 MiB | vendored inputs / our own outputs |

`cachix push` already knows to skip anything the cache's configured upstream
(`cache.nixos.org`) can serve, so handing it the whole closure is safe — it
uploads the delta, not 9.62 GiB. Note `libtorch` and OpenCV are the paths
where compression pays off most; the npm dep tarball is already compressed and
is the single biggest line item.

The one thing deliberately *not* seeded is the `ReUseX-tests` output itself —
see "Staying inside 5 GB" below.

## Seeding / topping up from a dev machine

```bash
# 1. Build the exact thing CI builds
nix build .#checks.x86_64-linux.tests

# 2. Push the full build closure; upstream-cached paths are skipped
nix-store -q --requisites --include-outputs \
  $(nix path-info --derivation .#checks.x86_64-linux.tests) \
  | grep -v '\.drv$' \
  | cachix push reusex
```

You need a personal Cachix auth token (`cachix authtoken <token>`, stored in
`~/.config/cachix/cachix.dhall`). `cachix` is not in the dev shell; use
`nix run nixpkgs#cachix -- …`.

To see what a push *would* cost before doing it:

```bash
nix path-info --json $(…path list…) \
  | jq '[.[].narSize] | add / 1024 / 1024 / 1024'
```

## Staying inside 5 GB

Two design decisions keep the cache from growing without bound:

1. **CI writes only from `main`.** Even with the `lib.fileset` source filter
   in place (see below), a commit that touches any build-relevant file still
   produces a distinct `ReUseX-tests` output path. If PR runs pushed, a few
   dozen PRs would exhaust the quota with outputs nothing will ever reuse.
   `ci.yml` therefore sets `skipPush` for everything that is not a push to
   `main`. PRs still *read* from the cache, which is where all the value is:
   the dependency closure, not the build product.

2. **Only the CPU variant is cached.** The CUDA closure (TensorRT, cuOpt,
   CUDA-enabled OpenCV, the `.cu` kernels) is many times larger than the whole
   free tier and is useless to GitHub-hosted runners, which have no GPU. CUDA
   coverage stays a local / self-hosted-runner concern (#202's optional task).

If the cache does fill up, the recovery is to delete old `ReUseX-*` outputs
from the Cachix dashboard — the dependency paths are the ones that must
survive.

## Runner disk

Not a problem, contrary to the usual assumption. Measured on a green run
carrying no cleanup step at all: `/` is **145 GB and still had 80 GB free at
the end of the job**, with `/nix/store` at **7.0 GB**. `ci.yml` therefore does
*not* carry the widely-copied "free up runner disk space" step — it cost 38 s
to reclaim space nothing needed. The `Report disk usage` step is the early
warning if a future dependency bump changes this.

## Where the time actually goes

Measured on a green PR run (25 min 39 s wall, all of it on `ubuntu-latest`,
4 vCPU):

| Phase | Time |
|---|---:|
| Job setup, checkout, install-nix, cachix-action | 50 s |
| **Pull the whole non-substitutable closure from Cachix** | **~38 s** |
| Compile ReUseX (CPU variant, `-j4`) | ~20 min |
| `ctest --parallel` (577 tests) | 3 min 48 s |
| Post steps | 2 s |

The cache does its job completely — 38 seconds for everything
`cache.nixos.org` could not serve. What it *cannot* fix is the ~19-minute
compile for a PR that touches build-relevant files, because every change in
`libs/`, `apps/`, `tests/`, etc. produces a new source hash and requires
recompiling ReUseX from scratch. PRs that touch only excluded paths (docs,
workflows, markdown) are pure cache hits — they skip the compile entirely.
That distinction is why run times split: doc-only PRs land in ~90 s, while
source-touching PRs still take ~25 min.

## Cold vs warm

- **Warm** (cache hit on every dependency): ~25 min, dominated by compiling
  ReUseX itself on a 4-vCPU runner.
- **Cold** (an overlay bump invalidated, say, OpenCV): the runner rebuilds
  that dependency from source. `timeout-minutes: 120` exists so such a run can
  still finish and repopulate the cache on `main` rather than being killed
  halfway, leaving the cache stale for the next PR.

### Filtered `src`: doc-only changes are pure cache hits

`default.nix` uses `lib.fileset.toSource` to include only the files the C++
build actually needs: `CMakeLists.txt`, `cmake/`, `libs/`, `apps/rux/`,
`apps/ruxd/`, `bindings/`, `tests/`, `scripts/check-openapi.py` (a ctest),
and two specific docs files the test suite reads at runtime
(`docs/CONTRACTS.md`, `docs/gui/openapi.yaml`, `docs/gui/events.schema.json`).

Everything else — `docs/`, `.github/`, `*.md`, `apps/blender/`, `tools/`,
`.claude/`, `overlays/`, `pkgs/`, `python/`, `scripts/` (except the one ctest
entry) — is excluded. A PR that only touches those paths produces the same
`ReUseX-tests` derivation hash, so CI resolves the entire build from the
Cachix cache in ~38 s instead of recompiling for ~20 min.

**Verification performed in #357:**

| Change | Derivation hash | Expected |
|---|---|---|
| Baseline (`src = ./.`) | `wm3ig0mn2sc1m8k3511fn5j8dabmgfmh` | — |
| After filter, no file change | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | changed (filter itself is new) |
| Append blank line to `docs/DIRECTION.md` | `vwfcqjy23w8q1d9mglqg3fh6093h57a6` | **unchanged** ✓ |
| Append comment to `libs/reusex/src/core/logging.cpp` | `qvngp54bppgmhihwrzqvidw24kcsipi2` | **changed** ✓ |
