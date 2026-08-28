title: Enable hosted CI (Cachix cache + push/PR triggers)
labels: infrastructure, phase-0

## Problem
`.github/workflows/ci.yml` exists but is `workflow_dispatch`-only: the nix
closure (overlaid PCL/OpenCV/RTABMap/OpenMVS/libtorch) is too large to build
from scratch on GitHub-hosted runners without a binary cache.

## Tasks
- [ ] Create Cachix cache (free OSS tier) and add `CACHIX_AUTH_TOKEN` secret
- [ ] Seed the cache from a machine with the CPU variant built:
      `nix build .#checks.x86_64-linux.tests && nix path-info -r .#cpu | cachix push reusex`
- [ ] Flip `ci.yml` triggers to `push`/`pull_request`
- [ ] Optional: register a self-hosted runner for the CUDA variant

## Acceptance
CI runs build + ctest on every PR in < 20 min warm.

category=CLI estimate=4h
