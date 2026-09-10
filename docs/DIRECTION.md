<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX — Direction

<!-- Source of truth for project heading. Agents: read this before planning
     or reviewing work. Update via PR; date every change in the changelog. -->

This document owns **intent**: where the project is heading and why. GitHub owns
**state**: what is in flight, what is done. Nothing here tracks per-issue status —
each workstream points at one anchor issue and says where it should end up.

Companion documents, referenced rather than duplicated here:

- [`docs/STANDARDS.md`](STANDARDS.md) — the objective bar every change must meet
  (module boundaries, label contract, determinism, testing, Definition of Done)
- [`docs/CONTRACTS.md`](CONTRACTS.md) — what each pipeline stage consumes and
  produces in a `.rux` project database
- [`CLAUDE.md`](../CLAUDE.md) — how to work in this repo (build, conventions, style)

---

## Mission (stable)

ReUseX turns 3D scans of existing building interiors into **semantic models that
support reuse and renovation decisions**. A capture goes in — RTABMap/ARKit
sensor frames, MuSHRoom or ARKitScenes benchmark data, E57/PLY clouds, 360°
panoramas — and what comes out is a labelled, room-partitioned geometric model
whose components (walls, windows, doors) carry structured **material passports**
in the spirit of the EU Digital Product Passport, exportable into the tools
architects and engineers actually use (Rhino/OpenNURBS, Speckle, CSV,
MaterialEPAS).

The value is not the point cloud and not the mesh. It is the answer to *"what is
in this building, in what quantity, in what condition, and can it be reused?"* —
derived from a scan rather than from drawings that no longer match reality.

Everything else in this document is in service of making that answer **accurate
enough to act on** and **cheap enough to produce** for a real building.

## Principles & constraints

- **GPL-3.0-or-later.** Dependencies and vendored code must be licence-compatible.
  Non-commercial or research-only weights and models are disqualifying, not
  negotiable — see the deferred-matcher note under Non-goals.
- **Nix-flake reproducibility is non-negotiable; Linux-first.** If it does not
  build from the flake, it is not done. Other platforms are not a goal.
- **Solo maintainer plus an agent workforce.** Prefer boring, maintainable,
  well-documented choices over clever ones. Code an agent can safely modify six
  months from now beats code that is 10% faster today.
- **Library-first.** Logic lives in `libs/reusex`; `rux`, `ruxd`, the Python
  bindings, the Blender add-on and any future GUI are thin shells over the same
  entry points. Anything that can only be done through the CLI is a design bug.
  (STANDARDS §1)
- **Every pipeline stage independently re-runnable against `ProjectDB`.** Re-annotate
  without re-importing; re-mesh without re-segmenting. `.rux` is the only
  handoff between stages. (CONTRACTS.md)
- **Measurable before improvable.** Quality claims need a metric and a fixture.
  `rux analyze quality|accuracy` and the benchmark harness exist so that
  "better" is a number, not an impression. (STANDARDS §8)
- **Deterministic by default.** Seeded, bounded, reproducible runs; no unbounded
  iteration counts. (STANDARDS §6)
- **Fail loudly.** A stage that produces empty or degenerate output must say so,
  with the reason and the numbers. (STANDARDS §5)

## Current priorities (ordered, dated)

1. **SLAM / reconstruction quality** — anchors #221, #225. Everything downstream
   inherits pose error: drifting poses smear walls, fragment plane segmentation
   and starve the solid-model solve. Target: **~10 mm plane-flatness RMS** on the
   reference office scan. As of 2026-09 the measured baseline sits around
   20 mm RMS, and joint pairwise registration alone saturates near 18 mm because
   pairwise point-to-plane cannot express global consistency — hence the owned
   plane-landmark back-end in #225. This is priority one because no amount of
   work on the stages above it can recover geometry the poses have destroyed.
2. **GUI application** — anchor #265. The pipeline is currently reachable only by
   a developer at a terminal plus a keyboard-driven PCL window. Without a
   discoverable interface the project cannot be used or evaluated by the
   architects it is for, which caps both adoption and the quality of feedback.
3. **Gaussian splatting** — anchor #240. Native C++/CUDA 3DGS seeded from the
   point cloud and trained on the posed frames and aligned panoramas. Two
   payoffs: a genuinely presentable visual record of a scanned building, and a
   photometric consistency signal that is complementary to the geometric one.
4. **Infrastructure and test health** — anchors #268, #262, plus CI (#202).
   Slow coverage runs and parallel-ctest flakiness tax every other workstream and
   directly degrade the agent development loop. This sits fourth by *ordering*,
   not by importance — it is the cost of everything above it.

Last reviewed: 2026-09-08

## Active workstreams

The anchor is where discussion belongs; sub-issues link back to it. Each carries
a `workstream: *` label so alignment can be queried with
`gh issue list --label "workstream: <name>"`.

| Workstream | Label | Anchor | Intent (one line) |
|---|---|---|---|
| SLAM / reconstruction quality | `workstream: slam-quality` | #221, #225 | Reach ~10 mm plane flatness via an owned global pose-optimization stage (plane landmarks + GNC), measured against fixture scans — not by tuning RTABMap forever |
| GUI application | `workstream: gui` | #265 | `rux gui` serving a designed web frontend locally; visual language authored in Claude Design, behaviour owned by the repo; `ruxd` stays headless and remote execution comes later over the same API contract |
| Gaussian splatting | `workstream: gsplat` | #240 | Native C++/CUDA 3DGS trained from `ProjectDB` — point-cloud-seeded Gaussians, sensor frames and sliced 360 panoramas as views, Apache-licensed gsplat kernels under a GPL trainer |
| 360 integration | `workstream: 360-integration` | #236 | Turn panoramas into wide-baseline pose-graph constraints; a panorama that sees many temporally-distant frames supplies exactly the loop closures the plane-landmark back-end cannot. Mechanism landed and tested (`rux optimize --use-panoramas`); **blocked on matcher quality** — cross-camera ORB resections are too weak to help, see [`research/panorama-loop-closure.md`](research/panorama-loop-closure.md) |
| Agent-driven modeling | `workstream: agent-modeling` | #267 | Evaluate an agent + MCP gateway + Blender path to a simplified, tagged building model as a complement to the geometric pipeline; requires headless rendering so the agent has eyes |
| Infrastructure & test health | `workstream: infra` | #268 | Keep the build, test and coverage loop fast and trustworthy so agents can iterate — includes #262 (parallel-ctest flakiness) and hosted CI (#202) |

Work that fits none of these rows is either a bugfix, a standing maintenance
obligation (STANDARDS/CONTRACTS drift, licence compliance), or a **new
direction** — and a new direction requires an edit to this document, not just an
issue.

## Non-goals / deferred

- **Non-Linux platforms.** Nix-flake-first with a CUDA-heavy dependency set; the
  cost of Windows/macOS parity buys nothing for the target user today.
  *Would change if:* a collaborator or deployment target makes it a hard
  requirement.
- **Non-commercially-licensed learned matchers (e.g. MASt3R).** Attractive on
  paper for wide-baseline loop closure, but CC-BY-NC weights are incompatible
  with this project's licence and intended use. Apache/MIT alternatives are the
  only candidates, and RGB-D input already makes the metric-scale argument for
  MASt3R moot. *Would change if:* a permissively-licensed model of comparable
  quality lands, or the matcher is fine-tuned from a permissive base.
- **Training foundation models from scratch.** Fine-tuning a permissive base is
  in scope; pretraining is not — no data, no compute, no reason.
  *Would change if:* nothing plausible.
- **HiGHS GPU (PDLP) acceleration.** Disabled at the overlay level on purpose:
  the solid-model solve is a MIP with binary variables and PDLP handles only
  continuous LP, so it could never be used. GPU acceleration for that solve goes
  through cuOpt instead. *Would change if:* HiGHS ships a GPU MIP path.
- **Remote / multi-tenant execution in `ruxd`.** `ruxd` stays a headless service
  worker; the GUI ships local-first against the same API contract so remote
  execution can be added later without redesigning it. *Would change if:* the
  local GUI is proven and a multi-machine use case appears.
- **Write/pipeline APIs in the Python bindings.** Read-only `.rux` inspection by
  design — the CLI and library remain the only way to mutate a project, which
  keeps the schema-migration surface small. *Would change if:* a consumer needs
  scripted pipeline control that MCP (#267) cannot serve.
- **Loading raw scan data into Blender.** Measured and rejected: per-frame
  objects make Blender unusable at scan scale. The Blender path is for
  *simplified* models and component inventories only (#267).
  *Would change if:* a bulk/instanced representation makes it tractable.

---

## Review-loop protocol

The workflow this document exists to support: a **supervising agent** reviews the
repository on a cadence, proposes work and files issues; **executor agents**
implement them and report back; the **maintainer** steers by editing this file.

### Supervisor / review agent

Runs as a scheduled routine or a manual session. Each pass:

1. **Read this document first** — Mission, Current priorities, Active workstreams.
2. **Review reality**: recent commits, open and recently-closed issues, PR queue,
   CI status, and any benchmark or quality regressions.
3. **File issues linked to a workstream.** Every filed issue names the workstream
   it serves, carries the matching `workstream: *` label, and references its
   anchor. An issue that serves no existing workstream must say so explicitly:
   *"new direction — requires DIRECTION.md update"*.
4. **Flag misalignment**: in-flight work serving no workstream, workstreams with
   no activity, or priorities contradicted by where effort is actually going.
   Flagging is reporting — not unilateral re-prioritisation.
5. **Propose direction edits as a PR, never silently.** When reality has drifted
   from this document — a workstream finished, an anchor closed, a priority
   overtaken — open a PR editing the workstream table and adding a dated
   changelog entry. Mission and Current priorities are proposed with reasoning
   and left for the maintainer to accept.

### Executor agents

- Read this document before starting; understand which workstream the task serves
  and why it matters.
- Report progress and outcomes **as comments on the issue**, not only in the
  session transcript.
- **Never edit Mission or Current priorities.** If the work reveals that a
  priority is wrong, say so in the issue comment and let the maintainer decide.
- If the work changes the project's direction, update this document **in the same
  PR**, with a dated changelog entry.

### Maintainer

Edits this document directly, whenever intentions change. This file is the
steering wheel; the workstream table, the labels, the issue backlog and the
agents' plans all follow from it. A direction change is a diff, dated in the
changelog — that history is the point of keeping it in the repo.

### Ownership rule

| Section | Who may change it |
|---|---|
| Mission | Maintainer only (or an agent explicitly asked to) |
| Principles & constraints | Maintainer only |
| Current priorities | Maintainer only |
| Active workstreams | Agents may propose via PR; maintainer accepts |
| Non-goals / deferred | Agents may propose via PR; maintainer accepts |
| Direction changelog | Appended by whoever makes the change, always dated |

---

## Direction changelog

- **2026-09-10** — The benchmark blocker named on 2026-09-09 is **cleared**, and
  it reverses the previous entry's headline. Synthetic, seeded drift on the
  ARKitScenes seed poses (`rux edit perturb-poses`, #338) turns three unusable
  scans into a drifting benchmark with absolute GT, and on it **`rux optimize`
  is GT-positive for the first time in this workstream**: at a moderate drift
  level (0.09–0.13 m, ~2x the measured loop-edge error) F@50mm goes 0.4640 →
  0.7927, 0.5393 → 0.6378 and 0.4182 → 0.7607, with median accuracy roughly
  halved. The mechanism is the **plane term** — `--no-plane-factors` reproduces
  *no pose stage* bit-exactly on every drifted row, because with the plane term
  off the graph is odometry plus a gauge prior and the odometry chain *is* the
  drifted chain. So "the plane term costs absolute accuracy on drifting
  captures" was an artefact of measuring it on captures with no drift to remove;
  the term is a drift-recovery mechanism whose benefit is conditional on drift
  existing. Two consequences worth carrying: the **GT-free flatness metric and
  absolute accuracy now agree** on every drifted row (they were only ever
  measured to disagree where the seed was already right), which raises the trust
  in the office scan's numbers; and recovery **collapses at heavy drift**
  (0.36–0.51 m) because plane association is seeded from the drifted poses
  through a 0.10 m gate, so the landmarks that would tie distant frames together
  are never formed — visible as the same surfaces shattering into ~2x the plane
  count. That gave a testable prediction about what a global constraint is
  needed *for* — not to replace the plane term but to pull the seed inside its
  association basin — and **the test passes**: at heavy drift, `optimize` with
  the plane term *and* XFeat loop edges reaches F@50mm **0.6042** (41069050) and
  **0.5406** (41069051), against 0.2980 / 0.2305 for the plane term alone and
  0.3478 / 0.3361 for the edges alone. Not additive — the edges restore the
  association basin and the plane term then does the fine correction, visible in
  the plane count returning from 35 to 15 on 41069051. This is the **first
  measurement in the workstream of loop closure paying against absolute GT**,
  and it re-reads #337's negative result as scan-dependent rather than
  fundamental. It also makes **matcher error a ranked objective**: the same
  configuration does nothing on 41069048, whose XFeat edges carry 169 mm of
  their own error versus ~58 mm on the other two. Loop closure (#236, #311) and
  an association-gate sweep are now both measurable experiments rather than
  guesses.
  Also landed: the loop-edge seed-disagreement gate is **scale-relative** (#339),
  since a 1.8 m room scan and an 18 m walk can no longer share an absolute
  threshold. Full write-up: `docs/research/registration-improvements.md` §10.

- **2026-09-09** — The #225 benchmark itself is now the blocker, not the
  algorithm. Two measured questions both answered no: a **plane-term weight
  sweep against absolute GT** has no interior optimum on the drifting scans
  (accuracy improves monotonically as the term is weakened; the best setting is
  the new `--no-plane-factors`, which reproduces *no pose stage* bit-exactly),
  and **XFeat loop edges** — the one front-end whose edges survive PCM and which
  closes 16.66 m of office drift — **lose GT accuracy in every configuration**
  on all three ARKitScenes scans. The reason is a signal-to-noise argument that
  reframes the workstream: those trajectories span only **1.8–2.6 m** and their
  seed poses already reconstruct to **15.7–18.6 mm** median accuracy, while an
  XFeat edge disagrees with the seed by **58–169 mm** — its own error, four
  times larger than the error it would correct (on office the same measurement
  is 14.5 m of disagreement on an 18 m trajectory, i.e. real drift). So **the
  ARKitScenes scans cannot adjudicate pose refinement at all**: on a well-posed
  capture the correct behaviour of every pose stage is to do nothing, and that
  is exactly what wins. #221 Tier 2 asked for a drifting scan *with* absolute
  GT; these supply the GT but not the drift. The next increment is therefore
  **benchmark work, not solver work** — cheapest first: synthesise realistic
  drift on the ARKitScenes seeds so their GT meshes become a drifting benchmark.
  Also measured and worth carrying: the plane term is *correctly* calibrated for
  well-posed captures (honka's GT optimum and office's GT-free optimum are both
  the shipped default), so this is a capture-dependent split, not a
  miscalibration; and the shipped 0.50 m seed-disagreement gate is
  scale-dependent — on a 2 m trajectory it selects the *wrong* edges and drops
  F to 0.29, so it should become relative to trajectory extent. Full write-up:
  `docs/research/registration-improvements.md` §9.

- **2026-09-09** — MapAnything evaluated as a loop-edge matcher (#264) and
  **rejected on measurement, not on licence**. The licence question closes
  cleanly in our favour: `facebook/map-anything-apache` is genuinely Apache-2.0
  (verified on the model card), so a pointmap *foundation* model is commercially
  available to us for the first time. But on the office scan's 1225-pair endcap
  set it produced 148 edges with the highest inlier support of any backend
  (9203, above the MASt3R NC oracle) — of which **PCM rejected 145 as mutually
  inconsistent**, leaving a 0.21 m correction on a scan that needs ~16 m, where
  XFeat's 163 edges survive intact and apply 16.66 m. This is the #236 failure
  repeated from a different front-end: a pointmap model asked about two blank
  repetitive walls always returns a geometry, so per-pair confidence is high and
  cross-pair agreement is absent. **Σ inliers is therefore not a matcher
  ranking** — it rewards precisely what PCM punishes, and any future matcher
  comparison should be judged on edges-surviving-PCM and applied correction.
  (#312 carried this into the tooling: `scripts/bench-loop-edges.sh` now emits
  `pcm_kept` and `max_shift_m` as its primary columns and §5.1's table leads
  with them, so the comparison cannot be read the wrong way round.)
  The shipping recommendation is unchanged (XFeat / LightGlue+ALIKED); the C++
  ingestion path for MapAnything is **not** worth building. Full write-up:
  [`research/loop-closure-learned-matchers.md`](research/loop-closure-learned-matchers.md)
  §5.5–5.6.

- **2026-09-08** — 360-driven pose-graph loop closure (#236) implemented and
  measured on the NewOffice drifted scan: the mechanism, guardrails and CLI
  landed, but the effect is negative (flatness_rms 25.15 → 27.08 mm vs the
  `optimize` baseline) because cross-camera ORB resections between the 360
  camera and the iPad are too weak — forcing the edges through `--loop-trust`
  collapses quality to 35.03 mm, confirming they are genuinely wrong rather than
  merely suppressed. The workstream's next step is therefore a **learned
  matcher**, which moves it behind the #221/#225 matcher work rather than ahead
  of it. Notably this does **not** settle the #225 odometry-trust hypothesis:
  edge quality is a confound, and the clean test still needs accurate
  wide-baseline edges. Full write-up:
  [`research/panorama-loop-closure.md`](research/panorama-loop-closure.md).

- **2026-09-08** — Document created (#266). Established the four-way priority
  ordering (SLAM/reconstruction quality → GUI → Gaussian splatting →
  infrastructure/test health) and the six active workstreams. Seeded from the
  open issue set, the "Reuse Development" project board, `docs/CONTRACTS.md`,
  `docs/STANDARDS.md`, and an unpublished 2026-05 progress draft — the last of
  which was verified against the tree and found stale in several places
  (module split #222, CLI spellings, SAM2 → SAM3/3.1, HiGHS-only → cuOpt/HiGHS
  solver choice, schema v4 → v11); the code won every disagreement.
