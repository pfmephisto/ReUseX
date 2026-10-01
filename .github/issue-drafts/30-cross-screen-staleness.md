title: Cross-screen staleness between Kortlægning and Miljø & prøver's per-page mutation queues
labels: gui, bug, frontend

## Problem
Every page keeps its own `useMutationQueue`/`SerialQueue` chain
(`docs/design/gui-kortlaegning-redesign.md`, Out of scope). A Miljø & prøver
write — recording a sample result, say — is queued on that page's own
chain. If the surveyor then navigates to Kortlægning before that write has
settled, Kortlægning's mount-time `GET /survey` can race ahead of it and
land first, so Kortlægning shows a stale approval gate (a type that should
now be un-gated still looks blocked, or vice versa) until something forces
a re-read.

This is a real sequence a surveyor hits: register a sample result from a
Miljø card's "Koblet:" link, then immediately follow the same card back to
its type in Kortlægning.

## Proposed fix
Two options, either is acceptable (pick one, pending maintainer input):
- [ ] **App-level `SerialQueue`.** Promote mutation ordering above the page:
      a single app-wide chain that every page's writes join, and every
      page's initial load awaits before issuing its first `GET`. Bigger
      change, closes the whole class of cross-screen races at once.
- [ ] **Kortlægning re-read on survey-count change.** `SurveyCountsContext`
      already polls/holds the review-queue and pending-sample counts; have
      Kortlægning re-issue `GET /survey` when those counts change under it,
      rather than relying on data being fresh at mount time only.
- [ ] Whichever is chosen, add a Playwright regression: Miljø write →
      immediate navigation to Kortlægning → assert the gate reflects the
      write, not the pre-write state.

category=I/O estimate=1d
