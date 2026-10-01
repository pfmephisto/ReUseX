title: AppShell overflows horizontally at 390px (phone width)
labels: gui, bug, frontend, phase-6

## Problem
`AppShell` overflows horizontally at a 390px viewport whenever the topbar
and the open sidebar are both shown — on every route, not just the case
screens. This predates Phase 4 and is explicitly out of scope through
Phase 5 (R12): `docs/design/gui-kortlaegning-redesign.md`'s Out of scope
list carries it, and Phase 5's own verification only checks that `<main>`
has no horizontal overflow at 768px; a 390px shot is taken for the record
only, not asserted on.

## Proposed fix
- [ ] Design a collapsible sidebar (hidden by default, opened by a topbar
      toggle) for widths below some breakpoint — this needs a design pass,
      not just a CSS squeeze, since the sidebar carries the whole case
      navigation and badge counts.
- [ ] Phase 6 (On-site) is where this belongs: it already designs a phone
      capture-sheet layout, so the collapsible-sidebar mechanism and the
      On-site layout should land together.
- [ ] Once built, extend the Phase 5/17-style screenshot verification to
      assert no horizontal overflow at 390px, not just record it.

category=Visualization estimate=2d
