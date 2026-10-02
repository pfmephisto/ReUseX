title: Commit a Playwright regression suite; close the test gaps it found
labels: gui, enhancement, frontend, testing

## Problem
Phase 6's regression checks for the responsive shell (R5), the app-wide
write chain (R11) and the cross-link style (R12) exist only as ad hoc
scripts in an agent's scratchpad, not in the repo:
`phase6_flow.py`, `phase6_shots.py` and `onsite_live.py`. Nothing stops
the next change from silently breaking a drawer focus trap, a stale-read
race, or a 390px overflow that these scripts caught and a human reviewer
would not reliably re-check by hand.

While writing and running them, a few gaps turned up that the suite does
not close yet:
- the 390px tap-target and overflow probe covers the seven case screens but
  not dialogs/forms opened over them — Kortlægning's `EditDialog` and
  Miljø & prøver's new-sample form were never shot at phone width;
- the drawer's `ThemeToggle` is a second instance of the title bar's
  (`Sidebar.tsx`), kept in sync only by both reading the same
  `useTheme()`/`localStorage` state — a future change that shows both at
  once (rather than one hidden by CSS) would silently desync or duplicate
  radiogroups;
- the 44px `::after` hit areas added for phone tap targets
  (`controls.module.css` `.textBtn`/`.crossLink`) are centred over their
  text and can overlap a link or button on an adjacent line in dense prose;
  none of today's screens hits this, but nothing asserts it won't;
- `gui_api_contract_parses` (`tests/CMakeLists.txt`) only runs
  `scripts/check-openapi.py`, which parses `docs/gui/openapi.yaml` as YAML —
  it does not compare the operations/schemas it declares against what the
  server actually registers or emits, so a route or a response field can
  drift from the spec with the test still green.

## Proposed fix
- [ ] Commit a Playwright suite (e.g. `tests/e2e/gui/`) built from the
      scratchpad scripts above, run against a `rux gui` instance the suite
      starts and stops itself, wired into CI alongside the existing
      frontend gates.
- [ ] Extend the 390px probe to dialogs and in-page forms opened over a
      screen (`EditDialog`, the new-sample form), not just the screens
      themselves.
- [ ] Either show the drawer `ThemeToggle` and the title-bar one from one
      shared instance (teleported, not duplicated), or add a test that
      fails if both are ever visible at once.
- [ ] Add a probe for overlapping `::after` hit areas on adjacent inline
      links/buttons, or scope the 44px expansion down where two targets
      sit on neighbouring lines.
- [ ] Extend `gui_api_contract_parses` (or add a sibling test) to diff the
      server's actual route/response shape against `openapi.yaml`, not just
      parse the YAML.

category=CLI estimate=3d
