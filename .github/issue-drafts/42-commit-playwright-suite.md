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
  Miljø & prøver's new-sample form were never shot at phone width. Read
  from the code, two of them miss the 44px target at 390: the `EditDialog`
  chrome buttons are 32px, and Miljø's `LinkPicker` rows are about 28px;
- the drawer's `ThemeToggle` (`Sidebar.tsx`) and the title bar's are two
  rendered instances. They now share one `useTheme()` state, called once in
  `AppShell` and passed to both, so they cannot drift; but CSS is all that
  keeps one of them hidden, so a change that shows both at once would
  duplicate the radiogroup;
- the 44px tap target is written two ways, as
  `var(--layout-titlebar-height)` (`controls.module.css`, the Kortlægning
  phone rules) and as `calc(var(--space-6) + var(--space-3))` (On-site's
  sheet and picker). A `--tap-target` token would name it once;
- the 44px `::after` hit areas added for phone tap targets
  (`controls.module.css` `.textBtn`/`.crossLink`) are centred over their
  text and can overlap a link or button on an adjacent line in dense prose;
  none of today's screens hits this, but nothing asserts it won't;
- `gui_api_contract_parses` (`tests/CMakeLists.txt`) only runs
  `scripts/check-openapi.py`, which parses `docs/gui/openapi.yaml` as YAML —
  it does not compare the operations/schemas it declares against what the
  server actually registers or emits, so a route or a response field can
  drift from the spec with the test still green. The contract test should
  assert that the keys each response actually emits match the properties
  of its schema in `openapi.yaml`.

## Proposed fix
- [ ] Commit a Playwright suite (e.g. `tests/e2e/gui/`) built from the
      scratchpad scripts above, run against a `rux gui` instance the suite
      starts and stops itself, wired into CI alongside the existing
      frontend gates.
- [ ] Extend the 390px probe to dialogs and in-page forms opened over a
      screen (`EditDialog`, the new-sample form, `LinkPicker`), not just
      the screens themselves, and bring the `EditDialog` chrome buttons
      and the `LinkPicker` rows up to 44px at phone width.
- [ ] Add a test that fails if both `ThemeToggle`s are ever visible at
      once (or render one instance, moved between title bar and drawer).
- [ ] Propose a `--tap-target` token (44px) to the Claude Design project
      and replace both spellings of the 44px target with it; `tokens.css`
      is synced from there, not edited here.
- [ ] Add a probe for overlapping `::after` hit areas on adjacent inline
      links/buttons, or scope the 44px expansion down where two targets
      sit on neighbouring lines.
- [ ] Extend `gui_api_contract_parses` (or add a sibling test) to diff the
      server's actual route/response shape against `openapi.yaml`, not just
      parse the YAML: assert that each response's emitted keys match its
      schema's properties.

category=CLI estimate=3d
