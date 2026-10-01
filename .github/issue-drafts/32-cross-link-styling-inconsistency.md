title: Kortlægning/Miljø cross-links are styled two different ways
labels: gui, bug, frontend

## Problem
The two deep-link styles between Kortlægning and Miljø & prøver don't
match. Kortlægning's `SampleLine` (the detail panel's link to a type's
linked samples) uses `--color-accent-deep` text with an underline; Miljø &
prøver's `.typeLink` (a sample card's "Koblet: <types>" link back to
Kortlægning) uses muted text with a `--color-border-strong` underline. A
surveyor following links back and forth between the two screens sees two
different conventions for "this is a link to the other screen", with no
design reason for the difference.

## Proposed fix
- [ ] Pick one: `SampleLine`'s accent-deep-with-underline reads as more
      clearly interactive and matches the rest of the app's link styling
      better than a muted-text link, so it is probably the one to keep —
      but this is a design call, not just an engineering one.
- [ ] Fold the chosen style into a shared class in `controls.module.css`
      (both already `compose` from it) so the two components can't drift
      again, rather than hand-tuning each `.module.css` to match.
- [ ] Re-screenshot both Kortlægning's detail panel and a Miljø sample card
      (light + dark) to confirm the unified style reads correctly in both
      themes.

category=Visualization estimate=2h
