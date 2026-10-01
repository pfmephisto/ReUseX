title: Esc in Kortlægning's EditDialog still closes and saves
labels: gui, bug, frontend

## Problem
R10 sets the project's Esc convention: Esc in a text field drops that
field's draft without committing and leaves the field; Esc outside a text
field closes or backs out. Phase 5 (Task 16) brought Kortlægning's
`DetailPanel` in line — Esc there now reverts the quantity/note draft and
returns focus to the table.

`EditDialog` was deliberately left alone (R10): Esc still closes the modal,
and the closing blur still commits whatever was being edited. This is the
opposite of the convention everywhere else in the app, and was parked
because the dialog's blur/commit ordering is "the most review-scarred code
in the frontend" and changing it needs its own pass — not because the
behaviour is correct.

## Proposed fix
- [ ] Give `EditDialog`'s fields the same `useTextDraft`/`editorKeyAction`
      treatment `DetailPanel` now uses: Esc in a field reverts that field's
      draft; a second Esc (or Esc outside any field) closes the dialog
      without committing an in-progress edit.
- [ ] Audit every blur handler in the dialog for the "commit on close"
      assumption the current code relies on, since removing it is exactly
      the kind of change the original authors flagged as risky.
- [ ] Add a Playwright assertion (next to the Task 17 flow) that Esc in an
      EditDialog field reverts rather than saves, mirroring the DetailPanel
      assertion already in place.

category=Geometry estimate=1d
