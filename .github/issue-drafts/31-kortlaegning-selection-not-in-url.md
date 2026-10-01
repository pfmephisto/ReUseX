title: Kortlægning's selected row is not reflected in the URL
labels: gui, enhancement, frontend

## Problem
`?type=<id>` on `/kortlaegning` is read once, at mount, to honour a
cross-screen deep link (a sample's "Koblet:" type name, or a new-sample
form's pre-linked type) — `KortlaegningPage.tsx`'s `initialViewFor`. Once
the page is open, selecting a different row in the table does not update
the URL at all: `replacePart`/`replaceType` update local state only.

So Back, after following a cross-screen link into Kortlægning and then
clicking around the table, lands on row 0 rather than wherever the
surveyor actually was — the browser history has nothing else to return to.

## Proposed fix
- [ ] Keep `?type=<selected>` (and, for a child row, whatever identifies
      the part) in sync with the current selection using `navigate(...,
      { replace: true })` — `replace`, not `push`, so every row click does
      not pollute history; only the initial deep-link navigation and
      genuine route changes should create history entries.
- [ ] Reconcile with the existing `initialViewFor` mount-time read so the
      two don't fight (the URL should be the single source of truth for
      "what's selected" once the page is open).
- [ ] Extend `kortlaegning.page.test.ts` / `model.ts` tests to cover the
      URL round-trip (select → URL reflects it → reload restores it).

category=CLI estimate=4h
