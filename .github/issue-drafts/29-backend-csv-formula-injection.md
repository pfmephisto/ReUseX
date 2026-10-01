title: Backend /exports/csv may lack CSV formula-injection guarding
labels: backend, bug, security

## Problem
Phase 5 (Task 15) found and fixed a CSV formula-injection gap in the new
Indberetning fraction-table download: a text field beginning with `=`,
`+`, `-` or `@` is interpreted as a formula by Excel/Sheets on open, so the
fix prefixes such fields with `'` before CSV-escaping (text columns only).

The material-passport CSV export (`GET /api/v1/exports/csv`, "Eksport
(XLS)" in Kortlægning and the CSV section of `/export`) was not audited for
the same issue. Its escaping, `csv_esc()` in `apps/rux/src/gui/api.cpp`,
only quotes a field containing `,`/`"`/newline — it does not guard a field
that starts with a formula-trigger character:

```cpp
std::string csv_esc(const std::string &f) {
  if (f.find_first_of(",\"\n\r") == std::string::npos)
    return f;
  ...
}
```

`apps/ruxd/src/handlers/exports.cpp` has its own, separately maintained
`csv_escape()` with the same shape — both need checking, since `rux gui`
and `ruxd` currently duplicate this export rather than sharing one
implementation.

Passport property values (free text, e.g. material notes) are
user-entered, so a value like `=cmd|'/c calc'!A1` pasted into a passport
field would round-trip into a formula-triggering cell on export.

## Proposed fix
- [ ] Confirm the gap with a test: a passport property value starting with
      `=` exported via `/exports/csv`, opened in a spreadsheet, executes as
      a formula.
- [ ] Apply the same `'` prefix guard the Indberetning CSV now uses, scoped
      to text-like columns, in both `csv_esc()` (`rux gui`) and
      `csv_escape()` (`ruxd`) — or better, factor the escaping into one
      shared function `reusex_core` exposes, closing the duplication at the
      same time.
- [ ] Add a unit test per server asserting the guard.

category=I/O estimate=2h
