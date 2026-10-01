title: Backend /exports/csv has no CSV formula-injection guard
labels: backend, bug, security

## Problem
Phase 5 (Task 15) found and fixed a CSV formula-injection gap in the new
Indberetning fraction-table download: a text field beginning with `=`,
`+`, `-` or `@` is interpreted as a formula by Excel/Sheets on open, so the
fix prefixes such fields with `'` before CSV-escaping (text columns only).

The material-passport CSV export (`GET /api/v1/exports/csv`, "Eksport
(XLS)" in Kortlægning and the CSV section of `/export`) has the same gap —
**confirmed**, not suspected. Both escapers only quote; neither guards a
field that starts with a formula-trigger character:

- `csv_esc()`, `apps/rux/src/gui/api.cpp:2656` (`rux gui`);
- `csv_escape()`, `apps/ruxd/src/handlers/exports.cpp:47` (`ruxd`), a
  separately maintained copy of the same shape.

```cpp
std::string csv_esc(const std::string &f) {
  if (f.find_first_of(",\"\n\r") == std::string::npos)
    return f;
  ...
}
```

This predates the Phase 5 branch: both functions came in with the CSV
export endpoint (#459, `422490a0`). Phase 5 did not touch them.

Passport property values (free text, e.g. material notes) are
user-entered, so a value like `=cmd|'/c calc'!A1` pasted into a passport
field round-trips into a formula-triggering cell on export.

A correct guard needs **per-column typing**, which the escapers do not have
today: they see a bare string. A numeric column must stay a number — a
value of `-1.5` (a confidence delta, a coordinate) must **not** get a `'`
prefix, or the spreadsheet reads it as text. So the fix is not a one-line
change to `csv_esc`; the caller has to say which columns are text.

## Proposed fix
- [ ] A test per server: a passport property value starting with `=`
      exported via `/exports/csv` comes out unguarded today.
- [ ] Carry a column kind (text / numeric) to the escaper, and apply the
      `'` prefix to text columns only — the rule the Indberetning CSV uses
      (`apps/rux/frontend/src/indberetning/model.ts`, `csvText`).
- [ ] Do it once: factor the escaping into one shared function both `rux gui`
      and `ruxd` call, closing the duplication at the same time.
- [ ] Tests asserting that a text `=1+1` is prefixed and a numeric `-1.5`
      is not, in both servers.

category=I/O estimate=4h
