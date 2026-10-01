title: Ressourcekortlægning PDF omits thousands grouping the GUI always shows
labels: gui, bug, backend

## Problem
Danish number formatting disagrees between the server-rendered PDF and the
GUI. The Typst report's `da_number` (`libs/reusex/src/core/
report_generator.cpp`) only swaps the decimal point for a comma:

```cpp
// 1234.5 -> "1234,5"; whole numbers lose the decimal ("640").
std::string da_number(double v) { ... }
```

so a quantity like 1334.8 prints as `1334,8`. The frontend's `formatNumber`
(`apps/rux/frontend/src/kortlaegning/vocab.ts`) uses
`n.toLocaleString('da-DK', { maximumFractionDigits: 1 })`, which groups
thousands with a period, printing `1.334,8` for the same value. The same
figure reads differently depending on which surface shows it.

## Proposed fix
- [ ] Add thousands grouping to `da_number` (period every three digits, as
      da-DK formats it) so the PDF matches the GUI, or document why the PDF
      deliberately omits it (e.g. Typst/fmt locale limitations) and instead
      simplify the GUI to match.
- [ ] While in there: the Overblik hero subline prints a stored date
      (`registreret 2026-08-09`) verbatim in `heroSubline` rather than
      through `danishDate`/`versionDate` — fold that fix in if touching the
      same area, since it's the same "Danish copy must match across the
      app" problem (F24).
- [ ] Add a report-generator unit test asserting the grouped form for a
      figure ≥ 1000.

category=Geometry estimate=2h
