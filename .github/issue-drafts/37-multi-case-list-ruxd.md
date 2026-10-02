title: Sager: list, switch and create cases through ruxd
labels: gui, enhancement, backend, frontend

## Problem
`rux gui` serves one `.rux`, so Phase 6's Sager screen (`/sager`) shows that
project as its single card and a panel saying how to open another
(`rux -p <fil>.rux gui`). The prototype shows four cases with statuses that
only a server deployment has (`Scanning i gang`, `Afventer upload`) and a
`+ Nyt projekt` button. The spec assigns multi-project listing and switching
to `ruxd` (#265's own Phase 6 — a different numbering from the redesign's).

## Proposed fix
- [ ] A `GET /api/v1/cases` contract (id, name, address, the survey summary
      figures the card shows, a status, a thumbnail URL) that `ruxd`
      implements and `rux gui` answers with its one project, so the frontend
      has one code path.
- [ ] Case switching in the shell (the selected case scopes every other
      route's requests), and `+ Nyt projekt` creating one on the server.
- [ ] Map `ruxd`'s capture/upload job states onto the card's status pill,
      next to Phase 6's derived Kladde / Gennemgang / Klar til indberetning /
      Gennemgået.
- [ ] The card grid already uses the prototype's `auto-fill` columns, so a
      longer list needs no layout change.

category=CLI estimate=1w
