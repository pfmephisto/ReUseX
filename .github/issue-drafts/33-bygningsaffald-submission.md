title: Indberetning sends nothing — post the fractions to bygningsaffald.dk
labels: gui, enhancement, backend, frontend

## Problem
Phase 5 built Indberetning (`/indberetning`) with the prototype's
`Send til bygningsaffald.dk` button and its gate (R9): the button is
disabled while any type blocks the report or there is no fraction to send,
and otherwise enabled. Clicked, it **posts nowhere**. It shows a notice
(`SEND_NOTICE` in `apps/rux/frontend/src/indberetning/model.ts`) saying
nothing was sent, and the surveyor downloads the fractions as CSV and types
them into the portal by hand.

So the screen's main action is a placeholder. The numbers are already in the
portal's structure (EAK code, behandling, contaminated, tonnes, from
`GET /survey/fractions`); only the submission is missing.

## Proposed fix
- [ ] Find out what bygningsaffald.dk accepts: an API (and its auth — MitID
      Erhverv / a system certificate), a bulk-upload file, or neither. This
      decides everything below.
- [ ] If there is an API: a server-side submission endpoint (credentials
      never reach the browser) that posts the ready fractions, records the
      submission (when, by whom, the portal's receipt / case id) in the
      project, and refuses while the report is not `ready`.
- [ ] Indberetning: the button calls it through the page's mutation queue,
      shows the receipt, and lists past submissions; resubmission after a
      change is explicit.
- [ ] If there is no API: keep the CSV path, and replace the button with an
      honest "Åbn bygningsaffald.dk" link so the screen stops implying a send.

category=I/O estimate=1w
