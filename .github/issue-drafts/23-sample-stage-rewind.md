title: Allow rewinding a sample's stage beyond "Fortryd svar"
labels: gui, enhancement, frontend

## Problem
Miljø & prøver's stage chain is Planlagt — Udtaget — Sendt til lab — Svar
modtaget. The only backward step the screen offers is `Fortryd svar`, which
returns a sample from *svar* to *sendt* with no result (Phase 4; chosen
deliberately over *svar* with no result, which would silently count as
clean and un-gate its types). There is no way back from *udtaget* to
*planlagt*, or from *sendt* to *udtaget* — a surveyor who advances a sample
by mistake, or whose lab returns a sample as unprocessable, is stuck.

## Proposed fix
- [ ] Generalise the one rewind button into a stage-aware "back" action:
      *sendt* → *udtaget* (clears nothing, since no result exists yet),
      *udtaget* → *planlagt*. Keep the existing *svar* → *sendt* behaviour
      and its "no silent clean result" guard.
- [ ] Decide whether rewinding from *sendt* should warn when a lab shipment
      is assumed to be in flight — a one-line confirmation is probably
      enough, not a full undo log.
- [ ] Re-run the approval-gate toast logic (how many types were un-gated or
      re-gated) for every rewind step, not just the *svar* one.
- [ ] Extend the `samples.ts` / Miljø `model.ts` stage-chain tests to cover
      each new transition.

category=I/O estimate=4h
