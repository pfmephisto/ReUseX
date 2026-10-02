title: On-site during a live capture
labels: gui, enhancement, vision

## Problem
The prototype frames On-site as "en afbrydelse, ikke listen": the scan is
running and the surveyor stops only to mark something. `rux gui` has no live
capture, so Phase 6 (R6) builds On-site as a walk through the parts that a
finished scan produced, with the best stored sensor frame in place of the
camera and the type's stored AI confidence in the detection chip.

## Proposed fix
- [ ] Decide where live capture runs (a phone app feeding RTABMap, or a
      `ruxd` ingest) and how its frames reach the project while scanning.
- [ ] A live detection stream (SAM3 on the incoming frames) that names the
      object under the reticle, so the chip says what is being looked at now.
- [ ] Attach ★/notes/samples made during capture to the instance once
      `rux create instances` and `rux create survey` have run, by position.

category=Vision estimate=2w
