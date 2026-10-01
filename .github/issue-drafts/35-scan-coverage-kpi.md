title: Overblik — a real scan-coverage KPI to replace "Klassificeret"
labels: gui, enhancement, backend

## Problem
The prototype's Overblik KPI row has **Scanningsdækning** (scan coverage).
Nothing in the project measures coverage, so Phase 5 (R4) shows
**Klassificeret** in that tile instead: the share of the instance cloud's
points that carry an instance label (`classified_share` in
`GET /survey/summary`, `apps/rux/src/gui/survey.cpp`).

That is a classification measure, not a coverage one. A building can be
100 % classified and still be half scanned — rooms never entered, walls seen
from one side only — and that is what a surveyor needs to know before the
inventory can be trusted.

## Proposed fix
- [ ] Define coverage for a scan: e.g. the share of the reconstructed
      surface area (or of the rooms' floor area / wall area from
      `create rooms` + `create planes`) observed by at least one sensor
      frame at a usable range and angle.
- [ ] Compute it in the library (it depends on poses, depth and the room
      segmentation, so `reconstruction` or `segmentation`, not the GUI), and
      store or derive it so `GET /survey/summary` can return
      `scan_coverage` (null when the inputs are missing).
- [ ] Overblik: put Scanningsdækning back in the tile when it is known; keep
      Klassificeret as a secondary figure or drop it.
- [ ] Validate on a dataset with ground truth (ARKitScenes / MuSHRoom) that
      the figure falls when rooms are left out.

category=Geometry estimate=1w
