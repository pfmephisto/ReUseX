# Kortlægning fixes + Segmentering view — design (2026-10-06)

Source: user request 2026-10-06 ("work through all the steps independently").
The user did not review this document; it records the controller's rulings so
implementers and reviewers share one authority. Investigation notes with
file:line anchors: `explore-ui.md` and `explore-seg.md` (session scratchpad,
copied into each plan workspace).

Two independent streams, each its own worktree/branch, merged locally:

- **Stream A — Kortlægning & navigation** (branch `fix-kortlaegning-nav`)
- **Stream B — Segmentering** (branch `feat-segmentering-view`)

Global rules (both streams): Danish copy on case screens; CSS Modules with
tokens only (`var(--…)`, never edit `src/tokens.css`); SPDX header on every new
file; types mirror `docs/gui/openapi.yaml` (update the contract with every
endpoint change); library-first (logic in `libs/reusex`, thin HTTP handlers);
STANDARDS §3 label contract, §4 parameters (no hard-coded magic thresholds in
library code — options structs with defaults), §5 no silent failure; pure logic
in `.ts` modules with vitest tests (vitest runs in node, no DOM); C++ Catch2
tests using `tests/support/temp_path.hpp`; `ctest --parallel`; visual check of
every changed screen in both themes and at 390px width.

---

## Stream A — Kortlægning & navigation

### A1. Værktøjer is a collapsible nav group, closed by default
- The "Værktøjer" label in `Sidebar.tsx` becomes a `<button aria-expanded
  aria-controls>` with a chevron; the tools `<nav>` is hidden when collapsed.
- Default closed. It **auto-opens while the current route is a tools route**
  (pure helper in `app/navigation.ts`, e.g. `isToolsPath(pathname)`), so the
  active link is never hidden. The user's explicit open/close choice is kept
  per viewer in localStorage (try/catch; storage failure = default closed).
- Clicking the toggle in the mobile drawer must not close the drawer.

### A2. Projektdata moves into Overblik, Projektdata retires
- Overblik gets a final, visually quiet section **"Projektdata"** below the
  existing panels: a collapsed-by-default disclosure (closed `<details>`-style
  toggle, muted text, small type, no cards/accent). Inside: a one-line
  technical summary (point clouds + total points, meshes, sensor frames
  (+segmented), panoramas (+matched), components, schema version, project
  path), **Seneste aktivitet** (`PipelineLogList compact` + link to the full
  log, own error surface), **Punktskyer**, **Meshes**, **Komponenter pr. type**
  tables (the column definitions extracted from `Dashboard.tsx` into a shared
  module, not duplicated).
- The duplicate project-card editor in `Dashboard.tsx` is dropped (Overblik's
  `ProjectMetaForm` already covers it). `Dashboard.tsx` is deleted, the
  Projektdata nav entry removed, and `/projektdata` redirects to `/`.

### A3. Reject ≠ delete
- **Reject** (type-level, existing `review_status=rejected`) keeps the row.
  Kortlægning gets a fourth tab **"Afvist (n)"** listing rejected types; they
  are excluded from the other tabs as today. In the Afvist tab the actions
  are "Genåbn" (back to queue) and "Slet". Toast after reject: "Afvist som
  fejldetektion — flyttet til Afvist".
- **Delete** is a separate, destructive action with the existing two-click
  armed confirm (`useArmedConfirm`), available in every tab, for both a type
  (deletes the type and all its parts) and a single part (any part, not only
  manual ones).
- Deleting scan-backed parts must stick: schema **v26** adds a tombstone table
  `survey_dismissed_instances(instance_guid TEXT PRIMARY KEY, dismissed_at)`.
  Deleting a part (or a type) records the `instance_guid` of every
  instance-backed part it removes; `sync_survey` skips dismissed guids. The
  passport of a deleted part is deleted unless linked elsewhere (existing
  `delete_resource` rule). Library: `core::delete_resource` loses its 409 for
  instance-backed parts; new `core::delete_survey_type(db, type_id)`.
  HTTP: `DELETE /api/v1/resources/<code>` (now any part) and new
  `DELETE /api/v1/survey/types/<id>`. A "Gendan slettede scan-ressourcer"
  action is out of scope (the tombstone table makes it possible later; note
  it in DIRECTION).

### A4. Resource images in Kortlægning (per prototype v2)
- Template: `docs/gui/images/prototype-v2/kortlaegning.png` and
  `dialog.png` (prototype artifact `At7n5is3zp7cYX54faGJkn`): part rows carry
  "n fotos"; the dialog has a Fotos strip; evidence has a 360° view.
- New batched endpoint `GET /api/v1/survey/photos` → `{parts: {<code>:
  {count, best_frame_id|null}}}` computed in one pass (load base cloud +
  instance cloud once, centroid per instance, `core::visible_frames`), so the
  table never does N per-row calls. Library function in core next to
  `frame_visibility`. Parts without an instance link are absent/zero.
- Table: each instance-backed part row shows a small thumbnail of its best
  frame (lazy `<img loading="lazy">`, `frameImageUrl(id,'color',{maxSize:96})`)
  and "n fotos" in the last column (replacing the literal "—"); type rows show
  the thumbnail of their first part with a photo. Rows without photos show a
  quiet placeholder, no layout shift.
- DetailPanel gets the same Fotos strip as the dialog (reuse the EditDialog
  `PhotoThumb`/strip — extract a shared component).

### A5. 360° in the evidence panel and the double-click dialog
- New library function + endpoint `GET /api/v1/instances/<cloud>/<id>/panoramas`
  → panoramas ranked for the instance: placeable panoramas (aligned or
  frame-matched pose), distance from the panorama centre to the instance
  centroid, and the equirect `u,v` (0..1) where the centroid appears. Sort by
  distance; an options struct carries max distance (default 15 m). Use
  `geometry/EquirectProjection.hpp` conventions if reachable from core, else
  replicate the bearing math with a test pinning it against the frontend's
  `bearingToUv`.
- Evidence tabs become **Plan · 360° · Foto · Punktsky · Rum** (keys 1–5,
  hint text updated). 360° renders the chosen panorama's equirect as a
  horizontally scrollable/pannable strip centred on the part's `u`, with a
  marker at `(u,v)`, the caption "<rum> · 360°", and a link "Åbn i viewport"
  (`/viewport?pano=<id>`). Empty state when no panorama is placeable:
  "Ingen 360°-optagelse nær denne ressource".
- **Ruling (2026-10-06, review of Task 3):** the endpoint stays sorted by
  distance, but the UI shows the nearest **resected** panorama (`rux align
  360` measured its heading, so `u` and the marker point at the part). Only
  when no resected panorama is in range does it fall back to the nearest
  levelled one (position borrowed from the matched frame, heading unknown):
  shown **without a marker** and captioned "<rum> · 360° · retning ukendt".
  Reason: on NewOffice the nearest panorama is often levelled while a
  resected one stands a few metres further off, and a marker on a guessed
  heading points at the wrong wall.
- The dialog's evidence stage shows the same 5 sources.

---

## Stream B — Segmentering

### B1. No model path in the UI; managed SAM3 provisioning
- Remove every model-path field, state and request field from the frontend
  (`SegmentPanel`, `PanoramaSegmentPanel`, `LabelQueuePanel`, `labelQueue`,
  `LabelQueueContext`, request types → `model_path?` omitted). Stored queue
  items carrying `modelPath` must still load (ignore the field).
- Add `Sam3ModelStatus` type + `api.sam3Status(cuda?)` for
  `GET /api/v1/models/sam3/status`. A shared pure module decides the UI state
  from (status, last segment response): `ready` → run; `absent|not_built` →
  "Første kørsel henter og bygger modellen (kan tage flere minutter)";
  `downloading|building` → progress bar + message, poll every 2 s, then
  automatically retry the pending run; `error` → message + "Prøv igen";
  a 503 while status is `ready` = busy DB → retry after 1 s, max 3 times.
  500 "SAM3 model preparation failed…" → show message. 409 keeps its copy.
- Fix the existing bug: a box/point prompt with an empty class name must
  work. Backend: `parse_prompts` accepts empty text when the prompt has boxes
  (the segmenter receives the SAM3 geometric-only convention — check what the
  Sam3 tokenizer path expects, e.g. `"visual"`, and use that). The "Pipeline
  page" hint after segmenting is removed.

### B2. Library + endpoint: project a mask into the cloud and create a resource
- Library (core or segmentation — must be reachable from `rux_gui_lib`,
  no RTABMap): `project_frame_mask(db, frame_id, mask, opts) -> pcl::Indices`
  (or `std::vector<std::size_t>`): pinhole projection with pose ·
  local_transform, intrinsics scaled to mask size, occlusion test against the
  frame's own depth image (`|z - depth| ≤ opts.depth_tolerance`, default
  0.10 m; `opts.max_depth` default 8 m), against the base cloud `cloud`.
  Zero hits → warn with numbers (STANDARDS §5).
- Library: `apply_mask_selection(db, indices, class_name, opts)`: find or
  create the class id in `label_definitions("labels")` by name (create an
  all-zero `labels` cloud if absent), set selected points; allocate a new
  instance id `max+1` in `instances` (create if absent), assign the points,
  append an `InstanceRecord` with a fresh GUID, extend
  `label_definitions("instances")` (`SM<cls>-<id> (<n>p)` format); update the
  point counts of instances that lost points. Then create a survey part linked
  to the new instance: in the given `type_id`, or find/create a survey type
  named after the class. One transaction. Pipeline log entry.
- Endpoint `POST /api/v1/frames/<id>/segment/resource`
  `{mask_label: <prompt index in the saved segmentation image>, class_name,
  type_id?}` → `{resource_code, type_id, instance_id, instance_guid,
  point_count, label_id}`; 422 when the frame has no pose/depth or no saved
  segmentation, 400 bad body, 409/503 per `with_write`. Documented in
  openapi.yaml.
- WebSocket event `clouds.changed {names:[…]}` broadcast after this endpoint
  (documented in `docs/gui/websocket-events.md`); the frontend bumps a cloud
  revision that `useCloudStream` depends on and reloads the clouds list, so
  the viewport shows new labels without a reload.

### B3. Segmentering view
- New route **`/segmentering`** in the main (Sag) group after Viewport, nav
  label "Segmentering". Query: `?frame=<id>&u=<px>&v=<px>` (path + href
  builder + parser in `app/links.ts`).
- Layout: the image is front and centre (fills the main area, aspect kept,
  zoom-to-fit), prompt drawing on top (box drag, click = point box), a
  side/bottom strip with: frame picker (filmstrip of frames, segmented
  indicator, prev/next keys), prompt list (class name per prompt, delete),
  confidence, model status chip (B1), "Kør segmentering", mask overlay toggle,
  and "Tilføj til kø" (neighbour window, existing label queue).
- After a run: the result lists each class (prompt) with its pixel count;
  selecting one highlights it; **"Opret ressource fra markering"** opens a
  dialog (class name prefilled from the prompt text; target type: existing
  survey type select or "Ny type: <class name>") → B2 endpoint → toast with
  the resource code and links "Vis i Kortlægning" (`/kortlaegning?part=`-style
  deep link if one exists, else the page) and "Vis i Viewport".
  Frames without pose/depth: button disabled with the reason.
- `u,v` in the URL seed a point prompt at that pixel.
- Viewport: clicking an image in `SourceImagePanel` (after picking a point)
  navigates to `/segmentering?frame=<id>&u=&v=` with the frame's `u,v`.
- Billeder: the FrameDetail "segment" tab is replaced by a "Segmentér" button
  linking to `/segmentering?frame=<id>`; `SegmentPanel` logic moves into the
  new view (no duplicate). Panorama segmentation stays where it is (only B1
  applies to it).
