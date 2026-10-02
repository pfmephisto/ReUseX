title: On-site: add extra photos to a bygningsdel from the phone
labels: gui, enhancement, backend, frontend

## Problem
The prototype's On-site sheet has `＋ Tilføj ekstra foto`. Phase 6 does
not draw it (R9): the spec's Phase 6 line names ★, note and sample only,
and a photo needs storage the project does not have — the same gap that
keeps Miljø & prøver's `Upload miljørapport (PDF)` undrawn (draft 22).
Today the only photos of a part are the sensor frames its instance was seen
in, which may not show what the surveyor stopped for (a label, a crack, a
fixing).

## Proposed fix
- [ ] Design the attachment storage once for both this and draft 22: a
      `part_photos` table (part code, blob, content type, taken_at), chunked
      like `report_pdfs` if a photo can be large, plus a size cap.
- [ ] `POST /api/v1/survey/parts/{code}/photos` (raw `image/jpeg` body, as
      the thumbnail PUT does) and `GET …/photos`, `GET …/photos/{id}`;
      openapi entries for all three.
- [ ] Draw `＋ Tilføj ekstra foto` on the sheet with
      `<input type="file" accept="image/*" capture="environment">`, through
      `useMutationQueue`; downscale on the client before upload.
- [ ] Show the photos in Kortlægning's evidence panel (Foto tab: stored
      photos first, then the best sensor frame) and in the edit dialog's
      photo strip, and fill the child rows' photo count the spec still owes.

category=I/O estimate=2d
