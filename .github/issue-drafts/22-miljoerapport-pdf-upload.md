title: Upload the lab's miljørapport (PDF) to a sample
labels: gui, enhancement, backend

## Problem
The prototype draws an `Upload miljørapport (PDF)` button on each sample
card in Miljø & prøver, but nothing in Phase 4 implements it: there is no
blob storage for a sample attachment and no endpoint to receive one. The
button is not drawn in v1 (`docs/design/gui-kortlaegning-redesign.md` §
"Out of scope / follow-up issues").

A surveyor's workflow for a sample currently ends at a stage/result pill —
the PDF the lab actually sends back (test results, chain of custody) has
nowhere to live in the project.

## Proposed fix
- [ ] A `sample_attachments` table (or a single nullable blob column on
      `samples`, if one file per sample is enough) storing the PDF bytes,
      content type and upload timestamp — same blob-chunking pattern as
      `point_cloud_data`/`report_pdfs` if a file can exceed sqlite's
      practical row size.
- [ ] `POST /api/v1/samples/{id}/attachment` (multipart or raw body) and
      `GET /api/v1/samples/{id}/attachment` to upload/download it.
- [ ] `openapi.yaml` entries for both.
- [ ] Draw the prototype's `Upload miljørapport (PDF)` button on
      `SampleCard`, wired through `useMutationQueue` like every other Miljø
      write; show a document icon + filename once one is attached, with a
      way to replace it.
- [ ] Decide a size cap and reject oversized uploads with a clear message
      (STANDARDS §5).

category=I/O estimate=1d
