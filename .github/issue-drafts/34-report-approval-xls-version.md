title: Rapport — approve a version with the MRK signature, and store the XLS inventory with it
labels: gui, enhancement, backend, frontend

## Problem
Rapport (`/rapport`) lists the stored Ressourcekortlægning PDFs, each
marked `Komplet` or `Udkast` from the blocking count recorded when it was
generated (schema v23). Two things the prototype shows are not modelled
(R7):

- **Approval and the MRK signature.** The prototype's version rows carry an
  approval state and the miljø- og ressourcekoordinator's (MRK) signature.
  Nothing in the project stores who approved a version or when, and there is
  no MRK on the case record either (see draft 24). `REPORT_FOOTNOTE` in
  `apps/rux/frontend/src/rapport/model.ts` is the prototype's footnote with
  the signature sentence removed for that reason.
- **The XLS inventory as a version.** The Inventarliste download is the
  live `/exports/csv` export, fetched fresh each time. A report version
  therefore has no matching inventory: the list a surveyor sends next week
  can differ from the one that stood behind v3.

## Proposed fix
- [ ] Schema: an approval on a report version (approved_by, approved_at,
      the MRK's name / role), immutable once set; a new generation is a new,
      unapproved version.
- [ ] Store the inventory export as a blob next to each PDF at generation
      time, and serve it per version (`/reports/{id}/inventory`).
- [ ] Rapport: an Approve action (gated: only a `Komplet` version), the
      signature line on the row and in the PDF, and the per-version
      inventory download in place of the live one.
- [ ] Restore the prototype's footnote sentence about the MRK signature once
      it is true.

category=I/O estimate=3d
