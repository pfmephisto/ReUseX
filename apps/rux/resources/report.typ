// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Ressourcekortlægning (Material Resource Mapping) report template.
//
// Invoked by ruxd via:
//   typst compile report.typ out.pdf --root <tmpdir>
//
// data.json must be present in <tmpdir> with the structure:
//   {
//     "project_name": "...",
//     "generated_at": "...",
//     "columns": [{"id": "...", "name": "...", "type": "..."}],
//     "materials": [{
//       "guid": "...",
//       "has_thumbnail": true,
//       "thumbnail_path": "thumbnails/guid.jpg",
//       "properties": {"col-name": "value"}
//     }]
//   }

#let data = json("data.json")
#let cols = data.columns
#let mats = data.materials

// ── Page layout ──────────────────────────────────────────────────────────────

#set page(
  paper: "a4",
  margin: (top: 2.5cm, bottom: 2.8cm, left: 2cm, right: 2cm),
  header: context [
    #set text(size: 8pt, fill: luma(140))
    #data.project_name
    #h(1fr)
    Ressourcekortlægning
  ],
  footer: context [
    #set text(size: 8pt, fill: luma(140))
    #data.generated_at
    #h(1fr)
    Side #counter(page).display() / #counter(page).final().at(0)
  ],
)

#set text(size: 10pt)
#set par(justify: false)

// ── Title block ───────────────────────────────────────────────────────────────

#v(0.5cm)
#align(center)[
  #text(size: 22pt, weight: "bold")[Ressourcekortlægning]
  #v(0.3cm)
  #text(size: 13pt)[#data.project_name]
  #v(0.2cm)
  #text(size: 9pt, fill: luma(120))[Genereret: #data.generated_at]
]
#v(0.6cm)
#line(length: 100%, stroke: 0.4pt + luma(180))
#v(0.5cm)

// ── Material table ────────────────────────────────────────────────────────────

#if mats.len() == 0 [
  #align(center)[_Ingen materialer i projektet._]
] else {
  // Build header cells: thumbnail + one cell per user-defined column.
  let header_cells = (
    table.cell(fill: luma(215), align: center)[*Billede*],
  ) + cols.map(col =>
    table.cell(fill: luma(215), align: center)[*#col.name*]
  )

  // Build body cells: one thumbnail + one property cell per column per row.
  let body_cells = ()
  for mat in mats {
    let thumb_cell = if mat.has_thumbnail {
      table.cell(align: center)[
        #image(mat.thumbnail_path, width: 2.8cm, height: 2.2cm, fit: "contain")
      ]
    } else {
      table.cell(fill: luma(245), align: center)[—]
    }
    body_cells = body_cells + (thumb_cell,)
    for col in cols {
      let val = mat.properties.at(col.name, default: "")
      body_cells = body_cells + (table.cell[#val],)
    }
  }

  // Column widths: fixed thumbnail + 1fr per user column.
  let col_widths = (2.9cm,) + cols.map(_ => 1fr)

  table(
    columns: col_widths,
    stroke: 0.3pt + luma(190),
    inset: (x: 5pt, y: 6pt),
    fill: (col, row) => {
      if row == 0 { luma(215) }
      else if calc.odd(row) { luma(250) }
      else { white }
    },
    ..header_cells,
    ..body_cells,
  )
}
