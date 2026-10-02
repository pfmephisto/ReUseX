// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/report_generator.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/resource_export.hpp>
#include <reusex/core/survey_service.hpp>

#include <fmt/format.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace reusex {

namespace {

// ── helpers ──────────────────────────────────────────────────────────────────

std::string now_iso8601_rg() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return std::string(buf);
}

/// 1234.5 -> "1234,5"; whole numbers lose the decimal ("640"). The report is
/// Danish, so the decimal mark is a comma.
std::string da_number(double v) {
  std::string s = fmt::format("{:.1f}", v);
  if (s.size() > 2 && s.compare(s.size() - 2, 2, ".0") == 0)
    s.resize(s.size() - 2);
  std::replace(s.begin(), s.end(), '.', ',');
  return s;
}

std::string ext_for_mime(const std::string &mime) {
  if (mime == "image/png")
    return ".png";
  if (mime == "image/webp")
    return ".webp";
  if (mime == "image/gif")
    return ".gif";
  return ".jpg";
}

// ── RAII temp directory ───────────────────────────────────────────────────

struct TempDir {
  std::filesystem::path path;

  explicit TempDir() {
    const auto tmp = std::filesystem::temp_directory_path();
    const auto pid = static_cast<long>(::getpid());
    const auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
    path = tmp /
           ("reusex_report_" + std::to_string(pid) + "_" + std::to_string(ts));
    std::filesystem::create_directories(path);
  }

  ~TempDir() noexcept {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;
};

// ── Typst template (embedded) ─────────────────────────────────────────────

// Canonical source: apps/rux/resources/report.typ. The two copies must be
// identical: ReportTemplate_CopiesInSync
// (tests/unit/core/test_report_survey.cpp) fails when they are not.
constexpr const char *kTypstTemplate = R"typst(
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
//     }],
//     "survey": {
//       "rows": [{"name", "bim7aa", "eak", "quantity", "mass", "treatment", "environment"}],
//       "circularity": [{"label": "Genanvendelse", "tonnes": "196,8 t"}],
//       "blocking": 7
//     }
//     "resources": null | {
//       "name": "template name",
//       "tables": [{"headers": ["Betegnelse", ...], "rows": [["...", ...]]}]
//     }
//   }
//
// The same text is embedded in libs/reusex/src/core/report_generator.cpp
// (kTypstTemplate); the ReportTemplate_CopiesInSync test fails when they differ.

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

// ── Kortlægning: approved survey types ───────────────────────────────────────

#let survey = data.survey

#text(size: 13pt, weight: "bold")[Kortlægning]
#v(0.2cm)
#if survey.blocking > 0 [
  #text(size: 9pt, style: "italic")[Udkast — #survey.blocking type(r) afventer gennemsyn eller prøvesvar, eller mangler tons, og indgår ikke i mængderne.]
  #v(0.2cm)
]
#if survey.rows.len() == 0 [
  _Ingen godkendte typer endnu._
] else {
  table(
    columns: (1.9fr, 1.5fr, 0.9fr, 0.9fr, 0.7fr, 1.35fr, 1.1fr),
    stroke: 0.3pt + luma(190),
    inset: (x: 5pt, y: 5pt),
    fill: (col, row) => if row == 0 { luma(215) } else { white },
    table.header([*Type*], [*BIM7AA*], [*EAK*], [*Mængde*], [*Tons*], [*Behandling*], [*Miljø*]),
    ..survey.rows.map(r => (r.name, r.bim7aa, r.eak, r.quantity, r.mass, r.treatment, r.environment)).flatten(),
  )
}
#if survey.circularity.len() > 0 [
  #v(0.2cm)
  #text(size: 9pt)[Cirkularitet (godkendte typer): #survey.circularity.map(c => c.label + " " + c.tonnes).join(" · ")]
]
#v(0.6cm)

// ── Ressourcetabel: resources through a chosen template ──────────────────────

#let res = data.at("resources", default: none)
#if res != none [
  #text(size: 13pt, weight: "bold")[Ressourcetabel — #res.name]
  #v(0.2cm)
  #if res.tables.len() == 0 [
    _Ingen ressourcer i projektet._
  ] else {
    for t in res.tables {
      table(
        columns: t.headers.len(),
        stroke: 0.3pt + luma(190),
        inset: (x: 5pt, y: 5pt),
        fill: (col, row) => if row == 0 { luma(215) } else { white },
        table.header(..t.headers.map(h => [*#h*])),
        ..t.rows.flatten(),
      )
      v(0.3cm)
    }
  }
  #v(0.6cm)
]

#text(size: 13pt, weight: "bold")[Materialepas]
#v(0.2cm)

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
)typst";

nlohmann::json resources_json(const core::ResourceReportSection &s) {
  nlohmann::json tables = nlohmann::json::array();
  for (const auto &t : s.tables)
    tables.push_back({{"headers", t.headers}, {"rows", t.rows}});
  return {{"name", s.template_name}, {"tables", std::move(tables)}};
}

// ── data assembly ─────────────────────────────────────────────────────────

nlohmann::json assemble_report_data(ProjectDB &db,
                                    const std::filesystem::path &tmpdir) {
  const auto summary = db.project_summary();
  const auto defs = db.list_property_definitions();

  std::string project_name;
  if (!summary.projects.empty() && !summary.projects.front().name.empty())
    project_name = summary.projects.front().name;
  else
    project_name = summary.path.stem().string();

  nlohmann::json data;
  data["project_name"] = project_name;
  data["generated_at"] = now_iso8601_rg();

  nlohmann::json cols = nlohmann::json::array();
  for (const auto &d : defs)
    cols.push_back({{"id", d.id}, {"name", d.name}, {"type", d.type}});
  data["columns"] = std::move(cols);

  const auto thumb_dir = tmpdir / "thumbnails";
  std::filesystem::create_directories(thumb_dir);

  nlohmann::json mats = nlohmann::json::array();
  for (const auto &info : summary.materials) {
    const auto &guid = info.guid;

    auto thumb_opt = db.material_thumbnail(guid);
    bool has_thumb = thumb_opt.has_value();
    std::string thumb_path;

    if (has_thumb) {
      const auto &[blob, mime] = *thumb_opt;
      const std::string fname = "thumbnails/" + guid + ext_for_mime(mime);
      std::ofstream f(tmpdir / fname, std::ios::binary);
      f.write(reinterpret_cast<const char *>(blob.data()),
              static_cast<std::streamsize>(blob.size()));
      if (f.good())
        thumb_path = fname;
      else
        has_thumb = false;
    }

    const auto props_map = db.passport_stored_properties(guid);
    nlohmann::json props = nlohmann::json::object();
    for (const auto &[k, v] : props_map)
      props[k] = v;

    mats.push_back({{"guid", guid},
                    {"has_thumbnail", has_thumb},
                    {"thumbnail_path", thumb_path},
                    {"properties", std::move(props)}});
  }
  data["materials"] = std::move(mats);

  // Survey (Kortlægning): reportable types only — "kun godkendte mængder
  // indgår i rapporten" (GUI Phase 5, R8). report_survey_rows already
  // withholds types awaiting a sample or missing tonnes, so the circularity
  // line below sums exactly the rows the table shows.
  nlohmann::json rows = nlohmann::json::array();
  std::vector<core::TypeTotals> approved;
  for (const auto &r : core::report_survey_rows(db)) {
    rows.push_back(
        {{"name", r.name},
         {"bim7aa", r.bim7aa_code},
         {"eak", r.eak_code},
         {"quantity", da_number(r.quantity) + " " + r.unit},
         {"mass", r.mass_t ? da_number(*r.mass_t) + " t" : std::string("—")},
         {"treatment", std::string(core::treatment_label_da(r.treatment))},
         {"environment",
          std::string(core::environment_label_da(r.environment))}});
    approved.push_back({r.treatment, core::ReviewStatus::approved, r.mass_t,
                        r.eak_code, r.environment});
  }
  const auto breakdown = core::circularity_breakdown(approved);
  nlohmann::json circ = nlohmann::json::array();
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i)
    if (breakdown[i] > 0.0)
      circ.push_back({{"label", std::string(core::treatment_label_da(
                                    static_cast<core::Treatment>(i)))},
                      {"tonnes", da_number(breakdown[i]) + " t"}});
  data["survey"] = {{"rows", std::move(rows)},
                    {"circularity", std::move(circ)},
                    {"blocking", report_blocking_types(db)}};
  return data;
}

// ── Typst invocation ──────────────────────────────────────────────────────

std::vector<std::uint8_t> run_typst(const std::filesystem::path &tmpdir) {
  const auto typ_path = tmpdir / "report.typ";
  const auto out_path = tmpdir / "out.pdf";
  const auto err_path = tmpdir / "typst.stderr";

  {
    std::ofstream f(typ_path);
    f << kTypstTemplate;
    if (!f.good())
      throw std::runtime_error(
          "report_generator: failed to write template to " + typ_path.string());
  }

  const std::string cmd = "typst compile '" + typ_path.string() + "' '" +
                          out_path.string() + "' --root '" + tmpdir.string() +
                          "' 2>'" + err_path.string() + "'";

  const int status = std::system(cmd.c_str()); // NOLINT(cert-env33-c)

  if (status != 0) {
    std::string err_msg;
    std::ifstream ef(err_path);
    if (ef)
      err_msg.assign(std::istreambuf_iterator<char>(ef),
                     std::istreambuf_iterator<char>{});
    throw std::runtime_error("typst compile failed (exit " +
                             std::to_string(WEXITSTATUS(status)) + "):\n" +
                             err_msg);
  }

  std::ifstream pf(out_path, std::ios::binary);
  if (!pf)
    throw std::runtime_error("report_generator: typst produced no output at " +
                             out_path.string());
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(pf),
                                   std::istreambuf_iterator<char>{});
}

} // namespace

// ── public API ────────────────────────────────────────────────────────────

int report_blocking_types(const ProjectDB &db) {
  return static_cast<int>(
      core::fractions_by_eak(core::type_totals(db)).blocking_types);
}

std::vector<std::uint8_t>
generate_ressourcekortlaegning_pdf(ProjectDB &db,
                                   std::optional<std::int64_t> template_id) {
  // Resolve the template first: an unknown id fails fast, typst or not.
  std::optional<core::ResourceReportSection> resources;
  if (template_id)
    resources = core::resource_report_section(db, *template_id);

  TempDir tmpdir;

  auto data = assemble_report_data(db, tmpdir.path);
  data["resources"] =
      resources ? resources_json(*resources) : nlohmann::json(nullptr);
  {
    std::ofstream f(tmpdir.path / "data.json");
    f << data.dump();
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write data.json");
  }

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns, {} resource table(s))",
      data.at("materials").size(), data.at("columns").size(),
      resources ? resources->tables.size() : 0);

  const auto pdf = run_typst(tmpdir.path);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}

} // namespace reusex
