// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/core/report_generator.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <nlohmann/json.hpp>

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

// Canonical source: apps/rux/resources/report.typ — keep in sync on edits.
constexpr const char *kTypstTemplate = R"typst(
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#let data = json("data.json")
#let cols = data.columns
#let mats = data.materials

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

#if mats.len() == 0 [
  #align(center)[_Ingen materialer i projektet._]
] else {
  let header_cells = (
    table.cell(fill: luma(215), align: center)[*Billede*],
  ) + cols.map(col =>
    table.cell(fill: luma(215), align: center)[*#col.name*]
  )

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

std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &db) {
  TempDir tmpdir;

  const auto data = assemble_report_data(db, tmpdir.path);
  {
    std::ofstream f(tmpdir.path / "data.json");
    f << data.dump();
    if (!f.good())
      throw std::runtime_error("report_generator: failed to write data.json");
  }

  reusex::info(
      "generate_ressourcekortlaegning_pdf: invoking typst ({} materials, "
      "{} columns)",
      data.at("materials").size(), data.at("columns").size());

  const auto pdf = run_typst(tmpdir.path);

  reusex::info("generate_ressourcekortlaegning_pdf: {} bytes", pdf.size());
  return pdf;
}

} // namespace reusex
