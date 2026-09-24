// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Server-side PDF generation for Ressourcekortlægning (#456).
//
// Three routes:
//   POST   /reports/ressourcekortlaegning  — generate, store, return metadata
//   GET    /reports/ressourcekortlaegning  — list stored versions
//   GET    /reports/ressourcekortlaegning/<int:id>  — fetch PDF bytes
//
// Generation assembles project metadata, column definitions, passport
// properties and thumbnails into data.json + thumbnails/ in a temporary
// directory, writes the embedded Typst template, then invokes
//   typst compile report.typ out.pdf --root <tmpdir>
// as a subprocess. The resulting bytes are stored in report_pdfs (schema v20)
// and returned as application/pdf.
//
// POST is writer-locked via a shared mutex so parallel requests never race on
// the temp directory or the ProjectDB write.

#include <handlers.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sys/wait.h>

namespace ruxd {

namespace {

// ── helpers ──────────────────────────────────────────────────────────────────

std::string now_iso8601_reports() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return std::string(buf);
}

crow::response error_json_r(crow::status code, const std::string &message) {
  return json_response(code, {{"error", message}});
}

void finish_r(crow::response &res, crow::response &&built) {
  res = std::move(built);
  res.end();
}

nlohmann::json record_json(const reusex::ProjectDB::ReportPdfRecord &r) {
  return nlohmann::json{{"id", r.id},
                        {"created_at", r.created_at},
                        {"label", r.label},
                        {"size_bytes", r.size_bytes}};
}

// ── Typst template (embedded) ─────────────────────────────────────────────

// The canonical source lives at apps/rux/resources/report.typ; this string
// is a copy embedded here so ruxd finds it at runtime without needing to know
// where the source tree is installed.
constexpr const char *kReportTypTemplate = R"typst(
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

// ── MIME → extension helper ───────────────────────────────────────────────

std::string ext_for_mime(const std::string &mime) {
  if (mime == "image/png")
    return ".png";
  if (mime == "image/webp")
    return ".webp";
  if (mime == "image/gif")
    return ".gif";
  return ".jpg"; // default: JPEG
}

// ── report data assembly ──────────────────────────────────────────────────

struct TempDir {
  std::filesystem::path path;

  explicit TempDir() {
    const auto tmp = std::filesystem::temp_directory_path();
    const auto pid = static_cast<long>(::getpid());
    const auto now =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path = tmp /
           ("ruxd_report_" + std::to_string(pid) + "_" + std::to_string(now));
    std::filesystem::create_directories(path);
  }

  ~TempDir() noexcept {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }

  TempDir(const TempDir &) = delete;
  TempDir &operator=(const TempDir &) = delete;
};

// Assemble report_data JSON and write thumbnail files into tmpdir/thumbnails/.
// Returns the populated JSON object.
nlohmann::json assemble_data(reusex::ProjectDB &db,
                             const std::filesystem::path &tmpdir) {
  const auto summary = db.project_summary();
  const auto defs = db.list_property_definitions();

  // Project name: use first project's name if available, else path stem.
  std::string project_name;
  if (!summary.projects.empty() && !summary.projects.front().name.empty())
    project_name = summary.projects.front().name;
  else
    project_name = summary.path.stem().string();

  nlohmann::json data;
  data["project_name"] = project_name;
  data["generated_at"] = now_iso8601_reports();

  // Column definitions (user-defined, sorted by sort_order).
  nlohmann::json cols = nlohmann::json::array();
  for (const auto &d : defs)
    cols.push_back({{"id", d.id}, {"name", d.name}, {"type", d.type}});
  data["columns"] = std::move(cols);

  // Thumbnail directory.
  const auto thumb_dir = tmpdir / "thumbnails";
  std::filesystem::create_directories(thumb_dir);

  // One JSON object per material.
  nlohmann::json mats = nlohmann::json::array();
  for (const auto &info : summary.materials) {
    const auto &guid = info.guid;

    auto thumb_opt = db.material_thumbnail(guid);
    bool has_thumb = thumb_opt.has_value();
    std::string thumb_path;

    if (has_thumb) {
      const auto &[blob, mime] = *thumb_opt;
      const std::string fname = "thumbnails/" + guid + ext_for_mime(mime);
      const auto full = tmpdir / fname;
      std::ofstream f(full, std::ios::binary);
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

// Invoke `typst compile` as a subprocess and return the PDF bytes.
// Throws std::runtime_error on failure (typst not found, compile error, etc.).
std::vector<std::uint8_t>
run_typst_compile(const std::filesystem::path &tmpdir) {
  const auto typ_path = tmpdir / "report.typ";
  const auto out_path = tmpdir / "out.pdf";
  const auto err_path = tmpdir / "typst.stderr";

  // Write template.
  {
    std::ofstream f(typ_path);
    f << kReportTypTemplate;
    if (!f.good())
      throw std::runtime_error("failed to write Typst template to " +
                               typ_path.string());
  }

  // Build command. All paths are under /tmp/ruxd_report_…, which cannot
  // contain shell metacharacters; single-quoting is an extra guard.
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

  // Read output PDF.
  std::ifstream pf(out_path, std::ios::binary);
  if (!pf)
    throw std::runtime_error("typst produced no output at " +
                             out_path.string());
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(pf),
                                   std::istreambuf_iterator<char>{});
}

} // namespace

// ── route registration ────────────────────────────────────────────────────

void register_report_routes(App &app, EndpointRegistry &reg,
                            reusex::ProjectDB &db) {
  // Shared mutex: serialize POST generation so two in-flight requests never
  // race on the same ProjectDB read+write sequence or temp directory.
  auto gen_mutex = std::make_shared<std::mutex>();

  // POST /reports/ressourcekortlaegning — generate, store, return metadata.
  add_route(
      app, reg,
      {"POST",
       "/reports/ressourcekortlaegning",
       "Generate a Ressourcekortlægning PDF and store it as a new version",
       true,
       {{201, "Version created"}, {500, "Generation failed"}}},
      [&db, gen_mutex](const crow::request &) -> crow::response {
        std::unique_lock lock(*gen_mutex);
        try {
          TempDir tmpdir;

          // 1. Assemble data and write data.json.
          const auto data = assemble_data(db, tmpdir.path);
          {
            std::ofstream f(tmpdir.path / "data.json");
            f << data.dump();
            if (!f.good())
              throw std::runtime_error("failed to write data.json");
          }

          // 2. Invoke Typst.
          const auto pdf = run_typst_compile(tmpdir.path);

          // 3. Store in DB.
          const auto rec = db.add_report_pdf(pdf, "Ressourcekortlægning");

          reusex::core::info(
              "POST /reports/ressourcekortlaegning: stored version {} "
              "({} bytes)",
              rec.id, rec.size_bytes);

          return json_response(crow::status::CREATED, record_json(rec));
        } catch (const std::exception &e) {
          reusex::core::error("POST /reports/ressourcekortlaegning failed: {}",
                              e.what());
          return error_json_r(crow::status::INTERNAL_SERVER_ERROR, e.what());
        }
      });

  // GET /reports/ressourcekortlaegning — list stored versions.
  add_route(app, reg,
            {"GET",
             "/reports/ressourcekortlaegning",
             "List stored Ressourcekortlægning PDF versions",
             true,
             {{200, "Version list"}}},
            [&db](const crow::request &) -> crow::response {
              try {
                const auto recs = db.list_report_pdfs();
                nlohmann::json arr = nlohmann::json::array();
                for (const auto &r : recs)
                  arr.push_back(record_json(r));
                return json_response(crow::status::OK,
                                     nlohmann::json{{"versions", arr}});
              } catch (const std::exception &e) {
                reusex::core::error(
                    "GET /reports/ressourcekortlaegning failed: {}", e.what());
                return error_json_r(crow::status::INTERNAL_SERVER_ERROR,
                                    e.what());
              }
            });

  // GET /reports/ressourcekortlaegning/<int> — fetch one PDF.
  add_route_dynamic(
      app, reg,
      {"GET",
       "/reports/ressourcekortlaegning/<int>",
       "Fetch a stored Ressourcekortlægning PDF by version id",
       true,
       {{200, "PDF bytes"}, {404, "Version not found"}}},
      [&db](const crow::request &, crow::response &res, int id) {
        try {
          const auto pdf = db.report_pdf(static_cast<int64_t>(id));
          if (!pdf.has_value()) {
            finish_r(res, error_json_r(crow::status::NOT_FOUND,
                                       "report version not found: " +
                                           std::to_string(id)));
            return;
          }

          crow::response out(crow::status::OK);
          out.set_header("Content-Type", "application/pdf");
          out.set_header("Content-Disposition",
                         "attachment; filename=\"ressourcekortlaegning_" +
                             std::to_string(id) + ".pdf\"");
          out.body.assign(reinterpret_cast<const char *>(pdf->data()),
                          pdf->size());
          finish_r(res, std::move(out));
        } catch (const std::exception &e) {
          reusex::core::error(
              "GET /reports/ressourcekortlaegning/{} failed: {}", id, e.what());
          finish_r(res,
                   error_json_r(crow::status::INTERNAL_SERVER_ERROR, e.what()));
        }
      });
}

} // namespace ruxd
