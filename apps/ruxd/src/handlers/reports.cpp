// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// ruxd routes for Ressourcekortlægning PDF reports (#456).
//
// Generation is delegated to reusex::generate_ressourcekortlaegning_pdf()
// (libs/reusex/src/core/report_generator.cpp); this file only handles HTTP
// concerns: request parsing, DB storage, serialising responses.
//
// Three routes:
//   POST   /reports/ressourcekortlaegning  — generate + store + 201 metadata
//   GET    /reports/ressourcekortlaegning  — list stored versions
//   GET    /reports/ressourcekortlaegning/<int>  — fetch PDF bytes
//
// POST is writer-locked so parallel requests never race on the DB write.

#include <handlers.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/logging.hpp>
#include <reusex/core/report_generator.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace ruxd {

namespace {

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

} // namespace

void register_report_routes(App &app, EndpointRegistry &reg,
                            reusex::ProjectDB &db) {
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
          const auto pdf = reusex::generate_ressourcekortlaegning_pdf(db);
          const auto rec = db.add_report_pdf(pdf, "Ressourcekortlægning");

          reusex::core::info("POST /reports/ressourcekortlaegning: stored "
                             "version {} ({} bytes)",
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
                nlohmann::json arr = nlohmann::json::array();
                for (const auto &r : db.list_report_pdfs())
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
