// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Resources and templates endpoints (docs/superpowers/specs/
// 2026-10-02-resources-templates-ia-design.md §4.4, §5.5, §6.3). Thin: each
// parses, calls one reusex::core function, and maps its exceptions through
// map_library_errors (gui/api.hpp) — KeyValueError/invalid_argument 400,
// out_of_range 404, NameConflictError 409.

#include "api/api.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

namespace ruxd::api {

/// `GET /resources/keys`: the key catalogue as a JSON array.
nlohmann::json resource_keys_json(const reusex::ProjectDB &db);
/// `GET /resources[?template=<id>]`: `{resources:[…], template?:{id,
/// resolved_keys, missing}}`. @throws HttpError 400 (bad id), 404 (unknown).
nlohmann::json resources_json(const reusex::ProjectDB &db,
                              const Params &params);
/// `PATCH /resources/<code>[?template=<id>]`, body `{values:{<key>:
/// string|null}}` → `{resource, siblings}`.
nlohmann::json patch_resource_json(reusex::ProjectDB &db,
                                   const std::string &code,
                                   const Params &params,
                                   const std::string &body);
/// `POST /resources`, body `{type_id, name?}` → the new resource.
nlohmann::json create_resource_json(reusex::ProjectDB &db,
                                    const std::string &body);
/// `DELETE /resources/<code>`: any part. A scan-backed part is tombstoned so
/// a survey sync does not re-create it (core::delete_resource).
void delete_resource(reusex::ProjectDB &db, const std::string &code);
/// `GET /resources/export.csv?template=<id>` (template required → 400).
Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params);

/// `GET /templates`: `{templates:[Template…]}`, by id.
nlohmann::json templates_json(const reusex::ProjectDB &db);
/// `POST /templates`, body `{name, members?, csv?}` (201).
nlohmann::json create_template_json(reusex::ProjectDB &db,
                                    const std::string &body);
/// `PATCH /templates/<id>`, body any of `{name, members, csv}`.
nlohmann::json patch_template_json(reusex::ProjectDB &db, int64_t id,
                                   const std::string &body);
/// `DELETE /templates/<id>` (204).
void delete_template(reusex::ProjectDB &db, int64_t id);
/// `POST /templates/<id>/duplicate` (201): "<name> (kopi)", numbered.
nlohmann::json duplicate_template_json(reusex::ProjectDB &db, int64_t id);
/// `POST /templates/restore-seeds`: `{restored:[names], templates:[…]}`.
nlohmann::json restore_seed_templates_json(reusex::ProjectDB &db);

} // namespace ruxd::api
