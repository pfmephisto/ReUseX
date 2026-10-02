// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Resources and templates endpoints (docs/superpowers/specs/
// 2026-10-02-resources-templates-ia-design.md §4.4, §5.5, §6.3). Thin: each
// parses, calls one reusex::core function, and maps its exceptions through
// map_library_errors (gui/api.hpp) — KeyValueError/invalid_argument 400,
// out_of_range 404, NameConflictError and ResourceConflictError 409.

#include "gui/api.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>

namespace rux::gui {

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
/// `DELETE /resources/<code>`: manual parts only (409 otherwise).
void delete_resource(reusex::ProjectDB &db, const std::string &code);
/// `GET /resources/export.csv?template=<id>` (template required → 400).
Blob resources_csv_blob(const reusex::ProjectDB &db, const Params &params);

} // namespace rux::gui
