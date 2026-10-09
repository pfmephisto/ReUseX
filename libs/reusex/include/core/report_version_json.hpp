// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Wire form of one stored report PDF version — the `ReportPdfVersion` schema
/// in `ruxd`'s API contract (`docs/gui/openapi.yaml`, in the `ruxd` repo).
/// The servers that answer it (`ruxd`) serialise through this one function so
/// their responses cannot drift.

#include "reusex/core/ProjectDB.hpp"

#include <nlohmann/json.hpp>

namespace reusex::core {

/// {id, created_at, label, size_bytes, version, blocking_types}; blocking_types
/// is null for a version generated before schema v23.
nlohmann::json report_version_json(const ProjectDB::ReportPdfRecord &r);

} // namespace reusex::core
