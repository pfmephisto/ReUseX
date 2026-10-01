// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/report_version_json.hpp"

namespace reusex::core {

nlohmann::json report_version_json(const ProjectDB::ReportPdfRecord &r) {
  return {{"id", r.id},
          {"created_at", r.created_at},
          {"label", r.label},
          {"size_bytes", r.size_bytes},
          {"version", r.version},
          {"blocking_types", r.blocking_types
                                 ? nlohmann::json(*r.blocking_types)
                                 : nlohmann::json(nullptr)}};
}

} // namespace reusex::core
