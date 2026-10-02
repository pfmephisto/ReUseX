// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Survey/resource test fixtures shared by the resource tests: an instance
// cloud, survey types, parts (manual or instance-backed) and bare passports.
// Include as "../../support/survey_fixture.hpp" from tests/unit/<module>/.

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>

#include <pcl/point_types.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace reusex::test_support {

/// An "instances" cloud with instances 1..n (one point each, class 3, guid
/// "guid-inst-<i>").
inline void make_instance_cloud(ProjectDB &db, std::uint32_t n) {
  CloudL labels;
  std::vector<ProjectDB::InstanceRecord> rows;
  for (std::uint32_t i = 1; i <= n; ++i) {
    pcl::Label p;
    p.label = i;
    labels.push_back(p);
    rows.push_back({i, "guid-inst-" + std::to_string(i), 3, 1});
  }
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", rows);
}

inline int64_t make_type(ProjectDB &db, const std::string &name,
                         core::Treatment t = core::Treatment::genbrug) {
  ProjectDB::SurveyTypeRecord rec;
  rec.name = name;
  rec.treatment = t;
  rec.unit = "stk";
  return db.add_survey_type(rec).id;
}

/// A part, instance-backed when @p instance_id is given.
inline void make_part(ProjectDB &db, const std::string &code, int64_t type_id,
                      std::optional<std::uint32_t> instance_id = std::nullopt,
                      double quantity = 1.0) {
  ProjectDB::SurveyPartRecord p;
  p.code = code;
  p.type_id = type_id;
  if (instance_id) {
    p.cloud_name = "instances";
    p.instance_id = *instance_id;
  }
  p.quantity = quantity;
  db.add_survey_part(p);
}

inline void make_passport(ProjectDB &db, const std::string &guid) {
  core::MaterialPassport p;
  p.metadata.document_guid = guid;
  p.metadata.creation_date = "2025-01-01T00:00:00Z";
  p.metadata.version_number = "1.0.0";
  db.add_material_passport(p, "");
}

} // namespace reusex::test_support
