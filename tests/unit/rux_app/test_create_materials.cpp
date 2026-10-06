// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// `rux create materials` must respect the schema v26 survey tombstones: a
// scan part the user deleted in Kortlægning keeps its instance guid in
// survey_dismissed_instances, and minting a fresh passport for that instance
// would bring the deleted part's material back.

#include <catch2/catch_test_macros.hpp>

#include <create/materials.hpp>
#include <exit_status.hpp>
#include <global-params.hpp>
#include <reusex/core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <pcl/point_types.h>

TEST_CASE("CreateMaterials_SkipsDismissedInstances", "[rux][materials]") {
  const reusex::test_support::TempPath tmp("create_materials_dismissed");
  {
    reusex::ProjectDB db(tmp.path, /*readOnly=*/false);
    reusex::CloudL labels;
    for (std::uint32_t id : {1u, 2u, 3u}) {
      pcl::Label l;
      l.label = id;
      labels.push_back(l);
    }
    db.save_point_cloud("instances", labels, "test", "{}");
    db.save_label_definitions(
        "instances", {{1, "SM3-1 (1p)"}, {2, "SM3-2 (1p)"}, {3, "SM3-3 (1p)"}});
    db.save_instances("instances", {{1, "guid-inst-1", 3, 1},
                                    {2, "guid-inst-2", 3, 1},
                                    {3, "guid-inst-3", 3, 1}});
    db.dismiss_instance("guid-inst-2");
  }

  RuxOptions global;
  global.project_db = tmp.path;
  SubcommandCreateMaterialsOptions opt;
  REQUIRE(run_subcommand_create_materials(opt, global) == RuxError::SUCCESS);

  reusex::ProjectDB db(tmp.path);
  CHECK(db.instance_material_guid("instances", 1).has_value());
  CHECK_FALSE(db.instance_material_guid("instances", 2).has_value());
  CHECK(db.instance_material_guid("instances", 3).has_value());

  SECTION("--clear does not resurrect it either") {
    opt.clear = true;
    REQUIRE(run_subcommand_create_materials(opt, global) == RuxError::SUCCESS);
    reusex::ProjectDB again(tmp.path);
    CHECK_FALSE(again.instance_material_guid("instances", 2).has_value());
  }
}
