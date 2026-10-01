// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Survey (Ressourcekortlægning) read endpoints: GET /survey, GET
// /survey/summary, GET /survey/fractions, GET /samples. Framework-free, like
// gui/api.hpp — these take a `const ProjectDB &` and return JSON built from
// the pure rules in reusex/core/survey.hpp and the ProjectDB-backed
// derivations in reusex/core/survey_service.hpp.

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/survey.hpp>

#include <nlohmann/json_fwd.hpp>

#include <cstdint>
#include <vector>

namespace rux::gui {

/// One SurveyPart (RX-### bygningsdel) as the wire shape documents it.
nlohmann::json survey_part_json(const reusex::ProjectDB::SurveyPartRecord &p);

/// One SurveyType (Kortlægning group row) with its parts, derived miljøstatus
/// and linked sample ids.
///
/// @param t          The type record.
/// @param parts      This type's parts, in the order they should be listed.
/// @param env        The type's derived miljøstatus (environment_statuses()).
/// @param sample_ids Ids of samples linked to this type, ascending.
nlohmann::json
survey_type_json(const reusex::ProjectDB::SurveyTypeRecord &t,
                 const std::vector<reusex::ProjectDB::SurveyPartRecord> &parts,
                 reusex::core::EnvironmentStatus env,
                 const std::vector<int64_t> &sample_ids);

/// One environmental sample.
nlohmann::json sample_json(const reusex::ProjectDB::SampleRecord &s);

/// `GET /survey`: every survey type (rejected included) with its parts, plus
/// the queue/approved/rejected/all counts.
nlohmann::json survey_json(const reusex::ProjectDB &db);

/// `GET /survey/summary`: counts, circularity (tonnes per affaldshierarki
/// step), reuse share, pending-sample count and the two optional coverage
/// signals (unlabeled points, rooms without a part) that depend on clouds the
/// project may not have yet.
nlohmann::json survey_summary_json(const reusex::ProjectDB &db);

/// `GET /survey/fractions`: approved tonnes per EAK code, with the count of
/// types still blocking a waste report.
nlohmann::json survey_fractions_json(const reusex::ProjectDB &db);

/// `GET /samples`: every environmental sample with its linked survey types.
nlohmann::json samples_json(const reusex::ProjectDB &db);

} // namespace rux::gui
