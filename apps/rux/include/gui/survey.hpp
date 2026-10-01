// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Survey (Ressourcekortlægning) read and write endpoints: GET /survey, GET
// /survey/summary, GET /survey/fractions, GET/POST /samples, plus the Task 7
// mutators (sync, create/patch type, patch part, create/patch/delete sample,
// set sample links). Framework-free, like gui/api.hpp — these take a
// `ProjectDB &` (or `const ProjectDB &` for reads) and return JSON built from
// the pure rules in reusex/core/survey.hpp and the ProjectDB-backed
// derivations in reusex/core/survey_service.hpp, signalling failure by
// throwing HttpError (gui/api.hpp).

#include "gui/api.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/survey.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <string>
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

// --- writes (#265 Phase 2) --------------------------------------------------
//
// Each handler parses and validates the whole body before writing anything,
// so a refused request leaves the project unchanged.

/// `POST /survey/sync`: fill survey_types/survey_parts from the instances
/// cloud (idempotent; see core::sync_survey). Body: `{}` or
/// `{ "instances_cloud"?, "semantic_cloud"?, "rooms_cloud"? }`.
/// @throws HttpError(400) when the body is not a JSON object.
/// @throws HttpError(422) when the instances cloud does not exist.
nlohmann::json sync_survey_json(reusex::ProjectDB &db, const std::string &body);

/// `POST /survey/types`: create a new survey type. Body: `{ "name"
/// (required, non-empty), "eak_code"?, "bim7aa_code"?, "unit"?, "treatment"?
/// }`.
/// @throws HttpError(400) when `name` is missing/empty or `treatment` names
///         an unknown value.
nlohmann::json create_survey_type_json(reusex::ProjectDB &db,
                                       const std::string &body);

/// `PATCH /survey/types/<int>`: sparse-update a survey type. Every refusal is
/// checked before the first write, so a rejected combination changes
/// nothing. `quantity` redistributes across the type's existing parts
/// (core::set_type_quantity); `review_status` goes through
/// core::set_review_status, which gates `approved` on a pending sample.
/// @throws HttpError(400) on malformed JSON, an unknown field type/enum
///         value, or a negative `quantity`.
/// @throws HttpError(404) when @p id is not a survey type.
/// @throws HttpError(422) when approval is blocked by a pending sample, or
///         `quantity` is given for a type with no parts.
nlohmann::json patch_survey_type_json(reusex::ProjectDB &db, int64_t id,
                                      const std::string &body);

/// `PATCH /survey/parts/<string>`: sparse-update or re-file a survey part.
/// Body: any of `type_id` (int), `quantity` (number >= 0), `starred` (bool),
/// `note`, `room_name` (string).
/// @throws HttpError(400) on malformed JSON, a wrong field type, or a
///         negative `quantity`.
/// @throws HttpError(404) when @p code or a given `type_id` is unknown.
nlohmann::json patch_survey_part_json(reusex::ProjectDB &db,
                                      const std::string &code,
                                      const std::string &body);

/// `POST /samples`: register a new environmental sample. Body: `{ "title"
/// (required), "what"?, "type_ids"? }`.
/// @throws HttpError(400) when `title` is missing/empty.
/// @throws HttpError(404) when a given `type_ids` entry is unknown.
nlohmann::json create_sample_json(reusex::ProjectDB &db,
                                  const std::string &body);

/// `PATCH /samples/<int>`: advance a sample's stage or record its result
/// (core::update_sample_checked). Body: any of `title, what` (string),
/// `stage`, `result` (string|null).
/// @throws HttpError(400) on malformed JSON, an unknown `stage`/`result`
///         value, or a `result` that is neither a known string nor null.
/// @throws HttpError(404) when @p id is not a sample.
/// @throws HttpError(422) when `result` is set before the stage is `svar`.
nlohmann::json patch_sample_json(reusex::ProjectDB &db, int64_t id,
                                 const std::string &body);

/// `DELETE /samples/<int>`.
/// @throws HttpError(404) when @p id is not a sample.
void delete_sample(reusex::ProjectDB &db, int64_t id);

/// `PUT /samples/<int>/links`: replace the set of survey types a sample
/// covers. Body: `{ "type_ids": int[] }`.
/// @throws HttpError(400) when `type_ids` is missing or not an array of
///         integers.
/// @throws HttpError(404) when @p id or a given type id is unknown.
nlohmann::json set_sample_links_json(reusex::ProjectDB &db, int64_t id,
                                     const std::string &body);

} // namespace rux::gui
