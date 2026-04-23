// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "MaterialPassport.hpp"

#include <nlohmann/json.hpp>

#include <string_view>
#include <vector>

namespace reusex::core::json_import {

/**
 * @brief Import a single MaterialPassport from a JSON object.
 *
 * Parses a JSON object matching the Danish "Materialepas for genbrugte
 * byggevarer" interchange format and populates a MaterialPassport struct.
 *
 * @param j JSON object containing sections, log, and metadata
 * @return Populated MaterialPassport
 * @throws std::runtime_error if required keys are missing
 */
[[nodiscard]] MaterialPassport from_json(const nlohmann::json &j);

/**
 * @brief Import multiple MaterialPassports from a JSON array.
 *
 * Each element in the array must be a full passport JSON object.
 *
 * @param j JSON array of passport objects
 * @return Vector of MaterialPassport instances
 * @throws std::runtime_error if j is not an array
 */
[[nodiscard]] std::vector<MaterialPassport>
from_json_array(const nlohmann::json &j);

/**
 * @brief Import MaterialPassport(s) from a JSON string.
 *
 * Auto-detects whether the string contains a single passport object
 * or an array of passports.
 *
 * @param json_str JSON string to parse
 * @return Vector of MaterialPassport instances (single-element for objects)
 * @throws nlohmann::json::parse_error if the string is not valid JSON
 * @throws std::runtime_error if required keys are missing
 */
[[nodiscard]] std::vector<MaterialPassport>
from_json_string(std::string_view json_str);

} // namespace reusex::core::json_import
