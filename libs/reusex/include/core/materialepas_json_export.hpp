// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "core/MaterialPassport.hpp"
#include "core/materialepas_traits.hpp"

#include <nlohmann/json.hpp>

#include <span>
#include <string>
#include <vector>

namespace ReUseX::core::json_export {

/**
 * @brief Describes a section in the JSON export template.
 *
 * Maps a MaterialPassport section struct to its JSON representation,
 * including English/Danish names and the property descriptors.
 */
struct SectionDescriptor {
  /// English section name (e.g., "Owner")
  const char *name_en;

  /// Danish section name (e.g., "Ejer")
  const char *name_da;

  /// Pointer to property descriptor array for this section
  const traits::PropertyDescriptor *properties;

  /// Number of properties in the array
  size_t property_count;

  /// Offset of the section struct within MaterialPassport
  size_t passport_offset;
};

/**
 * @brief Get the section descriptors in JSON template order.
 *
 * Returns 10 sections in the order defined by the Danish standard:
 * Owner, ConstructionItemDescription, ProductInformation, Certifications,
 * Dimensions, History, Condition, Pollution, EnvironmentalPotential,
 * FireProperties
 *
 * @return Span of SectionDescriptor instances
 */
std::span<const SectionDescriptor> section_descriptors();

/**
 * @brief Export a single MaterialPassport to JSON.
 *
 * Produces a JSON object matching the Danish "Materialepas for genbrugte
 * byggevarer" interchange format with sections, log, and metadata.
 *
 * @param passport The passport to export
 * @return JSON object
 */
[[nodiscard]] nlohmann::json to_json(const MaterialPassport &passport);

/**
 * @brief Export multiple MaterialPassports to a JSON array.
 *
 * Each element in the array is a full passport JSON object.
 *
 * @param passports Vector of passports to export
 * @return JSON array
 */
[[nodiscard]] nlohmann::json
to_json(const std::vector<MaterialPassport> &passports);

/**
 * @brief Export a single passport as a formatted JSON string.
 *
 * @param passport The passport to export
 * @param indent Number of spaces for indentation (default: 4)
 * @return Formatted JSON string
 */
[[nodiscard]] std::string to_json_string(const MaterialPassport &passport,
                                         int indent = 4);

/**
 * @brief Export multiple passports as a formatted JSON string.
 *
 * @param passports Vector of passports to export
 * @param indent Number of spaces for indentation (default: 4)
 * @return Formatted JSON string
 */
[[nodiscard]] std::string
to_json_string(const std::vector<MaterialPassport> &passports,
               int indent = 4);

} // namespace ReUseX::core::json_export
