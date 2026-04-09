// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "MaterialPassport.hpp"
#include "materialepas_traits.hpp"

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
 * @brief Export a single MaterialPassport to JSON with sensible defaults.
 *
 * Similar to to_json(), but populates missing optional fields with sensible
 * defaults to improve readability:
 * - optional<bool> fields → false (explicit "no")
 * - Empty vectors → [] (truly empty, no template entries)
 * - TriState → unchanged (unknown, yes, no)
 * - Numeric optionals → "" (ambiguous whether 0 means zero or unknown)
 *
 * This is useful for exporting partial passports in a user-friendly format.
 *
 * @param passport The passport to export
 * @return JSON object with defaults populated
 */
[[nodiscard]] nlohmann::json to_json_with_defaults(const MaterialPassport &passport);

/**
 * @brief Generate a blank MaterialPassport template as JSON.
 *
 * Creates a complete template with all 10 sections and all properties present,
 * with empty values for user to fill in. Useful for creating new material
 * passports from scratch.
 *
 * Template characteristics:
 * - All sections present with all properties
 * - All string fields: "" (empty for user to fill)
 * - All vectors: [] (empty arrays)
 * - All optionals: unset (will show as "")
 * - Metadata: Auto-generated GUID, current date, version "0.1.0"
 * - Transaction log: [] (empty)
 *
 * @return JSON template object
 */
[[nodiscard]] nlohmann::json generate_blank_template();

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
