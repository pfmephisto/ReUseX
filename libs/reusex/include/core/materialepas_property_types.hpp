// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <optional>
#include <string_view>

namespace ReUseX::core::traits {

/**
 * @brief Property type enumeration for MaterialPassport serialization
 *
 * Maps C++ types to database storage types for the MaterialPassport trait system.
 */
enum class PropertyType {
  /// std::string
  String,

  /// std::optional<int>
  Integer,

  /// std::optional<double>
  Double,

  /// std::optional<bool>
  Boolean,

  /// TriState enum (yes/no/unknown)
  TriState,

  /// std::vector<std::string>
  StringArray,

  /// Enum values (Material, etc.)
  EnumValue,

  /// std::vector<EnumType>
  EnumArray,

  /// std::vector<StructType> (nested objects)
  ObjectArray
};

/**
 * @brief Convert PropertyType to database data_type string
 *
 * @param type PropertyType enum value
 * @return String representation for database storage
 */
constexpr std::string_view to_data_type_string(PropertyType type) noexcept {
  switch (type) {
  case PropertyType::String:
    return "string";
  case PropertyType::Integer:
    return "integer";
  case PropertyType::Double:
    return "double";
  case PropertyType::Boolean:
    return "boolean";
  case PropertyType::TriState:
    return "tristate";
  case PropertyType::StringArray:
    return "string_array";
  case PropertyType::EnumValue:
    return "enum";
  case PropertyType::EnumArray:
    return "enum_array";
  case PropertyType::ObjectArray:
    return "object_array";
  }
  return "unknown";
}

/**
 * @brief Convert database data_type string to PropertyType
 *
 * @param str Database data_type string
 * @return PropertyType if valid, nullopt otherwise
 */
inline std::optional<PropertyType>
property_type_from_string(std::string_view str) noexcept {
  if (str == "string")
    return PropertyType::String;
  if (str == "integer")
    return PropertyType::Integer;
  if (str == "double")
    return PropertyType::Double;
  if (str == "boolean")
    return PropertyType::Boolean;
  if (str == "tristate")
    return PropertyType::TriState;
  if (str == "string_array")
    return PropertyType::StringArray;
  if (str == "enum")
    return PropertyType::EnumValue;
  if (str == "enum_array")
    return PropertyType::EnumArray;
  if (str == "object_array")
    return PropertyType::ObjectArray;

  return std::nullopt;
}

} // namespace ReUseX::core::traits
