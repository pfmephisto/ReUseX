// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "materialepas_property_types.hpp"
#include "materialepas_traits.hpp"
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core::serialization {

/**
 * @brief Type-erased property value holder
 *
 * Stores a database property value (BLOB data) along with its type metadata.
 * Used for deserializing MaterialPassport properties from database.
 */
class PropertyValue {
public:
  /**
   * @brief Construct from raw BLOB data
   * @param blob_data Pointer to BLOB data from database
   * @param blob_size Size of BLOB data in bytes
   * @param type PropertyType enum indicating how to deserialize
   */
  PropertyValue(const void *blob_data, size_t blob_size,
                traits::PropertyType type);

  /**
   * @brief Construct from string (for convenience)
   * @param str String value
   * @param type PropertyType enum
   */
  PropertyValue(std::string_view str, traits::PropertyType type);

  /**
   * @brief Get raw BLOB data
   * @return Vector containing BLOB bytes
   */
  const std::vector<uint8_t> &blob() const noexcept { return data_; }

  /**
   * @brief Get BLOB as string view (for String, Integer, Double, etc.)
   * @return String view of BLOB data
   */
  std::string_view as_string() const noexcept;

  /**
   * @brief Get property type
   * @return PropertyType enum value
   */
  traits::PropertyType type() const noexcept { return type_; }

private:
  std::vector<uint8_t> data_;
  traits::PropertyType type_;
};

/**
 * @brief Deserializer for converting database values to C++ structs
 *
 * Uses PropertyTraits metadata to map database property values (keyed by
 * leksikon_guid) to struct fields. Handles type conversions, JSON parsing
 * for arrays, and nested object deserialization.
 */
class Deserializer {
public:
  /**
   * @brief Deserialize a struct from property value map
   *
   * @tparam T The struct type (must have PropertyTraits specialization)
   * @param obj Output object to populate
   * @param values Map of leksikon_guid → PropertyValue
   *
   * Missing properties will leave fields with default values.
   * Type mismatches will throw std::runtime_error.
   */
  template <typename T>
  static void deserialize(T &obj,
                          const std::map<std::string, PropertyValue> &values);

private:
  // Type-specific deserializers (throws on type mismatch/parse error)
  static void deserialize_string(void *ptr, const PropertyValue &value);
  static void deserialize_integer(void *ptr, const PropertyValue &value);
  static void deserialize_double(void *ptr, const PropertyValue &value);
  static void deserialize_boolean(void *ptr, const PropertyValue &value);
  static void deserialize_tristate(void *ptr, const PropertyValue &value);
  static void deserialize_string_array(void *ptr, const PropertyValue &value);
  static void deserialize_enum_value(void *ptr, const PropertyValue &value,
                                      std::string_view enum_type);
  static void deserialize_enum_array(void *ptr, const PropertyValue &value,
                                      std::string_view enum_type);
  static void deserialize_object_array(void *ptr, const PropertyValue &value,
                                        const traits::PropertyDescriptor &desc);
};

/**
 * @brief Serializer for converting C++ structs to database values
 *
 * Inverse of Deserializer - converts struct fields to PropertyValue map
 * that can be stored in the database.
 */
class Serializer {
public:
  /**
   * @brief Serialize a struct to property value map
   *
   * @tparam T The struct type (must have PropertyTraits specialization)
   * @param obj Object to serialize
   * @return Map of leksikon_guid → PropertyValue for database storage
   */
  template <typename T>
  static std::map<std::string, PropertyValue>
  serialize(const T &obj);

private:
  // Type-specific serializers
  static PropertyValue serialize_string(const void *ptr);
  static PropertyValue serialize_integer(const void *ptr);
  static PropertyValue serialize_double(const void *ptr);
  static PropertyValue serialize_boolean(const void *ptr);
  static PropertyValue serialize_tristate(const void *ptr);
  static PropertyValue serialize_string_array(const void *ptr);
  static PropertyValue serialize_enum_value(const void *ptr,
                                             std::string_view enum_type);
  static PropertyValue serialize_enum_array(const void *ptr,
                                             std::string_view enum_type);
  static PropertyValue serialize_object_array(
      const void *ptr, const traits::PropertyDescriptor &desc);
};

// Template implementation must be in header for visibility
template <typename T>
void Deserializer::deserialize(
    T &obj, const std::map<std::string, PropertyValue> &values) {
  using Traits = traits::PropertyTraits<T>;

  for (size_t i = 0; i < Traits::property_count(); ++i) {
    const auto &desc = Traits::properties()[i];

    // Find property by leksikon_guid (or field_name for nested arrays)
    const char *lookup_key = desc.type == traits::PropertyType::ObjectArray
                                 ? desc.field_name
                                 : desc.leksikon_guid;
    auto it = values.find(lookup_key);
    if (it == values.end()) {
      continue; // Use default value (field already initialized)
    }

    // Calculate field pointer
    void *field_ptr = reinterpret_cast<char *>(&obj) + desc.offset;

    // Dispatch to type-specific deserializer
    switch (desc.type) {
    case traits::PropertyType::String:
      deserialize_string(field_ptr, it->second);
      break;
    case traits::PropertyType::Integer:
      deserialize_integer(field_ptr, it->second);
      break;
    case traits::PropertyType::Double:
      deserialize_double(field_ptr, it->second);
      break;
    case traits::PropertyType::Boolean:
      deserialize_boolean(field_ptr, it->second);
      break;
    case traits::PropertyType::TriState:
      deserialize_tristate(field_ptr, it->second);
      break;
    case traits::PropertyType::StringArray:
      deserialize_string_array(field_ptr, it->second);
      break;
    case traits::PropertyType::EnumValue:
      deserialize_enum_value(field_ptr, it->second, Traits::struct_name());
      break;
    case traits::PropertyType::EnumArray:
      deserialize_enum_array(field_ptr, it->second, Traits::struct_name());
      break;
    case traits::PropertyType::ObjectArray:
      deserialize_object_array(field_ptr, it->second, desc);
      break;
    }
  }
}

template <typename T>
std::map<std::string, PropertyValue> Serializer::serialize(const T &obj) {
  using Traits = traits::PropertyTraits<T>;
  std::map<std::string, PropertyValue> values;

  for (size_t i = 0; i < Traits::property_count(); ++i) {
    const auto &desc = Traits::properties()[i];
    const void *field_ptr =
        reinterpret_cast<const char *>(&obj) + desc.offset;

    // Dispatch to type-specific serializer
    PropertyValue value("", traits::PropertyType::String); // placeholder

    switch (desc.type) {
    case traits::PropertyType::String:
      value = serialize_string(field_ptr);
      break;
    case traits::PropertyType::Integer:
      value = serialize_integer(field_ptr);
      break;
    case traits::PropertyType::Double:
      value = serialize_double(field_ptr);
      break;
    case traits::PropertyType::Boolean:
      value = serialize_boolean(field_ptr);
      break;
    case traits::PropertyType::TriState:
      value = serialize_tristate(field_ptr);
      break;
    case traits::PropertyType::StringArray:
      value = serialize_string_array(field_ptr);
      break;
    case traits::PropertyType::EnumValue:
      value = serialize_enum_value(field_ptr, Traits::struct_name());
      break;
    case traits::PropertyType::EnumArray:
      value = serialize_enum_array(field_ptr, Traits::struct_name());
      break;
    case traits::PropertyType::ObjectArray:
      value = serialize_object_array(field_ptr, desc);
      break;
    }

    // Store with leksikon_guid as key (or field_name for nested arrays)
    const char *store_key = desc.type == traits::PropertyType::ObjectArray
                                ? desc.field_name
                                : desc.leksikon_guid;
    values.emplace(store_key, std::move(value));
  }

  return values;
}

} // namespace reusex::core::serialization
