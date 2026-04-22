// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/materialepas_serialization.hpp"
#include "core/materialepas_enums.hpp"
#include "core/materialepas_types.hpp"
#include "core/logging.hpp"

#include <nlohmann/json.hpp>
#include <charconv>
#include <stdexcept>

using json = nlohmann::json;

namespace ReUseX::core::serialization {

// ===========================================================================
// PropertyValue Implementation
// ===========================================================================

PropertyValue::PropertyValue(const void *blob_data, size_t blob_size,
                              traits::PropertyType type)
    : type_(type) {
  if (blob_data && blob_size > 0) {
    const uint8_t *bytes = static_cast<const uint8_t *>(blob_data);
    data_.assign(bytes, bytes + blob_size);
  }
}

PropertyValue::PropertyValue(std::string_view str, traits::PropertyType type)
    : type_(type) {
  data_.assign(str.begin(), str.end());
}

std::string_view PropertyValue::as_string() const noexcept {
  return std::string_view(reinterpret_cast<const char *>(data_.data()),
                          data_.size());
}

// ===========================================================================
// Deserializer Implementation
// ===========================================================================

void Deserializer::deserialize_string(void *ptr, const PropertyValue &value) {
  auto *field = static_cast<std::string *>(ptr);
  *field = std::string(value.as_string());
}

void Deserializer::deserialize_integer(void *ptr, const PropertyValue &value) {
  auto *field = static_cast<std::optional<int> *>(ptr);

  std::string_view str = value.as_string();
  if (str.empty()) {
    *field = std::nullopt;
    return;
  }

  int result = 0;
  auto [p, ec] = std::from_chars(str.data(), str.data() + str.size(), result);
  if (ec != std::errc()) {
    throw std::runtime_error("Failed to parse integer: " + std::string(str));
  }
  *field = result;
}

void Deserializer::deserialize_double(void *ptr, const PropertyValue &value) {
  auto *field = static_cast<std::optional<double> *>(ptr);

  std::string_view str = value.as_string();
  if (str.empty()) {
    *field = std::nullopt;
    return;
  }

  try {
    *field = std::stod(std::string(str));
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to parse double: " + std::string(str));
  }
}

void Deserializer::deserialize_boolean(void *ptr, const PropertyValue &value) {
  auto *field = static_cast<std::optional<bool> *>(ptr);

  std::string_view str = value.as_string();
  if (str.empty()) {
    *field = std::nullopt;
    return;
  }

  if (str == "true" || str == "1") {
    *field = true;
  } else if (str == "false" || str == "0") {
    *field = false;
  } else {
    throw std::runtime_error("Failed to parse boolean: " + std::string(str));
  }
}

void Deserializer::deserialize_tristate(void *ptr, const PropertyValue &value) {
  auto *field = static_cast<TriState *>(ptr);

  std::string_view str = value.as_string();
  if (auto result = tri_state_from_string(str)) {
    *field = *result;
  } else {
    ReUseX::warn("Unknown TriState value '{}', defaulting to 'unknown'", str);
    *field = TriState::unknown;
  }
}

void Deserializer::deserialize_string_array(void *ptr,
                                             const PropertyValue &value) {
  auto *field = static_cast<std::vector<std::string> *>(ptr);

  std::string_view str = value.as_string();
  if (str.empty()) {
    field->clear();
    return;
  }

  try {
    json j = json::parse(str);
    if (!j.is_array()) {
      throw std::runtime_error("Expected JSON array for string_array type");
    }

    field->clear();
    for (const auto &item : j) {
      if (item.is_string()) {
        field->push_back(item.get<std::string>());
      }
    }
  } catch (const json::exception &e) {
    throw std::runtime_error("Failed to parse JSON array: " + std::string(e.what()));
  }
}

void Deserializer::deserialize_enum_value(void *ptr,
                                           const PropertyValue &value,
                                           std::string_view enum_type) {
  std::string_view str = value.as_string();

  // Dispatch based on struct context to use the correct enum type
  if (enum_type == "DangerousSubstance") {
    if (auto result = substance_content_method_from_string(str)) {
      *static_cast<SubstanceContentMethod *>(ptr) = *result;
      return;
    }
    throw std::runtime_error("Failed to parse SubstanceContentMethod: " + std::string(str));
  }

  if (enum_type == "Emission") {
    if (auto result = emission_quantity_type_from_string(str)) {
      *static_cast<EmissionQuantityType *>(ptr) = *result;
      return;
    }
    throw std::runtime_error("Failed to parse EmissionQuantityType: " + std::string(str));
  }

  // Default: Material enum
  if (auto result = material_from_string(str)) {
    *static_cast<Material *>(ptr) = *result;
    return;
  }

  throw std::runtime_error("Failed to parse enum value: " + std::string(str));
}

void Deserializer::deserialize_enum_array(void *ptr,
                                           const PropertyValue &value,
                                           std::string_view enum_type) {
  auto *field = static_cast<std::vector<Material> *>(ptr);

  std::string_view str = value.as_string();
  if (str.empty()) {
    field->clear();
    return;
  }

  try {
    json j = json::parse(str);
    if (!j.is_array()) {
      throw std::runtime_error("Expected JSON array for enum_array type");
    }

    field->clear();
    for (const auto &item : j) {
      if (item.is_string()) {
        if (auto mat = material_from_string(item.get<std::string>())) {
          field->push_back(*mat);
        }
      }
    }
  } catch (const json::exception &e) {
    throw std::runtime_error("Failed to parse enum array: " + std::string(e.what()));
  }
}

void Deserializer::deserialize_object_array(
    void *ptr, const PropertyValue &value,
    const traits::PropertyDescriptor &desc) {

  std::string_view str = value.as_string();
  if (str.empty()) {
    return; // Leave vector empty
  }

  try {
    json j = json::parse(str);
    if (!j.is_array()) {
      throw std::runtime_error("Expected JSON array for object_array type");
    }

    // Helper lambda to extract JSON value as string without extra quotes
    auto json_to_string = [](const json &val) -> std::string {
      if (val.is_string()) {
        return val.get<std::string>();
      }
      return val.dump();
    };

    // Handle DangerousSubstance array
    if (desc.nested_properties == traits::PropertyTraits<DangerousSubstance>::properties()) {
      auto *field = static_cast<std::vector<DangerousSubstance> *>(ptr);
      field->clear();

      for (const auto &obj : j) {
        DangerousSubstance item;

        // Deserialize nested object properties
        std::map<std::string, PropertyValue> nested_values;
        for (size_t i = 0; i < desc.nested_count; ++i) {
          const auto &nested_desc = desc.nested_properties[i];
          if (obj.contains(nested_desc.field_name)) {
            std::string val_str = json_to_string(obj[nested_desc.field_name]);
            nested_values.emplace(nested_desc.leksikon_guid,
                                  PropertyValue(val_str, nested_desc.type));
          }
        }

        deserialize(item, nested_values);
        field->push_back(std::move(item));
      }
      return;
    }

    // Handle Emission array
    if (desc.nested_properties == traits::PropertyTraits<Emission>::properties()) {
      auto *field = static_cast<std::vector<Emission> *>(ptr);
      field->clear();

      for (const auto &obj : j) {
        Emission item;

        std::map<std::string, PropertyValue> nested_values;
        for (size_t i = 0; i < desc.nested_count; ++i) {
          const auto &nested_desc = desc.nested_properties[i];
          if (obj.contains(nested_desc.field_name)) {
            std::string val_str = json_to_string(obj[nested_desc.field_name]);
            nested_values.emplace(nested_desc.leksikon_guid,
                                  PropertyValue(val_str, nested_desc.type));
          }
        }

        deserialize(item, nested_values);
        field->push_back(std::move(item));
      }
      return;
    }

    throw std::runtime_error("Unknown nested object type for object_array");

  } catch (const json::exception &e) {
    throw std::runtime_error("Failed to parse object array: " + std::string(e.what()));
  }
}

// ===========================================================================
// Serializer Implementation
// ===========================================================================

PropertyValue Serializer::serialize_string(const void *ptr) {
  const auto *field = static_cast<const std::string *>(ptr);
  return PropertyValue(*field, traits::PropertyType::String);
}

PropertyValue Serializer::serialize_integer(const void *ptr) {
  const auto *field = static_cast<const std::optional<int> *>(ptr);
  if (!field->has_value()) {
    return PropertyValue("", traits::PropertyType::Integer);
  }
  return PropertyValue(std::to_string(**field), traits::PropertyType::Integer);
}

PropertyValue Serializer::serialize_double(const void *ptr) {
  const auto *field = static_cast<const std::optional<double> *>(ptr);
  if (!field->has_value()) {
    return PropertyValue("", traits::PropertyType::Double);
  }
  return PropertyValue(std::to_string(**field), traits::PropertyType::Double);
}

PropertyValue Serializer::serialize_boolean(const void *ptr) {
  const auto *field = static_cast<const std::optional<bool> *>(ptr);
  if (!field->has_value()) {
    return PropertyValue("", traits::PropertyType::Boolean);
  }
  return PropertyValue(**field ? "true" : "false",
                       traits::PropertyType::Boolean);
}

PropertyValue Serializer::serialize_tristate(const void *ptr) {
  const auto *field = static_cast<const TriState *>(ptr);
  return PropertyValue(to_string(*field), traits::PropertyType::TriState);
}

PropertyValue Serializer::serialize_string_array(const void *ptr) {
  const auto *field = static_cast<const std::vector<std::string> *>(ptr);
  json j = json::array();
  for (const auto &str : *field) {
    j.push_back(str);
  }
  return PropertyValue(j.dump(), traits::PropertyType::StringArray);
}

PropertyValue Serializer::serialize_enum_value(const void *ptr,
                                                 std::string_view enum_type) {
  // Dispatch based on struct context to avoid misinterpreting enum values
  if (enum_type == "DangerousSubstance") {
    const auto *sub = static_cast<const SubstanceContentMethod *>(ptr);
    return PropertyValue(to_string(*sub), traits::PropertyType::EnumValue);
  }

  if (enum_type == "Emission") {
    const auto *emit = static_cast<const EmissionQuantityType *>(ptr);
    return PropertyValue(to_string(*emit), traits::PropertyType::EnumValue);
  }

  // Default: Material enum (used in ConstructionItemDescription and others)
  const auto *mat = static_cast<const Material *>(ptr);
  return PropertyValue(to_string(*mat), traits::PropertyType::EnumValue);
}

PropertyValue Serializer::serialize_enum_array(const void *ptr,
                                                 std::string_view enum_type) {
  const auto *field = static_cast<const std::vector<Material> *>(ptr);
  json j = json::array();
  for (const auto &mat : *field) {
    j.push_back(std::string(to_string(mat)));
  }
  return PropertyValue(j.dump(), traits::PropertyType::EnumArray);
}

PropertyValue Serializer::serialize_object_array(
    const void *ptr, const traits::PropertyDescriptor &desc) {

  json j = json::array();

  // Handle DangerousSubstance array
  if (desc.nested_properties == traits::PropertyTraits<DangerousSubstance>::properties()) {
    const auto *field = static_cast<const std::vector<DangerousSubstance> *>(ptr);

    for (const auto &item : *field) {
      json obj = json::object();
      auto values = serialize(item);

      for (size_t i = 0; i < desc.nested_count; ++i) {
        const auto &nested_desc = desc.nested_properties[i];
        auto it = values.find(nested_desc.leksikon_guid);
        if (it != values.end()) {
          obj[nested_desc.field_name] = std::string(it->second.as_string());
        }
      }

      j.push_back(obj);
    }

    return PropertyValue(j.dump(), traits::PropertyType::ObjectArray);
  }

  // Handle Emission array
  if (desc.nested_properties == traits::PropertyTraits<Emission>::properties()) {
    const auto *field = static_cast<const std::vector<Emission> *>(ptr);

    for (const auto &item : *field) {
      json obj = json::object();
      auto values = serialize(item);

      for (size_t i = 0; i < desc.nested_count; ++i) {
        const auto &nested_desc = desc.nested_properties[i];
        auto it = values.find(nested_desc.leksikon_guid);
        if (it != values.end()) {
          obj[nested_desc.field_name] = std::string(it->second.as_string());
        }
      }

      j.push_back(obj);
    }

    return PropertyValue(j.dump(), traits::PropertyType::ObjectArray);
  }

  throw std::runtime_error("Unknown nested object type for serialization");
}

} // namespace ReUseX::core::serialization
