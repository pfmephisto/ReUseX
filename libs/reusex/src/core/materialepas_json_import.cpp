// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/materialepas_json_import.hpp"
#include "core/materialepas_enums.hpp"
#include "core/materialepas_json_export.hpp"
#include "core/logging.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using json = nlohmann::json;
using namespace reusex::core::traits;

namespace reusex::core::json_import {

// ===========================================================================
// Write helpers — inverse of the read helpers in export
// ===========================================================================

namespace {

/// Write a std::string field at the given byte offset within a struct
void write_string(void *section, size_t offset, const std::string &value) {
  *reinterpret_cast<std::string *>(static_cast<char *>(section) + offset) =
      value;
}

/// Write std::optional<int> field
void write_opt_int(void *section, size_t offset, std::optional<int> value) {
  *reinterpret_cast<std::optional<int> *>(static_cast<char *>(section) +
                                          offset) = value;
}

/// Write std::optional<double> field
void write_opt_double(void *section, size_t offset,
                      std::optional<double> value) {
  *reinterpret_cast<std::optional<double> *>(static_cast<char *>(section) +
                                             offset) = value;
}

/// Write std::optional<bool> field
void write_opt_bool(void *section, size_t offset, std::optional<bool> value) {
  *reinterpret_cast<std::optional<bool> *>(static_cast<char *>(section) +
                                           offset) = value;
}

/// Write TriState field
void write_tristate(void *section, size_t offset, TriState value) {
  *reinterpret_cast<TriState *>(static_cast<char *>(section) + offset) = value;
}

/// Write SubstanceContentMethod field
void write_substance_method(void *section, size_t offset,
                            SubstanceContentMethod value) {
  *reinterpret_cast<SubstanceContentMethod *>(static_cast<char *>(section) +
                                              offset) = value;
}

/// Write EmissionQuantityType field
void write_emission_qty_type(void *section, size_t offset,
                             EmissionQuantityType value) {
  *reinterpret_cast<EmissionQuantityType *>(static_cast<char *>(section) +
                                            offset) = value;
}

/// Write std::vector<std::string> field
void write_string_array(void *section, size_t offset,
                        std::vector<std::string> value) {
  *reinterpret_cast<std::vector<std::string> *>(static_cast<char *>(section) +
                                                offset) = std::move(value);
}

/// Write std::vector<Material> field
void write_material_array(void *section, size_t offset,
                          std::vector<Material> value) {
  *reinterpret_cast<std::vector<Material> *>(static_cast<char *>(section) +
                                             offset) = std::move(value);
}

/// Write std::vector<DangerousSubstance> field
void write_dangerous_substances(void *section, size_t offset,
                                std::vector<DangerousSubstance> value) {
  *reinterpret_cast<std::vector<DangerousSubstance> *>(
      static_cast<char *>(section) + offset) = std::move(value);
}

/// Write std::vector<Emission> field
void write_emissions(void *section, size_t offset,
                     std::vector<Emission> value) {
  *reinterpret_cast<std::vector<Emission> *>(static_cast<char *>(section) +
                                             offset) = std::move(value);
}

/// Determine struct_name for a section descriptor based on its properties
/// pointer
const char *
section_struct_name(const json_export::SectionDescriptor &sd) {
  if (sd.properties == PropertyTraits<Owner>::properties())
    return "Owner";
  if (sd.properties ==
      PropertyTraits<ConstructionItemDescription>::properties())
    return "ConstructionItemDescription";
  if (sd.properties == PropertyTraits<ProductInformation>::properties())
    return "ProductInformation";
  if (sd.properties == PropertyTraits<Certifications>::properties())
    return "Certifications";
  if (sd.properties == PropertyTraits<Dimensions>::properties())
    return "Dimensions";
  if (sd.properties == PropertyTraits<Condition>::properties())
    return "Condition";
  if (sd.properties == PropertyTraits<Pollution>::properties())
    return "Pollution";
  if (sd.properties == PropertyTraits<EnvironmentalPotential>::properties())
    return "EnvironmentalPotential";
  if (sd.properties == PropertyTraits<FireProperties>::properties())
    return "FireProperties";
  if (sd.properties == PropertyTraits<History>::properties())
    return "History";
  return "";
}

/// Check if all values in a nested properties array are empty strings
bool is_empty_template(const json &properties_array) {
  for (const auto &prop : properties_array) {
    if (prop.contains("value")) {
      auto val = prop["value"].get<std::string>();
      if (!val.empty())
        return false;
    }
  }
  return true;
}

/// Parse a single nested DangerousSubstance from a JSON properties array
DangerousSubstance
parse_dangerous_substance(const json &properties_array) {
  DangerousSubstance sub;

  // Build a name→value lookup for the nested properties
  for (const auto &prop : properties_array) {
    if (!prop.contains("name") || !prop.contains("value"))
      continue;
    auto name = prop["name"].get<std::string>();
    auto val = prop["value"].get<std::string>();

    // Match by json_name from DangerousSubstance PropertyTraits
    const auto *descs = PropertyTraits<DangerousSubstance>::properties();
    auto count = PropertyTraits<DangerousSubstance>::property_count();

    for (size_t i = 0; i < count; ++i) {
      const auto &desc = descs[i];
      if (desc.json_name[0] == '\0')
        continue;
      if (name != desc.json_name)
        continue;

      switch (desc.type) {
      case PropertyType::String:
        write_string(&sub, desc.offset, val);
        break;
      case PropertyType::Double: {
        if (!val.empty()) {
          try {
            write_opt_double(&sub, desc.offset, std::stod(val));
          } catch (const std::exception &e) {
            reusex::warn("Failed to parse double for '{}': {}", name,
                               e.what());
          }
        }
        break;
      }
      case PropertyType::EnumValue: {
        auto parsed = substance_content_method_from_string(val);
        if (parsed.has_value()) {
          write_substance_method(&sub, desc.offset, *parsed);
        }
        break;
      }
      default:
        break;
      }
      break; // Found matching descriptor
    }
  }
  return sub;
}

/// Parse a single nested Emission from a JSON properties array
Emission parse_emission(const json &properties_array) {
  Emission em;

  for (const auto &prop : properties_array) {
    if (!prop.contains("name") || !prop.contains("value"))
      continue;
    auto name = prop["name"].get<std::string>();
    auto val = prop["value"].get<std::string>();

    const auto *descs = PropertyTraits<Emission>::properties();
    auto count = PropertyTraits<Emission>::property_count();

    for (size_t i = 0; i < count; ++i) {
      const auto &desc = descs[i];
      if (desc.json_name[0] == '\0')
        continue;
      if (name != desc.json_name)
        continue;

      switch (desc.type) {
      case PropertyType::String:
        write_string(&em, desc.offset, val);
        break;
      case PropertyType::Double: {
        if (!val.empty()) {
          try {
            write_opt_double(&em, desc.offset, std::stod(val));
          } catch (const std::exception &e) {
            reusex::warn("Failed to parse double for '{}': {}", name,
                               e.what());
          }
        }
        break;
      }
      case PropertyType::EnumValue: {
        auto parsed = emission_quantity_type_from_string(val);
        if (parsed.has_value()) {
          write_emission_qty_type(&em, desc.offset, *parsed);
        }
        break;
      }
      default:
        break;
      }
      break; // Found matching descriptor
    }
  }
  return em;
}

/// Deserialize a single simple property value from JSON into its struct field
void deserialize_simple_property(void *section,
                                 const PropertyDescriptor &desc,
                                 const std::string &value,
                                 const char *struct_name) {
  switch (desc.type) {
  case PropertyType::String:
    write_string(section, desc.offset, value);
    break;

  case PropertyType::Integer: {
    if (value.empty()) {
      write_opt_int(section, desc.offset, std::nullopt);
    } else {
      try {
        write_opt_int(section, desc.offset, std::stoi(value));
      } catch (const std::exception &e) {
        reusex::warn("Failed to parse integer for '{}': {}", desc.json_name,
                           e.what());
      }
    }
    break;
  }

  case PropertyType::Double: {
    if (value.empty()) {
      write_opt_double(section, desc.offset, std::nullopt);
    } else {
      try {
        write_opt_double(section, desc.offset, std::stod(value));
      } catch (const std::exception &e) {
        reusex::warn("Failed to parse double for '{}': {}", desc.json_name,
                           e.what());
      }
    }
    break;
  }

  case PropertyType::Boolean: {
    if (value.empty()) {
      write_opt_bool(section, desc.offset, std::nullopt);
    } else if (value == "true") {
      write_opt_bool(section, desc.offset, true);
    } else if (value == "false") {
      write_opt_bool(section, desc.offset, false);
    }
    break;
  }

  case PropertyType::TriState: {
    auto parsed = tri_state_from_string(value);
    write_tristate(section, desc.offset,
                   parsed.value_or(TriState::unknown));
    break;
  }

  case PropertyType::EnumValue: {
    if (std::strcmp(struct_name, "DangerousSubstance") == 0) {
      auto parsed = substance_content_method_from_string(value);
      if (parsed.has_value()) {
        write_substance_method(section, desc.offset, *parsed);
      }
    } else if (std::strcmp(struct_name, "Emission") == 0) {
      auto parsed = emission_quantity_type_from_string(value);
      if (parsed.has_value()) {
        write_emission_qty_type(section, desc.offset, *parsed);
      }
    }
    break;
  }

  default:
    break;
  }
}

/// Parse a "values" array of {"value": ...} objects into a vector of strings
std::vector<std::string> parse_values_array(const json &prop) {
  std::vector<std::string> result;
  if (!prop.contains("values"))
    return result;
  for (const auto &item : prop["values"]) {
    if (item.contains("value")) {
      result.push_back(item["value"].get<std::string>());
    }
  }
  return result;
}

} // anonymous namespace

// ===========================================================================
// Public API
// ===========================================================================

MaterialPassport from_json(const json &j) {
  if (!j.contains("sections")) {
    throw std::runtime_error(
        "Invalid material passport JSON: missing 'sections' key");
  }
  if (!j["sections"].is_array()) {
    throw std::runtime_error(
        "Invalid material passport JSON: 'sections' must be an array");
  }

  MaterialPassport passport;

  // Build a lookup map from nameEN → JSON section object
  std::unordered_map<std::string, const json *> section_map;
  for (const auto &sec : j["sections"]) {
    if (sec.contains("nameEN")) {
      section_map[sec["nameEN"].get<std::string>()] = &sec;
    }
  }

  // Iterate section descriptors and match by nameEN
  for (const auto &sd : json_export::section_descriptors()) {
    auto it = section_map.find(sd.name_en);
    if (it == section_map.end())
      continue;

    const json &json_section = *it->second;
    if (!json_section.contains("properties"))
      continue;

    void *section_ptr =
        reinterpret_cast<char *>(&passport) + sd.passport_offset;
    const char *sname = section_struct_name(sd);

    // Build a lookup map from property name → list of JSON property objects
    // (multiple entries possible for ObjectArray types)
    std::unordered_map<std::string, std::vector<const json *>> prop_map;
    for (const auto &prop : json_section["properties"]) {
      if (prop.contains("name")) {
        prop_map[prop["name"].get<std::string>()].push_back(&prop);
      }
    }

    // Process each property descriptor
    for (size_t i = 0; i < sd.property_count; ++i) {
      const auto &desc = sd.properties[i];

      // Skip properties with empty json_name
      if (desc.json_name[0] == '\0')
        continue;

      auto pit = prop_map.find(desc.json_name);
      if (pit == prop_map.end())
        continue;

      const auto &entries = pit->second;
      if (entries.empty())
        continue;

      switch (desc.type) {
      case PropertyType::StringArray: {
        const json &prop = *entries[0];
        if (prop.contains("values")) {
          write_string_array(section_ptr, desc.offset,
                             parse_values_array(prop));
        }
        // If "value": "" → leave as empty vector (default)
        break;
      }

      case PropertyType::EnumArray: {
        const json &prop = *entries[0];
        if (prop.contains("values")) {
          std::vector<Material> materials;
          for (const auto &item : prop["values"]) {
            if (item.contains("value")) {
              auto parsed =
                  material_from_string(item["value"].get<std::string>());
              if (parsed.has_value()) {
                materials.push_back(*parsed);
              }
            }
          }
          write_material_array(section_ptr, desc.offset, std::move(materials));
        }
        break;
      }

      case PropertyType::ObjectArray: {
        if (desc.nested_properties ==
            PropertyTraits<DangerousSubstance>::properties()) {
          // Check if single empty template entry
          if (entries.size() == 1 && entries[0]->contains("properties") &&
              is_empty_template((*entries[0])["properties"])) {
            // Leave as empty vector
            break;
          }

          std::vector<DangerousSubstance> substances;
          for (const json *entry : entries) {
            if (entry->contains("properties")) {
              substances.push_back(
                  parse_dangerous_substance((*entry)["properties"]));
            }
          }
          write_dangerous_substances(section_ptr, desc.offset,
                                     std::move(substances));
        } else if (desc.nested_properties ==
                   PropertyTraits<Emission>::properties()) {
          if (entries.size() == 1 && entries[0]->contains("properties") &&
              is_empty_template((*entries[0])["properties"])) {
            break;
          }

          std::vector<Emission> emissions;
          for (const json *entry : entries) {
            if (entry->contains("properties")) {
              emissions.push_back(parse_emission((*entry)["properties"]));
            }
          }
          write_emissions(section_ptr, desc.offset, std::move(emissions));
        }
        break;
      }

      default: {
        // Simple types: String, Integer, Double, Boolean, TriState, EnumValue
        const json &prop = *entries[0];
        if (prop.contains("value")) {
          auto val = prop["value"].get<std::string>();
          deserialize_simple_property(section_ptr, desc, val, sname);
        }
        break;
      }
      }
    }
  }

  // --- Transaction Log ---
  if (j.contains("log") && j["log"].is_array()) {
    for (const auto &log_entry : j["log"]) {
      if (!log_entry.contains("properties"))
        continue;

      TransactionLogEntry entry;
      for (const auto &prop : log_entry["properties"]) {
        if (!prop.contains("name") || !prop.contains("value"))
          continue;
        auto name = prop["name"].get<std::string>();
        auto val = prop["value"].get<std::string>();

        if (name == "type") {
          auto parsed = transaction_type_from_string(val);
          if (parsed.has_value())
            entry.type = *parsed;
        } else if (name == "guid") {
          entry.guid = val;
        } else if (name == "edited by") {
          entry.edited_by = val;
        } else if (name == "edited date") {
          entry.edited_date = val;
        } else if (name == "old value") {
          entry.old_value = val;
        } else if (name == "new value") {
          entry.new_value = val;
        }
      }
      passport.transaction_log.push_back(std::move(entry));
    }
  }

  // --- Metadata ---
  if (j.contains("metadata") && j["metadata"].is_object()) {
    const auto &meta = j["metadata"];
    if (meta.contains("document guid"))
      passport.metadata.document_guid =
          meta["document guid"].get<std::string>();
    if (meta.contains("document creation date"))
      passport.metadata.creation_date =
          meta["document creation date"].get<std::string>();
    if (meta.contains("document revision date"))
      passport.metadata.revision_date =
          meta["document revision date"].get<std::string>();
    if (meta.contains("version number"))
      passport.metadata.version_number =
          meta["version number"].get<std::string>();
    if (meta.contains("version date"))
      passport.metadata.version_date =
          meta["version date"].get<std::string>();
  }

  return passport;
}

std::vector<MaterialPassport> from_json_array(const json &j) {
  if (!j.is_array()) {
    throw std::runtime_error(
        "Expected JSON array for from_json_array, got " +
        std::string(j.type_name()));
  }
  std::vector<MaterialPassport> result;
  result.reserve(j.size());
  for (const auto &elem : j) {
    result.push_back(from_json(elem));
  }
  return result;
}

std::vector<MaterialPassport> from_json_string(std::string_view json_str) {
  auto j = json::parse(json_str);
  if (j.is_array()) {
    return from_json_array(j);
  }
  return {from_json(j)};
}

} // namespace reusex::core::json_import
