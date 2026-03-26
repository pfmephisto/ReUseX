// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/materialepas_json_export.hpp"
#include "core/materialepas_enums.hpp"

#include <array>
#include <cstring>
#include <string>

using json = nlohmann::json;
using namespace ReUseX::core::traits;

namespace ReUseX::core::json_export {

// ===========================================================================
// Section Registry (JSON template order)
// ===========================================================================

static const std::array<SectionDescriptor, 10> k_sections = {{
    {"Owner", "Ejer",
     PropertyTraits<Owner>::properties(),
     PropertyTraits<Owner>::property_count(),
     offsetof(MaterialPassport, owner)},

    {"Construction item description", "Byggevarebeskrivelse",
     PropertyTraits<ConstructionItemDescription>::properties(),
     PropertyTraits<ConstructionItemDescription>::property_count(),
     offsetof(MaterialPassport, description)},

    {"Product information", "Produktinformation",
     PropertyTraits<ProductInformation>::properties(),
     PropertyTraits<ProductInformation>::property_count(),
     offsetof(MaterialPassport, product)},

    {"Certifications, approvements and declarations",
     "Certificeringer, godkendelser og deklarationer",
     PropertyTraits<Certifications>::properties(),
     PropertyTraits<Certifications>::property_count(),
     offsetof(MaterialPassport, certifications)},

    {"Dimensions and geometri", "Dimensioner og geometri",
     PropertyTraits<Dimensions>::properties(),
     PropertyTraits<Dimensions>::property_count(),
     offsetof(MaterialPassport, dimensions)},

    {"History", "Historik",
     PropertyTraits<History>::properties(),
     PropertyTraits<History>::property_count(),
     offsetof(MaterialPassport, history)},

    {"Condition", "Tilstand",
     PropertyTraits<Condition>::properties(),
     PropertyTraits<Condition>::property_count(),
     offsetof(MaterialPassport, condition)},

    {"Pollution - content and emissions",
     "Forurening - indhold og afgasning",
     PropertyTraits<Pollution>::properties(),
     PropertyTraits<Pollution>::property_count(),
     offsetof(MaterialPassport, pollution)},

    {"Environmental and resource potential or sustainable utilization of "
     "natural resources",
     "Milj\xc3\xb8- og ressourcepotentiale eller b\xc3\xa6redygtig udnyttelse "
     "af naturressourcer",
     PropertyTraits<EnvironmentalPotential>::properties(),
     PropertyTraits<EnvironmentalPotential>::property_count(),
     offsetof(MaterialPassport, environmental)},

    {"Other essential properties",
     "\xc3\x98vrige v\xc3\xa6sentlige egenskaber",
     PropertyTraits<FireProperties>::properties(),
     PropertyTraits<FireProperties>::property_count(),
     offsetof(MaterialPassport, fire)},
}};

std::span<const SectionDescriptor> section_descriptors() { return k_sections; }

// ===========================================================================
// Value extraction helpers
// ===========================================================================

namespace {

/// Read a std::string field at the given byte offset within a struct
const std::string &read_string(const void *section, size_t offset) {
  return *reinterpret_cast<const std::string *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::optional<int> field
const std::optional<int> &read_opt_int(const void *section, size_t offset) {
  return *reinterpret_cast<const std::optional<int> *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::optional<double> field
const std::optional<double> &read_opt_double(const void *section,
                                              size_t offset) {
  return *reinterpret_cast<const std::optional<double> *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::optional<bool> field
const std::optional<bool> &read_opt_bool(const void *section, size_t offset) {
  return *reinterpret_cast<const std::optional<bool> *>(
      static_cast<const char *>(section) + offset);
}

/// Read TriState field
const TriState &read_tristate(const void *section, size_t offset) {
  return *reinterpret_cast<const TriState *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::vector<std::string> field
const std::vector<std::string> &read_string_array(const void *section,
                                                    size_t offset) {
  return *reinterpret_cast<const std::vector<std::string> *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::vector<Material> field
const std::vector<Material> &read_material_array(const void *section,
                                                   size_t offset) {
  return *reinterpret_cast<const std::vector<Material> *>(
      static_cast<const char *>(section) + offset);
}

/// Read SubstanceContentMethod enum field
const SubstanceContentMethod &read_substance_method(const void *section,
                                                      size_t offset) {
  return *reinterpret_cast<const SubstanceContentMethod *>(
      static_cast<const char *>(section) + offset);
}

/// Read EmissionQuantityType enum field
const EmissionQuantityType &read_emission_qty_type(const void *section,
                                                     size_t offset) {
  return *reinterpret_cast<const EmissionQuantityType *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::vector<DangerousSubstance> field
const std::vector<DangerousSubstance> &
read_dangerous_substances(const void *section, size_t offset) {
  return *reinterpret_cast<const std::vector<DangerousSubstance> *>(
      static_cast<const char *>(section) + offset);
}

/// Read std::vector<Emission> field
const std::vector<Emission> &read_emissions(const void *section,
                                              size_t offset) {
  return *reinterpret_cast<const std::vector<Emission> *>(
      static_cast<const char *>(section) + offset);
}

/// Format a double value as string (removing trailing zeros)
std::string format_double(double val) {
  std::string s = std::to_string(val);
  // Remove trailing zeros after decimal point
  auto dot_pos = s.find('.');
  if (dot_pos != std::string::npos) {
    auto last_nonzero = s.find_last_not_of('0');
    if (last_nonzero != std::string::npos && last_nonzero > dot_pos) {
      s.erase(last_nonzero + 1);
    } else if (last_nonzero == dot_pos) {
      // e.g., "3." -> "3.0" (keep at least one decimal)
      s.erase(dot_pos + 2);
    }
  }
  return s;
}

/// Convert a simple property to its JSON value string
std::string property_value_string(const void *section,
                                   const PropertyDescriptor &desc,
                                   const char *struct_name) {
  switch (desc.type) {
  case PropertyType::String:
    return read_string(section, desc.offset);

  case PropertyType::Integer: {
    const auto &opt = read_opt_int(section, desc.offset);
    return opt.has_value() ? std::to_string(*opt) : "";
  }

  case PropertyType::Double: {
    const auto &opt = read_opt_double(section, desc.offset);
    return opt.has_value() ? format_double(*opt) : "";
  }

  case PropertyType::Boolean: {
    const auto &opt = read_opt_bool(section, desc.offset);
    if (!opt.has_value())
      return "";
    return *opt ? "true" : "false";
  }

  case PropertyType::TriState:
    return std::string(to_string(read_tristate(section, desc.offset)));

  case PropertyType::EnumValue: {
    if (std::strcmp(struct_name, "DangerousSubstance") == 0) {
      return std::string(
          to_string(read_substance_method(section, desc.offset)));
    }
    if (std::strcmp(struct_name, "Emission") == 0) {
      return std::string(
          to_string(read_emission_qty_type(section, desc.offset)));
    }
    return "";
  }

  default:
    return "";
  }
}

/// Convert a single nested object (DangerousSubstance or Emission) to JSON
/// properties array
json nested_object_properties(const void *obj,
                               const PropertyDescriptor *nested_props,
                               size_t nested_count,
                               const char *struct_name) {
  json props = json::array();
  for (size_t i = 0; i < nested_count; ++i) {
    const auto &nd = nested_props[i];

    // Skip properties with empty json_name
    if (nd.json_name[0] == '\0')
      continue;

    json prop;
    prop["name"] = nd.json_name;
    prop["guid"] = nd.leksikon_guid;
    prop["value"] = property_value_string(obj, nd, struct_name);
    props.push_back(std::move(prop));
  }
  return props;
}

/// Serialize a property to the appropriate JSON format
void serialize_property(json &properties_array, const void *section,
                        const PropertyDescriptor &desc,
                        const char *struct_name) {
  // Skip properties with empty json_name
  if (desc.json_name[0] == '\0')
    return;

  json prop;
  prop["name"] = desc.json_name;

  switch (desc.type) {
  case PropertyType::StringArray: {
    prop["guid"] = desc.leksikon_guid;
    const auto &arr = read_string_array(section, desc.offset);
    if (arr.empty()) {
      prop["value"] = "";
    } else {
      json values = json::array();
      for (const auto &s : arr) {
        json val;
        val["value"] = s;
        values.push_back(std::move(val));
      }
      prop["values"] = std::move(values);
    }
    break;
  }

  case PropertyType::EnumArray: {
    prop["guid"] = desc.leksikon_guid;
    const auto &arr = read_material_array(section, desc.offset);
    if (arr.empty()) {
      json values = json::array();
      prop["values"] = std::move(values);
    } else {
      json values = json::array();
      for (const auto &m : arr) {
        json val;
        val["value"] = std::string(to_string(m));
        val["guid"] = "";
        values.push_back(std::move(val));
      }
      prop["values"] = std::move(values);
    }
    break;
  }

  case PropertyType::ObjectArray: {
    // Each instance of the nested type becomes a separate entry in the
    // parent section's properties array with its own "properties" sub-array.
    prop["guid"] = desc.leksikon_guid;

    // Determine which type of nested object
    if (desc.nested_properties ==
        PropertyTraits<DangerousSubstance>::properties()) {
      const auto &substances =
          read_dangerous_substances(section, desc.offset);
      if (substances.empty()) {
        // Empty template: one entry with empty nested properties
        prop["properties"] =
            nested_object_properties(nullptr, desc.nested_properties,
                                      0, "DangerousSubstance");
        // Actually, show the empty template with all fields
        json nested = json::array();
        for (size_t i = 0; i < desc.nested_count; ++i) {
          const auto &nd = desc.nested_properties[i];
          if (nd.json_name[0] == '\0')
            continue;
          json np;
          np["name"] = nd.json_name;
          np["guid"] = nd.leksikon_guid;
          np["value"] = "";
          nested.push_back(std::move(np));
        }
        prop["properties"] = std::move(nested);
        properties_array.push_back(std::move(prop));
      } else {
        for (const auto &sub : substances) {
          json entry;
          entry["name"] = desc.json_name;
          entry["guid"] = desc.leksikon_guid;
          entry["properties"] = nested_object_properties(
              &sub, desc.nested_properties, desc.nested_count,
              "DangerousSubstance");
          properties_array.push_back(std::move(entry));
        }
      }
      return; // Already pushed directly
    }

    if (desc.nested_properties == PropertyTraits<Emission>::properties()) {
      const auto &emissions = read_emissions(section, desc.offset);
      if (emissions.empty()) {
        json nested = json::array();
        for (size_t i = 0; i < desc.nested_count; ++i) {
          const auto &nd = desc.nested_properties[i];
          if (nd.json_name[0] == '\0')
            continue;
          json np;
          np["name"] = nd.json_name;
          np["guid"] = nd.leksikon_guid;
          np["value"] = "";
          nested.push_back(std::move(np));
        }
        prop["properties"] = std::move(nested);
        properties_array.push_back(std::move(prop));
      } else {
        for (const auto &em : emissions) {
          json entry;
          entry["name"] = desc.json_name;
          entry["guid"] = desc.leksikon_guid;
          entry["properties"] = nested_object_properties(
              &em, desc.nested_properties, desc.nested_count, "Emission");
          properties_array.push_back(std::move(entry));
        }
      }
      return; // Already pushed directly
    }
    break;
  }

  default: {
    prop["guid"] = desc.leksikon_guid;
    prop["value"] = property_value_string(section, desc, struct_name);
    break;
  }
  }

  properties_array.push_back(std::move(prop));
}

/// Determine struct_name for a section descriptor based on its properties pointer
const char *section_struct_name(const SectionDescriptor &sd) {
  if (sd.properties == PropertyTraits<Owner>::properties())
    return "Owner";
  if (sd.properties == PropertyTraits<ConstructionItemDescription>::properties())
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

} // anonymous namespace

// ===========================================================================
// Public API
// ===========================================================================

json to_json(const MaterialPassport &passport) {
  json result;

  // --- Sections ---
  json sections = json::array();
  for (const auto &sd : k_sections) {
    json section;
    section["nameEN"] = sd.name_en;
    section["nameDA"] = sd.name_da;

    json props = json::array();
    const void *section_ptr =
        reinterpret_cast<const char *>(&passport) + sd.passport_offset;
    const char *sname = section_struct_name(sd);

    for (size_t i = 0; i < sd.property_count; ++i) {
      serialize_property(props, section_ptr, sd.properties[i], sname);
    }

    section["properties"] = std::move(props);
    sections.push_back(std::move(section));
  }
  result["sections"] = std::move(sections);

  // --- Transaction Log ---
  json log = json::array();
  for (const auto &entry : passport.transaction_log) {
    json log_entry;
    json log_props = json::array();

    json type_prop;
    type_prop["name"] = "type";
    type_prop["value"] = std::string(to_string(entry.type));
    log_props.push_back(std::move(type_prop));

    json guid_prop;
    guid_prop["name"] = "guid";
    guid_prop["value"] = entry.guid;
    log_props.push_back(std::move(guid_prop));

    json edited_by_prop;
    edited_by_prop["name"] = "edited by";
    edited_by_prop["value"] = entry.edited_by;
    log_props.push_back(std::move(edited_by_prop));

    json edited_date_prop;
    edited_date_prop["name"] = "edited date";
    edited_date_prop["value"] = entry.edited_date;
    log_props.push_back(std::move(edited_date_prop));

    json old_value_prop;
    old_value_prop["name"] = "old value";
    old_value_prop["value"] = entry.old_value;
    log_props.push_back(std::move(old_value_prop));

    json new_value_prop;
    new_value_prop["name"] = "new value";
    new_value_prop["value"] = entry.new_value;
    log_props.push_back(std::move(new_value_prop));

    log_entry["properties"] = std::move(log_props);
    log.push_back(std::move(log_entry));
  }
  result["log"] = std::move(log);

  // --- Metadata ---
  json metadata;
  metadata["document guid"] = passport.metadata.document_guid;
  metadata["document creation date"] = passport.metadata.creation_date;
  metadata["document revision date"] = passport.metadata.revision_date;
  metadata["version number"] = passport.metadata.version_number;
  metadata["version date"] = passport.metadata.version_date;
  result["metadata"] = std::move(metadata);

  return result;
}

json to_json(const std::vector<MaterialPassport> &passports) {
  json result = json::array();
  for (const auto &p : passports) {
    result.push_back(to_json(p));
  }
  return result;
}

std::string to_json_string(const MaterialPassport &passport, int indent) {
  return to_json(passport).dump(indent);
}

std::string to_json_string(const std::vector<MaterialPassport> &passports,
                           int indent) {
  return to_json(passports).dump(indent);
}

} // namespace ReUseX::core::json_export
