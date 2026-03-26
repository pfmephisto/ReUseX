// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "materialepas_property_types.hpp"
#include <array>
#include <cstddef>

namespace ReUseX::core::traits {

/**
 * @brief Property metadata descriptor
 *
 * Contains all metadata needed to serialize/deserialize a single struct field.
 * GUIDs are stored here (NOT in the struct itself) to maintain clean separation.
 */
struct PropertyDescriptor {
  /// C++ field name (e.g., "contact_email")
  const char *field_name;

  /// Molio leksiCon GUID (e.g., "0Bwj05D$55V931bq9VaBE5")
  const char *leksikon_guid;

  /// JSON export name matching the Danish standard template
  /// (e.g., "materialpassport owner contact email")
  /// Empty string means skip in JSON export.
  const char *json_name;

  /// Property type enum
  PropertyType type;

  /// Offset of field in struct (calculated via offsetof macro)
  size_t offset;

  /// For nested types (ObjectArray): pointer to nested property array
  const PropertyDescriptor *nested_properties = nullptr;

  /// For nested types: number of properties in nested array
  size_t nested_count = 0;
};

/**
 * @brief Trait interface for property metadata
 *
 * Specialize this template for each serializable struct to provide:
 * - Array of PropertyDescriptor instances
 * - Property count
 * - Struct name for debugging/logging
 *
 * @tparam T The struct type to provide traits for
 */
template <typename T> struct PropertyTraits {
  /**
   * @brief Get array of property descriptors
   * @return Pointer to static array of PropertyDescriptor instances
   */
  static constexpr const PropertyDescriptor *properties() noexcept;

  /**
   * @brief Get number of properties in the struct
   * @return Property count
   */
  static constexpr size_t property_count() noexcept;

  /**
   * @brief Get human-readable struct name
   * @return Struct name string (for debugging/logging)
   */
  static constexpr const char *struct_name() noexcept;
};

// ============================================================================
// Helper Functions for Creating PropertyDescriptors
// ============================================================================

namespace detail {

/**
 * @brief Create a simple property descriptor (non-nested)
 *
 * Helper function to construct PropertyDescriptor for simple fields.
 * Provides type-safe, constexpr property creation without macros.
 *
 * @tparam T The struct type containing the field
 * @param field_name C++ field name (e.g., "contact_email")
 * @param leksikon_guid Molio leksiCon GUID string
 * @param json_name JSON export name matching Danish standard template
 * @param type PropertyType enum value
 * @param field_offset Offset of field in struct (use offsetof)
 * @return PropertyDescriptor instance
 */
template <typename T>
constexpr PropertyDescriptor make_property(
    const char* field_name,
    const char* leksikon_guid,
    const char* json_name,
    PropertyType type,
    size_t field_offset) noexcept {
  return PropertyDescriptor{
    field_name,
    leksikon_guid,
    json_name,
    type,
    field_offset,
    nullptr,
    0
  };
}

/**
 * @brief Create a nested object array property descriptor
 *
 * Helper function for fields like std::vector<DangerousSubstance>.
 * Links to nested type's PropertyTraits for recursive serialization.
 *
 * @tparam T The parent struct type
 * @tparam NestedT The nested struct type (must have PropertyTraits specialization)
 * @param field_name C++ field name
 * @param json_name JSON export name matching Danish standard template
 * @param field_offset Offset of field in struct (use offsetof)
 * @return PropertyDescriptor instance with nested properties linked
 */
template <typename T, typename NestedT>
constexpr PropertyDescriptor make_nested_property(
    const char* field_name,
    const char* json_name,
    size_t field_offset) noexcept {
  return PropertyDescriptor{
    field_name,
    "",  // No GUID for nested arrays
    json_name,
    PropertyType::ObjectArray,
    field_offset,
    PropertyTraits<NestedT>::properties(),
    PropertyTraits<NestedT>::property_count()
  };
}

} // namespace detail

} // namespace ReUseX::core::traits

// ===========================================================================
// Include types before specializations
// ===========================================================================

#include "materialepas_types.hpp"

namespace ReUseX::core::traits {

// Bring types into scope for convenience
using ::ReUseX::core::Owner;
using ::ReUseX::core::ConstructionItemDescription;
using ::ReUseX::core::ProductInformation;
using ::ReUseX::core::Certifications;
using ::ReUseX::core::Dimensions;
using ::ReUseX::core::Condition;
using ::ReUseX::core::DangerousSubstance;
using ::ReUseX::core::Emission;
using ::ReUseX::core::Pollution;
using ::ReUseX::core::EnvironmentalPotential;
using ::ReUseX::core::FireProperties;
using ::ReUseX::core::History;

// ===========================================================================
// Template Specializations
// ===========================================================================

// ===========================================================================
// Section 1: Owner (3 properties)
// ===========================================================================

template <>
struct PropertyTraits<Owner> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Owner>(
      "contact_email", "0Bwj05D$55V931bq9VaBE5",
      "materialpassport owner contact email",
      PropertyType::String, offsetof(Owner, contact_email)),
    detail::make_property<Owner>(
      "contact_name", "17BdeQk152gR5YUt5oSLfp",
      "materialpassport owner contact name",
      PropertyType::String, offsetof(Owner, contact_name)),
    detail::make_property<Owner>(
      "company_name", "1bb51PIefCe9uHGCmR0gFJ",
      "materialpassport owner company name",
      PropertyType::String, offsetof(Owner, company_name))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Owner";
  }
};

// ===========================================================================
// Section 2: Construction Item Description (8 properties)
// ===========================================================================

template <>
struct PropertyTraits<ConstructionItemDescription> {
  static inline constexpr std::array properties_array = {
    detail::make_property<ConstructionItemDescription>(
      "designation", "2_mYRkA$9EcwXAxqBGoMrM",
      "construction item designation",
      PropertyType::String, offsetof(ConstructionItemDescription, designation)),
    detail::make_property<ConstructionItemDescription>(
      "images", "1dvQfmc2z9lB_rwknYkM3y",
      "construction item image",
      PropertyType::StringArray, offsetof(ConstructionItemDescription, images)),
    detail::make_property<ConstructionItemDescription>(
      "has_qr_code", "06moAxe0j7EutROb0dD3B7",
      "has QR code",
      PropertyType::Boolean, offsetof(ConstructionItemDescription, has_qr_code)),
    detail::make_property<ConstructionItemDescription>(
      "has_rfid_tag", "2$Owq1iHrAuw$ASWOXAk9p",
      "has RFID tag",
      PropertyType::Boolean, offsetof(ConstructionItemDescription, has_rfid_tag)),
    detail::make_property<ConstructionItemDescription>(
      "materials", "1R_z75Gd52OQYxVkJ$E3ZU",
      "material",
      PropertyType::EnumArray, offsetof(ConstructionItemDescription, materials)),
    detail::make_property<ConstructionItemDescription>(
      "assembly_methods", "0N7Qwopp125xrULezqArw4",
      "assembly method",
      PropertyType::StringArray, offsetof(ConstructionItemDescription, assembly_methods)),
    detail::make_property<ConstructionItemDescription>(
      "year_of_installation", "294ZkUyYn6TfdwCsbo2X63",
      "year of installation",
      PropertyType::Integer, offsetof(ConstructionItemDescription, year_of_installation)),
    detail::make_property<ConstructionItemDescription>(
      "year_of_construction", "17LFECVML6buNXwBLz416Y",
      "year of construction of the building",
      PropertyType::Integer, offsetof(ConstructionItemDescription, year_of_construction))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "ConstructionItemDescription";
  }
};

// ===========================================================================
// Section 3: Product Information (5 properties)
// ===========================================================================

template <>
struct PropertyTraits<ProductInformation> {
  static inline constexpr std::array properties_array = {
    detail::make_property<ProductInformation>(
      "manufacturer", "1QwFy1F3T4dPe2N2cbsuRv",
      "manufacturer",
      PropertyType::String, offsetof(ProductInformation, manufacturer)),
    detail::make_property<ProductInformation>(
      "gtin", "1rQ9Z5vy5AygUSncyWuhXr",
      "global trade item number (GTIN)",
      PropertyType::String, offsetof(ProductInformation, gtin)),
    detail::make_property<ProductInformation>(
      "product_name", "198tMTFIX5PecEfAwMEIeo",
      "product name",
      PropertyType::String, offsetof(ProductInformation, product_name)),
    detail::make_property<ProductInformation>(
      "model_label", "06t2gkj49FNvMebmltrHXV",
      "model label",
      PropertyType::String, offsetof(ProductInformation, model_label)),
    detail::make_property<ProductInformation>(
      "production_year", "3nif2XJKH8r9I09LfEYaRm",
      "production year",
      PropertyType::Integer, offsetof(ProductInformation, production_year))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "ProductInformation";
  }
};

// ===========================================================================
// Section 4: Certifications (13 properties)
// ===========================================================================

template <>
struct PropertyTraits<Certifications> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Certifications>(
      "has_epd", "2_jn1$mEr1780H5QOiEj6O",
      "environmental product declaration (EPD)",
      PropertyType::Boolean, offsetof(Certifications, has_epd)),
    detail::make_property<Certifications>(
      "epd_programme_operator", "0ORiYBKRnCFQzUwOsNqbYn",
      "EPD programme operator",
      PropertyType::String, offsetof(Certifications, epd_programme_operator)),
    detail::make_property<Certifications>(
      "epd_operator_web_domain", "2kqTixLBr9sBqGKYs9lNSI",
      "EPD programme operator web domain",
      PropertyType::String, offsetof(Certifications, epd_operator_web_domain)),
    detail::make_property<Certifications>(
      "epd_registration_number", "2aCraAVB91qw8CzrM6x3ia",
      "EPD registration number",
      PropertyType::String, offsetof(Certifications, epd_registration_number)),
    detail::make_property<Certifications>(
      "reference_service_life", "28mSsPe_z69BksBt5oRXMl",
      "reference service life",
      PropertyType::Double, offsetof(Certifications, reference_service_life)),
    detail::make_property<Certifications>(
      "has_safety_data_sheet", "3D1KFVwbDBuumnIBZZHQId",
      "has safety data sheet",
      PropertyType::Boolean, offsetof(Certifications, has_safety_data_sheet)),
    detail::make_property<Certifications>(
      "declaration_of_performance", "1tqCmrJwn3yvF$k2lU2fFa",
      "declaration of performance",
      PropertyType::StringArray, offsetof(Certifications, declaration_of_performance)),
    detail::make_property<Certifications>(
      "technical_documentation", "1IfrbnqGv5Pu3GBuocLZgi",
      "technical documentation",
      PropertyType::StringArray, offsetof(Certifications, technical_documentation)),
    detail::make_property<Certifications>(
      "non_destructive_tests", "0aTBceworF$QTXM6CqyOdV",
      "non-destructive tests",
      PropertyType::StringArray, offsetof(Certifications, non_destructive_tests)),
    detail::make_property<Certifications>(
      "assessed_period_of_use", "3xW23JRMTAzOQb79uZW3qC",
      "assessed period of use",
      PropertyType::Double, offsetof(Certifications, assessed_period_of_use)),
    detail::make_property<Certifications>(
      "avg_service_life_build", "2TtVRWo_56mBphAcSiFIHG",
      "average service life (BUILD)",
      PropertyType::Double, offsetof(Certifications, avg_service_life_build)),
    detail::make_property<Certifications>(
      "remaining_service_life_rsl", "2id4UZ8ynAF9DEzV0NC_wT",
      "service life (RSL)",
      PropertyType::Double, offsetof(Certifications, remaining_service_life_rsl)),
    detail::make_property<Certifications>(
      "remaining_service_life_build", "27gXAA5Iz7lgcOytMRqOBv",
      "remaining service life (BUILD)",
      PropertyType::Double, offsetof(Certifications, remaining_service_life_build))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Certifications";
  }
};

// ===========================================================================
// Section 5: Dimensions (11 properties)
// ===========================================================================

template <>
struct PropertyTraits<Dimensions> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Dimensions>(
      "width_mm", "3NHBUedX9438Hi3mwD15$Z",
      "width",
      PropertyType::Double, offsetof(Dimensions, width_mm)),
    detail::make_property<Dimensions>(
      "height_mm", "2G$wMhUvL2LgKNY0J66oAT",
      "height",
      PropertyType::Double, offsetof(Dimensions, height_mm)),
    detail::make_property<Dimensions>(
      "length_mm", "1uUn3YWZfBaPqYQMo_$Om$",
      "length",
      PropertyType::Double, offsetof(Dimensions, length_mm)),
    detail::make_property<Dimensions>(
      "thickness_mm", "0SyXPZ9an9vh49k$Lgkvly",
      "thickness",
      PropertyType::Double, offsetof(Dimensions, thickness_mm)),
    detail::make_property<Dimensions>(
      "depth_mm", "1$OV5du3LFWwa$P7SfJhAr",
      "depth",
      PropertyType::Double, offsetof(Dimensions, depth_mm)),
    detail::make_property<Dimensions>(
      "volume_m3", "3IHRMkfs9EDPXVvPJ0hbFn",
      "volume",
      PropertyType::Double, offsetof(Dimensions, volume_m3)),
    detail::make_property<Dimensions>(
      "surface_area_m2", "3Cs4d96Ff1nPGHWlsTyYIS",
      "surface area of product",
      PropertyType::Double, offsetof(Dimensions, surface_area_m2)),
    detail::make_property<Dimensions>(
      "inner_diameter_mm", "0ayREcz2zBuPWmCDTem2a8",
      "inner diameter",
      PropertyType::Double, offsetof(Dimensions, inner_diameter_mm)),
    detail::make_property<Dimensions>(
      "outer_diameter_mm", "3BWwvo0lf3iAd_34y7ac$4",
      "outer diameter",
      PropertyType::Double, offsetof(Dimensions, outer_diameter_mm)),
    detail::make_property<Dimensions>(
      "weight_kg", "1XyLvnxf94pwJW6LWaD6Zw",
      "weight",
      PropertyType::Double, offsetof(Dimensions, weight_kg)),
    detail::make_property<Dimensions>(
      "technical_drawing", "2d01jZqvP7GOR3V$coAprV",
      "technical drawing",
      PropertyType::String, offsetof(Dimensions, technical_drawing))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Dimensions";
  }
};

// ===========================================================================
// Section 6: Condition (8 properties)
// ===========================================================================

template <>
struct PropertyTraits<Condition> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Condition>(
      "photo_documentation", "3$DH1Zm_r9_gzqdwKK39Ua",
      "photo documentation",
      PropertyType::StringArray, offsetof(Condition, photo_documentation)),
    detail::make_property<Condition>(
      "visual_inspection_performed", "2jB20LC_nEzBN4_j2ypip8",
      "visual inspection performed",
      PropertyType::Boolean, offsetof(Condition, visual_inspection_performed)),
    detail::make_property<Condition>(
      "has_signs_of_damage", "0rQimGJFHEueqtqwbwBaC5",
      "has signs of damage",
      PropertyType::Boolean, offsetof(Condition, has_signs_of_damage)),
    detail::make_property<Condition>(
      "is_deformed", "1g6mt3SrvDlho6YqBIu1lW",
      "is deformed",
      PropertyType::Boolean, offsetof(Condition, is_deformed)),
    detail::make_property<Condition>(
      "is_scratched", "365luGupDCfwgtD32gGGrS",
      "is scratched",
      PropertyType::Boolean, offsetof(Condition, is_scratched)),
    detail::make_property<Condition>(
      "is_surface_intact", "2GO932BuX5EvXOecmhbbl3",
      "is surface intact",
      PropertyType::Boolean, offsetof(Condition, is_surface_intact)),
    detail::make_property<Condition>(
      "has_intact_edges", "0AoBmqQ9v1pw7ca1VVp_Za",
      "has intact edges",
      PropertyType::Boolean, offsetof(Condition, has_intact_edges)),
    detail::make_property<Condition>(
      "has_signs_of_degradation", "01lVHQF$XAneE8CFtajgr_",
      "has signs of degradation",
      PropertyType::Boolean, offsetof(Condition, has_signs_of_degradation))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Condition";
  }
};

// ===========================================================================
// Section 7: Pollution - Nested Types (DangerousSubstance, Emission)
// ===========================================================================

// DangerousSubstance (5 properties - nested)
template <>
struct PropertyTraits<DangerousSubstance> {
  static inline constexpr std::array properties_array = {
    detail::make_property<DangerousSubstance>(
      "content_method", "2ympiI4PL6c8ooh9Agqnz4",
      "content of dangerous substances",
      PropertyType::EnumValue, offsetof(DangerousSubstance, content_method)),
    detail::make_property<DangerousSubstance>(
      "analyzed_substance", "3Y2oTiV9r3NfoZQBVP_Bpx",
      "analyzed for chemical substances",
      PropertyType::String, offsetof(DangerousSubstance, analyzed_substance)),
    detail::make_property<DangerousSubstance>(
      "cas_number", "2tZYDby2H2PADWYcTMn50O",
      "CAS-number",
      PropertyType::String, offsetof(DangerousSubstance, cas_number)),
    detail::make_property<DangerousSubstance>(
      "ec_number", "1cV5alHQ93Zu44Uv4Iuqoj",
      "EC-number",
      PropertyType::String, offsetof(DangerousSubstance, ec_number)),
    detail::make_property<DangerousSubstance>(
      "concentration_mg_per_kg", "0yw_0Ggsf6ihdShOA1niRl",
      "concentration of substance",
      PropertyType::Double, offsetof(DangerousSubstance, concentration_mg_per_kg))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "DangerousSubstance";
  }
};

// Emission (7 properties - nested)
template <>
struct PropertyTraits<Emission> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Emission>(
      "standard", "1Kx6G$uIXEZO3Br$MEsdaB",
      "emission standard",
      PropertyType::String, offsetof(Emission, standard)),
    detail::make_property<Emission>(
      "type", "12ZQ3obYHB1OhXiyy3LcfK",
      "emission type",
      PropertyType::String, offsetof(Emission, type)),
    detail::make_property<Emission>(
      "lower_interval", "1dSnTNTGD5YQPcgcbzMJaD",
      "emission lower Interval",
      PropertyType::Double, offsetof(Emission, lower_interval)),
    detail::make_property<Emission>(
      "upper_interval", "3YTotdivHEeQS4ij2Xa7PU",
      "emission upper Interval",
      PropertyType::Double, offsetof(Emission, upper_interval)),
    detail::make_property<Emission>(
      "quantity_type", "33HQrsPDn9uhOu$7D3HhWU",
      "emission quantity type",
      PropertyType::EnumValue, offsetof(Emission, quantity_type)),
    detail::make_property<Emission>(
      "measuring_unit", "3d1HQUJVb23ReixUzvSyvS",
      "measuring unit",
      PropertyType::String, offsetof(Emission, measuring_unit)),
    detail::make_property<Emission>(
      "quantity", "0lZ6gflK18WBte7dMyQQ$h",
      "",
      PropertyType::Double, offsetof(Emission, quantity))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Emission";
  }
};

// Pollution (9 properties including 2 nested arrays)
template <>
struct PropertyTraits<Pollution> {
  static inline constexpr std::array properties_array = {
    detail::make_property<Pollution>(
      "contains_reach_substances", "2E6_MQ4Cn36f0s6CVYxQOR",
      "contains substances on REACH's candidate list",
      PropertyType::TriState, offsetof(Pollution, contains_reach_substances)),
    detail::make_property<Pollution>(
      "is_chemically_treated", "1Cf0jFllr2HAk8FOmT2VGr",
      "is construction item chemically treated",
      PropertyType::TriState, offsetof(Pollution, is_chemically_treated)),
    detail::make_property<Pollution>(
      "surface_treatments", "1zV5ZyYKT6CPWjKdTEZ1K0",
      "surface treatment",
      PropertyType::StringArray, offsetof(Pollution, surface_treatments)),
    detail::make_nested_property<Pollution, DangerousSubstance>(
      "dangerous_substances", "dangerous substances",
      offsetof(Pollution, dangerous_substances)),
    detail::make_nested_property<Pollution, Emission>(
      "emissions", "release of dangerous substances",
      offsetof(Pollution, emissions)),
    detail::make_property<Pollution>(
      "intended_for_indoor_use", "2W1zl8TFb459xOVytRmX3J",
      "intended for indoor use",
      PropertyType::Boolean, offsetof(Pollution, intended_for_indoor_use)),
    detail::make_property<Pollution>(
      "labelling_scheme", "3Txv$QA7zCp8azbWeuubx2",
      "labelling scheme",
      PropertyType::String, offsetof(Pollution, labelling_scheme)),
    detail::make_property<Pollution>(
      "emission_level", "3areU4Rpv9gfbeDNE9eBgA",
      "emission level",
      PropertyType::String, offsetof(Pollution, emission_level)),
    detail::make_property<Pollution>(
      "has_asbestos_analysis", "2962p$Lwj68Br6satSNLAn",
      "has asbestos analysis",
      PropertyType::Boolean, offsetof(Pollution, has_asbestos_analysis))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "Pollution";
  }
};

// ===========================================================================
// Section 8: Environmental Potential (2 properties)
// ===========================================================================

template <>
struct PropertyTraits<EnvironmentalPotential> {
  static inline constexpr std::array properties_array = {
    detail::make_property<EnvironmentalPotential>(
      "takeback_scheme_available", "2__I$nkL94YRI_YCBy0ou8",
      "take-back scheme available",
      PropertyType::Boolean, offsetof(EnvironmentalPotential, takeback_scheme_available)),
    detail::make_property<EnvironmentalPotential>(
      "consists_of_separate_parts", "2nZqcaXkz7YPHhKI$FMowg",
      "construction item consists of separate parts naturally ",
      PropertyType::Boolean, offsetof(EnvironmentalPotential, consists_of_separate_parts))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "EnvironmentalPotential";
  }
};

// ===========================================================================
// Section 9: Fire Properties (4 properties)
// ===========================================================================

template <>
struct PropertyTraits<FireProperties> {
  static inline constexpr std::array properties_array = {
    detail::make_property<FireProperties>(
      "reaction_to_fire", "2bipZ4JrzDnwNZZHkwfk8O",
      "reaction to fire",
      PropertyType::String, offsetof(FireProperties, reaction_to_fire)),
    detail::make_property<FireProperties>(
      "resistance_to_fire", "0NlhENUQr4YA5$p0zBrXSp",
      "resistance to fire",
      PropertyType::String, offsetof(FireProperties, resistance_to_fire)),
    detail::make_property<FireProperties>(
      "documentation_of_fire_classification", "1DdkDjj1H7POQtnzDtu7CC",
      "documentation of fire classification",
      PropertyType::String, offsetof(FireProperties, documentation_of_fire_classification)),
    detail::make_property<FireProperties>(
      "field_of_application", "37sNiQNwHC399AW2B60MrY",
      "field of application in relation to fire",
      PropertyType::String, offsetof(FireProperties, field_of_application))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "FireProperties";
  }
};

// ===========================================================================
// Section 10: History (1 property)
// ===========================================================================

template <>
struct PropertyTraits<History> {
  static inline constexpr std::array properties_array = {
    detail::make_property<History>(
      "previous_usage_environments", "3BiwSSLEz9l8YLgoWcixHq",
      "previous usage environment",
      PropertyType::StringArray, offsetof(History, previous_usage_environments))
  };

  static constexpr const PropertyDescriptor* properties() noexcept {
    return properties_array.data();
  }

  static constexpr size_t property_count() noexcept {
    return properties_array.size();
  }

  static constexpr const char* struct_name() noexcept {
    return "History";
  }
};

} // namespace ReUseX::core::traits
