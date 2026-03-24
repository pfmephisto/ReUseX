// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "core/materialepas_property_types.hpp"
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
// Registration Macros
// ============================================================================

/**
 * @brief Calculate field offset in struct
 */
#define REUSEX_PROPERTY_OFFSET(Struct, Field) offsetof(Struct, Field)

/**
 * @brief Define a simple property (non-nested)
 *
 * @param Struct The struct type
 * @param Field The field name
 * @param Guid The Molio leksiCon GUID string
 * @param Type The PropertyType enum value (without namespace)
 */
#define REUSEX_PROP(Struct, Field, Guid, Type)                                 \
  PropertyDescriptor { #Field, Guid, PropertyType::Type,                       \
                       REUSEX_PROPERTY_OFFSET(Struct, Field), nullptr, 0 }

/**
 * @brief Define a nested object array property
 *
 * For fields like std::vector<DangerousSubstance>, links to nested traits.
 *
 * @param Struct The parent struct type
 * @param Field The field name
 * @param NestedStruct The nested struct type (must have PropertyTraits
 * specialization)
 */
#define REUSEX_PROP_NESTED(Struct, Field, NestedStruct)                        \
  PropertyDescriptor {                                                         \
    #Field, "", PropertyType::ObjectArray,                                     \
        REUSEX_PROPERTY_OFFSET(Struct, Field),                                 \
        PropertyTraits<NestedStruct>::properties(),                            \
        PropertyTraits<NestedStruct>::property_count()                         \
  }

/**
 * @brief Declare PropertyTraits specialization for a struct
 *
 * Usage example:
 * @code
 * REUSEX_DECLARE_PROPERTIES(Owner,
 *   REUSEX_PROP(Owner, contact_email, "0Bwj05D$55V931bq9VaBE5", String),
 *   REUSEX_PROP(Owner, contact_name, "17BdeQk152gR5YUt5oSLfp", String),
 *   REUSEX_PROP(Owner, company_name, "1bb51PIefCe9uHGCmR0gFJ", String)
 * );
 * @endcode
 *
 * @param StructName The struct type to register
 * @param ... Variadic list of REUSEX_PROP or REUSEX_PROP_NESTED macro calls
 */
#define REUSEX_DECLARE_PROPERTIES(StructName, ...)                             \
  template <> struct PropertyTraits<StructName> {                              \
    static constexpr PropertyDescriptor properties_array[] = {__VA_ARGS__};    \
                                                                               \
    static constexpr const PropertyDescriptor *properties() noexcept {         \
      return properties_array;                                                 \
    }                                                                          \
                                                                               \
    static constexpr size_t property_count() noexcept {                        \
      return sizeof(properties_array) / sizeof(PropertyDescriptor);            \
    }                                                                          \
                                                                               \
    static constexpr const char *struct_name() noexcept { return #StructName; }\
  };

} // namespace ReUseX::core::traits
