// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "materialepas_enums.hpp"
#include "materialepas_types.hpp"

#include <fmt/format.h>

#include <string>
#include <vector>

namespace ReUseX::core {

/** @brief A single entry in the material passport transaction log.
 *
 *  Records changes to either the document metadata or individual properties.
 *  @note Danish: 'Transaktionslog'
 */
struct TransactionLogEntry {
  /// @brief Type of transaction (document or property change)
  /// @note Danish: 'Type'
  TransactionType type = TransactionType::property;

  /// @brief GUID of the changed document or property
  /// @note Danish: 'GUID'
  std::string guid;

  /// @brief Who made the change
  /// @note Danish: 'Redigeret af'
  std::string edited_by;

  /// @brief Date of the change (ISO 8601)
  /// @note Danish: 'Redigeringsdato'
  std::string edited_date;

  /// @brief Previous value before the change
  /// @note Danish: 'Gammel værdi'
  std::string old_value;

  /// @brief New value after the change
  /// @note Danish: 'Ny værdi'
  std::string new_value;
};

/** @brief Document-level metadata for the material passport.
 *  @note Danish: 'Metadata'
 */
struct MaterialPassportMetadata {
  /// @brief Unique document identifier (GUID)
  /// @note Danish: 'Dokument-GUID'
  std::string document_guid;

  /// @brief Date the document was created (ISO 8601)
  /// @note Danish: 'Dokument oprettelsesdato'
  std::string creation_date;

  /// @brief Date of last revision (ISO 8601)
  /// @note Danish: 'Dokument revisionsdato'
  std::string revision_date;

  /// @brief Version number (semver, e.g. "0.1.1")
  /// @note Danish: 'Versionsnummer'
  std::string version_number;

  /// @brief Date of the version specification (ISO 8601)
  /// @note Danish: 'Versionsdato'
  std::string version_date;
};

/** @brief A complete material passport for a reused building material.
 *
 *  Implements the Danish "Materialepas for genbrugte byggevarer" standard
 *  (v1.0, September 2024). Contains 10 sections with 75 properties covering
 *  ownership, description, certifications, dimensions, condition, pollution,
 *  environmental potential, fire properties, and history.
 *
 *  @note Danish: 'Materialepas'
 */
struct MaterialPassport {
  /// @brief Section 1: Owner information
  Owner owner;

  /// @brief Section 2: Construction item description
  ConstructionItemDescription description;

  /// @brief Section 3: Product information
  ProductInformation product;

  /// @brief Section 4: Certifications, approvals and declarations
  Certifications certifications;

  /// @brief Section 5: Dimensions and geometry
  Dimensions dimensions;

  /// @brief Section 6: Condition assessment
  Condition condition;

  /// @brief Section 7: Pollution - content and emissions
  Pollution pollution;

  /// @brief Section 8: Environmental and resource potential
  EnvironmentalPotential environmental;

  /// @brief Section 9: Fire properties
  FireProperties fire;

  /// @brief Section 10: History
  History history;

  /// @brief Transaction log recording all changes
  std::vector<TransactionLogEntry> transaction_log;

  /// @brief Document metadata
  MaterialPassportMetadata metadata;
};

} // namespace ReUseX::core

// ===========================================================================
// fmt formatters
// ===========================================================================

template <>
struct fmt::formatter<ReUseX::core::Material> : fmt::formatter<std::string_view> {
  auto format(ReUseX::core::Material value, format_context &ctx) const
      -> format_context::iterator {
    return fmt::formatter<std::string_view>::format(
        ReUseX::core::to_string(value), ctx);
  }
};

template <>
struct fmt::formatter<ReUseX::core::TriState>
    : fmt::formatter<std::string_view> {
  auto format(ReUseX::core::TriState value, format_context &ctx) const
      -> format_context::iterator {
    return fmt::formatter<std::string_view>::format(
        ReUseX::core::to_string(value), ctx);
  }
};

template <>
struct fmt::formatter<ReUseX::core::EmissionQuantityType>
    : fmt::formatter<std::string_view> {
  auto format(ReUseX::core::EmissionQuantityType value,
              format_context &ctx) const -> format_context::iterator {
    return fmt::formatter<std::string_view>::format(
        ReUseX::core::to_string(value), ctx);
  }
};

template <>
struct fmt::formatter<ReUseX::core::SubstanceContentMethod>
    : fmt::formatter<std::string_view> {
  auto format(ReUseX::core::SubstanceContentMethod value,
              format_context &ctx) const -> format_context::iterator {
    return fmt::formatter<std::string_view>::format(
        ReUseX::core::to_string(value), ctx);
  }
};

template <>
struct fmt::formatter<ReUseX::core::TransactionType>
    : fmt::formatter<std::string_view> {
  auto format(ReUseX::core::TransactionType value, format_context &ctx) const
      -> format_context::iterator {
    return fmt::formatter<std::string_view>::format(
        ReUseX::core::to_string(value), ctx);
  }
};

template <>
struct fmt::formatter<ReUseX::core::MaterialPassport>
    : fmt::formatter<std::string_view> {
  auto format(const ReUseX::core::MaterialPassport &mp,
              format_context &ctx) const -> format_context::iterator {
    return fmt::format_to(
        ctx.out(),
        "MaterialPassport(designation='{}', materials={}, version='{}')",
        mp.description.designation, mp.description.materials.size(),
        mp.metadata.version_number);
  }
};
