// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <optional>
#include <string_view>

namespace reusex::core {

/** @brief Material types from the Danish material passport standard (v1.0).
 *
 *  Closed value list with 43 entries. Each value corresponds to a building
 *  material category defined in the "Materialepas for genbrugte byggevarer"
 *  specification.
 */
enum class Material {
  /// @brief Natural stone
  /// @note Danish: 'Natursten'
  natural_stone,
  /// @brief Lime mortar
  /// @note Danish: 'Kalkmørtel'
  lime_mortar,
  /// @brief Concrete
  /// @note Danish: 'Beton'
  concrete,
  /// @brief Terrazzo
  /// @note Danish: 'Terrazzo'
  terrazzo,
  /// @brief Cement mortar
  /// @note Danish: 'Cementmørtel'
  cement_mortar,
  /// @brief Aerated concrete
  /// @note Danish: 'Porebeton'
  aerated_concrete,
  /// @brief Lightweight clinker concrete
  /// @note Danish: 'Letklinkerbeton'
  lightweight_clinker_concrete,
  /// @brief Plaster
  /// @note Danish: 'Puds'
  plaster,
  /// @brief Brick
  /// @note Danish: 'Mursten/tegl'
  brick,
  /// @brief Glazed tile
  /// @note Danish: 'Glaseret tegl'
  glazed_tile,
  /// @brief Metal (general)
  /// @note Danish: 'Metal (generelt)'
  metal_general,
  /// @brief Iron
  /// @note Danish: 'Jern'
  iron,
  /// @brief Steel
  /// @note Danish: 'Stål'
  steel,
  /// @brief Stainless steel
  /// @note Danish: 'Rustfrit stål'
  stainless_steel,
  /// @brief Aluminum
  /// @note Danish: 'Aluminium'
  aluminum,
  /// @brief Copper
  /// @note Danish: 'Kobber'
  copper,
  /// @brief Zinc
  /// @note Danish: 'Zink'
  zinc,
  /// @brief Lead
  /// @note Danish: 'Bly'
  lead,
  /// @brief Construction wood
  /// @note Danish: 'Konstruktionstræ'
  construction_wood,
  /// @brief Wood (unclassified)
  /// @note Danish: 'Træ (uklassificeret)'
  wood_unclassified,
  /// @brief Precious wood
  /// @note Danish: 'Ædeltræ'
  precious_wood,
  /// @brief Laminated wood
  /// @note Danish: 'Lamineret træ'
  laminated_wood,
  /// @brief Veneer
  /// @note Danish: 'Finer'
  veneer,
  /// @brief Wood fibers
  /// @note Danish: 'Træfibre'
  wood_fibers,
  /// @brief Wood fiber boards
  /// @note Danish: 'Træfiberplader'
  wood_fiber_boards,
  /// @brief Wood shavings
  /// @note Danish: 'Træspåner'
  wood_shavings,
  /// @brief Chipboards
  /// @note Danish: 'Spånplader'
  chipboards,
  /// @brief Paper
  /// @note Danish: 'Papir'
  paper,
  /// @brief Corrugated cardboard
  /// @note Danish: 'Bølgepap'
  corrugated_cardboard,
  /// @brief Impregnated cardboard
  /// @note Danish: 'Imprægneret pap'
  impregnated_cardboard,
  /// @brief Wood concrete / cement fibers
  /// @note Danish: 'Træbeton/cementfibre'
  wood_concrete_cement_fibers,
  /// @brief Mineral wool
  /// @note Danish: 'Mineraluld'
  mineral_wool,
  /// @brief Asphalt
  /// @note Danish: 'Asfalt'
  asphalt,
  /// @brief Fibers
  /// @note Danish: 'Fibre'
  fibers,
  /// @brief Lining paper
  /// @note Danish: 'Vægbeklædningspapir'
  lining_paper,
  /// @brief Linoleum
  /// @note Danish: 'Linoleum'
  linoleum,
  /// @brief Rubber
  /// @note Danish: 'Gummi'
  rubber,
  /// @brief Plastic
  /// @note Danish: 'Plast'
  plastic,
  /// @brief Foam plastic
  /// @note Danish: 'Skumplast'
  foam_plastic,
  /// @brief Glass
  /// @note Danish: 'Glas'
  glass,
  /// @brief Natural filling / aggregate
  /// @note Danish: 'Naturligt fyld/tilslag'
  natural_filling_aggregate,
  /// @brief Painting supplies
  /// @note Danish: 'Malerartikler'
  painting_supplies,
  /// @brief Other
  /// @note Danish: 'Andet'
  other,
};

/** @brief Tri-state value for Yes/No/Unknown fields.
 *
 *  Used where the standard defines three possible states, as distinct from
 *  std::optional<bool> which represents present-or-absent.
 */
enum class TriState {
  /// @brief Yes / true
  /// @note Danish: 'Ja'
  yes,
  /// @brief No / false
  /// @note Danish: 'Nej'
  no,
  /// @brief Unknown / not determined
  /// @note Danish: 'Ukendt'
  unknown,
};

/** @brief Emission quantity type.
 *  @note Danish: 'Emissionsmængdetype'
 */
enum class EmissionQuantityType {
  /// @brief Exact measured value
  /// @note Danish: 'Eksakt'
  exact,
  /// @brief Minimum value
  /// @note Danish: 'Minimum'
  minimum,
  /// @brief Maximum value
  /// @note Danish: 'Maksimum'
  maximum,
  /// @brief Interval (use lower/upper interval fields)
  /// @note Danish: 'Interval'
  interval,
};

/** @brief Method used to determine substance content.
 *  @note Danish: 'Indhold af farlige stoffer (metode)'
 */
enum class SubstanceContentMethod {
  /// @brief Assumed based on knowledge or documentation
  /// @note Danish: 'Antaget'
  assumed,
  /// @brief Measured on the surface of the material
  /// @note Danish: 'Målt (overflade)'
  measured_surface,
  /// @brief Measured in the material itself
  /// @note Danish: 'Målt (materiale)'
  measured_material,
};

/** @brief Transaction log entry type.
 *  @note Danish: 'Type'
 */
enum class TransactionType {
  /// @brief Change to the document itself (metadata)
  /// @note Danish: 'Dokument'
  document,
  /// @brief Change to a property value
  /// @note Danish: 'Egenskab'
  property,
};

// ---------------------------------------------------------------------------
// String conversion declarations
// ---------------------------------------------------------------------------

/** @brief Convert Material enum to its string identifier. */
[[nodiscard]] auto to_string(Material value) -> std::string_view;

/** @brief Parse a Material from its string identifier.
 *  @return The matching Material, or std::nullopt if not found.
 */
[[nodiscard]] auto material_from_string(std::string_view str)
    -> std::optional<Material>;

/** @brief Convert TriState enum to its string identifier. */
[[nodiscard]] auto to_string(TriState value) -> std::string_view;

/** @brief Parse a TriState from its string identifier. */
[[nodiscard]] auto tri_state_from_string(std::string_view str)
    -> std::optional<TriState>;

/** @brief Convert EmissionQuantityType enum to its string identifier. */
[[nodiscard]] auto to_string(EmissionQuantityType value) -> std::string_view;

/** @brief Parse an EmissionQuantityType from its string identifier. */
[[nodiscard]] auto emission_quantity_type_from_string(std::string_view str)
    -> std::optional<EmissionQuantityType>;

/** @brief Convert SubstanceContentMethod enum to its string identifier. */
[[nodiscard]] auto to_string(SubstanceContentMethod value) -> std::string_view;

/** @brief Parse a SubstanceContentMethod from its string identifier. */
[[nodiscard]] auto substance_content_method_from_string(std::string_view str)
    -> std::optional<SubstanceContentMethod>;

/** @brief Convert TransactionType enum to its string identifier. */
[[nodiscard]] auto to_string(TransactionType value) -> std::string_view;

/** @brief Parse a TransactionType from its string identifier. */
[[nodiscard]] auto transaction_type_from_string(std::string_view str)
    -> std::optional<TransactionType>;

} // namespace reusex::core
