// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/materialepas_enums.hpp"

#include <array>
#include <algorithm>
#include <string_view>
#include <utility>

namespace reusex::core {

// ===========================================================================
// Material
// ===========================================================================

auto to_string(Material value) -> std::string_view {
  switch (value) {
  case Material::natural_stone:                return "natural_stone";
  case Material::lime_mortar:                  return "lime_mortar";
  case Material::concrete:                     return "concrete";
  case Material::terrazzo:                     return "terrazzo";
  case Material::cement_mortar:                return "cement_mortar";
  case Material::aerated_concrete:             return "aerated_concrete";
  case Material::lightweight_clinker_concrete: return "lightweight_clinker_concrete";
  case Material::plaster:                      return "plaster";
  case Material::brick:                        return "brick";
  case Material::glazed_tile:                  return "glazed_tile";
  case Material::metal_general:                return "metal_general";
  case Material::iron:                         return "iron";
  case Material::steel:                        return "steel";
  case Material::stainless_steel:              return "stainless_steel";
  case Material::aluminum:                     return "aluminum";
  case Material::copper:                       return "copper";
  case Material::zinc:                         return "zinc";
  case Material::lead:                         return "lead";
  case Material::construction_wood:            return "construction_wood";
  case Material::wood_unclassified:            return "wood_unclassified";
  case Material::precious_wood:                return "precious_wood";
  case Material::laminated_wood:               return "laminated_wood";
  case Material::veneer:                       return "veneer";
  case Material::wood_fibers:                  return "wood_fibers";
  case Material::wood_fiber_boards:            return "wood_fiber_boards";
  case Material::wood_shavings:                return "wood_shavings";
  case Material::chipboards:                   return "chipboards";
  case Material::paper:                        return "paper";
  case Material::corrugated_cardboard:         return "corrugated_cardboard";
  case Material::impregnated_cardboard:        return "impregnated_cardboard";
  case Material::wood_concrete_cement_fibers:  return "wood_concrete_cement_fibers";
  case Material::mineral_wool:                 return "mineral_wool";
  case Material::asphalt:                      return "asphalt";
  case Material::fibers:                       return "fibers";
  case Material::lining_paper:                 return "lining_paper";
  case Material::linoleum:                     return "linoleum";
  case Material::rubber:                       return "rubber";
  case Material::plastic:                      return "plastic";
  case Material::foam_plastic:                 return "foam_plastic";
  case Material::glass:                        return "glass";
  case Material::natural_filling_aggregate:    return "natural_filling_aggregate";
  case Material::painting_supplies:            return "painting_supplies";
  case Material::other:                        return "other";
  }
  return "unknown";
}

// clang-format off
static constexpr std::array<std::pair<std::string_view, Material>, 43>
    material_lookup{{
        {"natural_stone",                Material::natural_stone},
        {"lime_mortar",                  Material::lime_mortar},
        {"concrete",                     Material::concrete},
        {"terrazzo",                     Material::terrazzo},
        {"cement_mortar",                Material::cement_mortar},
        {"aerated_concrete",             Material::aerated_concrete},
        {"lightweight_clinker_concrete", Material::lightweight_clinker_concrete},
        {"plaster",                      Material::plaster},
        {"brick",                        Material::brick},
        {"glazed_tile",                  Material::glazed_tile},
        {"metal_general",                Material::metal_general},
        {"iron",                         Material::iron},
        {"steel",                        Material::steel},
        {"stainless_steel",              Material::stainless_steel},
        {"aluminum",                     Material::aluminum},
        {"copper",                       Material::copper},
        {"zinc",                         Material::zinc},
        {"lead",                         Material::lead},
        {"construction_wood",            Material::construction_wood},
        {"wood_unclassified",            Material::wood_unclassified},
        {"precious_wood",                Material::precious_wood},
        {"laminated_wood",               Material::laminated_wood},
        {"veneer",                       Material::veneer},
        {"wood_fibers",                  Material::wood_fibers},
        {"wood_fiber_boards",            Material::wood_fiber_boards},
        {"wood_shavings",                Material::wood_shavings},
        {"chipboards",                   Material::chipboards},
        {"paper",                        Material::paper},
        {"corrugated_cardboard",         Material::corrugated_cardboard},
        {"impregnated_cardboard",        Material::impregnated_cardboard},
        {"wood_concrete_cement_fibers",  Material::wood_concrete_cement_fibers},
        {"mineral_wool",                 Material::mineral_wool},
        {"asphalt",                      Material::asphalt},
        {"fibers",                       Material::fibers},
        {"lining_paper",                 Material::lining_paper},
        {"linoleum",                     Material::linoleum},
        {"rubber",                       Material::rubber},
        {"plastic",                      Material::plastic},
        {"foam_plastic",                 Material::foam_plastic},
        {"glass",                        Material::glass},
        {"natural_filling_aggregate",    Material::natural_filling_aggregate},
        {"painting_supplies",            Material::painting_supplies},
        {"other",                        Material::other},
    }};
// clang-format on

auto material_from_string(std::string_view str) -> std::optional<Material> {
  auto it = std::find_if(
      material_lookup.begin(), material_lookup.end(),
      [&str](const auto &pair) { return pair.first == str; });
  if (it != material_lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

// ===========================================================================
// TriState
// ===========================================================================

auto to_string(TriState value) -> std::string_view {
  switch (value) {
  case TriState::yes:     return "yes";
  case TriState::no:      return "no";
  case TriState::unknown: return "unknown";
  }
  return "unknown";
}

static constexpr std::array<std::pair<std::string_view, TriState>, 3>
    tri_state_lookup{{
        {"yes",     TriState::yes},
        {"no",      TriState::no},
        {"unknown", TriState::unknown},
    }};

auto tri_state_from_string(std::string_view str) -> std::optional<TriState> {
  auto it = std::find_if(
      tri_state_lookup.begin(), tri_state_lookup.end(),
      [&str](const auto &pair) { return pair.first == str; });
  if (it != tri_state_lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

// ===========================================================================
// EmissionQuantityType
// ===========================================================================

auto to_string(EmissionQuantityType value) -> std::string_view {
  switch (value) {
  case EmissionQuantityType::exact:    return "exact";
  case EmissionQuantityType::minimum:  return "minimum";
  case EmissionQuantityType::maximum:  return "maximum";
  case EmissionQuantityType::interval: return "interval";
  }
  return "unknown";
}

static constexpr std::array<std::pair<std::string_view, EmissionQuantityType>, 4>
    emission_quantity_type_lookup{{
        {"exact",    EmissionQuantityType::exact},
        {"minimum",  EmissionQuantityType::minimum},
        {"maximum",  EmissionQuantityType::maximum},
        {"interval", EmissionQuantityType::interval},
    }};

auto emission_quantity_type_from_string(std::string_view str)
    -> std::optional<EmissionQuantityType> {
  auto it = std::find_if(
      emission_quantity_type_lookup.begin(),
      emission_quantity_type_lookup.end(),
      [&str](const auto &pair) { return pair.first == str; });
  if (it != emission_quantity_type_lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

// ===========================================================================
// SubstanceContentMethod
// ===========================================================================

auto to_string(SubstanceContentMethod value) -> std::string_view {
  switch (value) {
  case SubstanceContentMethod::assumed:          return "assumed";
  case SubstanceContentMethod::measured_surface: return "measured_surface";
  case SubstanceContentMethod::measured_material: return "measured_material";
  }
  return "unknown";
}

static constexpr std::array<std::pair<std::string_view, SubstanceContentMethod>, 3>
    substance_content_method_lookup{{
        {"assumed",           SubstanceContentMethod::assumed},
        {"measured_surface",  SubstanceContentMethod::measured_surface},
        {"measured_material", SubstanceContentMethod::measured_material},
    }};

auto substance_content_method_from_string(std::string_view str)
    -> std::optional<SubstanceContentMethod> {
  auto it = std::find_if(
      substance_content_method_lookup.begin(),
      substance_content_method_lookup.end(),
      [&str](const auto &pair) { return pair.first == str; });
  if (it != substance_content_method_lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

// ===========================================================================
// TransactionType
// ===========================================================================

auto to_string(TransactionType value) -> std::string_view {
  switch (value) {
  case TransactionType::document: return "document";
  case TransactionType::property: return "property";
  }
  return "unknown";
}

static constexpr std::array<std::pair<std::string_view, TransactionType>, 2>
    transaction_type_lookup{{
        {"document", TransactionType::document},
        {"property", TransactionType::property},
    }};

auto transaction_type_from_string(std::string_view str)
    -> std::optional<TransactionType> {
  auto it = std::find_if(
      transaction_type_lookup.begin(), transaction_type_lookup.end(),
      [&str](const auto &pair) { return pair.first == str; });
  if (it != transaction_type_lookup.end()) {
    return it->second;
  }
  return std::nullopt;
}

} // namespace reusex::core
