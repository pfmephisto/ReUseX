// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "core/materialepas_enums.hpp"

#include <optional>
#include <string>
#include <vector>

namespace ReUseX::core {

// ===========================================================================
// Section 1: Owner
// ===========================================================================

/** @brief Owner contact information for the material passport.
 *  @note Danish section: 'Ejer'
 */
struct Owner {
  /// @brief Contact email of the material passport owner
  /// @note Danish: 'Materialepas ejer kontakt email'
  std::string contact_email;

  /// @brief Contact name of the material passport owner
  /// @note Danish: 'Materialepas ejer kontaktnavn'
  std::string contact_name;

  /// @brief Company name of the material passport owner
  /// @note Danish: 'Materialepas ejer virksomhedsnavn'
  std::string company_name;
};

// ===========================================================================
// Section 2: Construction Item Description
// ===========================================================================

/** @brief Description of the reused construction item.
 *  @note Danish section: 'Byggevarebeskrivelse'
 */
struct ConstructionItemDescription {
  /// @brief Designation of the construction item
  /// @note Danish: 'Byggevarebetegnelse'
  std::string designation;

  /// @brief Images of the construction item (URLs or base64)
  /// @note Danish: 'Byggevarebillede'
  std::vector<std::string> images;

  /// @brief Whether the item has a QR code
  /// @note Danish: 'Har QR-kode'
  std::optional<bool> has_qr_code;

  /// @brief Whether the item has an RFID tag
  /// @note Danish: 'Har RFID-tag'
  std::optional<bool> has_rfid_tag;

  /// @brief Materials the construction item is made of (closed value list)
  /// @note Danish: 'Materiale'
  std::vector<Material> materials;

  /// @brief Assembly methods used for the construction item
  /// @note Danish: 'Sammenføjningsmetode'
  /// @note Open value list. Predefined values include: bolted, glued, grouted,
  ///       hung, keyed, laid, mortared, nailed, pressed, riveted, screwed,
  ///       soldered, stacked, suspended, wedged, welded, wound, other.
  std::vector<std::string> assembly_methods;

  /// @brief Year the item was installed in the building
  /// @note Danish: 'Installationsår'
  std::optional<int> year_of_installation;

  /// @brief Year of construction of the building
  /// @note Danish: 'Bygningens opførelsesår'
  std::optional<int> year_of_construction;
};

// ===========================================================================
// Section 3: Product Information
// ===========================================================================

/** @brief Original product information.
 *  @note Danish section: 'Produktinformation'
 */
struct ProductInformation {
  /// @brief Original manufacturer name
  /// @note Danish: 'Producent'
  std::string manufacturer;

  /// @brief Global Trade Item Number (GTIN / EAN barcode)
  /// @note Danish: 'Global Trade Item Number (GTIN)'
  std::string gtin;

  /// @brief Product name
  /// @note Danish: 'Produktnavn'
  std::string product_name;

  /// @brief Model label / product identifier
  /// @note Danish: 'Modelbetegnelse'
  std::string model_label;

  /// @brief Year the product was manufactured
  /// @note Danish: 'Produktionsår'
  std::optional<int> production_year;
};

// ===========================================================================
// Section 4: Certifications, Approvals and Declarations
// ===========================================================================

/** @brief Certifications, approvals and declarations for the item.
 *  @note Danish section: 'Certificeringer, godkendelser og deklarationer'
 */
struct Certifications {
  /// @brief Whether an Environmental Product Declaration (EPD) exists
  /// @note Danish: 'Miljøvaredeklaration (EPD)'
  std::optional<bool> has_epd;

  /// @brief EPD programme operator name
  /// @note Danish: 'EPD programoperatør'
  /// @note Closed value list (22 operators). Examples: EPD International,
  ///       IBU, BRE, INIES, EPD Norge, EPD Danmark, etc.
  std::string epd_programme_operator;

  /// @brief EPD programme operator web domain (URL)
  /// @note Danish: 'EPD programoperatør webdomæne'
  std::string epd_operator_web_domain;

  /// @brief EPD registration number
  /// @note Danish: 'EPD registreringsnummer'
  std::string epd_registration_number;

  /// @brief Reference service life in years
  /// @note Danish: 'Referencebrugstid'
  std::optional<double> reference_service_life;

  /// @brief Whether a safety data sheet exists
  /// @note Danish: 'Har sikkerhedsdatablad'
  std::optional<bool> has_safety_data_sheet;

  /// @brief Declaration of performance references
  /// @note Danish: 'Ydeevnedeklaration'
  /// @note Open value list of document references.
  std::vector<std::string> declaration_of_performance;

  /// @brief Technical documentation references
  /// @note Danish: 'Teknisk dokumentation'
  /// @note Open value list of document references.
  std::vector<std::string> technical_documentation;

  /// @brief Non-destructive test results
  /// @note Danish: 'Ikke-destruktive prøvninger'
  /// @note Open value list of test references.
  std::vector<std::string> non_destructive_tests;

  /// @brief Assessed period of use in years
  /// @note Danish: 'Vurderet brugsperiode'
  std::optional<double> assessed_period_of_use;

  /// @brief Average service life according to BUILD methodology (years)
  /// @note Danish: 'Gennemsnitlig levetid (BUILD)'
  std::optional<double> avg_service_life_build;

  /// @brief Remaining service life (RSL) in years
  /// @note Danish: 'Restlevetid (RSL)'
  std::optional<double> remaining_service_life_rsl;

  /// @brief Remaining service life according to BUILD methodology (years)
  /// @note Danish: 'Restlevetid (BUILD)'
  std::optional<double> remaining_service_life_build;
};

// ===========================================================================
// Section 5: Dimensions and Geometry
// ===========================================================================

/** @brief Physical dimensions and geometry of the construction item.
 *  @note Danish section: 'Dimensioner og geometri'
 */
struct Dimensions {
  /// @brief Width in millimeters
  /// @note Danish: 'Bredde'
  std::optional<double> width_mm;

  /// @brief Height in millimeters
  /// @note Danish: 'Højde'
  std::optional<double> height_mm;

  /// @brief Length in millimeters
  /// @note Danish: 'Længde'
  std::optional<double> length_mm;

  /// @brief Thickness in millimeters
  /// @note Danish: 'Tykkelse'
  std::optional<double> thickness_mm;

  /// @brief Depth in millimeters
  /// @note Danish: 'Dybde'
  std::optional<double> depth_mm;

  /// @brief Volume in cubic meters
  /// @note Danish: 'Volumen'
  std::optional<double> volume_m3;

  /// @brief Surface area of product in square meters
  /// @note Danish: 'Overfladeareal'
  std::optional<double> surface_area_m2;

  /// @brief Inner diameter in millimeters
  /// @note Danish: 'Indvendig diameter'
  std::optional<double> inner_diameter_mm;

  /// @brief Outer diameter in millimeters
  /// @note Danish: 'Udvendig diameter'
  std::optional<double> outer_diameter_mm;

  /// @brief Weight in kilograms
  /// @note Danish: 'Vægt'
  std::optional<double> weight_kg;

  /// @brief Technical drawing (image URL or base64)
  /// @note Danish: 'Teknisk tegning'
  std::string technical_drawing;
};

// ===========================================================================
// Section 6: Condition
// ===========================================================================

/** @brief Condition assessment of the construction item.
 *  @note Danish section: 'Tilstand'
 */
struct Condition {
  /// @brief Photo documentation images (URLs or base64)
  /// @note Danish: 'Fotodokumentation'
  std::vector<std::string> photo_documentation;

  /// @brief Whether a visual inspection has been performed
  /// @note Danish: 'Visuel inspektion udført'
  std::optional<bool> visual_inspection_performed;

  /// @brief Whether the item has signs of damage
  /// @note Danish: 'Har tegn på skade'
  std::optional<bool> has_signs_of_damage;

  /// @brief Whether the item is deformed
  /// @note Danish: 'Er deformeret'
  std::optional<bool> is_deformed;

  /// @brief Whether the item is scratched
  /// @note Danish: 'Er ridset'
  std::optional<bool> is_scratched;

  /// @brief Whether the surface is intact
  /// @note Danish: 'Er overfladen intakt'
  std::optional<bool> is_surface_intact;

  /// @brief Whether edges are intact
  /// @note Danish: 'Har intakte kanter'
  std::optional<bool> has_intact_edges;

  /// @brief Whether the item has signs of degradation
  /// @note Danish: 'Har tegn på nedbrydning'
  std::optional<bool> has_signs_of_degradation;
};

// ===========================================================================
// Section 7: Pollution - Content and Emissions (with nested types)
// ===========================================================================

/** @brief Dangerous substance analysis record.
 *
 *  Nested within the Pollution section. Each entry documents one substance
 *  that has been analyzed for.
 *  @note Danish: 'Farlige stoffer'
 */
struct DangerousSubstance {
  /// @brief Method used to determine substance content
  /// @note Danish: 'Indhold af farlige stoffer'
  SubstanceContentMethod content_method = SubstanceContentMethod::assumed;

  /// @brief Which chemical substance was analyzed for
  /// @note Danish: 'Analyseret for kemiske stoffer'
  /// @note Open value list. Predefined values include: asbestos, chromium VI,
  ///       formaldehyde, lead, mercury, cadmium, PCB, PAH, pentachlorophenol,
  ///       chloroparaffins, phthalates, PFAS, dioxins, radon, VOC, etc.
  std::string analyzed_substance;

  /// @brief CAS registry number of the substance
  /// @note Danish: 'CAS-nummer'
  std::string cas_number;

  /// @brief EC (EINECS) number of the substance
  /// @note Danish: 'EC-nummer'
  std::string ec_number;

  /// @brief Concentration of substance in mg/kg
  /// @note Danish: 'Koncentration af stof'
  std::optional<double> concentration_mg_per_kg;
};

/** @brief Emission measurement record.
 *
 *  Nested within the Pollution section. Each entry documents one emission
 *  measurement.
 *  @note Danish: 'Afgasning af farlige stoffer'
 */
struct Emission {
  /// @brief Emission standard (ISO or other standard code)
  /// @note Danish: 'Emissionsstandard'
  /// @note Open value list. Predefined values include: ISO 16000-3,
  ///       ISO 16000-6, ISO 16000-9, EN 717-1, CEN/TS 16516, etc.
  std::string standard;

  /// @brief Type of emission measured
  /// @note Danish: 'Emissionstype'
  /// @note Open value list. Predefined values include: TVOC, formaldehyde,
  ///       ammonia, acetaldehyde, toluene, etc.
  std::string type;

  /// @brief Lower bound of emission interval
  /// @note Danish: 'Emission nedre interval'
  std::optional<double> lower_interval;

  /// @brief Upper bound of emission interval
  /// @note Danish: 'Emission øvre interval'
  std::optional<double> upper_interval;

  /// @brief How the emission quantity is expressed
  /// @note Danish: 'Emissionsmængdetype'
  EmissionQuantityType quantity_type = EmissionQuantityType::exact;

  /// @brief Measuring unit for the emission quantity
  /// @note Danish: 'Måleenhed'
  std::string measuring_unit;

  /// @brief Measured emission quantity
  /// @note Danish: 'Emissionsmængde'
  std::optional<double> quantity;
};

/** @brief Pollution, content and emission information.
 *  @note Danish section: 'Forurening - indhold og afgasning'
 */
struct Pollution {
  /// @brief Whether the item contains substances on REACH candidate list
  /// @note Danish: 'Indeholder stoffer på REACHs kandidatliste'
  TriState contains_reach_substances = TriState::unknown;

  /// @brief Whether the item has been chemically treated
  /// @note Danish: 'Er byggevaren kemisk behandlet'
  TriState is_chemically_treated = TriState::unknown;

  /// @brief Surface treatments applied to the item
  /// @note Danish: 'Overfladebehandling'
  /// @note Open value list. Predefined values include: anodized, brushed,
  ///       chromium-plated, coated, galvanized, glazed, impregnated, lacquered,
  ///       laminated, oiled, painted, patinated, polished, powder-coated,
  ///       varnished, waxed, other.
  std::vector<std::string> surface_treatments;

  /// @brief Dangerous substance analysis records
  /// @note Danish: 'Farlige stoffer'
  std::vector<DangerousSubstance> dangerous_substances;

  /// @brief Emission measurement records
  /// @note Danish: 'Afgasning af farlige stoffer'
  std::vector<Emission> emissions;

  /// @brief Whether the item is intended for indoor use
  /// @note Danish: 'Beregnet til indendørs brug'
  std::optional<bool> intended_for_indoor_use;

  /// @brief Labelling scheme for emissions
  /// @note Danish: 'Mærkningsordning'
  /// @note Closed value list. Examples: Danish Indoor Climate Labelling,
  ///       M1, Emicode, Blue Angel, GUT, EMICODE, etc.
  std::string labelling_scheme;

  /// @brief Emission level classification
  /// @note Danish: 'Emissionsniveau'
  /// @note Closed value list. Values depend on the labelling scheme.
  std::string emission_level;

  /// @brief Whether an asbestos analysis has been performed
  /// @note Danish: 'Har asbestanalyse'
  std::optional<bool> has_asbestos_analysis;
};

// ===========================================================================
// Section 8: Environmental and Resource Potential
// ===========================================================================

/** @brief Environmental and resource potential for sustainable utilization.
 *  @note Danish section: 'Miljø- og ressourcepotentiale'
 */
struct EnvironmentalPotential {
  /// @brief Whether a take-back scheme is available
  /// @note Danish: 'Tilbagetagningsordning tilgængelig'
  std::optional<bool> takeback_scheme_available;

  /// @brief Whether the item consists of naturally separate parts
  /// @note Danish: 'Byggevaren består naturligt af separate dele'
  std::optional<bool> consists_of_separate_parts;
};

// ===========================================================================
// Section 9: Other Essential Properties (Fire)
// ===========================================================================

/** @brief Fire-related properties of the construction item.
 *  @note Danish section: 'Øvrige væsentlige egenskaber'
 */
struct FireProperties {
  /// @brief Reaction to fire classification (Euroclass code)
  /// @note Danish: 'Reaktion ved brand'
  /// @note Closed value list of Euroclass codes. Examples: A1, A2-s1,d0,
  ///       B-s1,d0, C-s2,d0, D-s2,d0, E, F, etc.
  std::string reaction_to_fire;

  /// @brief Resistance to fire classification
  /// @note Danish: 'Modstandsevne ved brand'
  /// @note Closed value list of fire resistance codes. Examples: EI30, EI60,
  ///       REI30, REI60, REI90, REI120, R30, R60, etc.
  std::string resistance_to_fire;

  /// @brief Documentation of fire classification
  /// @note Danish: 'Dokumentation af brandklassifikation'
  /// @note Closed value list. Examples: test report, classification report,
  ///       EXAP (extended application), tabulated data, etc.
  std::string documentation_of_fire_classification;

  /// @brief Field of application in relation to fire
  /// @note Danish: 'Anvendelsesområde i relation til brand'
  std::string field_of_application;
};

// ===========================================================================
// Section 10: History
// ===========================================================================

/** @brief Historical usage information for the construction item.
 *  @note Danish section: 'Historik'
 */
struct History {
  /// @brief Previous usage environments
  /// @note Danish: 'Tidligere brugsmiljø'
  /// @note Open value list. Predefined values include: indoor, outdoor,
  ///       underground, marine, industrial, residential, commercial,
  ///       agricultural, etc.
  std::vector<std::string> previous_usage_environments;
};

} // namespace ReUseX::core
