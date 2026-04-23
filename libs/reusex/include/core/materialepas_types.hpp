// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "materialepas_enums.hpp"

#include <optional>
#include <string>
#include <vector>

namespace reusex::core {

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
  std::string leksikon_guid_contact_email = "0Bwj05D$55V931bq9VaBE5";

  /// @brief Contact name of the material passport owner
  /// @note Danish: 'Materialepas ejer kontaktnavn'
  std::string contact_name;
  std::string leksikon_guid_contact_name = "17BdeQk152gR5YUt5oSLfp";

  /// @brief Company name of the material passport owner
  /// @note Danish: 'Materialepas ejer virksomhedsnavn'
  std::string company_name;
  std::string leksikon_guid_company_name = "1bb51PIefCe9uHGCmR0gFJ";
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
  std::string leksikon_guid_designation = "2_mYRkA$9EcwXAxqBGoMrM";

  /// @brief Images of the construction item (URLs or base64)
  /// @note Danish: 'Byggevarebillede'
  std::vector<std::string> images;
  std::string leksikon_guid_images = "1dvQfmc2z9lB_rwknYkM3y";

  /// @brief Whether the item has a QR code
  /// @note Danish: 'Har QR-kode'
  std::optional<bool> has_qr_code;
  std::string leksikon_guid_has_qr_code = "06moAxe0j7EutROb0dD3B7";

  /// @brief Whether the item has an RFID tag
  /// @note Danish: 'Har RFID-tag'
  std::optional<bool> has_rfid_tag;
  std::string leksikon_guid_has_rfid_tag = "2$Owq1iHrAuw$ASWOXAk9p";

  /// @brief Materials the construction item is made of (closed value list)
  /// @note Danish: 'Materiale'
  std::vector<Material> materials;
  std::string leksikon_guid_materials = "1R_z75Gd52OQYxVkJ$E3ZU";

  /// @brief Assembly methods used for the construction item
  /// @note Danish: 'Sammenføjningsmetode'
  /// @note Open value list. Predefined values include: bolted, glued, grouted,
  ///       hung, keyed, laid, mortared, nailed, pressed, riveted, screwed,
  ///       soldered, stacked, suspended, wedged, welded, wound, other.
  std::vector<std::string> assembly_methods;
  std::string leksikon_guid_assembly_methods = "0N7Qwopp125xrULezqArw4";

  /// @brief Year the item was installed in the building
  /// @note Danish: 'Installationsår'
  std::optional<int> year_of_installation;
  std::string leksikon_guid_year_of_installation = "294ZkUyYn6TfdwCsbo2X63";

  /// @brief Year of construction of the building
  /// @note Danish: 'Bygningens opførelsesår'
  std::optional<int> year_of_construction;
  std::string leksikon_guid_year_of_construction = "17LFECVML6buNXwBLz416Y";
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
  std::string leksikon_guid_manufacturer = "1QwFy1F3T4dPe2N2cbsuRv";

  /// @brief Global Trade Item Number (GTIN / EAN barcode)
  /// @note Danish: 'Global Trade Item Number (GTIN)'
  std::string gtin;
  std::string leksikon_guid_gtin = "1rQ9Z5vy5AygUSncyWuhXr";

  /// @brief Product name
  /// @note Danish: 'Produktnavn'
  std::string product_name;
  std::string leksikon_guid_product_name = "198tMTFIX5PecEfAwMEIeo";

  /// @brief Model label / product identifier
  /// @note Danish: 'Modelbetegnelse'
  std::string model_label;
  std::string leksikon_guid_model_label = "06t2gkj49FNvMebmltrHXV";

  /// @brief Year the product was manufactured
  /// @note Danish: 'Produktionsår'
  std::optional<int> production_year;
  std::string leksikon_guid_production_year = "3nif2XJKH8r9I09LfEYaRm";
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
  std::string leksikon_guid_has_epd = "2_jn1$mEr1780H5QOiEj6O";

  /// @brief EPD programme operator name
  /// @note Danish: 'EPD programoperatør'
  /// @note Closed value list (22 operators). Examples: EPD International,
  ///       IBU, BRE, INIES, EPD Norge, EPD Danmark, etc.
  std::string epd_programme_operator;
  std::string leksikon_guid_epd_programme_operator = "0ORiYBKRnCFQzUwOsNqbYn";

  /// @brief EPD programme operator web domain (URL)
  /// @note Danish: 'EPD programoperatør webdomæne'
  std::string epd_operator_web_domain;
  std::string leksikon_guid_epd_operator_web_domain = "2kqTixLBr9sBqGKYs9lNSI";

  /// @brief EPD registration number
  /// @note Danish: 'EPD registreringsnummer'
  std::string epd_registration_number;
  std::string leksikon_guid_epd_registration_number = "2aCraAVB91qw8CzrM6x3ia";

  /// @brief Reference service life in years
  /// @note Danish: 'Referencebrugstid'
  std::optional<double> reference_service_life;
  std::string leksikon_guid_reference_service_life = "28mSsPe_z69BksBt5oRXMl";

  /// @brief Whether a safety data sheet exists
  /// @note Danish: 'Har sikkerhedsdatablad'
  std::optional<bool> has_safety_data_sheet;
  std::string leksikon_guid_has_safety_data_sheet = "3D1KFVwbDBuumnIBZZHQId";

  /// @brief Declaration of performance references
  /// @note Danish: 'Ydeevnedeklaration'
  /// @note Open value list of document references.
  std::vector<std::string> declaration_of_performance;
  std::string leksikon_guid_declaration_of_performance = "1tqCmrJwn3yvF$k2lU2fFa";

  /// @brief Technical documentation references
  /// @note Danish: 'Teknisk dokumentation'
  /// @note Open value list of document references.
  std::vector<std::string> technical_documentation;
  std::string leksikon_guid_technical_documentation = "1IfrbnqGv5Pu3GBuocLZgi";

  /// @brief Non-destructive test results
  /// @note Danish: 'Ikke-destruktive prøvninger'
  /// @note Open value list of test references.
  std::vector<std::string> non_destructive_tests;
  std::string leksikon_guid_non_destructive_tests = "0aTBceworF$QTXM6CqyOdV";

  /// @brief Assessed period of use in years
  /// @note Danish: 'Vurderet brugsperiode'
  std::optional<double> assessed_period_of_use;
  std::string leksikon_guid_assessed_period_of_use = "3xW23JRMTAzOQb79uZW3qC";

  /// @brief Average service life according to BUILD methodology (years)
  /// @note Danish: 'Gennemsnitlig levetid (BUILD)'
  std::optional<double> avg_service_life_build;
  std::string leksikon_guid_avg_service_life_build = "2TtVRWo_56mBphAcSiFIHG";

  /// @brief Remaining service life (RSL) in years
  /// @note Danish: 'Restlevetid (RSL)'
  std::optional<double> remaining_service_life_rsl;
  std::string leksikon_guid_remaining_service_life_rsl = "2id4UZ8ynAF9DEzV0NC_wT";

  /// @brief Remaining service life according to BUILD methodology (years)
  /// @note Danish: 'Restlevetid (BUILD)'
  std::optional<double> remaining_service_life_build;
  std::string leksikon_guid_remaining_service_life_build = "27gXAA5Iz7lgcOytMRqOBv";
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
  std::string leksikon_guid_width_mm = "3NHBUedX9438Hi3mwD15$Z";

  /// @brief Height in millimeters
  /// @note Danish: 'Højde'
  std::optional<double> height_mm;
  std::string leksikon_guid_height_mm = "2G$wMhUvL2LgKNY0J66oAT";

  /// @brief Length in millimeters
  /// @note Danish: 'Længde'
  std::optional<double> length_mm;
  std::string leksikon_guid_length_mm = "1uUn3YWZfBaPqYQMo_$Om$";

  /// @brief Thickness in millimeters
  /// @note Danish: 'Tykkelse'
  std::optional<double> thickness_mm;
  std::string leksikon_guid_thickness_mm = "0SyXPZ9an9vh49k$Lgkvly";

  /// @brief Depth in millimeters
  /// @note Danish: 'Dybde'
  std::optional<double> depth_mm;
  std::string leksikon_guid_depth_mm = "1$OV5du3LFWwa$P7SfJhAr";

  /// @brief Volume in cubic meters
  /// @note Danish: 'Volumen'
  std::optional<double> volume_m3;
  std::string leksikon_guid_volume_m3 = "3IHRMkfs9EDPXVvPJ0hbFn";

  /// @brief Surface area of product in square meters
  /// @note Danish: 'Overfladeareal'
  std::optional<double> surface_area_m2;
  std::string leksikon_guid_surface_area_m2 = "3Cs4d96Ff1nPGHWlsTyYIS";

  /// @brief Inner diameter in millimeters
  /// @note Danish: 'Indvendig diameter'
  std::optional<double> inner_diameter_mm;
  std::string leksikon_guid_inner_diameter_mm = "0ayREcz2zBuPWmCDTem2a8";

  /// @brief Outer diameter in millimeters
  /// @note Danish: 'Udvendig diameter'
  std::optional<double> outer_diameter_mm;
  std::string leksikon_guid_outer_diameter_mm = "3BWwvo0lf3iAd_34y7ac$4";

  /// @brief Weight in kilograms
  /// @note Danish: 'Vægt'
  std::optional<double> weight_kg;
  std::string leksikon_guid_weight_kg = "1XyLvnxf94pwJW6LWaD6Zw";

  /// @brief Technical drawing (image URL or base64)
  /// @note Danish: 'Teknisk tegning'
  std::string technical_drawing;
  std::string leksikon_guid_technical_drawing = "2d01jZqvP7GOR3V$coAprV";
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
  std::string leksikon_guid_photo_documentation = "3$DH1Zm_r9_gzqdwKK39Ua";

  /// @brief Whether a visual inspection has been performed
  /// @note Danish: 'Visuel inspektion udført'
  std::optional<bool> visual_inspection_performed;
  std::string leksikon_guid_visual_inspection_performed = "2jB20LC_nEzBN4_j2ypip8";

  /// @brief Whether the item has signs of damage
  /// @note Danish: 'Har tegn på skade'
  std::optional<bool> has_signs_of_damage;
  std::string leksikon_guid_has_signs_of_damage = "0rQimGJFHEueqtqwbwBaC5";

  /// @brief Whether the item is deformed
  /// @note Danish: 'Er deformeret'
  std::optional<bool> is_deformed;
  std::string leksikon_guid_is_deformed = "1g6mt3SrvDlho6YqBIu1lW";

  /// @brief Whether the item is scratched
  /// @note Danish: 'Er ridset'
  std::optional<bool> is_scratched;
  std::string leksikon_guid_is_scratched = "365luGupDCfwgtD32gGGrS";

  /// @brief Whether the surface is intact
  /// @note Danish: 'Er overfladen intakt'
  std::optional<bool> is_surface_intact;
  std::string leksikon_guid_is_surface_intact = "2GO932BuX5EvXOecmhbbl3";

  /// @brief Whether edges are intact
  /// @note Danish: 'Har intakte kanter'
  std::optional<bool> has_intact_edges;
  std::string leksikon_guid_has_intact_edges = "0AoBmqQ9v1pw7ca1VVp_Za";

  /// @brief Whether the item has signs of degradation
  /// @note Danish: 'Har tegn på nedbrydning'
  std::optional<bool> has_signs_of_degradation;
  std::string leksikon_guid_has_signs_of_degradation = "01lVHQF$XAneE8CFtajgr_";
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
  std::string leksikon_guid_content_method = "2ympiI4PL6c8ooh9Agqnz4";

  /// @brief Which chemical substance was analyzed for
  /// @note Danish: 'Analyseret for kemiske stoffer'
  /// @note Open value list. Predefined values include: asbestos, chromium VI,
  ///       formaldehyde, lead, mercury, cadmium, PCB, PAH, pentachlorophenol,
  ///       chloroparaffins, phthalates, PFAS, dioxins, radon, VOC, etc.
  std::string analyzed_substance;
  std::string leksikon_guid_analyzed_substance = "3Y2oTiV9r3NfoZQBVP_Bpx";

  /// @brief CAS registry number of the substance
  /// @note Danish: 'CAS-nummer'
  std::string cas_number;
  std::string leksikon_guid_cas_number = "2tZYDby2H2PADWYcTMn50O";

  /// @brief EC (EINECS) number of the substance
  /// @note Danish: 'EC-nummer'
  std::string ec_number;
  std::string leksikon_guid_ec_number = "1cV5alHQ93Zu44Uv4Iuqoj";

  /// @brief Concentration of substance in mg/kg
  /// @note Danish: 'Koncentration af stof'
  std::optional<double> concentration_mg_per_kg;
  std::string leksikon_guid_concentration_mg_per_kg = "0yw_0Ggsf6ihdShOA1niRl";
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
  std::string leksikon_guid_standard = "1Kx6G$uIXEZO3Br$MEsdaB";

  /// @brief Type of emission measured
  /// @note Danish: 'Emissionstype'
  /// @note Open value list. Predefined values include: TVOC, formaldehyde,
  ///       ammonia, acetaldehyde, toluene, etc.
  std::string type;
  std::string leksikon_guid_type = "12ZQ3obYHB1OhXiyy3LcfK";

  /// @brief Lower bound of emission interval
  /// @note Danish: 'Emission nedre interval'
  std::optional<double> lower_interval;
  std::string leksikon_guid_lower_interval = "1dSnTNTGD5YQPcgcbzMJaD";

  /// @brief Upper bound of emission interval
  /// @note Danish: 'Emission øvre interval'
  std::optional<double> upper_interval;
  std::string leksikon_guid_upper_interval = "3YTotdivHEeQS4ij2Xa7PU";

  /// @brief How the emission quantity is expressed
  /// @note Danish: 'Emissionsmængdetype'
  EmissionQuantityType quantity_type = EmissionQuantityType::exact;
  std::string leksikon_guid_quantity_type = "33HQrsPDn9uhOu$7D3HhWU";

  /// @brief Measuring unit for the emission quantity
  /// @note Danish: 'Måleenhed'
  std::string measuring_unit;
  std::string leksikon_guid_measuring_unit = "3d1HQUJVb23ReixUzvSyvS";

  /// @brief Measured emission quantity
  /// @note Danish: 'Emissionsmængde'
  std::optional<double> quantity;
  std::string leksikon_guid_quantity = "0lZ6gflK18WBte7dMyQQ$h";
};

/** @brief Pollution, content and emission information.
 *  @note Danish section: 'Forurening - indhold og afgasning'
 */
struct Pollution {
  /// @brief Whether the item contains substances on REACH candidate list
  /// @note Danish: 'Indeholder stoffer på REACHs kandidatliste'
  TriState contains_reach_substances = TriState::unknown;
  std::string leksikon_guid_contains_reach_substances = "2E6_MQ4Cn36f0s6CVYxQOR";

  /// @brief Whether the item has been chemically treated
  /// @note Danish: 'Er byggevaren kemisk behandlet'
  TriState is_chemically_treated = TriState::unknown;
  std::string leksikon_guid_is_chemically_treated = "1Cf0jFllr2HAk8FOmT2VGr";

  /// @brief Surface treatments applied to the item
  /// @note Danish: 'Overfladebehandling'
  /// @note Open value list. Predefined values include: anodized, brushed,
  ///       chromium-plated, coated, galvanized, glazed, impregnated, lacquered,
  ///       laminated, oiled, painted, patinated, polished, powder-coated,
  ///       varnished, waxed, other.
  std::vector<std::string> surface_treatments;
  std::string leksikon_guid_surface_treatments = "1zV5ZyYKT6CPWjKdTEZ1K0";

  /// @brief Dangerous substance analysis records
  /// @note Danish: 'Farlige stoffer'
  std::vector<DangerousSubstance> dangerous_substances;

  /// @brief Emission measurement records
  /// @note Danish: 'Afgasning af farlige stoffer'
  std::vector<Emission> emissions;

  /// @brief Whether the item is intended for indoor use
  /// @note Danish: 'Beregnet til indendørs brug'
  std::optional<bool> intended_for_indoor_use;
  std::string leksikon_guid_intended_for_indoor_use = "2W1zl8TFb459xOVytRmX3J";

  /// @brief Labelling scheme for emissions
  /// @note Danish: 'Mærkningsordning'
  /// @note Closed value list. Examples: Danish Indoor Climate Labelling,
  ///       M1, Emicode, Blue Angel, GUT, EMICODE, etc.
  std::string labelling_scheme;
  std::string leksikon_guid_labelling_scheme = "3Txv$QA7zCp8azbWeuubx2";

  /// @brief Emission level classification
  /// @note Danish: 'Emissionsniveau'
  /// @note Closed value list. Values depend on the labelling scheme.
  std::string emission_level;
  std::string leksikon_guid_emission_level = "3areU4Rpv9gfbeDNE9eBgA";

  /// @brief Whether an asbestos analysis has been performed
  /// @note Danish: 'Har asbestanalyse'
  std::optional<bool> has_asbestos_analysis;
  std::string leksikon_guid_has_asbestos_analysis = "2962p$Lwj68Br6satSNLAn";
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
  std::string leksikon_guid_takeback_scheme_available = "2__I$nkL94YRI_YCBy0ou8";

  /// @brief Whether the item consists of naturally separate parts
  /// @note Danish: 'Byggevaren består naturligt af separate dele'
  std::optional<bool> consists_of_separate_parts;
  std::string leksikon_guid_consists_of_separate_parts = "2nZqcaXkz7YPHhKI$FMowg";
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
  std::string leksikon_guid_reaction_to_fire = "2bipZ4JrzDnwNZZHkwfk8O";

  /// @brief Resistance to fire classification
  /// @note Danish: 'Modstandsevne ved brand'
  /// @note Closed value list of fire resistance codes. Examples: EI30, EI60,
  ///       REI30, REI60, REI90, REI120, R30, R60, etc.
  std::string resistance_to_fire;
  std::string leksikon_guid_resistance_to_fire = "0NlhENUQr4YA5$p0zBrXSp";

  /// @brief Documentation of fire classification
  /// @note Danish: 'Dokumentation af brandklassifikation'
  /// @note Closed value list. Examples: test report, classification report,
  ///       EXAP (extended application), tabulated data, etc.
  std::string documentation_of_fire_classification;
  std::string leksikon_guid_documentation_of_fire_classification = "1DdkDjj1H7POQtnzDtu7CC";

  /// @brief Field of application in relation to fire
  /// @note Danish: 'Anvendelsesområde i relation til brand'
  std::string field_of_application;
  std::string leksikon_guid_field_of_application = "37sNiQNwHC399AW2B60MrY";
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
  std::string leksikon_guid_previous_usage_environments = "3BiwSSLEz9l8YLgoWcixHq";
};

} // namespace reusex::core
