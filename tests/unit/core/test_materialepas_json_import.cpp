// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/materialepas_json_export.hpp>
#include <core/materialepas_json_import.hpp>
#include <core/materialepas_types.hpp>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using namespace ReUseX::core;
using json = nlohmann::json;

// Helper: Create a fully-populated MaterialPassport for testing
static MaterialPassport createSamplePassport() {
  MaterialPassport passport;

  // Section 1: Owner
  passport.owner.contact_email = "owner@example.com";
  passport.owner.contact_name = "John Smith";
  passport.owner.company_name = "Green Construction Ltd";

  // Section 2: Construction Item Description
  passport.description.designation = "Recycled Steel Beam I-Profile 300x150";
  passport.description.images = {"beam_photo1.jpg", "beam_photo2.jpg"};
  passport.description.has_qr_code = true;
  passport.description.has_rfid_tag = false;
  passport.description.materials = {Material::steel,
                                    Material::painting_supplies};
  passport.description.assembly_methods = {"Welded", "Bolted"};
  passport.description.year_of_installation = 2010;
  passport.description.year_of_construction = 2010;

  // Section 3: Product Information
  passport.product.manufacturer = "ArcelorMittal";
  passport.product.gtin = "5901234123457";
  passport.product.product_name = "HEB 300 Steel Beam";
  passport.product.model_label = "HEB300-S355";
  passport.product.production_year = 2010;

  // Section 4: Certifications
  passport.certifications.has_epd = true;
  passport.certifications.epd_programme_operator = "EPD International";
  passport.certifications.epd_operator_web_domain =
      "https://www.environdec.com";
  passport.certifications.epd_registration_number = "S-P-00123";
  passport.certifications.reference_service_life = 50.0;
  passport.certifications.has_safety_data_sheet = true;
  passport.certifications.declaration_of_performance = {"DoP-12345.pdf"};
  passport.certifications.technical_documentation = {"technical_spec.pdf"};
  passport.certifications.non_destructive_tests = {
      "ultrasonic_test_report.pdf"};
  passport.certifications.assessed_period_of_use = 15.0;
  passport.certifications.avg_service_life_build = 75.0;
  passport.certifications.remaining_service_life_rsl = 60.0;
  passport.certifications.remaining_service_life_build = 60.0;

  // Section 5: Dimensions
  passport.dimensions.width_mm = 300.0;
  passport.dimensions.height_mm = 300.0;
  passport.dimensions.length_mm = 6000.0;
  passport.dimensions.thickness_mm = 15.0;
  passport.dimensions.weight_kg = 450.0;
  passport.dimensions.volume_m3 = 0.54;
  passport.dimensions.surface_area_m2 = 7.2;
  passport.dimensions.technical_drawing = "HEB300_drawing.pdf";

  // Section 6: Condition
  passport.condition.photo_documentation = {"condition_photo1.jpg"};
  passport.condition.visual_inspection_performed = true;
  passport.condition.has_signs_of_damage = false;
  passport.condition.is_deformed = false;
  passport.condition.is_scratched = true;
  passport.condition.is_surface_intact = true;
  passport.condition.has_intact_edges = true;
  passport.condition.has_signs_of_degradation = false;

  // Section 7: Pollution (with nested objects)
  passport.pollution.contains_reach_substances = TriState::no;
  passport.pollution.is_chemically_treated = TriState::yes;
  passport.pollution.surface_treatments = {"Primer coating",
                                           "Protective paint"};
  passport.pollution.intended_for_indoor_use = false;
  passport.pollution.labelling_scheme = "DGNoeB";
  passport.pollution.emission_level = "C";
  passport.pollution.has_asbestos_analysis = true;

  DangerousSubstance substance;
  substance.content_method = SubstanceContentMethod::measured_material;
  substance.analyzed_substance = "Lead (Pb)";
  substance.cas_number = "7439-92-1";
  substance.ec_number = "231-100-4";
  substance.concentration_mg_per_kg = 0.5;
  passport.pollution.dangerous_substances.push_back(substance);

  Emission emission;
  emission.standard = "EN ISO 16000";
  emission.type = "TVOC";
  emission.lower_interval = 0.0;
  emission.upper_interval = 0.05;
  emission.quantity_type = EmissionQuantityType::interval;
  emission.measuring_unit = "mg/m3";
  emission.quantity = 0.02;
  passport.pollution.emissions.push_back(emission);

  // Section 8: Environmental Potential
  passport.environmental.takeback_scheme_available = true;
  passport.environmental.consists_of_separate_parts = false;

  // Section 9: Fire Properties
  passport.fire.reaction_to_fire = "A1";
  passport.fire.resistance_to_fire = "R120";
  passport.fire.documentation_of_fire_classification = "fire_cert.pdf";
  passport.fire.field_of_application = "Load-bearing structures";

  // Section 10: History
  passport.history.previous_usage_environments = {
      "Industrial warehouse - 2010-2025"};

  // Metadata
  passport.metadata.document_guid = "test-guid-12345";
  passport.metadata.creation_date = "2025-03-25T10:30:00Z";
  passport.metadata.revision_date = "2025-03-25T10:30:00Z";
  passport.metadata.version_number = "1.0.0";
  passport.metadata.version_date = "2025-03-25T10:30:00Z";

  // Transaction log
  TransactionLogEntry log_entry;
  log_entry.type = TransactionType::document;
  log_entry.guid = passport.metadata.document_guid;
  log_entry.edited_by = "test_user";
  log_entry.edited_date = "2025-03-25T10:30:00Z";
  log_entry.old_value = "";
  log_entry.new_value = "Created";
  passport.transaction_log.push_back(log_entry);

  return passport;
}

// ===========================================================================
// Test Cases
// ===========================================================================

TEST_CASE("Round-trip: export then import preserves all fields",
          "[core][json][import]") {
  auto original = createSamplePassport();
  auto j = json_export::to_json(original);
  auto imported = json_import::from_json(j);

  // Owner
  CHECK(imported.owner.contact_email == original.owner.contact_email);
  CHECK(imported.owner.contact_name == original.owner.contact_name);
  CHECK(imported.owner.company_name == original.owner.company_name);

  // Construction Item Description
  CHECK(imported.description.designation == original.description.designation);
  CHECK(imported.description.images == original.description.images);
  CHECK(imported.description.has_qr_code == original.description.has_qr_code);
  CHECK(imported.description.has_rfid_tag ==
        original.description.has_rfid_tag);
  CHECK(imported.description.materials == original.description.materials);
  CHECK(imported.description.assembly_methods ==
        original.description.assembly_methods);
  CHECK(imported.description.year_of_installation ==
        original.description.year_of_installation);
  CHECK(imported.description.year_of_construction ==
        original.description.year_of_construction);

  // Product Information
  CHECK(imported.product.manufacturer == original.product.manufacturer);
  CHECK(imported.product.gtin == original.product.gtin);
  CHECK(imported.product.product_name == original.product.product_name);
  CHECK(imported.product.model_label == original.product.model_label);
  CHECK(imported.product.production_year == original.product.production_year);

  // Certifications
  CHECK(imported.certifications.has_epd == original.certifications.has_epd);
  CHECK(imported.certifications.epd_programme_operator ==
        original.certifications.epd_programme_operator);
  CHECK(imported.certifications.epd_operator_web_domain ==
        original.certifications.epd_operator_web_domain);
  CHECK(imported.certifications.epd_registration_number ==
        original.certifications.epd_registration_number);
  CHECK(imported.certifications.reference_service_life ==
        original.certifications.reference_service_life);
  CHECK(imported.certifications.has_safety_data_sheet ==
        original.certifications.has_safety_data_sheet);
  CHECK(imported.certifications.declaration_of_performance ==
        original.certifications.declaration_of_performance);
  CHECK(imported.certifications.technical_documentation ==
        original.certifications.technical_documentation);
  CHECK(imported.certifications.non_destructive_tests ==
        original.certifications.non_destructive_tests);
  CHECK(imported.certifications.assessed_period_of_use ==
        original.certifications.assessed_period_of_use);
  CHECK(imported.certifications.avg_service_life_build ==
        original.certifications.avg_service_life_build);
  CHECK(imported.certifications.remaining_service_life_rsl ==
        original.certifications.remaining_service_life_rsl);
  CHECK(imported.certifications.remaining_service_life_build ==
        original.certifications.remaining_service_life_build);

  // Dimensions
  CHECK(imported.dimensions.width_mm == original.dimensions.width_mm);
  CHECK(imported.dimensions.height_mm == original.dimensions.height_mm);
  CHECK(imported.dimensions.length_mm == original.dimensions.length_mm);
  CHECK(imported.dimensions.thickness_mm == original.dimensions.thickness_mm);
  CHECK(imported.dimensions.weight_kg == original.dimensions.weight_kg);
  CHECK(imported.dimensions.volume_m3 == original.dimensions.volume_m3);
  CHECK(imported.dimensions.surface_area_m2 ==
        original.dimensions.surface_area_m2);
  CHECK(imported.dimensions.technical_drawing ==
        original.dimensions.technical_drawing);

  // Condition
  CHECK(imported.condition.photo_documentation ==
        original.condition.photo_documentation);
  CHECK(imported.condition.visual_inspection_performed ==
        original.condition.visual_inspection_performed);
  CHECK(imported.condition.has_signs_of_damage ==
        original.condition.has_signs_of_damage);
  CHECK(imported.condition.is_deformed == original.condition.is_deformed);
  CHECK(imported.condition.is_scratched == original.condition.is_scratched);
  CHECK(imported.condition.is_surface_intact ==
        original.condition.is_surface_intact);
  CHECK(imported.condition.has_intact_edges ==
        original.condition.has_intact_edges);
  CHECK(imported.condition.has_signs_of_degradation ==
        original.condition.has_signs_of_degradation);

  // Pollution (simple fields)
  CHECK(imported.pollution.contains_reach_substances ==
        original.pollution.contains_reach_substances);
  CHECK(imported.pollution.is_chemically_treated ==
        original.pollution.is_chemically_treated);
  CHECK(imported.pollution.surface_treatments ==
        original.pollution.surface_treatments);
  CHECK(imported.pollution.intended_for_indoor_use ==
        original.pollution.intended_for_indoor_use);
  CHECK(imported.pollution.labelling_scheme ==
        original.pollution.labelling_scheme);
  CHECK(imported.pollution.emission_level ==
        original.pollution.emission_level);
  CHECK(imported.pollution.has_asbestos_analysis ==
        original.pollution.has_asbestos_analysis);

  // Pollution: DangerousSubstance
  REQUIRE(imported.pollution.dangerous_substances.size() ==
          original.pollution.dangerous_substances.size());
  CHECK(imported.pollution.dangerous_substances[0].content_method ==
        original.pollution.dangerous_substances[0].content_method);
  CHECK(imported.pollution.dangerous_substances[0].analyzed_substance ==
        original.pollution.dangerous_substances[0].analyzed_substance);
  CHECK(imported.pollution.dangerous_substances[0].cas_number ==
        original.pollution.dangerous_substances[0].cas_number);
  CHECK(imported.pollution.dangerous_substances[0].ec_number ==
        original.pollution.dangerous_substances[0].ec_number);
  CHECK(imported.pollution.dangerous_substances[0].concentration_mg_per_kg ==
        original.pollution.dangerous_substances[0].concentration_mg_per_kg);

  // Pollution: Emission
  REQUIRE(imported.pollution.emissions.size() ==
          original.pollution.emissions.size());
  CHECK(imported.pollution.emissions[0].standard ==
        original.pollution.emissions[0].standard);
  CHECK(imported.pollution.emissions[0].type ==
        original.pollution.emissions[0].type);
  CHECK(imported.pollution.emissions[0].lower_interval ==
        original.pollution.emissions[0].lower_interval);
  CHECK(imported.pollution.emissions[0].upper_interval ==
        original.pollution.emissions[0].upper_interval);
  CHECK(imported.pollution.emissions[0].quantity_type ==
        original.pollution.emissions[0].quantity_type);
  CHECK(imported.pollution.emissions[0].measuring_unit ==
        original.pollution.emissions[0].measuring_unit);
  // Note: quantity has empty json_name, so it's not exported/imported

  // Environmental Potential
  CHECK(imported.environmental.takeback_scheme_available ==
        original.environmental.takeback_scheme_available);
  CHECK(imported.environmental.consists_of_separate_parts ==
        original.environmental.consists_of_separate_parts);

  // Fire Properties
  CHECK(imported.fire.reaction_to_fire == original.fire.reaction_to_fire);
  CHECK(imported.fire.resistance_to_fire == original.fire.resistance_to_fire);
  CHECK(imported.fire.documentation_of_fire_classification ==
        original.fire.documentation_of_fire_classification);
  CHECK(imported.fire.field_of_application ==
        original.fire.field_of_application);

  // History
  CHECK(imported.history.previous_usage_environments ==
        original.history.previous_usage_environments);

  // Metadata
  CHECK(imported.metadata.document_guid == original.metadata.document_guid);
  CHECK(imported.metadata.creation_date == original.metadata.creation_date);
  CHECK(imported.metadata.revision_date == original.metadata.revision_date);
  CHECK(imported.metadata.version_number == original.metadata.version_number);
  CHECK(imported.metadata.version_date == original.metadata.version_date);

  // Transaction log
  REQUIRE(imported.transaction_log.size() ==
          original.transaction_log.size());
  CHECK(imported.transaction_log[0].type ==
        original.transaction_log[0].type);
  CHECK(imported.transaction_log[0].guid ==
        original.transaction_log[0].guid);
  CHECK(imported.transaction_log[0].edited_by ==
        original.transaction_log[0].edited_by);
  CHECK(imported.transaction_log[0].edited_date ==
        original.transaction_log[0].edited_date);
  CHECK(imported.transaction_log[0].old_value ==
        original.transaction_log[0].old_value);
  CHECK(imported.transaction_log[0].new_value ==
        original.transaction_log[0].new_value);
}

TEST_CASE("Empty passport round-trip preserves defaults",
          "[core][json][import]") {
  MaterialPassport empty;
  auto j = json_export::to_json(empty);
  auto imported = json_import::from_json(j);

  // Strings default to empty
  CHECK(imported.owner.contact_email.empty());
  CHECK(imported.owner.contact_name.empty());

  // Optionals default to nullopt
  CHECK_FALSE(imported.description.has_qr_code.has_value());
  CHECK_FALSE(imported.description.year_of_installation.has_value());
  CHECK_FALSE(imported.product.production_year.has_value());

  // TriState defaults to unknown
  CHECK(imported.pollution.contains_reach_substances == TriState::unknown);
  CHECK(imported.pollution.is_chemically_treated == TriState::unknown);

  // Vectors are empty
  CHECK(imported.description.images.empty());
  CHECK(imported.description.materials.empty());
  CHECK(imported.pollution.dangerous_substances.empty());
  CHECK(imported.pollution.emissions.empty());

  // Empty metadata
  CHECK(imported.metadata.document_guid.empty());
  CHECK(imported.metadata.version_number.empty());

  // Empty log
  CHECK(imported.transaction_log.empty());
}

TEST_CASE("Multiple passports round-trip via from_json_array",
          "[core][json][import]") {
  MaterialPassport p1 = createSamplePassport();
  p1.metadata.document_guid = "guid-001";
  p1.owner.contact_name = "Alice";

  MaterialPassport p2 = createSamplePassport();
  p2.metadata.document_guid = "guid-002";
  p2.owner.contact_name = "Bob";

  MaterialPassport p3 = createSamplePassport();
  p3.metadata.document_guid = "guid-003";
  p3.owner.contact_name = "Charlie";

  std::vector<MaterialPassport> originals = {p1, p2, p3};
  auto j = json_export::to_json(originals);
  auto imported = json_import::from_json_array(j);

  REQUIRE(imported.size() == 3);
  CHECK(imported[0].metadata.document_guid == "guid-001");
  CHECK(imported[1].metadata.document_guid == "guid-002");
  CHECK(imported[2].metadata.document_guid == "guid-003");
  CHECK(imported[0].owner.contact_name == "Alice");
  CHECK(imported[1].owner.contact_name == "Bob");
  CHECK(imported[2].owner.contact_name == "Charlie");
}

TEST_CASE("from_json_string auto-detects single vs array",
          "[core][json][import]") {
  auto passport = createSamplePassport();

  SECTION("Single object") {
    auto json_str = json_export::to_json_string(passport);
    auto imported = json_import::from_json_string(json_str);
    REQUIRE(imported.size() == 1);
    CHECK(imported[0].owner.contact_email == "owner@example.com");
  }

  SECTION("Array of objects") {
    std::vector<MaterialPassport> passports = {passport, passport};
    auto json_str = json_export::to_json_string(passports);
    auto imported = json_import::from_json_string(json_str);
    REQUIRE(imported.size() == 2);
  }
}

TEST_CASE("Multiple DangerousSubstance entries round-trip correctly",
          "[core][json][import]") {
  MaterialPassport passport;

  DangerousSubstance sub1;
  sub1.content_method = SubstanceContentMethod::measured_material;
  sub1.analyzed_substance = "Lead (Pb)";
  sub1.cas_number = "7439-92-1";
  sub1.ec_number = "231-100-4";
  sub1.concentration_mg_per_kg = 0.5;

  DangerousSubstance sub2;
  sub2.content_method = SubstanceContentMethod::assumed;
  sub2.analyzed_substance = "Asbestos";
  sub2.cas_number = "1332-21-4";
  sub2.ec_number = "215-481-4";
  sub2.concentration_mg_per_kg = 100.0;

  passport.pollution.dangerous_substances = {sub1, sub2};

  auto j = json_export::to_json(passport);
  auto imported = json_import::from_json(j);

  REQUIRE(imported.pollution.dangerous_substances.size() == 2);

  CHECK(imported.pollution.dangerous_substances[0].content_method ==
        SubstanceContentMethod::measured_material);
  CHECK(imported.pollution.dangerous_substances[0].analyzed_substance ==
        "Lead (Pb)");
  CHECK(imported.pollution.dangerous_substances[0].cas_number == "7439-92-1");

  CHECK(imported.pollution.dangerous_substances[1].content_method ==
        SubstanceContentMethod::assumed);
  CHECK(imported.pollution.dangerous_substances[1].analyzed_substance ==
        "Asbestos");
  CHECK(imported.pollution.dangerous_substances[1].cas_number == "1332-21-4");
  CHECK(imported.pollution.dangerous_substances[1].concentration_mg_per_kg ==
        100.0);
}

TEST_CASE("Missing sections key produces defaults with metadata parsed",
          "[core][json][import]") {
  // JSON with empty sections array but valid metadata
  json j;
  j["sections"] = json::array();
  j["log"] = json::array();
  j["metadata"] = {{"document guid", "meta-guid"},
                    {"document creation date", "2025-01-01"},
                    {"document revision date", "2025-01-02"},
                    {"version number", "2.0"},
                    {"version date", "2025-01-03"}};

  auto imported = json_import::from_json(j);

  // All section fields should be defaults
  CHECK(imported.owner.contact_email.empty());
  CHECK_FALSE(imported.description.has_qr_code.has_value());
  CHECK(imported.pollution.contains_reach_substances == TriState::unknown);

  // Metadata should be parsed
  CHECK(imported.metadata.document_guid == "meta-guid");
  CHECK(imported.metadata.creation_date == "2025-01-01");
  CHECK(imported.metadata.revision_date == "2025-01-02");
  CHECK(imported.metadata.version_number == "2.0");
  CHECK(imported.metadata.version_date == "2025-01-03");
}

TEST_CASE("Invalid JSON throws parse_error", "[core][json][import]") {
  REQUIRE_THROWS_AS(json_import::from_json_string("not valid json {{{"),
                    nlohmann::json::parse_error);
}

TEST_CASE("Missing required sections key throws runtime_error",
          "[core][json][import]") {
  json j;
  j["log"] = json::array();
  j["metadata"] = json::object();
  // No "sections" key

  REQUIRE_THROWS_AS(json_import::from_json(j), std::runtime_error);
}

TEST_CASE("JSON string round-trip produces identical re-export",
          "[core][json][import]") {
  auto original = createSamplePassport();
  auto json_str1 = json_export::to_json_string(original);

  auto imported = json_import::from_json_string(json_str1);
  REQUIRE(imported.size() == 1);

  auto json_str2 = json_export::to_json_string(imported[0]);

  // Parse both and compare as JSON objects for structural equality
  auto j1 = json::parse(json_str1);
  auto j2 = json::parse(json_str2);
  CHECK(j1 == j2);
}
