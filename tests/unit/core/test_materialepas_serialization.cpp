// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <core/MaterialPassport.hpp>
#include <core/ProjectDB.hpp>
#include <core/materialepas_types.hpp>

#include <filesystem>
#include <string>

using namespace reusex::core;
using namespace Catch::Matchers;

// Helper function: Create a fully-populated MaterialPassport for testing
MaterialPassport createSamplePassport() {
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
    passport.description.materials = {Material::steel, Material::painting_supplies};
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
    passport.certifications.epd_operator_web_domain = "https://www.environdec.com";
    passport.certifications.epd_registration_number = "S-P-00123";
    passport.certifications.reference_service_life = 50.0;
    passport.certifications.has_safety_data_sheet = true;
    passport.certifications.declaration_of_performance = {"DoP-12345.pdf"};
    passport.certifications.technical_documentation = {"technical_spec.pdf"};
    passport.certifications.non_destructive_tests = {"ultrasonic_test_report.pdf"};
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
    passport.pollution.surface_treatments = {"Primer coating", "Protective paint"};
    passport.pollution.intended_for_indoor_use = false;
    passport.pollution.labelling_scheme = "DGNøB";
    passport.pollution.emission_level = "C";
    passport.pollution.has_asbestos_analysis = true;

    // Nested: DangerousSubstance
    DangerousSubstance substance;
    substance.content_method = SubstanceContentMethod::measured_material;
    substance.analyzed_substance = "Lead (Pb)";
    substance.cas_number = "7439-92-1";
    substance.ec_number = "231-100-4";
    substance.concentration_mg_per_kg = 0.5;
    passport.pollution.dangerous_substances.push_back(substance);

    // Nested: Emission
    Emission emission;
    emission.standard = "EN ISO 16000";
    emission.type = "TVOC";
    emission.lower_interval = 0.0;
    emission.upper_interval = 0.05;
    emission.quantity_type = EmissionQuantityType::interval;
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
        "Industrial warehouse - 2010-2025"
    };

    // Metadata
    passport.metadata.document_guid = ""; // Set by caller
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

// Helper function: Deep comparison of two MaterialPassport objects
void verifyPassportEquality(const MaterialPassport& original,
                           const MaterialPassport& retrieved) {
    // Section 1: Owner
    REQUIRE(retrieved.owner.contact_email == original.owner.contact_email);
    REQUIRE(retrieved.owner.contact_name == original.owner.contact_name);
    REQUIRE(retrieved.owner.company_name == original.owner.company_name);

    // Section 2: Construction Item Description
    REQUIRE(retrieved.description.designation == original.description.designation);
    REQUIRE(retrieved.description.images == original.description.images);
    REQUIRE(retrieved.description.has_qr_code == original.description.has_qr_code);
    REQUIRE(retrieved.description.has_rfid_tag == original.description.has_rfid_tag);
    REQUIRE(retrieved.description.materials == original.description.materials);
    REQUIRE(retrieved.description.assembly_methods == original.description.assembly_methods);
    REQUIRE(retrieved.description.year_of_installation == original.description.year_of_installation);
    REQUIRE(retrieved.description.year_of_construction == original.description.year_of_construction);

    // Section 3: Product Information
    REQUIRE(retrieved.product.manufacturer == original.product.manufacturer);
    REQUIRE(retrieved.product.gtin == original.product.gtin);
    REQUIRE(retrieved.product.product_name == original.product.product_name);
    REQUIRE(retrieved.product.model_label == original.product.model_label);
    REQUIRE(retrieved.product.production_year == original.product.production_year);

    // Section 4: Certifications (13 properties)
    REQUIRE(retrieved.certifications.has_epd == original.certifications.has_epd);
    REQUIRE(retrieved.certifications.epd_programme_operator == original.certifications.epd_programme_operator);
    REQUIRE(retrieved.certifications.epd_operator_web_domain == original.certifications.epd_operator_web_domain);
    REQUIRE(retrieved.certifications.epd_registration_number == original.certifications.epd_registration_number);
    REQUIRE_THAT(retrieved.certifications.reference_service_life.value(),
                 WithinAbs(original.certifications.reference_service_life.value(), 0.001));
    REQUIRE(retrieved.certifications.has_safety_data_sheet == original.certifications.has_safety_data_sheet);
    REQUIRE(retrieved.certifications.declaration_of_performance == original.certifications.declaration_of_performance);
    REQUIRE(retrieved.certifications.technical_documentation == original.certifications.technical_documentation);
    REQUIRE(retrieved.certifications.non_destructive_tests == original.certifications.non_destructive_tests);
    REQUIRE_THAT(retrieved.certifications.assessed_period_of_use.value(),
                 WithinAbs(original.certifications.assessed_period_of_use.value(), 0.001));
    REQUIRE_THAT(retrieved.certifications.avg_service_life_build.value(),
                 WithinAbs(original.certifications.avg_service_life_build.value(), 0.001));
    REQUIRE_THAT(retrieved.certifications.remaining_service_life_rsl.value(),
                 WithinAbs(original.certifications.remaining_service_life_rsl.value(), 0.001));
    REQUIRE_THAT(retrieved.certifications.remaining_service_life_build.value(),
                 WithinAbs(original.certifications.remaining_service_life_build.value(), 0.001));

    // Section 5: Dimensions (8 properties)
    REQUIRE_THAT(retrieved.dimensions.width_mm.value(), WithinAbs(original.dimensions.width_mm.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.height_mm.value(), WithinAbs(original.dimensions.height_mm.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.length_mm.value(), WithinAbs(original.dimensions.length_mm.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.thickness_mm.value(), WithinAbs(original.dimensions.thickness_mm.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.weight_kg.value(), WithinAbs(original.dimensions.weight_kg.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.volume_m3.value(), WithinAbs(original.dimensions.volume_m3.value(), 0.001));
    REQUIRE_THAT(retrieved.dimensions.surface_area_m2.value(), WithinAbs(original.dimensions.surface_area_m2.value(), 0.001));
    REQUIRE(retrieved.dimensions.technical_drawing == original.dimensions.technical_drawing);

    // Section 6: Condition (8 properties)
    REQUIRE(retrieved.condition.photo_documentation == original.condition.photo_documentation);
    REQUIRE(retrieved.condition.visual_inspection_performed == original.condition.visual_inspection_performed);
    REQUIRE(retrieved.condition.has_signs_of_damage == original.condition.has_signs_of_damage);
    REQUIRE(retrieved.condition.is_deformed == original.condition.is_deformed);
    REQUIRE(retrieved.condition.is_scratched == original.condition.is_scratched);
    REQUIRE(retrieved.condition.is_surface_intact == original.condition.is_surface_intact);
    REQUIRE(retrieved.condition.has_intact_edges == original.condition.has_intact_edges);
    REQUIRE(retrieved.condition.has_signs_of_degradation == original.condition.has_signs_of_degradation);

    // Section 7: Pollution (with nested objects)
    REQUIRE(retrieved.pollution.contains_reach_substances == original.pollution.contains_reach_substances);
    REQUIRE(retrieved.pollution.is_chemically_treated == original.pollution.is_chemically_treated);
    REQUIRE(retrieved.pollution.surface_treatments == original.pollution.surface_treatments);
    REQUIRE(retrieved.pollution.intended_for_indoor_use == original.pollution.intended_for_indoor_use);
    REQUIRE(retrieved.pollution.labelling_scheme == original.pollution.labelling_scheme);
    REQUIRE(retrieved.pollution.emission_level == original.pollution.emission_level);
    REQUIRE(retrieved.pollution.has_asbestos_analysis == original.pollution.has_asbestos_analysis);

    // Verify nested DangerousSubstance array
    REQUIRE(retrieved.pollution.dangerous_substances.size() == original.pollution.dangerous_substances.size());
    for (size_t i = 0; i < original.pollution.dangerous_substances.size(); ++i) {
        const auto& orig_sub = original.pollution.dangerous_substances[i];
        const auto& retr_sub = retrieved.pollution.dangerous_substances[i];

        REQUIRE(retr_sub.content_method == orig_sub.content_method);
        REQUIRE(retr_sub.analyzed_substance == orig_sub.analyzed_substance);
        REQUIRE(retr_sub.cas_number == orig_sub.cas_number);
        REQUIRE(retr_sub.ec_number == orig_sub.ec_number);
        REQUIRE_THAT(retr_sub.concentration_mg_per_kg.value(),
                     WithinAbs(orig_sub.concentration_mg_per_kg.value(), 0.001));
    }

    // Verify nested Emission array
    REQUIRE(retrieved.pollution.emissions.size() == original.pollution.emissions.size());
    for (size_t i = 0; i < original.pollution.emissions.size(); ++i) {
        const auto& orig_em = original.pollution.emissions[i];
        const auto& retr_em = retrieved.pollution.emissions[i];

        REQUIRE(retr_em.standard == orig_em.standard);
        REQUIRE(retr_em.type == orig_em.type);
        REQUIRE_THAT(retr_em.lower_interval.value(), WithinAbs(orig_em.lower_interval.value(), 0.001));
        REQUIRE_THAT(retr_em.upper_interval.value(), WithinAbs(orig_em.upper_interval.value(), 0.001));
        REQUIRE(retr_em.quantity_type == orig_em.quantity_type);
        REQUIRE_THAT(retr_em.quantity.value(), WithinAbs(orig_em.quantity.value(), 0.001));
    }

    // Section 8: Environmental Potential
    REQUIRE(retrieved.environmental.takeback_scheme_available == original.environmental.takeback_scheme_available);
    REQUIRE(retrieved.environmental.consists_of_separate_parts == original.environmental.consists_of_separate_parts);

    // Section 9: Fire Properties
    REQUIRE(retrieved.fire.reaction_to_fire == original.fire.reaction_to_fire);
    REQUIRE(retrieved.fire.resistance_to_fire == original.fire.resistance_to_fire);
    REQUIRE(retrieved.fire.documentation_of_fire_classification == original.fire.documentation_of_fire_classification);
    REQUIRE(retrieved.fire.field_of_application == original.fire.field_of_application);

    // Section 10: History
    REQUIRE(retrieved.history.previous_usage_environments == original.history.previous_usage_environments);

    // Metadata
    REQUIRE(retrieved.metadata.document_guid == original.metadata.document_guid);
    REQUIRE(retrieved.metadata.creation_date == original.metadata.creation_date);
    REQUIRE(retrieved.metadata.revision_date == original.metadata.revision_date);
    REQUIRE(retrieved.metadata.version_number == original.metadata.version_number);
    REQUIRE(retrieved.metadata.version_date == original.metadata.version_date);

    // Transaction log
    REQUIRE(retrieved.transaction_log.size() == original.transaction_log.size());
    for (size_t i = 0; i < original.transaction_log.size(); ++i) {
        REQUIRE(retrieved.transaction_log[i].type == original.transaction_log[i].type);
        REQUIRE(retrieved.transaction_log[i].guid == original.transaction_log[i].guid);
        REQUIRE(retrieved.transaction_log[i].edited_by == original.transaction_log[i].edited_by);
        REQUIRE(retrieved.transaction_log[i].edited_date == original.transaction_log[i].edited_date);
        REQUIRE(retrieved.transaction_log[i].old_value == original.transaction_log[i].old_value);
        REQUIRE(retrieved.transaction_log[i].new_value == original.transaction_log[i].new_value);
    }
}

// ============================================================================
// Test Cases
// ============================================================================

TEST_CASE("MaterialPassport database round-trip", "[core][serialization][database]") {
    // 1. Create temporary database path
    auto temp_db = std::filesystem::temp_directory_path() /
                   "test_materialpassport_roundtrip.db";

    // Ensure clean state
    if (std::filesystem::exists(temp_db)) {
        std::filesystem::remove(temp_db);
    }

    // 2. Create a fully-populated MaterialPassport
    MaterialPassport original = createSamplePassport();
    original.metadata.document_guid = "test-passport-uuid-12345";

    SECTION("Write and read back passport") {
        // 3. Create database and write passport
        {
            reusex::ProjectDB db(temp_db, false);  // false = write mode
            REQUIRE(db.is_open());

            db.add_material_passport(original, "test-project-001");

            // Verify it was written
            REQUIRE(std::filesystem::exists(temp_db));
            REQUIRE(std::filesystem::file_size(temp_db) > 0);
        }  // db closed (RAII cleanup)

        // 4. Reopen database and read passport back
        MaterialPassport retrieved;
        {
            reusex::ProjectDB db(temp_db, true);  // true = read-only mode
            REQUIRE(db.is_open());

            retrieved = db.material_passport(original.metadata.document_guid);
        }  // db closed

        // 5. Verify all fields match
        verifyPassportEquality(original, retrieved);

        // Cleanup
        std::filesystem::remove(temp_db);
    }
}

TEST_CASE("ProjectDB handles empty passport", "[core][serialization][database]") {
    auto temp_db = std::filesystem::temp_directory_path() / "test_empty_passport.db";

    // Ensure clean state
    if (std::filesystem::exists(temp_db)) {
        std::filesystem::remove(temp_db);
    }

    MaterialPassport empty_passport;
    empty_passport.metadata.document_guid = "empty-guid-001";

    {
        reusex::ProjectDB db(temp_db, false);
        db.add_material_passport(empty_passport, "test-project");
    }

    {
        reusex::ProjectDB db(temp_db, true);
        auto retrieved = db.material_passport("empty-guid-001");

        // Verify default/empty values preserved
        REQUIRE(retrieved.owner.contact_email.empty());
        REQUIRE(retrieved.description.images.empty());
        REQUIRE_FALSE(retrieved.certifications.has_epd.value_or(false));
    }

    std::filesystem::remove(temp_db);
}

TEST_CASE("ProjectDB read non-existent passport throws", "[core][serialization][database]") {
    auto temp_db = std::filesystem::temp_directory_path() / "test_missing_passport.db";

    // Ensure clean state
    if (std::filesystem::exists(temp_db)) {
        std::filesystem::remove(temp_db);
    }

    {
        reusex::ProjectDB db(temp_db, false);
        // Create empty database
    }

    {
        reusex::ProjectDB db(temp_db, true);
        REQUIRE_THROWS_AS(
            db.material_passport("non-existent-guid"),
            std::runtime_error
        );
    }

    std::filesystem::remove(temp_db);
}

TEST_CASE("ProjectDB handles multiple passports", "[core][serialization][database]") {
    auto temp_db = std::filesystem::temp_directory_path() / "test_multiple_passports.db";

    // Ensure clean state
    if (std::filesystem::exists(temp_db)) {
        std::filesystem::remove(temp_db);
    }

    // Create three different passports
    MaterialPassport passport1 = createSamplePassport();
    passport1.metadata.document_guid = "passport-001";
    passport1.owner.contact_name = "Alice Johnson";

    MaterialPassport passport2 = createSamplePassport();
    passport2.metadata.document_guid = "passport-002";
    passport2.owner.contact_name = "Bob Smith";

    MaterialPassport passport3 = createSamplePassport();
    passport3.metadata.document_guid = "passport-003";
    passport3.owner.contact_name = "Charlie Brown";

    // Write all three passports
    {
        reusex::ProjectDB db(temp_db, false);

        db.add_material_passport(passport1, "project-A");
        db.add_material_passport(passport2, "project-B");
        db.add_material_passport(passport3, "project-A");  // Same project as passport1
    }

    // Read them back and verify
    {
        reusex::ProjectDB db(temp_db, true);

        auto retrieved1 = db.material_passport("passport-001");
        auto retrieved2 = db.material_passport("passport-002");
        auto retrieved3 = db.material_passport("passport-003");

        verifyPassportEquality(passport1, retrieved1);
        verifyPassportEquality(passport2, retrieved2);
        verifyPassportEquality(passport3, retrieved3);

        // Verify they have different names as expected
        REQUIRE(retrieved1.owner.contact_name == "Alice Johnson");
        REQUIRE(retrieved2.owner.contact_name == "Bob Smith");
        REQUIRE(retrieved3.owner.contact_name == "Charlie Brown");
    }

    std::filesystem::remove(temp_db);
}
