// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>

#include <core/MaterialPassport.hpp>
#include <core/materialepas_json_export.hpp>
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
        "Industrial warehouse - 2010-2025"
    };

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

TEST_CASE("Empty passport JSON matches template structure",
          "[core][json][template]") {
    MaterialPassport empty;
    auto j = json_export::to_json(empty);

    // Top-level structure
    REQUIRE(j.contains("sections"));
    REQUIRE(j.contains("log"));
    REQUIRE(j.contains("metadata"));

    // 10 sections in correct order
    const auto &sections = j["sections"];
    REQUIRE(sections.size() == 10);

    // Verify section order matches JSON template
    CHECK(sections[0]["nameEN"] == "Owner");
    CHECK(sections[0]["nameDA"] == "Ejer");
    CHECK(sections[1]["nameEN"] == "Construction item description");
    CHECK(sections[2]["nameEN"] == "Product information");
    CHECK(sections[3]["nameEN"] == "Certifications, approvements and declarations");
    CHECK(sections[4]["nameEN"] == "Dimensions and geometri");
    CHECK(sections[5]["nameEN"] == "History");
    CHECK(sections[6]["nameEN"] == "Condition");
    CHECK(sections[7]["nameEN"] == "Pollution - content and emissions");
    CHECK(sections[8]["nameEN"].get<std::string>().find("Environmental") != std::string::npos);
    CHECK(sections[9]["nameEN"] == "Other essential properties");

    // Verify property counts per section
    CHECK(sections[0]["properties"].size() == 3);   // Owner
    CHECK(sections[1]["properties"].size() == 8);   // ConstructionItemDescription
    CHECK(sections[2]["properties"].size() == 5);   // ProductInformation
    CHECK(sections[3]["properties"].size() == 13);  // Certifications
    CHECK(sections[4]["properties"].size() == 11);  // Dimensions
    CHECK(sections[5]["properties"].size() == 1);   // History

    // Every property has at least name + guid + (value or values or properties)
    for (const auto &sec : sections) {
        for (const auto &prop : sec["properties"]) {
            REQUIRE(prop.contains("name"));
            bool has_content = prop.contains("value") || prop.contains("values")
                            || prop.contains("properties");
            REQUIRE(has_content);
        }
    }

    // Empty log and metadata
    CHECK(j["log"].empty());
    CHECK(j["metadata"]["document guid"] == "");
    CHECK(j["metadata"]["version number"] == "");
}

TEST_CASE("Populated passport JSON contains correct values",
          "[core][json][export]") {
    auto passport = createSamplePassport();
    auto j = json_export::to_json(passport);

    const auto &sections = j["sections"];

    // Owner section - string values
    const auto &owner = sections[0]["properties"];
    CHECK(owner[0]["value"] == "owner@example.com");
    CHECK(owner[1]["value"] == "John Smith");
    CHECK(owner[2]["value"] == "Green Construction Ltd");

    // ConstructionItemDescription - mixed types
    const auto &desc = sections[1]["properties"];
    CHECK(desc[0]["value"] == "Recycled Steel Beam I-Profile 300x150");
    // images is StringArray -> should have "values" array
    REQUIRE(desc[1].contains("values"));
    CHECK(desc[1]["values"].size() == 2);
    CHECK(desc[1]["values"][0]["value"] == "beam_photo1.jpg");
    // Boolean as "true"/"false"
    CHECK(desc[2]["value"] == "true");  // has_qr_code
    CHECK(desc[3]["value"] == "false"); // has_rfid_tag
    // EnumArray (materials)
    REQUIRE(desc[4].contains("values"));
    CHECK(desc[4]["values"].size() == 2);
    CHECK(desc[4]["values"][0]["value"] == "steel");
    // Integer as string
    CHECK(desc[6]["value"] == "2010"); // year_of_installation

    // Product information
    const auto &product = sections[2]["properties"];
    CHECK(product[0]["value"] == "ArcelorMittal");
    CHECK(product[4]["value"] == "2010"); // production_year as string

    // Certifications - double as string
    const auto &certs = sections[3]["properties"];
    CHECK(certs[0]["value"] == "true"); // has_epd
    // reference_service_life (double) should be stringified
    std::string rsl = certs[4]["value"].get<std::string>();
    REQUIRE_FALSE(rsl.empty());

    // Dimensions
    const auto &dims = sections[4]["properties"];
    std::string width = dims[0]["value"].get<std::string>();
    REQUIRE_FALSE(width.empty());

    // History (position 5 in JSON template order!)
    const auto &history = sections[5]["properties"];
    // StringArray with values
    CHECK(history[0].contains("values"));

    // Condition section
    const auto &cond = sections[6]["properties"];
    // photo_documentation is StringArray
    REQUIRE(cond[0].contains("values"));
    CHECK(cond[0]["values"].size() == 1);
    CHECK(cond[1]["value"] == "true"); // visual_inspection_performed

    // Pollution section - with nested objects
    const auto &poll = sections[7]["properties"];
    // TriState values
    CHECK(poll[0]["value"] == "no");  // contains_reach_substances
    CHECK(poll[1]["value"] == "yes"); // is_chemically_treated

    // Find dangerous substances entry (ObjectArray)
    bool found_dangerous = false;
    bool found_emission = false;
    for (const auto &prop : poll) {
        if (prop["name"] == "dangerous substances" && prop.contains("properties")) {
            found_dangerous = true;
            const auto &nested = prop["properties"];
            // Should have 5 properties for DangerousSubstance
            CHECK(nested.size() == 5);
            CHECK(nested[0]["name"] == "content of dangerous substances");
            CHECK(nested[0]["value"] == "measured_material");
            CHECK(nested[1]["value"] == "Lead (Pb)");
        }
        if (prop["name"] == "release of dangerous substances" && prop.contains("properties")) {
            found_emission = true;
            const auto &nested = prop["properties"];
            // Should have 6 properties (quantity is skipped due to empty json_name)
            CHECK(nested.size() == 6);
            CHECK(nested[0]["name"] == "emission standard");
            CHECK(nested[0]["value"] == "EN ISO 16000");
            // Check measuring_unit
            bool found_unit = false;
            for (const auto &np : nested) {
                if (np["name"] == "measuring unit") {
                    found_unit = true;
                    CHECK(np["value"] == "mg/m3");
                }
            }
            CHECK(found_unit);
        }
    }
    CHECK(found_dangerous);
    CHECK(found_emission);

    // Environmental potential
    const auto &env = sections[8]["properties"];
    CHECK(env[0]["value"] == "true");  // takeback_scheme_available
    CHECK(env[1]["value"] == "false"); // consists_of_separate_parts

    // Fire properties
    const auto &fire = sections[9]["properties"];
    CHECK(fire[0]["value"] == "A1");

    // Metadata
    CHECK(j["metadata"]["document guid"] == "test-guid-12345");
    CHECK(j["metadata"]["document creation date"] == "2025-03-25T10:30:00Z");
    CHECK(j["metadata"]["version number"] == "1.0.0");

    // Transaction log
    REQUIRE(j["log"].size() == 1);
    const auto &log_props = j["log"][0]["properties"];
    CHECK(log_props[0]["value"] == "document");
    CHECK(log_props[2]["value"] == "test_user");
    CHECK(log_props[5]["value"] == "Created");
}

TEST_CASE("Multiple passports export as JSON array",
          "[core][json][merge]") {
    MaterialPassport p1 = createSamplePassport();
    p1.metadata.document_guid = "guid-001";
    p1.owner.contact_name = "Alice";

    MaterialPassport p2 = createSamplePassport();
    p2.metadata.document_guid = "guid-002";
    p2.owner.contact_name = "Bob";

    MaterialPassport p3 = createSamplePassport();
    p3.metadata.document_guid = "guid-003";
    p3.owner.contact_name = "Charlie";

    std::vector<MaterialPassport> passports = {p1, p2, p3};
    auto j = json_export::to_json(passports);

    REQUIRE(j.is_array());
    REQUIRE(j.size() == 3);

    // Each element has full structure
    for (const auto &elem : j) {
        REQUIRE(elem.contains("sections"));
        REQUIRE(elem.contains("log"));
        REQUIRE(elem.contains("metadata"));
        REQUIRE(elem["sections"].size() == 10);
    }

    // Distinct values preserved
    CHECK(j[0]["metadata"]["document guid"] == "guid-001");
    CHECK(j[1]["metadata"]["document guid"] == "guid-002");
    CHECK(j[2]["metadata"]["document guid"] == "guid-003");

    CHECK(j[0]["sections"][0]["properties"][1]["value"] == "Alice");
    CHECK(j[1]["sections"][0]["properties"][1]["value"] == "Bob");
    CHECK(j[2]["sections"][0]["properties"][1]["value"] == "Charlie");
}

TEST_CASE("Empty fields produce correct JSON format",
          "[core][json][empty]") {
    MaterialPassport empty;
    auto j = json_export::to_json(empty);

    const auto &sections = j["sections"];

    // Empty strings produce ""
    CHECK(sections[0]["properties"][0]["value"] == "");

    // Unset optionals produce ""
    // has_qr_code in ConstructionItemDescription (index 2 in section 1)
    CHECK(sections[1]["properties"][2]["value"] == "");

    // Empty vectors produce "value": "" for StringArray
    // images in ConstructionItemDescription (index 1 in section 1)
    CHECK(sections[1]["properties"][1]["value"] == "");

    // Empty EnumArray produces "values": []
    CHECK(sections[1]["properties"][4]["values"].empty());

    // Default TriState -> "unknown"
    CHECK(sections[7]["properties"][0]["value"] == "unknown");
    CHECK(sections[7]["properties"][1]["value"] == "unknown");

    // Empty metadata
    CHECK(j["metadata"]["document guid"] == "");
    CHECK(j["metadata"]["document creation date"] == "");
}

TEST_CASE("JSON string export is valid parseable JSON",
          "[core][json][string]") {
    auto passport = createSamplePassport();

    // Pretty-printed
    std::string pretty = json_export::to_json_string(passport, 4);
    REQUIRE_FALSE(pretty.empty());
    auto j_pretty = json::parse(pretty);
    REQUIRE(j_pretty.contains("sections"));

    // Compact
    std::string compact = json_export::to_json_string(passport, -1);
    REQUIRE_FALSE(compact.empty());
    auto j_compact = json::parse(compact);
    REQUIRE(j_compact.contains("sections"));

    // Both should produce equivalent JSON
    CHECK(j_pretty == j_compact);
}
