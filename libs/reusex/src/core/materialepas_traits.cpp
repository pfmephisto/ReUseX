// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/materialepas_traits.hpp"
#include "core/materialepas_types.hpp"

namespace ReUseX::core::traits {

// ===========================================================================
// Section 1: Owner (3 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    Owner,
    REUSEX_PROP(Owner, contact_email, "0Bwj05D$55V931bq9VaBE5", String),
    REUSEX_PROP(Owner, contact_name, "17BdeQk152gR5YUt5oSLfp", String),
    REUSEX_PROP(Owner, company_name, "1bb51PIefCe9uHGCmR0gFJ", String));

// ===========================================================================
// Section 2: Construction Item Description (8 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    ConstructionItemDescription,
    REUSEX_PROP(ConstructionItemDescription, designation, "2_mYRkA$9EcwXAxqBGoMrM",
                String),
    REUSEX_PROP(ConstructionItemDescription, images, "1dvQfmc2z9lB_rwknYkM3y",
                StringArray),
    REUSEX_PROP(ConstructionItemDescription, has_qr_code, "06moAxe0j7EutROb0dD3B7",
                Boolean),
    REUSEX_PROP(ConstructionItemDescription, has_rfid_tag, "2$Owq1iHrAuw$ASWOXAk9p",
                Boolean),
    REUSEX_PROP(ConstructionItemDescription, materials, "1R_z75Gd52OQYxVkJ$E3ZU",
                EnumArray),
    REUSEX_PROP(ConstructionItemDescription, assembly_methods,
                "0N7Qwopp125xrULezqArw4", StringArray),
    REUSEX_PROP(ConstructionItemDescription, year_of_installation,
                "294ZkUyYn6TfdwCsbo2X63", Integer),
    REUSEX_PROP(ConstructionItemDescription, year_of_construction,
                "17LFECVML6buNXwBLz416Y", Integer));

// ===========================================================================
// Section 3: Product Information (5 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    ProductInformation,
    REUSEX_PROP(ProductInformation, manufacturer, "1QwFy1F3T4dPe2N2cbsuRv", String),
    REUSEX_PROP(ProductInformation, gtin, "1rQ9Z5vy5AygUSncyWuhXr", String),
    REUSEX_PROP(ProductInformation, product_name, "198tMTFIX5PecEfAwMEIeo", String),
    REUSEX_PROP(ProductInformation, model_label, "06t2gkj49FNvMebmltrHXV", String),
    REUSEX_PROP(ProductInformation, production_year, "3nif2XJKH8r9I09LfEYaRm",
                Integer));

// ===========================================================================
// Section 4: Certifications (13 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    Certifications,
    REUSEX_PROP(Certifications, has_epd, "2_jn1$mEr1780H5QOiEj6O", Boolean),
    REUSEX_PROP(Certifications, epd_programme_operator, "0ORiYBKRnCFQzUwOsNqbYn",
                String),
    REUSEX_PROP(Certifications, epd_operator_web_domain, "2kqTixLBr9sBqGKYs9lNSI",
                String),
    REUSEX_PROP(Certifications, epd_registration_number, "2aCraAVB91qw8CzrM6x3ia",
                String),
    REUSEX_PROP(Certifications, reference_service_life, "28mSsPe_z69BksBt5oRXMl",
                Double),
    REUSEX_PROP(Certifications, has_safety_data_sheet, "3D1KFVwbDBuumnIBZZHQId",
                Boolean),
    REUSEX_PROP(Certifications, declaration_of_performance,
                "1tqCmrJwn3yvF$k2lU2fFa", StringArray),
    REUSEX_PROP(Certifications, technical_documentation, "1IfrbnqGv5Pu3GBuocLZgi",
                StringArray),
    REUSEX_PROP(Certifications, non_destructive_tests, "0aTBceworF$QTXM6CqyOdV",
                StringArray),
    REUSEX_PROP(Certifications, assessed_period_of_use, "3xW23JRMTAzOQb79uZW3qC",
                Double),
    REUSEX_PROP(Certifications, avg_service_life_build, "2TtVRWo_56mBphAcSiFIHG",
                Double),
    REUSEX_PROP(Certifications, remaining_service_life_rsl,
                "2id4UZ8ynAF9DEzV0NC_wT", Double),
    REUSEX_PROP(Certifications, remaining_service_life_build,
                "27gXAA5Iz7lgcOytMRqOBv", Double));

// ===========================================================================
// Section 5: Dimensions (11 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    Dimensions,
    REUSEX_PROP(Dimensions, width_mm, "3NHBUedX9438Hi3mwD15$Z", Double),
    REUSEX_PROP(Dimensions, height_mm, "2G$wMhUvL2LgKNY0J66oAT", Double),
    REUSEX_PROP(Dimensions, length_mm, "1uUn3YWZfBaPqYQMo_$Om$", Double),
    REUSEX_PROP(Dimensions, thickness_mm, "0SyXPZ9an9vh49k$Lgkvly", Double),
    REUSEX_PROP(Dimensions, depth_mm, "1$OV5du3LFWwa$P7SfJhAr", Double),
    REUSEX_PROP(Dimensions, volume_m3, "3IHRMkfs9EDPXVvPJ0hbFn", Double),
    REUSEX_PROP(Dimensions, surface_area_m2, "3Cs4d96Ff1nPGHWlsTyYIS", Double),
    REUSEX_PROP(Dimensions, inner_diameter_mm, "0ayREcz2zBuPWmCDTem2a8", Double),
    REUSEX_PROP(Dimensions, outer_diameter_mm, "3BWwvo0lf3iAd_34y7ac$4", Double),
    REUSEX_PROP(Dimensions, weight_kg, "1XyLvnxf94pwJW6LWaD6Zw", Double),
    REUSEX_PROP(Dimensions, technical_drawing, "2d01jZqvP7GOR3V$coAprV", String));

// ===========================================================================
// Section 6: Condition (8 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    Condition,
    REUSEX_PROP(Condition, photo_documentation, "3$DH1Zm_r9_gzqdwKK39Ua",
                StringArray),
    REUSEX_PROP(Condition, visual_inspection_performed, "2jB20LC_nEzBN4_j2ypip8",
                Boolean),
    REUSEX_PROP(Condition, has_signs_of_damage, "0rQimGJFHEueqtqwbwBaC5", Boolean),
    REUSEX_PROP(Condition, is_deformed, "1g6mt3SrvDlho6YqBIu1lW", Boolean),
    REUSEX_PROP(Condition, is_scratched, "365luGupDCfwgtD32gGGrS", Boolean),
    REUSEX_PROP(Condition, is_surface_intact, "2GO932BuX5EvXOecmhbbl3", Boolean),
    REUSEX_PROP(Condition, has_intact_edges, "0AoBmqQ9v1pw7ca1VVp_Za", Boolean),
    REUSEX_PROP(Condition, has_signs_of_degradation, "01lVHQF$XAneE8CFtajgr_",
                Boolean));

// ===========================================================================
// Section 7: Pollution - Nested Types (DangerousSubstance, Emission)
// ===========================================================================

// DangerousSubstance (5 properties - nested)
REUSEX_DECLARE_PROPERTIES(
    DangerousSubstance,
    REUSEX_PROP(DangerousSubstance, content_method, "2ympiI4PL6c8ooh9Agqnz4",
                EnumValue),
    REUSEX_PROP(DangerousSubstance, analyzed_substance, "3Y2oTiV9r3NfoZQBVP_Bpx",
                String),
    REUSEX_PROP(DangerousSubstance, cas_number, "2tZYDby2H2PADWYcTMn50O", String),
    REUSEX_PROP(DangerousSubstance, ec_number, "1cV5alHQ93Zu44Uv4Iuqoj", String),
    REUSEX_PROP(DangerousSubstance, concentration_mg_per_kg,
                "0yw_0Ggsf6ihdShOA1niRl", Double));

// Emission (5 properties - nested, excluding archived measuring_unit field)
REUSEX_DECLARE_PROPERTIES(
    Emission,
    REUSEX_PROP(Emission, standard, "1Kx6G$uIXEZO3Br$MEsdaB", String),
    REUSEX_PROP(Emission, type, "12ZQ3obYHB1OhXiyy3LcfK", String),
    REUSEX_PROP(Emission, lower_interval, "1dSnTNTGD5YQPcgcbzMJaD", Double),
    REUSEX_PROP(Emission, upper_interval, "3YTotdivHEeQS4ij2Xa7PU", Double),
    REUSEX_PROP(Emission, quantity_type, "33HQrsPDn9uhOu$7D3HhWU", EnumValue),
    REUSEX_PROP(Emission, quantity, "0lZ6gflK18WBte7dMyQQ$h", Double));

// Pollution (7 properties + 2 nested arrays)
REUSEX_DECLARE_PROPERTIES(
    Pollution,
    REUSEX_PROP(Pollution, contains_reach_substances, "2E6_MQ4Cn36f0s6CVYxQOR",
                TriState),
    REUSEX_PROP(Pollution, is_chemically_treated, "1Cf0jFllr2HAk8FOmT2VGr",
                TriState),
    REUSEX_PROP(Pollution, surface_treatments, "1zV5ZyYKT6CPWjKdTEZ1K0",
                StringArray),
    REUSEX_PROP_NESTED(Pollution, dangerous_substances, DangerousSubstance),
    REUSEX_PROP_NESTED(Pollution, emissions, Emission),
    REUSEX_PROP(Pollution, intended_for_indoor_use, "2W1zl8TFb459xOVytRmX3J",
                Boolean),
    REUSEX_PROP(Pollution, labelling_scheme, "3Txv$QA7zCp8azbWeuubx2", String),
    REUSEX_PROP(Pollution, emission_level, "3areU4Rpv9gfbeDNE9eBgA", String),
    REUSEX_PROP(Pollution, has_asbestos_analysis, "2962p$Lwj68Br6satSNLAn",
                Boolean));

// ===========================================================================
// Section 8: Environmental Potential (2 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    EnvironmentalPotential,
    REUSEX_PROP(EnvironmentalPotential, takeback_scheme_available,
                "2__I$nkL94YRI_YCBy0ou8", Boolean),
    REUSEX_PROP(EnvironmentalPotential, consists_of_separate_parts,
                "2nZqcaXkz7YPHhKI$FMowg", Boolean));

// ===========================================================================
// Section 9: Fire Properties (4 properties)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    FireProperties,
    REUSEX_PROP(FireProperties, reaction_to_fire, "2bipZ4JrzDnwNZZHkwfk8O", String),
    REUSEX_PROP(FireProperties, resistance_to_fire, "0NlhENUQr4YA5$p0zBrXSp",
                String),
    REUSEX_PROP(FireProperties, documentation_of_fire_classification,
                "1DdkDjj1H7POQtnzDtu7CC", String),
    REUSEX_PROP(FireProperties, field_of_application, "37sNiQNwHC399AW2B60MrY",
                String));

// ===========================================================================
// Section 10: History (1 property)
// ===========================================================================

REUSEX_DECLARE_PROPERTIES(
    History, REUSEX_PROP(History, previous_usage_environments,
                         "3BiwSSLEz9l8YLgoWcixHq", StringArray));

} // namespace ReUseX::core::traits
