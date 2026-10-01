// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Pure survey rules (Kortlægning): wire strings, miljøstatus, circularity,
// waste fractions, room majority vote, quantity redistribution, codes.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <core/survey.hpp>

#include <stdexcept>
#include <vector>

using namespace reusex::core;
using Catch::Approx;

TEST_CASE("SurveyEnums_WireStrings_RoundTrip", "[survey]") {
  for (auto t :
       {Treatment::bevaring, Treatment::genbrug, Treatment::genanvendelse,
        Treatment::nyttiggoerelse, Treatment::bortskaffelse})
    CHECK(treatment_from_string(to_string(t)) == t);
  CHECK(to_string(Treatment::nyttiggoerelse) == "nyttiggoerelse");
  for (auto s :
       {ReviewStatus::queue, ReviewStatus::approved, ReviewStatus::rejected})
    CHECK(review_status_from_string(to_string(s)) == s);
  for (auto s : {SampleStage::planlagt, SampleStage::udtaget,
                 SampleStage::sendt, SampleStage::svar})
    CHECK(sample_stage_from_string(to_string(s)) == s);
  CHECK(to_string(SampleResult::none).empty());
  CHECK(sample_result_from_string("") == SampleResult::none);
  CHECK(sample_result_from_string("forurenet") == SampleResult::forurenet);
  CHECK_FALSE(treatment_from_string("Genbrug").has_value()); // case-sensitive
  CHECK(to_string(EnvironmentStatus::ren_proevesvar) == "ren_proevesvar");
}

TEST_CASE("EnvironmentStatus_Derivation", "[survey]") {
  CHECK(environment_status({}) == EnvironmentStatus::ren_screening);
  CHECK(environment_status({{SampleStage::sendt, SampleResult::none}}) ==
        EnvironmentStatus::afventer);
  CHECK(environment_status({{SampleStage::svar, SampleResult::ren}}) ==
        EnvironmentStatus::ren_proevesvar);
  // Any contaminated answer wins, even while another sample is still out.
  CHECK(environment_status({{SampleStage::svar, SampleResult::forurenet},
                            {SampleStage::planlagt, SampleResult::none}}) ==
        EnvironmentStatus::forurenet);
  CHECK(environment_status({{SampleStage::svar, SampleResult::ren},
                            {SampleStage::udtaget, SampleResult::none}}) ==
        EnvironmentStatus::afventer);
}

TEST_CASE("ValidateSampleState_ResultOnlyWithAnswer", "[survey]") {
  CHECK_NOTHROW(validate_sample_state({SampleStage::svar, SampleResult::ren}));
  CHECK_NOTHROW(
      validate_sample_state({SampleStage::sendt, SampleResult::none}));
  CHECK_THROWS_AS(
      validate_sample_state({SampleStage::sendt, SampleResult::ren}),
      std::invalid_argument);
}

TEST_CASE("CircularityBreakdown_SkipsRejected_NullMassIsZero", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::bevaring, ReviewStatus::approved, 640.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::queue, 58.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::rejected, 999.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::bortskaffelse, ReviewStatus::queue, std::nullopt, "17.06.04",
       EnvironmentStatus::ren_screening},
  };
  const auto b = circularity_breakdown(types);
  CHECK(b[static_cast<std::size_t>(Treatment::bevaring)] == Approx(640.0));
  CHECK(b[static_cast<std::size_t>(Treatment::genbrug)] == Approx(58.0));
  CHECK(b[static_cast<std::size_t>(Treatment::bortskaffelse)] == Approx(0.0));
}

TEST_CASE("FractionsByEak_GroupsApprovedByCodeAndTreatment_LeavesBevaringOut",
          "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::genanvendelse, ReviewStatus::approved, 380.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 190.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::bevaring, ReviewStatus::approved, 640.0, "17.01.01",
       EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 6.8, "17.04.05",
       EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::queue, 58.0, "17.01.01",
       EnvironmentStatus::ren_screening, 2, "Betonsøjler, bærende"},
      {Treatment::genbrug, ReviewStatus::rejected, 5.0, "17.02.01",
       EnvironmentStatus::ren_screening},
  };
  const auto r = fractions_by_eak(types);
  REQUIRE(r.fractions.size() == 2);
  CHECK(r.fractions[0].eak_code == "17.01.01");
  CHECK(r.fractions[0].treatment == Treatment::genanvendelse);
  CHECK(r.fractions[0].mass_t == Approx(570.0));
  CHECK(r.fractions[0].name == "Beton");
  CHECK_FALSE(r.fractions[0].contaminated);
  CHECK(r.fractions[1].eak_code == "17.04.05");
  CHECK(r.total_t == Approx(576.8)); // bevaring stays in the building
  REQUIRE(r.blocking.size() == 1);   // the queued type; rejected never blocks
  CHECK(r.blocking_types == 1);
  CHECK(r.blocking[0].type_id == 2);
  CHECK(r.blocking[0].name == "Betonsøjler, bærende");
  CHECK(r.blocking[0].reason == BlockingReason::review);
  REQUIRE(r.blocking[0].mass_t.has_value());
  CHECK(*r.blocking[0].mass_t == Approx(58.0));
}

TEST_CASE("FractionsByEak_ApprovedButAfventer_IsWithheldAndBlocks",
          "[survey]") {
  // The answer can make it contaminated, which changes how it is reported.
  std::vector<TypeTotals> types{{Treatment::genbrug, ReviewStatus::approved,
                                 3.1, "17.04.02", EnvironmentStatus::afventer,
                                 6, "Vinduespartier, aluminium"}};
  const auto r = fractions_by_eak(types);
  CHECK(r.fractions.empty());
  CHECK(r.total_t == Approx(0.0));
  REQUIRE(r.blocking.size() == 1);
  CHECK(r.blocking[0].reason == BlockingReason::sample);
  CHECK(r.blocking[0].type_id == 6);
}

TEST_CASE("FractionsByEak_QueuedBevaring_BlocksButNeverCounts", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::bevaring, ReviewStatus::queue, 640.0, "17.01.01",
       EnvironmentStatus::ren_screening, 1, "Fundamenter & terrændæk, beton"}};
  const auto r = fractions_by_eak(types);
  CHECK(r.fractions.empty());
  REQUIRE(r.blocking.size() == 1);
  CHECK(r.blocking[0].reason == BlockingReason::review);
}

TEST_CASE("FractionsByEak_ContaminatedTonnes_GetTheirOwnRow", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::bortskaffelse, ReviewStatus::approved, 38.0, "17.01.02",
       EnvironmentStatus::forurenet},
      {Treatment::bortskaffelse, ReviewStatus::approved, 4.0, "17.01.02",
       EnvironmentStatus::ren_proevesvar},
  };
  const auto r = fractions_by_eak(types);
  REQUIRE(r.fractions.size() == 2);
  CHECK_FALSE(r.fractions[0].contaminated); // clean first
  CHECK(r.fractions[0].mass_t == Approx(4.0));
  CHECK(r.fractions[1].contaminated);
  CHECK(r.fractions[1].mass_t == Approx(38.0));
  CHECK(r.fractions[1].name == "Mursten");
  CHECK(r.total_t == Approx(42.0));
  CHECK(r.blocking.empty());
}

TEST_CASE("BlockingReason_WireStrings", "[survey]") {
  CHECK(to_string(BlockingReason::review) == "review");
  CHECK(to_string(BlockingReason::sample) == "sample");
}

TEST_CASE("Reportable_ApprovedAndNotAwaitingSample", "[survey]") {
  CHECK(reportable(ReviewStatus::approved, EnvironmentStatus::ren_screening));
  CHECK(reportable(ReviewStatus::approved, EnvironmentStatus::forurenet));
  CHECK_FALSE(reportable(ReviewStatus::approved, EnvironmentStatus::afventer));
  CHECK_FALSE(
      reportable(ReviewStatus::queue, EnvironmentStatus::ren_screening));
  CHECK_FALSE(
      reportable(ReviewStatus::rejected, EnvironmentStatus::ren_screening));
}

TEST_CASE("EakFractionName_KnownAndUnknown", "[survey]") {
  CHECK(eak_fraction_name("17.04.05") == "Jern og stål");
  CHECK(eak_fraction_name("99.99.99").empty());
}

TEST_CASE("MajorityRoom_VotesPerInstance_IgnoresZero_TieToLowerRoom",
          "[survey]") {
  //                 idx: 0  1  2  3  4  5  6  7
  std::vector<std::uint32_t> inst{1, 1, 1, 2, 2, 0, 3, 3};
  std::vector<std::uint32_t> room{4, 4, 5, 6, 7, 4, 0, 0};
  const auto m = majority_room(inst, room);
  CHECK(m.at(1) == 4);
  CHECK(m.at(2) == 6);        // tie 6 vs 7 -> lower id
  CHECK_FALSE(m.contains(3)); // only unlabeled room points
  CHECK_FALSE(m.contains(0));
  CHECK_THROWS_AS(majority_room({1, 2}, {1}), std::invalid_argument);
}

TEST_CASE("RedistributeQuantity_Proportional_EqualWhenZero_RemainderOnLast",
          "[survey]") {
  CHECK(redistribute_quantity({18, 6}, 30) == std::vector<double>{22.5, 7.5});
  CHECK(redistribute_quantity({0, 0, 0}, 10) ==
        std::vector<double>{3.33, 3.33, 3.34});
  CHECK(redistribute_quantity({1, 1, 1}, 1) ==
        std::vector<double>{0.33, 0.33, 0.34});
  CHECK(redistribute_quantity({}, 5).empty());
  CHECK_THROWS_AS(redistribute_quantity({1}, -1), std::invalid_argument);
}

TEST_CASE("Codes_ZeroPadded", "[survey]") {
  CHECK(part_code(1) == "RX-001");
  CHECK(part_code(42) == "RX-042");
  CHECK(part_code(1000) == "RX-1000");
  CHECK(sample_code(3) == "P-03");
}
