// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/// Survey (Ressourcekortlægning) rules that need no database: the wire
/// vocabulary shared by ProjectDB, the REST API and the frontend, and the
/// derivations the GUI shows — miljøstatus from linked samples, tonnes per
/// affaldshierarki step, approved tonnes per EAK code. Everything here is pure
/// so it is testable without a project, and so `rux`, `rux gui` and a future
/// Qt client all compute the same numbers.

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace reusex::core {

enum class Treatment {
  bevaring,
  genbrug,
  genanvendelse,
  nyttiggoerelse,
  bortskaffelse
};
inline constexpr std::size_t kTreatmentCount = 5;
enum class ReviewStatus { queue, approved, rejected };
enum class SampleStage { planlagt, udtaget, sendt, svar };
enum class SampleResult { none, ren, forurenet };
enum class EnvironmentStatus {
  ren_screening,
  afventer,
  forurenet,
  ren_proevesvar
};

std::string_view to_string(Treatment);
std::string_view to_string(ReviewStatus);
std::string_view to_string(SampleStage);
std::string_view to_string(SampleResult); // none -> ""
std::string_view to_string(EnvironmentStatus);
std::optional<Treatment> treatment_from_string(std::string_view);
std::optional<ReviewStatus> review_status_from_string(std::string_view);
std::optional<SampleStage> sample_stage_from_string(std::string_view);
std::optional<SampleResult>
    sample_result_from_string(std::string_view); // "" -> none

struct SampleState {
  SampleStage stage = SampleStage::planlagt;
  SampleResult result = SampleResult::none;
};
void validate_sample_state(const SampleState &); // throws std::invalid_argument
EnvironmentStatus environment_status(const std::vector<SampleState> &linked);

class SamplePendingError : public std::runtime_error {
    public:
  using std::runtime_error::runtime_error;
};

struct TypeTotals {
  Treatment treatment = Treatment::genanvendelse;
  ReviewStatus status = ReviewStatus::queue;
  std::optional<double> mass_t;
  std::string eak_code;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
};
std::array<double, kTreatmentCount>
circularity_breakdown(const std::vector<TypeTotals> &);
struct Fraction {
  std::string eak_code;
  std::string name;
  Treatment treatment;
  double mass_t = 0.0;
};
struct FractionReport {
  std::vector<Fraction> fractions;
  double total_t = 0.0;
  std::size_t blocking_types = 0;
};
FractionReport fractions_by_eak(const std::vector<TypeTotals> &);
std::string_view eak_fraction_name(std::string_view code); // "" when unknown
std::map<std::uint32_t, std::uint32_t>
majority_room(const std::vector<std::uint32_t> &instance_labels,
              const std::vector<std::uint32_t> &room_labels);
std::vector<double> redistribute_quantity(const std::vector<double> &current,
                                          double total);
std::string part_code(int number);   // 1 -> "RX-001"
std::string sample_code(int number); // 1 -> "P-01"

} // namespace reusex::core
