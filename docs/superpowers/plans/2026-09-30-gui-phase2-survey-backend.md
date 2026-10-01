<!--
SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# GUI Phase 2 — Survey Backend Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the Kortlægning / Miljø & prøver / Indberetning screens typed storage, library logic and a REST API: survey types grouping instance-parts, environmental samples that gate approval, derived circularity and waste fractions, `rux create survey`, and server-rendered evidence images with the selected instance highlighted.

**Architecture:** Library-first (DIRECTION.md). Pure rules live in `libs/reusex/include/core/survey.hpp` (no DB); storage in `ProjectDB` (schema v22, four tables); DB-aware operations — approval gate, quantity redistribution, `sync_survey` — in `core/survey_service.hpp`. `rux create survey` and the `rux gui` handlers are thin shells over those. Rendering reuses `visualize::render_view()` with a new instance-highlight option; `rux_gui_lib` stays VTK-free by taking an injected `IViewRenderer`, exactly like `IFrameSegmenter`. The frontend contract layer (types + client) lands here so Phase 3 is UI only.

**Tech Stack:** C++20, sqlite3, nlohmann_json, Crow (via `rux_gui_lib`), CLI11, VTK (via `reusex_visualize`), Catch2 v3, TypeScript + vitest.

**Spec:** `docs/design/gui-kortlaegning-redesign.md` (§ Kortlægning domain model, § Phases 2)

## Global Constraints

- Naming per CLAUDE.md: snake_case functions, PascalCase types, enum values snake_case, no `get_` prefix, members `name_`. Namespace `reusex::core` for library code; `rux::gui` for server code.
- Public headers include siblings with the `reusex/` prefix (`#include "reusex/core/survey.hpp"`), matching `ProjectDB.hpp`.
- Every new file carries the SPDX header (`// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen` + `// SPDX-License-Identifier: GPL-3.0-or-later`).
- Enum wire strings (DB and JSON) are exactly: treatment `bevaring|genbrug|genanvendelse|nyttiggoerelse|bortskaffelse`; review status `queue|approved|rejected`; sample stage `planlagt|udtaget|sendt|svar`; sample result `ren|forurenet` or JSON `null` / DB `''` for none; environment status `ren_screening|afventer|forurenet|ren_proevesvar`.
- Codes: parts `RX-###` (3-digit zero pad, grows past 999 naturally: `RX-1000`); samples `P-##`.
- HTTP: reads through `with_db`, writes through `with_write`; `409` stays "a job holds the writer lock"; approval blocked by a pending sample is **`422`**; invalid body **`400`**; unknown id **`404`**; renderer absent or no GL **`503`**; render asks for data the project lacks **`422`**.
- Every new route: `endpoint_table()` row + `Server.cpp` registration + the literal set in `tests/unit/rux_gui/test_gui_api.cpp` + a `docs/gui/openapi.yaml` path. `scripts/check-openapi.py` must still pass (`ctest -R gui_api_contract_parses`).
- `rux_gui_lib` must not link `reusex_visualize` or `reusex_vision`.
- No silent failure (STANDARDS §5): `sync_survey` with no instance cloud throws naming `rux create instances`; missing/misaligned rooms cloud logs `warn` with the numbers and continues without rooms.
- Build/test inside `nix develop --command …` (gtsam is only in the dev shell). Worktree builds use their own build dir: `nix develop --command cmake -B build -DCMAKE_BUILD_TYPE=Release` then `cmake --build build --parallel`, tests `ctest --test-dir build -R <pattern> --output-on-failure --parallel $(nproc)`. Never pipe a gate command without `set -o pipefail`.
- Frontend: `npm --prefix apps/rux/frontend test` and `run typecheck`.

## Review Focus

- **Re-running sync after a user re-filed a part.** A part moved to another type must stay there; sync only creates parts for instances without one, and never renames a type the user renamed. (Task 5 test `SyncSurvey_Rerun_PreservesUserEdits`.)
- **Approve while a *new* sample is pending on an already-approved type.** Linking a pending sample to an approved type does not un-approve it, but Indberetning must count it as blocking. (Task 1 `FractionsByEak_ApprovedButAfventer_CountsAsBlocking`.)
- **Setting a type's quantity when all parts are 0, or when it has no parts.** Equal split for all-zero; no parts → 422, not a divide-by-zero. (Tasks 1 and 5.)
- **Deleting an instance cloud (`rux create instances --clear`) after survey parts reference it.** Parts keep their code, quantity and room; the part keeps its `instance_guid`; `cloud`/`instance_id` read back null because the guid no longer resolves (see the Task 2 ruling). (Task 2 `SurveyParts_InstanceDeleted_KeepsPartWithNullInstance`.)
- **Concurrent GUI renders.** Two browser tabs requesting renders must not run VTK concurrently; the renderer serialises with a mutex. (Task 8, stated in the implementation; covered by the fake-renderer handler test only for dispatch — VTK concurrency itself is not unit-testable.)

---

## File Structure

| File | Responsibility |
|---|---|
| `libs/reusex/include/core/survey.hpp` + `src/core/survey.cpp` (new) | enums, wire strings, pure derivations |
| `libs/reusex/include/core/ProjectDB.hpp`, `src/core/ProjectDB.cpp` | v22 migration; survey type/part and sample CRUD |
| `libs/reusex/include/core/survey_service.hpp` + `src/core/survey_service.cpp` (new) | gate, redistribution, totals, `sync_survey` |
| `apps/rux/include/create/survey.hpp`, `apps/rux/src/create/survey.cpp` (new), `apps/rux/src/create.cpp` | `rux create survey` |
| `apps/rux/include/gui/survey.hpp`, `apps/rux/src/gui/survey.cpp` (new) | JSON builders + write handlers for survey and samples |
| `apps/rux/src/gui/api.cpp`, `apps/rux/src/gui/Server.cpp`, `apps/rux/include/gui/Server.hpp` | route table, registration, renderer injection |
| `libs/reusex/include/visualize/highlight.hpp` + `src/visualize/highlight.cpp` (new), `render_view.hpp/.cpp` | instance highlight |
| `apps/rux/include/gui/ViewRenderer.hpp` (new), `apps/rux/src/gui/render.cpp` (new), `apps/rux/src/gui.cpp` | renderer interface, `/renders` handler, concrete renderer |
| `docs/gui/openapi.yaml`, `docs/CONTRACTS.md` | contract |
| `apps/rux/frontend/src/api/types.ts`, `client.ts`, `src/test/survey.client.test.ts` (new) | frontend contract layer |
| tests: `tests/unit/core/test_survey.cpp`, `test_project_db_survey.cpp`, `test_project_db_samples.cpp`, `test_survey_service.cpp`, `tests/unit/rux_gui/test_gui_survey.cpp`, `tests/unit/visualize/test_highlight.cpp` (all new) | |

---

### Task 1: Survey rules (pure library)

**Files:**
- Create: `libs/reusex/include/core/survey.hpp`, `libs/reusex/src/core/survey.cpp`
- Test: `tests/unit/core/test_survey.cpp`

**Interfaces:**
- Produces (exact):

```cpp
namespace reusex::core {
enum class Treatment { bevaring, genbrug, genanvendelse, nyttiggoerelse, bortskaffelse };
inline constexpr std::size_t kTreatmentCount = 5;
enum class ReviewStatus { queue, approved, rejected };
enum class SampleStage { planlagt, udtaget, sendt, svar };
enum class SampleResult { none, ren, forurenet };
enum class EnvironmentStatus { ren_screening, afventer, forurenet, ren_proevesvar };

std::string_view to_string(Treatment);
std::string_view to_string(ReviewStatus);
std::string_view to_string(SampleStage);
std::string_view to_string(SampleResult);   // none -> ""
std::string_view to_string(EnvironmentStatus);
std::optional<Treatment> treatment_from_string(std::string_view);
std::optional<ReviewStatus> review_status_from_string(std::string_view);
std::optional<SampleStage> sample_stage_from_string(std::string_view);
std::optional<SampleResult> sample_result_from_string(std::string_view); // "" -> none

struct SampleState { SampleStage stage = SampleStage::planlagt; SampleResult result = SampleResult::none; };
void validate_sample_state(const SampleState &);          // throws std::invalid_argument
EnvironmentStatus environment_status(const std::vector<SampleState> &linked);

class SamplePendingError : public std::runtime_error { public: using std::runtime_error::runtime_error; };

struct TypeTotals {
  Treatment treatment = Treatment::genanvendelse;
  ReviewStatus status = ReviewStatus::queue;
  std::optional<double> mass_t;
  std::string eak_code;
  EnvironmentStatus environment = EnvironmentStatus::ren_screening;
};
std::array<double, kTreatmentCount> circularity_breakdown(const std::vector<TypeTotals> &);
struct Fraction { std::string eak_code; std::string name; Treatment treatment; double mass_t = 0.0; };
struct FractionReport { std::vector<Fraction> fractions; double total_t = 0.0; std::size_t blocking_types = 0; };
FractionReport fractions_by_eak(const std::vector<TypeTotals> &);
std::string_view eak_fraction_name(std::string_view code);   // "" when unknown
std::map<std::uint32_t, std::uint32_t> majority_room(const std::vector<std::uint32_t> &instance_labels,
                                                     const std::vector<std::uint32_t> &room_labels);
std::vector<double> redistribute_quantity(const std::vector<double> &current, double total);
std::string part_code(int number);    // 1 -> "RX-001"
std::string sample_code(int number);  // 1 -> "P-01"
}
```

- [ ] **Step 1: Write the failing test** — create `tests/unit/core/test_survey.cpp`:

```cpp
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
  for (auto t : {Treatment::bevaring, Treatment::genbrug, Treatment::genanvendelse,
                 Treatment::nyttiggoerelse, Treatment::bortskaffelse})
    CHECK(treatment_from_string(to_string(t)) == t);
  CHECK(to_string(Treatment::nyttiggoerelse) == "nyttiggoerelse");
  for (auto s : {ReviewStatus::queue, ReviewStatus::approved, ReviewStatus::rejected})
    CHECK(review_status_from_string(to_string(s)) == s);
  for (auto s : {SampleStage::planlagt, SampleStage::udtaget, SampleStage::sendt, SampleStage::svar})
    CHECK(sample_stage_from_string(to_string(s)) == s);
  CHECK(to_string(SampleResult::none).empty());
  CHECK(sample_result_from_string("") == SampleResult::none);
  CHECK(sample_result_from_string("forurenet") == SampleResult::forurenet);
  CHECK_FALSE(treatment_from_string("Genbrug").has_value()); // case-sensitive
  CHECK(to_string(EnvironmentStatus::ren_proevesvar) == "ren_proevesvar");
}

TEST_CASE("EnvironmentStatus_Derivation", "[survey]") {
  CHECK(environment_status({}) == EnvironmentStatus::ren_screening);
  CHECK(environment_status({{SampleStage::sendt, SampleResult::none}}) == EnvironmentStatus::afventer);
  CHECK(environment_status({{SampleStage::svar, SampleResult::ren}}) == EnvironmentStatus::ren_proevesvar);
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
  CHECK_NOTHROW(validate_sample_state({SampleStage::sendt, SampleResult::none}));
  CHECK_THROWS_AS(validate_sample_state({SampleStage::sendt, SampleResult::ren}), std::invalid_argument);
}

TEST_CASE("CircularityBreakdown_SkipsRejected_NullMassIsZero", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::bevaring, ReviewStatus::approved, 640.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::queue, 58.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::rejected, 999.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::bortskaffelse, ReviewStatus::queue, std::nullopt, "17.06.04", EnvironmentStatus::ren_screening},
  };
  const auto b = circularity_breakdown(types);
  CHECK(b[static_cast<std::size_t>(Treatment::bevaring)] == Approx(640.0));
  CHECK(b[static_cast<std::size_t>(Treatment::genbrug)] == Approx(58.0));
  CHECK(b[static_cast<std::size_t>(Treatment::bortskaffelse)] == Approx(0.0));
}

TEST_CASE("FractionsByEak_GroupsApprovedByCodeAndTreatment", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::genanvendelse, ReviewStatus::approved, 380.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 190.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::bevaring, ReviewStatus::approved, 640.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::genanvendelse, ReviewStatus::approved, 6.8, "17.04.05", EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::queue, 58.0, "17.01.01", EnvironmentStatus::ren_screening},
      {Treatment::genbrug, ReviewStatus::rejected, 5.0, "17.02.01", EnvironmentStatus::ren_screening},
  };
  const auto r = fractions_by_eak(types);
  REQUIRE(r.fractions.size() == 3);
  CHECK(r.fractions[0].eak_code == "17.01.01");
  CHECK(r.fractions[0].treatment == Treatment::bevaring); // treatment order within a code
  CHECK(r.fractions[1].mass_t == Approx(570.0));
  CHECK(r.fractions[1].name == "Beton");
  CHECK(r.fractions[2].eak_code == "17.04.05");
  CHECK(r.total_t == Approx(1216.8));
  CHECK(r.blocking_types == 1); // the queued type; rejected never blocks
}

TEST_CASE("FractionsByEak_ApprovedButAfventer_CountsAsBlocking", "[survey]") {
  std::vector<TypeTotals> types{
      {Treatment::genbrug, ReviewStatus::approved, 3.1, "17.04.02", EnvironmentStatus::afventer}};
  const auto r = fractions_by_eak(types);
  CHECK(r.blocking_types == 1);
  CHECK(r.total_t == Approx(3.1)); // still counted: it is approved
}

TEST_CASE("EakFractionName_KnownAndUnknown", "[survey]") {
  CHECK(eak_fraction_name("17.04.05") == "Jern og stål");
  CHECK(eak_fraction_name("99.99.99").empty());
}

TEST_CASE("MajorityRoom_VotesPerInstance_IgnoresZero_TieToLowerRoom", "[survey]") {
  //                 idx: 0  1  2  3  4  5  6  7
  std::vector<std::uint32_t> inst{1, 1, 1, 2, 2, 0, 3, 3};
  std::vector<std::uint32_t> room{4, 4, 5, 6, 7, 4, 0, 0};
  const auto m = majority_room(inst, room);
  CHECK(m.at(1) == 4);
  CHECK(m.at(2) == 6);      // tie 6 vs 7 -> lower id
  CHECK_FALSE(m.contains(3)); // only unlabeled room points
  CHECK_FALSE(m.contains(0));
  CHECK_THROWS_AS(majority_room({1, 2}, {1}), std::invalid_argument);
}

TEST_CASE("RedistributeQuantity_Proportional_EqualWhenZero_RemainderOnLast", "[survey]") {
  CHECK(redistribute_quantity({18, 6}, 30) == std::vector<double>{22.5, 7.5});
  CHECK(redistribute_quantity({0, 0, 0}, 10) == std::vector<double>{3.33, 3.33, 3.34});
  CHECK(redistribute_quantity({1, 1, 1}, 1) == std::vector<double>{0.33, 0.33, 0.34});
  CHECK(redistribute_quantity({}, 5).empty());
  CHECK_THROWS_AS(redistribute_quantity({1}, -1), std::invalid_argument);
}

TEST_CASE("Codes_ZeroPadded", "[survey]") {
  CHECK(part_code(1) == "RX-001");
  CHECK(part_code(42) == "RX-042");
  CHECK(part_code(1000) == "RX-1000");
  CHECK(sample_code(3) == "P-03");
}
```

- [ ] **Step 2: Configure/build and confirm the failure**

Run: `nix develop --command cmake -B build -DCMAKE_BUILD_TYPE=Release` (first time in this worktree; takes a while), then `nix develop --command cmake --build build --parallel --target reusex_unit_tests`
Expected: FAIL — `core/survey.hpp: No such file or directory`.

- [ ] **Step 3: Implement the header** — `libs/reusex/include/core/survey.hpp` with exactly the declarations in **Interfaces** above, plus includes `<array> <cstddef> <cstdint> <map> <optional> <stdexcept> <string> <string_view> <vector>`, `#pragma once`, and a file comment:

```cpp
/// Survey (Ressourcekortlægning) rules that need no database: the wire
/// vocabulary shared by ProjectDB, the REST API and the frontend, and the
/// derivations the GUI shows — miljøstatus from linked samples, tonnes per
/// affaldshierarki step, approved tonnes per EAK code. Everything here is pure
/// so it is testable without a project, and so `rux`, `rux gui` and a future
/// Qt client all compute the same numbers.
```

- [ ] **Step 4: Implement `libs/reusex/src/core/survey.cpp`**

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <tuple>
#include <unordered_map>

namespace reusex::core {
namespace {

template <typename E, std::size_t N>
std::optional<E> from_table(std::string_view s, const std::array<std::pair<E, std::string_view>, N> &t) {
  for (const auto &[e, name] : t)
    if (name == s)
      return e;
  return std::nullopt;
}

template <typename E, std::size_t N>
std::string_view to_table(E e, const std::array<std::pair<E, std::string_view>, N> &t) {
  for (const auto &[x, name] : t)
    if (x == e)
      return name;
  return {};
}

constexpr std::array<std::pair<Treatment, std::string_view>, 5> kTreatments{{
    {Treatment::bevaring, "bevaring"},
    {Treatment::genbrug, "genbrug"},
    {Treatment::genanvendelse, "genanvendelse"},
    {Treatment::nyttiggoerelse, "nyttiggoerelse"},
    {Treatment::bortskaffelse, "bortskaffelse"},
}};
constexpr std::array<std::pair<ReviewStatus, std::string_view>, 3> kStatuses{{
    {ReviewStatus::queue, "queue"},
    {ReviewStatus::approved, "approved"},
    {ReviewStatus::rejected, "rejected"},
}};
constexpr std::array<std::pair<SampleStage, std::string_view>, 4> kStages{{
    {SampleStage::planlagt, "planlagt"},
    {SampleStage::udtaget, "udtaget"},
    {SampleStage::sendt, "sendt"},
    {SampleStage::svar, "svar"},
}};
constexpr std::array<std::pair<SampleResult, std::string_view>, 3> kResults{{
    {SampleResult::none, ""},
    {SampleResult::ren, "ren"},
    {SampleResult::forurenet, "forurenet"},
}};
constexpr std::array<std::pair<EnvironmentStatus, std::string_view>, 4> kEnvironment{{
    {EnvironmentStatus::ren_screening, "ren_screening"},
    {EnvironmentStatus::afventer, "afventer"},
    {EnvironmentStatus::forurenet, "forurenet"},
    {EnvironmentStatus::ren_proevesvar, "ren_proevesvar"},
}};

/// EAK (European Waste Catalogue) chapter 17 codes a building survey meets.
constexpr std::array<std::pair<std::string_view, std::string_view>, 15> kEak{{
    {"17.01.01", "Beton"},
    {"17.01.02", "Mursten"},
    {"17.01.03", "Tegl og keramik"},
    {"17.01.07", "Blandinger af beton, mursten, tegl"},
    {"17.02.01", "Træ"},
    {"17.02.02", "Glas"},
    {"17.02.03", "Plast"},
    {"17.03.02", "Bitumenblandinger"},
    {"17.04.01", "Kobber, bronze, messing"},
    {"17.04.02", "Aluminium"},
    {"17.04.05", "Jern og stål"},
    {"17.04.07", "Blandede metaller"},
    {"17.06.04", "Isoleringsmateriale"},
    {"17.08.02", "Gipsbaserede materialer"},
    {"17.09.04", "Blandet bygge- og nedrivningsaffald"},
}};

double round2(double x) { return std::round(x * 100.0) / 100.0; }

} // namespace

std::string_view to_string(Treatment v) { return to_table(v, kTreatments); }
std::string_view to_string(ReviewStatus v) { return to_table(v, kStatuses); }
std::string_view to_string(SampleStage v) { return to_table(v, kStages); }
std::string_view to_string(SampleResult v) { return to_table(v, kResults); }
std::string_view to_string(EnvironmentStatus v) { return to_table(v, kEnvironment); }
std::optional<Treatment> treatment_from_string(std::string_view s) { return from_table(s, kTreatments); }
std::optional<ReviewStatus> review_status_from_string(std::string_view s) { return from_table(s, kStatuses); }
std::optional<SampleStage> sample_stage_from_string(std::string_view s) { return from_table(s, kStages); }
std::optional<SampleResult> sample_result_from_string(std::string_view s) { return from_table(s, kResults); }

void validate_sample_state(const SampleState &s) {
  if (s.result != SampleResult::none && s.stage != SampleStage::svar)
    throw std::invalid_argument("a sample result can only be set once the stage is 'svar' "
                                "(answer received), not '" +
                                std::string(to_string(s.stage)) + "'");
}

EnvironmentStatus environment_status(const std::vector<SampleState> &linked) {
  if (linked.empty())
    return EnvironmentStatus::ren_screening;
  bool pending = false;
  for (const auto &s : linked) {
    if (s.result == SampleResult::forurenet)
      return EnvironmentStatus::forurenet;
    if (s.stage != SampleStage::svar)
      pending = true;
  }
  return pending ? EnvironmentStatus::afventer : EnvironmentStatus::ren_proevesvar;
}

std::array<double, kTreatmentCount> circularity_breakdown(const std::vector<TypeTotals> &types) {
  std::array<double, kTreatmentCount> out{};
  for (const auto &t : types)
    if (t.status != ReviewStatus::rejected)
      out[static_cast<std::size_t>(t.treatment)] += t.mass_t.value_or(0.0);
  return out;
}

FractionReport fractions_by_eak(const std::vector<TypeTotals> &types) {
  FractionReport report;
  std::map<std::pair<std::string, int>, double> grouped;
  for (const auto &t : types) {
    if (t.status == ReviewStatus::rejected)
      continue;
    if (t.status != ReviewStatus::approved || t.environment == EnvironmentStatus::afventer)
      ++report.blocking_types;
    if (t.status == ReviewStatus::approved)
      grouped[{t.eak_code, static_cast<int>(t.treatment)}] += t.mass_t.value_or(0.0);
  }
  for (const auto &[key, mass] : grouped) {
    report.fractions.push_back(Fraction{key.first, std::string(eak_fraction_name(key.first)),
                                        static_cast<Treatment>(key.second), mass});
    report.total_t += mass;
  }
  return report;
}

std::string_view eak_fraction_name(std::string_view code) {
  for (const auto &[c, name] : kEak)
    if (c == code)
      return name;
  return {};
}

std::map<std::uint32_t, std::uint32_t> majority_room(const std::vector<std::uint32_t> &instance_labels,
                                                     const std::vector<std::uint32_t> &room_labels) {
  if (instance_labels.size() != room_labels.size())
    throw std::invalid_argument("majority_room: " + std::to_string(instance_labels.size()) +
                                " instance labels but " + std::to_string(room_labels.size()) +
                                " room labels — the clouds are not index-aligned");
  std::map<std::uint32_t, std::map<std::uint32_t, std::size_t>> votes;
  for (std::size_t i = 0; i < instance_labels.size(); ++i)
    if (instance_labels[i] != 0 && room_labels[i] != 0)
      ++votes[instance_labels[i]][room_labels[i]];
  std::map<std::uint32_t, std::uint32_t> out;
  for (const auto &[inst, rooms] : votes) {
    // std::map iterates rooms ascending, and only a strictly larger count
    // replaces the leader, so a tie keeps the lower room id.
    std::uint32_t best = 0;
    std::size_t best_n = 0;
    for (const auto &[room, n] : rooms)
      if (n > best_n) {
        best = room;
        best_n = n;
      }
    out[inst] = best;
  }
  return out;
}

std::vector<double> redistribute_quantity(const std::vector<double> &current, double total) {
  if (total < 0.0)
    throw std::invalid_argument("quantity must not be negative");
  std::vector<double> out(current.size());
  if (current.empty())
    return out;
  const double sum = std::accumulate(current.begin(), current.end(), 0.0);
  double assigned = 0.0;
  for (std::size_t i = 0; i + 1 < current.size(); ++i) {
    const double share = sum > 0.0 ? current[i] / sum : 1.0 / static_cast<double>(current.size());
    out[i] = round2(total * share);
    assigned += out[i];
  }
  out.back() = std::max(0.0, round2(total - assigned));
  return out;
}

std::string part_code(int number) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "RX-%03d", number);
  return buf;
}

std::string sample_code(int number) {
  char buf[16];
  std::snprintf(buf, sizeof buf, "P-%02d", number);
  return buf;
}

} // namespace reusex::core
```

(The test file includes `<core/survey.hpp>` like `test_project_db_reports.cpp` includes `<core/ProjectDB.hpp>`; the source compiles through the `reusex/` prefix. If the include root for tests differs, match whatever `test_project_db_reports.cpp` does.)

- [ ] **Step 5: Build, run, pass**

Run: `nix develop --command cmake --build build --parallel --target reusex_unit_tests && ctest --test-dir build -R 'Survey|Environment|Circularity|Fractions|Eak|Majority|Redistribute|Codes_' --output-on-failure --parallel $(nproc)`
Expected: all PASS. `ctest -N | grep -c Survey` > 0 confirms discovery.

- [ ] **Step 6: Commit**

```bash
git add libs/reusex/include/core/survey.hpp libs/reusex/src/core/survey.cpp tests/unit/core/test_survey.cpp
git commit -m "feat(core): survey rules — wire vocabulary, miljøstatus, circularity, EAK fractions"
```

---

### Task 2: Schema v22 and survey type / part storage

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp` (new section after `// --- Export Templates (schema v21) ---`)
- Modify: `libs/reusex/src/core/ProjectDB.cpp` (`LATEST_SCHEMA_VERSION`, `migrateToV22`, `runMigrations`, CRUD)
- Test: `tests/unit/core/test_project_db_survey.cpp`

**Interfaces:**
- Consumes: Task 1 enums and `to_string`/`*_from_string`, `part_code`.
- Produces (members of `reusex::ProjectDB`):

```cpp
  // --- Survey (Ressourcekortlægning, schema v22) ---
  struct SurveyTypeRecord {
    int64_t id = 0;
    std::string name;
    std::string eak_code;
    std::string bim7aa_code;
    std::string unit = "stk";
    core::Treatment treatment = core::Treatment::genanvendelse;
    core::ReviewStatus review_status = core::ReviewStatus::queue;
    std::optional<double> confidence; // 0..1, AI detection confidence
    std::optional<double> mass_t;     // tonnes
    std::string note;
    bool starred = false;
    int semantic_class = -1;          // the class sync_survey seeded it from
    std::string created_at;           // ISO 8601 UTC
    std::string updated_at;
  };
  /// Sparse update; an engaged optional sets the field. For the nullable
  /// numbers the outer optional says "change it", the inner one the new value.
  struct SurveyTypePatch {
    std::optional<std::string> name, eak_code, bim7aa_code, unit, note;
    std::optional<core::Treatment> treatment;
    std::optional<core::ReviewStatus> review_status;
    std::optional<std::optional<double>> confidence, mass_t;
    std::optional<bool> starred;
  };
  struct SurveyPartRecord {
    std::string code;                          // "RX-001"
    int64_t type_id = 0;
    std::optional<std::string> cloud_name;     // null once the instance is gone
    std::optional<std::uint32_t> instance_id;
    std::optional<std::uint32_t> room_id;
    std::string room_name;
    double quantity = 1.0;
    bool starred = false;
    std::string note;
    std::optional<std::string> material_guid;  // read-only: from instance_materials
  };
  struct SurveyPartPatch {
    std::optional<int64_t> type_id;
    std::optional<double> quantity;
    std::optional<bool> starred;
    std::optional<std::string> note, room_name;
  };

  SurveyTypeRecord add_survey_type(const SurveyTypeRecord &rec); // id/timestamps ignored
  [[nodiscard]] std::vector<SurveyTypeRecord> survey_types() const; // by id
  [[nodiscard]] std::optional<SurveyTypeRecord> survey_type(int64_t id) const;
  SurveyTypeRecord update_survey_type(int64_t id, const SurveyTypePatch &patch);
  void add_survey_part(const SurveyPartRecord &rec);
  [[nodiscard]] std::vector<SurveyPartRecord> survey_parts() const; // by code
  [[nodiscard]] std::optional<SurveyPartRecord> survey_part(std::string_view code) const;
  SurveyPartRecord update_survey_part(std::string_view code, const SurveyPartPatch &patch);
  [[nodiscard]] bool has_survey_part_for(std::string_view cloud_name, std::uint32_t instance_id) const;
  [[nodiscard]] int max_survey_part_number() const; // 0 when there are none
```

Errors: unknown type/part id → `std::out_of_range` with the id in the message (the API maps it to 404); an invalid `type_id` in a part patch → `std::out_of_range`; a part whose `cloud_name` does not exist → `std::runtime_error` (from `getCloudId`).

- [ ] **Step 1: Write the failing test** — `tests/unit/core/test_project_db_survey.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22: survey_types / survey_parts round trips, patches, the
// instance-deleted SET NULL rule, and the v21 -> v22 migration.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <sqlite3.h>

#include <stdexcept>
#include <string>

using reusex::ProjectDB;
namespace core = reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_survey") {}
};

/// A 3-point instance cloud with instance 1 on two points and instance 2 on one.
void add_instance_cloud(ProjectDB &db) {
  reusex::CloudL labels;
  for (std::uint32_t l : {1u, 1u, 2u}) {
    pcl::Label p;
    p.label = l;
    labels.push_back(p);
  }
  db.save_point_cloud("instances", labels, "test", "{}");
  db.save_instances("instances", {{1, "guid-inst-1", 3, 2}, {2, "guid-inst-2", 5, 1}});
}

ProjectDB::SurveyTypeRecord window_type() {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Vinduespartier, aluminium";
  t.eak_code = "17.04.02";
  t.bim7aa_code = "312 Udv. vinduer";
  t.treatment = core::Treatment::genbrug;
  t.confidence = 0.82;
  t.mass_t = 3.1;
  t.note = "Ved ren fuge: salg som brugte partier.";
  t.semantic_class = 3;
  return t;
}
} // namespace

TEST_CASE("SurveyTypes_AddListGet_RoundTrip", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto added = db.add_survey_type(window_type());
  REQUIRE(added.id > 0);
  CHECK(!added.created_at.empty());
  const auto all = db.survey_types();
  REQUIRE(all.size() == 1);
  const auto &t = all.front();
  CHECK(t.name == "Vinduespartier, aluminium");
  CHECK(t.treatment == core::Treatment::genbrug);
  CHECK(t.review_status == core::ReviewStatus::queue);
  CHECK(t.confidence == 0.82);
  CHECK(t.mass_t == 3.1);
  CHECK(t.unit == "stk");
  CHECK(t.semantic_class == 3);
  CHECK(db.survey_type(added.id).has_value());
  CHECK_FALSE(db.survey_type(added.id + 100).has_value());
}

TEST_CASE("SurveyTypes_Patch_ChangesOnlyEngagedFields_AndClearsNullables", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = db.add_survey_type(window_type()).id;
  ProjectDB::SurveyTypePatch p;
  p.review_status = core::ReviewStatus::approved;
  p.mass_t = std::optional<double>{}; // clear
  p.starred = true;
  const auto t = db.update_survey_type(id, p);
  CHECK(t.review_status == core::ReviewStatus::approved);
  CHECK_FALSE(t.mass_t.has_value());
  CHECK(t.starred);
  CHECK(t.name == "Vinduespartier, aluminium"); // untouched
  CHECK(t.confidence == 0.82);
  CHECK_THROWS_AS(db.update_survey_type(id + 100, p), std::out_of_range);
}

TEST_CASE("SurveyParts_AddList_JoinsMaterialGuid_AndCodeOrder", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_instance_cloud(db);
  const auto type_id = db.add_survey_type(window_type()).id;
  db.add_survey_part({"RX-009", type_id, "instances", 2, 7, "Production Hall", 12, false, "", {}});
  db.add_survey_part({"RX-008", type_id, "instances", 1, 4, "Office Zone", 26, true, "note", {}});
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 2);
  CHECK(parts[0].code == "RX-008");
  CHECK(parts[0].cloud_name == "instances");
  CHECK(parts[0].instance_id == 1u);
  CHECK(parts[0].room_name == "Office Zone");
  CHECK(parts[0].quantity == 26);
  CHECK(parts[0].starred);
  CHECK_FALSE(parts[0].material_guid.has_value()); // no passport linked yet
  CHECK(db.has_survey_part_for("instances", 2));
  CHECK_FALSE(db.has_survey_part_for("instances", 9));
  CHECK(db.max_survey_part_number() == 9);
}

TEST_CASE("SurveyParts_Patch_MovesToOtherType_RejectsUnknownType", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = db.add_survey_type(window_type()).id;
  auto other = window_type();
  other.name = "Indvendige døre, træ";
  const auto b = db.add_survey_type(other).id;
  db.add_survey_part({"RX-001", a, std::nullopt, std::nullopt, std::nullopt, "", 1, false, "", {}});
  ProjectDB::SurveyPartPatch p;
  p.type_id = b;
  p.quantity = 14;
  const auto part = db.update_survey_part("RX-001", p);
  CHECK(part.type_id == b);
  CHECK(part.quantity == 14);
  p.type_id = b + 100;
  CHECK_THROWS_AS(db.update_survey_part("RX-001", p), std::out_of_range);
  CHECK_THROWS_AS(db.update_survey_part("RX-404", {}), std::out_of_range);
}

TEST_CASE("SurveyParts_InstanceDeleted_KeepsPartWithNullInstance", "[ProjectDB][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_instance_cloud(db);
  const auto type_id = db.add_survey_type(window_type()).id;
  db.add_survey_part({"RX-001", type_id, "instances", 1, std::nullopt, "", 5, false, "", {}});
  db.delete_point_cloud("instances"); // cascades to instances(cloud_id, …)
  const auto part = db.survey_part("RX-001");
  REQUIRE(part.has_value());
  CHECK(part->quantity == 5);
  CHECK_FALSE(part->cloud_name.has_value());
  CHECK_FALSE(part->instance_id.has_value());
}

TEST_CASE("SurveySchema_MigratesFromV21", "[ProjectDB][survey][migration]") {
  TempDB tmp;
  { ProjectDB db(tmp.path); } // fresh DB at the latest version
  {
    // Roll it back to v21 by removing everything v22 adds.
    sqlite3 *raw = nullptr;
    REQUIRE(sqlite3_open(tmp.path.string().c_str(), &raw) == SQLITE_OK);
    const char *sql = "DROP TABLE sample_links; DROP TABLE samples; DROP TABLE survey_parts;"
                      "DROP TABLE survey_types; DELETE FROM schema_version WHERE version = 22;";
    REQUIRE(sqlite3_exec(raw, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(raw);
  }
  ProjectDB db(tmp.path, /*readOnly=*/false);
  CHECK(db.survey_types().empty());
  CHECK(db.add_survey_type(window_type()).id > 0);
}
```

Before writing it, confirm the helper names this test uses against `ProjectDB.hpp`: `save_point_cloud(name, CloudL, stage, params)`, `save_instances(cloud, {InstanceRecord…})` with `InstanceRecord{instance_id, guid, semantic_class, point_count}`, and `delete_point_cloud(name)` (grep `delete_point_cloud\|remove_point_cloud`). Adjust the calls to the real signatures — the assertions stay as written. If `delete_point_cloud` does not cascade to `instances`, delete the `instances` rows via the existing API or raw sqlite in the test and keep the assertion (the rule under test is `ON DELETE SET NULL` on `survey_parts`).

- [ ] **Step 2: Build and confirm failure**

Run: `nix develop --command cmake --build build --parallel --target reusex_unit_tests`
Expected: FAIL — `SurveyTypeRecord` is not a member of `ProjectDB`.

- [ ] **Step 3: Declarations** — add `#include "reusex/core/survey.hpp"` next to the other `reusex/core/` include in `ProjectDB.hpp` and paste the **Interfaces** block after the v21 export-template section.

- [ ] **Step 4: Migration** — in `ProjectDB.cpp` set `LATEST_SCHEMA_VERSION = 22`, add `if (current < 22) migrateToV22();` after the v21 line in `runMigrations`, and add after `migrateToV21()`:

```cpp
  void migrateToV22() {
    reusex::info("Migrating database to schema version 22");

    // Ressourcekortlægning (docs/design/gui-kortlaegning-redesign.md): survey
    // types group instance-parts; samples gate approval. Enum columns are
    // CHECK-constrained to the wire strings in core/survey.hpp.
    const char *v22_schema = R"(
      CREATE TABLE IF NOT EXISTS survey_types (
        id             INTEGER PRIMARY KEY AUTOINCREMENT,
        name           TEXT    NOT NULL,
        eak_code       TEXT    NOT NULL DEFAULT '',
        bim7aa_code    TEXT    NOT NULL DEFAULT '',
        unit           TEXT    NOT NULL DEFAULT 'stk',
        treatment      TEXT    NOT NULL DEFAULT 'genanvendelse'
          CHECK (treatment IN ('bevaring','genbrug','genanvendelse','nyttiggoerelse','bortskaffelse')),
        review_status  TEXT    NOT NULL DEFAULT 'queue'
          CHECK (review_status IN ('queue','approved','rejected')),
        confidence     REAL,
        mass_t         REAL,
        note           TEXT    NOT NULL DEFAULT '',
        starred        INTEGER NOT NULL DEFAULT 0,
        semantic_class INTEGER NOT NULL DEFAULT -1,
        created_at     TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        updated_at     TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS survey_parts (
        code        TEXT    PRIMARY KEY,
        type_id     INTEGER NOT NULL REFERENCES survey_types(id) ON DELETE CASCADE,
        cloud_id    INTEGER,
        instance_id INTEGER,
        room_id     INTEGER,
        room_name   TEXT    NOT NULL DEFAULT '',
        quantity    REAL    NOT NULL DEFAULT 1,
        starred     INTEGER NOT NULL DEFAULT 0,
        note        TEXT    NOT NULL DEFAULT '',
        UNIQUE (cloud_id, instance_id),
        FOREIGN KEY (cloud_id, instance_id)
          REFERENCES instances(cloud_id, instance_id) ON DELETE SET NULL
      );
      CREATE INDEX IF NOT EXISTS idx_survey_parts_type ON survey_parts(type_id);
      CREATE TABLE IF NOT EXISTS samples (
        id         INTEGER PRIMARY KEY AUTOINCREMENT,
        code       TEXT    NOT NULL UNIQUE,
        title      TEXT    NOT NULL,
        what       TEXT    NOT NULL DEFAULT '',
        stage      TEXT    NOT NULL DEFAULT 'planlagt'
          CHECK (stage IN ('planlagt','udtaget','sendt','svar')),
        result     TEXT    NOT NULL DEFAULT ''
          CHECK (result IN ('','ren','forurenet')),
        created_at TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now')),
        updated_at TEXT    NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%SZ','now'))
      );
      CREATE TABLE IF NOT EXISTS sample_links (
        sample_id INTEGER NOT NULL REFERENCES samples(id) ON DELETE CASCADE,
        type_id   INTEGER NOT NULL REFERENCES survey_types(id) ON DELETE CASCADE,
        PRIMARY KEY (sample_id, type_id)
      );
    )";

    char *errMsg = nullptr;
    if (sqlite3_exec(db, v22_schema, nullptr, nullptr, &errMsg) != SQLITE_OK) {
      std::string error = errMsg ? errMsg : "unknown error";
      sqlite3_free(errMsg);
      throw std::runtime_error("Migration to v22 failed: " + error);
    }

    insertSchemaVersion(22, "Add survey types/parts and samples for Kortlægning");
    reusex::info("Migration to schema version 22 complete");
  }
```

(The samples tables are created here because one migration per schema version is the house style; their CRUD lands in Task 3. `ON DELETE SET NULL` on a composite key nulls both columns — which is exactly the "instance gone, part kept" rule.)

- [ ] **Step 5: CRUD** — add near the export-template methods in `ProjectDB.cpp`. First a small file-local helper block (put it in the anonymous namespace at the top of the file, next to `StmtGuard`):

```cpp
std::string column_text(sqlite3_stmt *s, int i) {
  const auto *t = reinterpret_cast<const char *>(sqlite3_column_text(s, i));
  return t ? t : "";
}
std::optional<double> column_opt_double(sqlite3_stmt *s, int i) {
  if (sqlite3_column_type(s, i) == SQLITE_NULL)
    return std::nullopt;
  return sqlite3_column_double(s, i);
}
std::optional<std::int64_t> column_opt_int64(sqlite3_stmt *s, int i) {
  if (sqlite3_column_type(s, i) == SQLITE_NULL)
    return std::nullopt;
  return sqlite3_column_int64(s, i);
}
void bind_opt_double(sqlite3_stmt *s, int i, const std::optional<double> &v) {
  v ? sqlite3_bind_double(s, i, *v) : sqlite3_bind_null(s, i);
}
void bind_text(sqlite3_stmt *s, int i, std::string_view v) {
  sqlite3_bind_text(s, i, v.data(), static_cast<int>(v.size()), SQLITE_TRANSIENT);
}
/// prepare_v2 that throws with the caller's name and sqlite's message.
sqlite3_stmt *prepare_or_throw(sqlite3 *db, const char *sql, const char *who) {
  sqlite3_stmt *stmt = nullptr;
  if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK)
    throw std::runtime_error(std::string(who) + ": prepare failed: " + sqlite3_errmsg(db));
  return stmt;
}
```

(If any of these names already exist in the file, reuse them instead of redefining.)

Then the methods:

```cpp
namespace {
constexpr const char *kSurveyTypeColumns =
    "id, name, eak_code, bim7aa_code, unit, treatment, review_status, confidence, mass_t, "
    "note, starred, semantic_class, created_at, updated_at";

ProjectDB::SurveyTypeRecord read_survey_type(sqlite3_stmt *s) {
  ProjectDB::SurveyTypeRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.name = column_text(s, 1);
  r.eak_code = column_text(s, 2);
  r.bim7aa_code = column_text(s, 3);
  r.unit = column_text(s, 4);
  r.treatment = core::treatment_from_string(column_text(s, 5)).value_or(core::Treatment::genanvendelse);
  r.review_status = core::review_status_from_string(column_text(s, 6)).value_or(core::ReviewStatus::queue);
  r.confidence = column_opt_double(s, 7);
  r.mass_t = column_opt_double(s, 8);
  r.note = column_text(s, 9);
  r.starred = sqlite3_column_int(s, 10) != 0;
  r.semantic_class = sqlite3_column_int(s, 11);
  r.created_at = column_text(s, 12);
  r.updated_at = column_text(s, 13);
  return r;
}
} // namespace

ProjectDB::SurveyTypeRecord ProjectDB::add_survey_type(const SurveyTypeRecord &rec) {
  impl_->checkWritable();
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
      "INSERT INTO survey_types (name, eak_code, bim7aa_code, unit, treatment, review_status, "
      "confidence, mass_t, note, starred, semantic_class) VALUES (?,?,?,?,?,?,?,?,?,?,?) "
      "RETURNING id;", "add_survey_type");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, rec.name);
  bind_text(stmt, 2, rec.eak_code);
  bind_text(stmt, 3, rec.bim7aa_code);
  bind_text(stmt, 4, rec.unit);
  bind_text(stmt, 5, core::to_string(rec.treatment));
  bind_text(stmt, 6, core::to_string(rec.review_status));
  bind_opt_double(stmt, 7, rec.confidence);
  bind_opt_double(stmt, 8, rec.mass_t);
  bind_text(stmt, 9, rec.note);
  sqlite3_bind_int(stmt, 10, rec.starred ? 1 : 0);
  sqlite3_bind_int(stmt, 11, rec.semantic_class);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("add_survey_type: insert failed: " + std::string(sqlite3_errmsg(impl_->db)));
  const auto id = sqlite3_column_int64(stmt, 0);
  return *survey_type(id);
}

std::vector<ProjectDB::SurveyTypeRecord> ProjectDB::survey_types() const {
  const std::string sql = std::string("SELECT ") + kSurveyTypeColumns + " FROM survey_types ORDER BY id;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_types");
  StmtGuard guard(stmt);
  std::vector<SurveyTypeRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_survey_type(stmt));
  return out;
}

std::optional<ProjectDB::SurveyTypeRecord> ProjectDB::survey_type(int64_t id) const {
  const std::string sql = std::string("SELECT ") + kSurveyTypeColumns + " FROM survey_types WHERE id = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_type");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_survey_type(stmt);
}

ProjectDB::SurveyTypeRecord ProjectDB::update_survey_type(int64_t id, const SurveyTypePatch &p) {
  impl_->checkWritable();
  if (!survey_type(id))
    throw std::out_of_range("no survey type " + std::to_string(id));
  // Built from a fixed column vocabulary, never from input: only the bound
  // values come from the caller.
  std::vector<std::string> sets;
  if (p.name) sets.emplace_back("name = ?");
  if (p.eak_code) sets.emplace_back("eak_code = ?");
  if (p.bim7aa_code) sets.emplace_back("bim7aa_code = ?");
  if (p.unit) sets.emplace_back("unit = ?");
  if (p.note) sets.emplace_back("note = ?");
  if (p.treatment) sets.emplace_back("treatment = ?");
  if (p.review_status) sets.emplace_back("review_status = ?");
  if (p.confidence) sets.emplace_back("confidence = ?");
  if (p.mass_t) sets.emplace_back("mass_t = ?");
  if (p.starred) sets.emplace_back("starred = ?");
  if (sets.empty())
    return *survey_type(id);
  std::string sql = "UPDATE survey_types SET ";
  for (const auto &s : sets)
    sql += s + ", ";
  sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "update_survey_type");
  StmtGuard guard(stmt);
  int i = 1;
  if (p.name) bind_text(stmt, i++, *p.name);
  if (p.eak_code) bind_text(stmt, i++, *p.eak_code);
  if (p.bim7aa_code) bind_text(stmt, i++, *p.bim7aa_code);
  if (p.unit) bind_text(stmt, i++, *p.unit);
  if (p.note) bind_text(stmt, i++, *p.note);
  if (p.treatment) bind_text(stmt, i++, core::to_string(*p.treatment));
  if (p.review_status) bind_text(stmt, i++, core::to_string(*p.review_status));
  if (p.confidence) bind_opt_double(stmt, i++, *p.confidence);
  if (p.mass_t) bind_opt_double(stmt, i++, *p.mass_t);
  if (p.starred) sqlite3_bind_int(stmt, i++, *p.starred ? 1 : 0);
  sqlite3_bind_int64(stmt, i, id);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("update_survey_type: " + std::string(sqlite3_errmsg(impl_->db)));
  return *survey_type(id);
}

namespace {
/// Parts joined to their cloud name and linked passport; one query shape for
/// list and single lookups.
constexpr const char *kSurveyPartSelect =
    "SELECT p.code, p.type_id, pc.name, p.instance_id, p.room_id, p.room_name, p.quantity, "
    "p.starred, p.note, im.material_guid "
    "FROM survey_parts p "
    "LEFT JOIN point_clouds pc ON pc.id = p.cloud_id "
    "LEFT JOIN instance_materials im ON im.cloud_id = p.cloud_id AND im.instance_id = p.instance_id ";

ProjectDB::SurveyPartRecord read_survey_part(sqlite3_stmt *s) {
  ProjectDB::SurveyPartRecord r;
  r.code = column_text(s, 0);
  r.type_id = sqlite3_column_int64(s, 1);
  if (sqlite3_column_type(s, 2) != SQLITE_NULL) r.cloud_name = column_text(s, 2);
  if (auto v = column_opt_int64(s, 3)) r.instance_id = static_cast<std::uint32_t>(*v);
  if (auto v = column_opt_int64(s, 4)) r.room_id = static_cast<std::uint32_t>(*v);
  r.room_name = column_text(s, 5);
  r.quantity = sqlite3_column_double(s, 6);
  r.starred = sqlite3_column_int(s, 7) != 0;
  r.note = column_text(s, 8);
  if (sqlite3_column_type(s, 9) != SQLITE_NULL) r.material_guid = column_text(s, 9);
  return r;
}
} // namespace

void ProjectDB::add_survey_part(const SurveyPartRecord &rec) {
  impl_->checkWritable();
  if (!survey_type(rec.type_id))
    throw std::out_of_range("no survey type " + std::to_string(rec.type_id));
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
      "INSERT INTO survey_parts (code, type_id, cloud_id, instance_id, room_id, room_name, "
      "quantity, starred, note) VALUES (?,?,?,?,?,?,?,?,?);", "add_survey_part");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, rec.code);
  sqlite3_bind_int64(stmt, 2, rec.type_id);
  if (rec.cloud_name && rec.instance_id) {
    sqlite3_bind_int(stmt, 3, impl_->getCloudId(*rec.cloud_name));
    sqlite3_bind_int64(stmt, 4, *rec.instance_id);
  } else {
    sqlite3_bind_null(stmt, 3);
    sqlite3_bind_null(stmt, 4);
  }
  rec.room_id ? sqlite3_bind_int64(stmt, 5, *rec.room_id) : sqlite3_bind_null(stmt, 5);
  bind_text(stmt, 6, rec.room_name);
  sqlite3_bind_double(stmt, 7, rec.quantity);
  sqlite3_bind_int(stmt, 8, rec.starred ? 1 : 0);
  bind_text(stmt, 9, rec.note);
  if (sqlite3_step(stmt) != SQLITE_DONE)
    throw std::runtime_error("add_survey_part '" + rec.code + "': " + sqlite3_errmsg(impl_->db));
}

std::vector<ProjectDB::SurveyPartRecord> ProjectDB::survey_parts() const {
  const std::string sql = std::string(kSurveyPartSelect) + "ORDER BY p.code;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_parts");
  StmtGuard guard(stmt);
  std::vector<SurveyPartRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_survey_part(stmt));
  return out;
}

std::optional<ProjectDB::SurveyPartRecord> ProjectDB::survey_part(std::string_view code) const {
  const std::string sql = std::string(kSurveyPartSelect) + "WHERE p.code = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "survey_part");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, code);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  return read_survey_part(stmt);
}

ProjectDB::SurveyPartRecord ProjectDB::update_survey_part(std::string_view code, const SurveyPartPatch &p) {
  impl_->checkWritable();
  if (!survey_part(code))
    throw std::out_of_range("no survey part '" + std::string(code) + "'");
  if (p.type_id && !survey_type(*p.type_id))
    throw std::out_of_range("no survey type " + std::to_string(*p.type_id));
  std::vector<std::string> sets;
  if (p.type_id) sets.emplace_back("type_id = ?");
  if (p.quantity) sets.emplace_back("quantity = ?");
  if (p.starred) sets.emplace_back("starred = ?");
  if (p.note) sets.emplace_back("note = ?");
  if (p.room_name) sets.emplace_back("room_name = ?");
  if (!sets.empty()) {
    std::string sql = "UPDATE survey_parts SET ";
    for (std::size_t k = 0; k < sets.size(); ++k)
      sql += (k ? ", " : "") + sets[k];
    sql += " WHERE code = ?;";
    sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "update_survey_part");
    StmtGuard guard(stmt);
    int i = 1;
    if (p.type_id) sqlite3_bind_int64(stmt, i++, *p.type_id);
    if (p.quantity) sqlite3_bind_double(stmt, i++, *p.quantity);
    if (p.starred) sqlite3_bind_int(stmt, i++, *p.starred ? 1 : 0);
    if (p.note) bind_text(stmt, i++, *p.note);
    if (p.room_name) bind_text(stmt, i++, *p.room_name);
    bind_text(stmt, i, code);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("update_survey_part: " + std::string(sqlite3_errmsg(impl_->db)));
  }
  return *survey_part(code);
}

bool ProjectDB::has_survey_part_for(std::string_view cloud_name, std::uint32_t instance_id) const {
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
      "SELECT 1 FROM survey_parts p JOIN point_clouds pc ON pc.id = p.cloud_id "
      "WHERE pc.name = ? AND p.instance_id = ?;", "has_survey_part_for");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, cloud_name);
  sqlite3_bind_int64(stmt, 2, instance_id);
  return sqlite3_step(stmt) == SQLITE_ROW;
}

int ProjectDB::max_survey_part_number() const {
  // Codes are "RX-<n>"; substr from 4 skips the prefix.
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
      "SELECT COALESCE(MAX(CAST(substr(code, 4) AS INTEGER)), 0) FROM survey_parts "
      "WHERE code LIKE 'RX-%';", "max_survey_part_number");
  StmtGuard guard(stmt);
  sqlite3_step(stmt);
  return sqlite3_column_int(stmt, 0);
}
```

(`impl_->getCloudId` is the Impl helper at `ProjectDB.cpp` ~2904; if it is private to Impl, it is already reachable here because these are `ProjectDB` members using `impl_`.)

- [ ] **Step 6: Build, run, pass**

Run: `nix develop --command cmake --build build --parallel --target reusex_unit_tests && ctest --test-dir build -R 'SurveyTypes|SurveyParts|SurveySchema' --output-on-failure --parallel $(nproc)`
Expected: PASS. Also run the existing migration/report tests: `ctest --test-dir build -R 'ProjectDB|Migration|ReportPdfs|ExportTemplate' --output-on-failure --parallel $(nproc)` → PASS (v22 bump must not break them; if one asserts `LATEST == 21`, update it to 22).

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp tests/unit/core/test_project_db_survey.cpp
git commit -m "feat(core): schema v22 — survey types and parts storage"
```

---

### Task 3: Sample storage

**Files:**
- Modify: `libs/reusex/include/core/ProjectDB.hpp`, `libs/reusex/src/core/ProjectDB.cpp`
- Test: `tests/unit/core/test_project_db_samples.cpp`

**Interfaces:**
- Consumes: Task 2 tables and helpers (`column_text`, `bind_text`, `prepare_or_throw`), Task 1 `sample_code`, stage/result conversions.
- Produces:

```cpp
  struct SampleRecord {
    int64_t id = 0;
    std::string code;  // "P-01"
    std::string title; // "PCB i fugemasse"
    std::string what;  // what was sampled, where
    core::SampleStage stage = core::SampleStage::planlagt;
    core::SampleResult result = core::SampleResult::none;
    std::vector<int64_t> type_ids; // linked survey types, ascending
    std::string created_at;
    std::string updated_at;
  };
  struct SamplePatch {
    std::optional<std::string> title, what;
    std::optional<core::SampleStage> stage;
    std::optional<core::SampleResult> result;
  };
  SampleRecord add_sample(std::string_view title, std::string_view what);
  [[nodiscard]] std::vector<SampleRecord> samples() const; // by id
  [[nodiscard]] std::optional<SampleRecord> sample(int64_t id) const;
  SampleRecord update_sample(int64_t id, const SamplePatch &patch); // storage only; no stage rules
  bool delete_sample(int64_t id);                                    // false when absent
  void set_sample_links(int64_t id, const std::vector<int64_t> &type_ids); // replaces the set
  [[nodiscard]] std::vector<SampleRecord> samples_for_type(int64_t type_id) const;
```

Unknown sample → `std::out_of_range`; unknown type in `set_sample_links` → `std::out_of_range` naming it, with no partial write (run in one transaction).

- [ ] **Step 1: Failing test** — `tests/unit/core/test_project_db_samples.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Schema v22 samples: codes, stage/result storage, link replacement,
// per-type lookup, and cascade on type deletion.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>

using reusex::ProjectDB;
namespace core = reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_projectdb_samples") {}
};
int64_t add_type(ProjectDB &db, const char *name) {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  return db.add_survey_type(t).id;
}
} // namespace

TEST_CASE("Samples_Add_AssignsSequentialCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = db.add_sample("PCB i fugemasse", "Fugemasse omkring vinduespartier");
  const auto b = db.add_sample("Bly i maling", "Malede indervægge");
  CHECK(a.code == "P-01");
  CHECK(b.code == "P-02");
  CHECK(a.stage == core::SampleStage::planlagt);
  CHECK(a.result == core::SampleResult::none);
  CHECK(db.samples().size() == 2);
}

TEST_CASE("Samples_Delete_DoesNotReuseCodes", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_sample("a", "");
  const auto b = db.add_sample("b", "");
  CHECK(db.delete_sample(b.id));
  CHECK_FALSE(db.delete_sample(b.id));
  // A lab report quoting P-02 must never come to mean a different sample.
  CHECK(db.add_sample("c", "").code == "P-03");
}

TEST_CASE("Samples_UpdateStageAndResult", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("Asbest i linoleumslim", "Gulvlim, Office Zone");
  ProjectDB::SamplePatch p;
  p.stage = core::SampleStage::svar;
  p.result = core::SampleResult::forurenet;
  const auto u = db.update_sample(s.id, p);
  CHECK(u.stage == core::SampleStage::svar);
  CHECK(u.result == core::SampleResult::forurenet);
  CHECK(u.title == "Asbest i linoleumslim");
  CHECK_THROWS_AS(db.update_sample(s.id + 50, p), std::out_of_range);
}

TEST_CASE("Samples_Links_ReplaceSet_PerTypeLookup_AtomicOnUnknownType", "[ProjectDB][samples]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto walls = add_type(db, "Indvendige murvægge, malet");
  const auto windows = add_type(db, "Vinduespartier");
  const auto s = db.add_sample("Bly i maling", "");
  db.set_sample_links(s.id, {windows, walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls, windows});
  db.set_sample_links(s.id, {walls});
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls});
  CHECK(db.samples_for_type(walls).size() == 1);
  CHECK(db.samples_for_type(windows).empty());
  CHECK_THROWS_AS(db.set_sample_links(s.id, {walls, 9999}), std::out_of_range);
  CHECK(db.sample(s.id)->type_ids == std::vector<int64_t>{walls}); // unchanged
}
```

- [ ] **Step 2: Build → FAIL** (`SampleRecord` not a member).

- [ ] **Step 3: Implement** — add the declarations from **Interfaces** after the survey block. Implementation in `ProjectDB.cpp`:

```cpp
namespace {
ProjectDB::SampleRecord read_sample_row(sqlite3_stmt *s) {
  ProjectDB::SampleRecord r;
  r.id = sqlite3_column_int64(s, 0);
  r.code = column_text(s, 1);
  r.title = column_text(s, 2);
  r.what = column_text(s, 3);
  r.stage = core::sample_stage_from_string(column_text(s, 4)).value_or(core::SampleStage::planlagt);
  r.result = core::sample_result_from_string(column_text(s, 5)).value_or(core::SampleResult::none);
  r.created_at = column_text(s, 6);
  r.updated_at = column_text(s, 7);
  return r;
}
constexpr const char *kSampleColumns = "id, code, title, what, stage, result, created_at, updated_at";
} // namespace

// Fills type_ids for each record in place.
static void attach_sample_links(sqlite3 *db, std::vector<ProjectDB::SampleRecord> &records) {
  sqlite3_stmt *stmt = prepare_or_throw(db,
      "SELECT type_id FROM sample_links WHERE sample_id = ? ORDER BY type_id;", "sample_links");
  StmtGuard guard(stmt);
  for (auto &r : records) {
    sqlite3_reset(stmt);
    sqlite3_bind_int64(stmt, 1, r.id);
    while (sqlite3_step(stmt) == SQLITE_ROW)
      r.type_ids.push_back(sqlite3_column_int64(stmt, 0));
  }
}

ProjectDB::SampleRecord ProjectDB::add_sample(std::string_view title, std::string_view what) {
  impl_->checkWritable();
  // Codes are never reused: take the highest ever issued (AUTOINCREMENT's
  // sqlite_sequence survives deletes), not the current count.
  sqlite3_stmt *seq = prepare_or_throw(impl_->db,
      "SELECT COALESCE((SELECT seq FROM sqlite_sequence WHERE name = 'samples'), 0);", "add_sample");
  StmtGuard seq_guard(seq);
  sqlite3_step(seq);
  const int next = sqlite3_column_int(seq, 0) + 1;

  sqlite3_stmt *stmt = prepare_or_throw(impl_->db,
      "INSERT INTO samples (code, title, what) VALUES (?,?,?) RETURNING id;", "add_sample");
  StmtGuard guard(stmt);
  bind_text(stmt, 1, core::sample_code(next));
  bind_text(stmt, 2, title);
  bind_text(stmt, 3, what);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    throw std::runtime_error("add_sample: " + std::string(sqlite3_errmsg(impl_->db)));
  return *sample(sqlite3_column_int64(stmt, 0));
}

std::vector<ProjectDB::SampleRecord> ProjectDB::samples() const {
  const std::string sql = std::string("SELECT ") + kSampleColumns + " FROM samples ORDER BY id;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "samples");
  StmtGuard guard(stmt);
  std::vector<SampleRecord> out;
  while (sqlite3_step(stmt) == SQLITE_ROW)
    out.push_back(read_sample_row(stmt));
  attach_sample_links(impl_->db, out);
  return out;
}

std::optional<ProjectDB::SampleRecord> ProjectDB::sample(int64_t id) const {
  const std::string sql = std::string("SELECT ") + kSampleColumns + " FROM samples WHERE id = ?;";
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "sample");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  if (sqlite3_step(stmt) != SQLITE_ROW)
    return std::nullopt;
  std::vector<SampleRecord> one{read_sample_row(stmt)};
  attach_sample_links(impl_->db, one);
  return one.front();
}

ProjectDB::SampleRecord ProjectDB::update_sample(int64_t id, const SamplePatch &p) {
  impl_->checkWritable();
  if (!sample(id))
    throw std::out_of_range("no sample " + std::to_string(id));
  std::vector<std::string> sets;
  if (p.title) sets.emplace_back("title = ?");
  if (p.what) sets.emplace_back("what = ?");
  if (p.stage) sets.emplace_back("stage = ?");
  if (p.result) sets.emplace_back("result = ?");
  if (!sets.empty()) {
    std::string sql = "UPDATE samples SET ";
    for (const auto &s : sets)
      sql += s + ", ";
    sql += "updated_at = strftime('%Y-%m-%dT%H:%M:%SZ','now') WHERE id = ?;";
    sqlite3_stmt *stmt = prepare_or_throw(impl_->db, sql.c_str(), "update_sample");
    StmtGuard guard(stmt);
    int i = 1;
    if (p.title) bind_text(stmt, i++, *p.title);
    if (p.what) bind_text(stmt, i++, *p.what);
    if (p.stage) bind_text(stmt, i++, core::to_string(*p.stage));
    if (p.result) bind_text(stmt, i++, core::to_string(*p.result));
    sqlite3_bind_int64(stmt, i, id);
    if (sqlite3_step(stmt) != SQLITE_DONE)
      throw std::runtime_error("update_sample: " + std::string(sqlite3_errmsg(impl_->db)));
  }
  return *sample(id);
}

bool ProjectDB::delete_sample(int64_t id) {
  impl_->checkWritable();
  sqlite3_stmt *stmt = prepare_or_throw(impl_->db, "DELETE FROM samples WHERE id = ?;", "delete_sample");
  StmtGuard guard(stmt);
  sqlite3_bind_int64(stmt, 1, id);
  sqlite3_step(stmt);
  return sqlite3_changes(impl_->db) > 0;
}

void ProjectDB::set_sample_links(int64_t id, const std::vector<int64_t> &type_ids) {
  impl_->checkWritable();
  if (!sample(id))
    throw std::out_of_range("no sample " + std::to_string(id));
  for (auto t : type_ids)
    if (!survey_type(t))
      throw std::out_of_range("no survey type " + std::to_string(t));
  sqlite3_exec(impl_->db, "BEGIN;", nullptr, nullptr, nullptr);
  try {
    sqlite3_stmt *del = prepare_or_throw(impl_->db, "DELETE FROM sample_links WHERE sample_id = ?;", "set_sample_links");
    StmtGuard del_guard(del);
    sqlite3_bind_int64(del, 1, id);
    sqlite3_step(del);
    sqlite3_stmt *ins = prepare_or_throw(impl_->db,
        "INSERT OR IGNORE INTO sample_links (sample_id, type_id) VALUES (?, ?);", "set_sample_links");
    StmtGuard ins_guard(ins);
    for (auto t : type_ids) {
      sqlite3_reset(ins);
      sqlite3_bind_int64(ins, 1, id);
      sqlite3_bind_int64(ins, 2, t);
      if (sqlite3_step(ins) != SQLITE_DONE)
        throw std::runtime_error("set_sample_links: " + std::string(sqlite3_errmsg(impl_->db)));
    }
    sqlite3_exec(impl_->db, "COMMIT;", nullptr, nullptr, nullptr);
  } catch (...) {
    sqlite3_exec(impl_->db, "ROLLBACK;", nullptr, nullptr, nullptr);
    throw;
  }
}

std::vector<ProjectDB::SampleRecord> ProjectDB::samples_for_type(int64_t type_id) const {
  std::vector<SampleRecord> out;
  for (auto &s : samples())
    if (std::find(s.type_ids.begin(), s.type_ids.end(), type_id) != s.type_ids.end())
      out.push_back(std::move(s));
  return out;
}
```

- [ ] **Step 4: Build, run** `ctest --test-dir build -R 'Samples_' --output-on-failure --parallel $(nproc)` → PASS.

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/core/ProjectDB.hpp libs/reusex/src/core/ProjectDB.cpp tests/unit/core/test_project_db_samples.cpp
git commit -m "feat(core): environmental sample storage with type links"
```

---

### Task 4: Survey service — gate, redistribution, totals

**Files:**
- Create: `libs/reusex/include/core/survey_service.hpp`, `libs/reusex/src/core/survey_service.cpp`
- Test: `tests/unit/core/test_survey_service.cpp`

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces:

```cpp
namespace reusex::core {
std::map<int64_t, EnvironmentStatus> environment_statuses(const ProjectDB &db); // every type
EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id);
/// Approving a type whose miljøstatus is `afventer` throws SamplePendingError.
ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id, ReviewStatus status);
/// Scales the type's parts to sum to `total` (redistribute_quantity).
/// Throws std::invalid_argument when the type has no parts or total < 0.
std::vector<ProjectDB::SurveyPartRecord> set_type_quantity(ProjectDB &db, int64_t type_id, double total);
std::vector<TypeTotals> type_totals(const ProjectDB &db); // one per survey type, same order
/// Sample edit with the stage/result rule enforced (validate_sample_state on the merged state).
ProjectDB::SampleRecord update_sample_checked(ProjectDB &db, int64_t id, const ProjectDB::SamplePatch &patch);
}
```

- [ ] **Step 1: Failing test** — `tests/unit/core/test_survey_service.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey operations that need a project: the sample approval gate, quantity
// redistribution across parts, totals, and the checked sample edit.

#include <catch2/catch_test_macros.hpp>
#include <core/ProjectDB.hpp>
#include <core/survey_service.hpp>

#include "../../support/temp_path.hpp"

#include <stdexcept>

using reusex::ProjectDB;
using namespace reusex::core;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_survey_service") {}
};
int64_t type_with_parts(ProjectDB &db, std::vector<double> quantities) {
  ProjectDB::SurveyTypeRecord t;
  t.name = "Betonsøjler, bærende";
  t.mass_t = 58.0;
  const auto id = db.add_survey_type(t).id;
  int n = db.max_survey_part_number();
  for (double q : quantities)
    db.add_survey_part({part_code(++n), id, std::nullopt, std::nullopt, std::nullopt, "", q, false, "", {}});
  return id;
}
} // namespace

TEST_CASE("SetReviewStatus_PendingSample_BlocksApproval_NotRejection", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {1});
  const auto s = db.add_sample("PCB i fugemasse", "");
  db.set_sample_links(s.id, {id});
  CHECK(environment_status_of(db, id) == EnvironmentStatus::afventer);
  CHECK_THROWS_AS(set_review_status(db, id, ReviewStatus::approved), SamplePendingError);
  CHECK(db.survey_type(id)->review_status == ReviewStatus::queue);
  CHECK(set_review_status(db, id, ReviewStatus::rejected).review_status == ReviewStatus::rejected);

  ProjectDB::SamplePatch answered;
  answered.stage = SampleStage::svar;
  answered.result = SampleResult::ren;
  update_sample_checked(db, s.id, answered);
  CHECK(environment_status_of(db, id) == EnvironmentStatus::ren_proevesvar);
  CHECK(set_review_status(db, id, ReviewStatus::approved).review_status == ReviewStatus::approved);
}

TEST_CASE("SetTypeQuantity_RedistributesAcrossParts", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = type_with_parts(db, {18, 6});
  const auto parts = set_type_quantity(db, id, 30);
  REQUIRE(parts.size() == 2);
  CHECK(parts[0].quantity == 22.5);
  CHECK(parts[1].quantity == 7.5);
  const auto empty = type_with_parts(db, {});
  CHECK_THROWS_AS(set_type_quantity(db, empty, 5), std::invalid_argument);
  CHECK_THROWS_AS(set_type_quantity(db, id, -1), std::invalid_argument);
}

TEST_CASE("UpdateSampleChecked_ResultBeforeAnswer_Throws", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto s = db.add_sample("x", "");
  ProjectDB::SamplePatch p;
  p.result = SampleResult::ren;
  CHECK_THROWS_AS(update_sample_checked(db, s.id, p), std::invalid_argument);
  CHECK(db.sample(s.id)->result == SampleResult::none);
}

TEST_CASE("TypeTotals_CarryEnvironment", "[survey][service]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = type_with_parts(db, {1});
  type_with_parts(db, {1});
  const auto s = db.add_sample("x", "");
  db.set_sample_links(s.id, {a});
  const auto totals = type_totals(db);
  REQUIRE(totals.size() == 2);
  CHECK(totals[0].environment == EnvironmentStatus::afventer);
  CHECK(totals[1].environment == EnvironmentStatus::ren_screening);
  CHECK(totals[0].mass_t == 58.0);
}
```

- [ ] **Step 2: Build → FAIL** (`core/survey_service.hpp` missing).

- [ ] **Step 3: Implement**

`libs/reusex/include/core/survey_service.hpp`: `#pragma once`, `#include "reusex/core/ProjectDB.hpp"`, `#include "reusex/core/survey.hpp"`, `<map> <vector>`, the **Interfaces** declarations, and a file comment: "Survey operations that need a project. Thin over ProjectDB storage and the pure rules in survey.hpp — the gate and redistribution live here, not in ProjectDB, which stays storage-only."

`libs/reusex/src/core/survey_service.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/core/survey_service.hpp"

#include <stdexcept>

namespace reusex::core {

std::map<int64_t, EnvironmentStatus> environment_statuses(const ProjectDB &db) {
  std::map<int64_t, std::vector<SampleState>> linked;
  for (const auto &s : db.samples())
    for (auto t : s.type_ids)
      linked[t].push_back({s.stage, s.result});
  std::map<int64_t, EnvironmentStatus> out;
  for (const auto &t : db.survey_types())
    out[t.id] = environment_status(linked[t.id]);
  return out;
}

EnvironmentStatus environment_status_of(const ProjectDB &db, int64_t type_id) {
  std::vector<SampleState> linked;
  for (const auto &s : db.samples_for_type(type_id))
    linked.push_back({s.stage, s.result});
  return environment_status(linked);
}

ProjectDB::SurveyTypeRecord set_review_status(ProjectDB &db, int64_t type_id, ReviewStatus status) {
  if (!db.survey_type(type_id))
    throw std::out_of_range("no survey type " + std::to_string(type_id));
  if (status == ReviewStatus::approved &&
      environment_status_of(db, type_id) == EnvironmentStatus::afventer) {
    std::string codes;
    for (const auto &s : db.samples_for_type(type_id))
      if (s.stage != SampleStage::svar)
        codes += (codes.empty() ? "" : ", ") + s.code;
    throw SamplePendingError("survey type " + std::to_string(type_id) +
                             " cannot be approved while sample(s) " + codes +
                             " await a lab answer");
  }
  ProjectDB::SurveyTypePatch p;
  p.review_status = status;
  return db.update_survey_type(type_id, p);
}

std::vector<ProjectDB::SurveyPartRecord> set_type_quantity(ProjectDB &db, int64_t type_id, double total) {
  if (total < 0.0)
    throw std::invalid_argument("quantity must not be negative");
  std::vector<ProjectDB::SurveyPartRecord> parts;
  for (auto &p : db.survey_parts())
    if (p.type_id == type_id)
      parts.push_back(std::move(p));
  if (parts.empty())
    throw std::invalid_argument("survey type " + std::to_string(type_id) +
                                " has no parts to distribute a quantity over");
  std::vector<double> current;
  for (const auto &p : parts)
    current.push_back(p.quantity);
  const auto next = redistribute_quantity(current, total);
  std::vector<ProjectDB::SurveyPartRecord> out;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    ProjectDB::SurveyPartPatch patch;
    patch.quantity = next[i];
    out.push_back(db.update_survey_part(parts[i].code, patch));
  }
  return out;
}

std::vector<TypeTotals> type_totals(const ProjectDB &db) {
  const auto env = environment_statuses(db);
  std::vector<TypeTotals> out;
  for (const auto &t : db.survey_types())
    out.push_back({t.treatment, t.review_status, t.mass_t, t.eak_code, env.at(t.id)});
  return out;
}

ProjectDB::SampleRecord update_sample_checked(ProjectDB &db, int64_t id, const ProjectDB::SamplePatch &patch) {
  const auto current = db.sample(id);
  if (!current)
    throw std::out_of_range("no sample " + std::to_string(id));
  validate_sample_state({patch.stage.value_or(current->stage), patch.result.value_or(current->result)});
  return db.update_sample(id, patch);
}

} // namespace reusex::core
```

- [ ] **Step 4: Build, run** `ctest --test-dir build -R 'SetReviewStatus|SetTypeQuantity|UpdateSampleChecked|TypeTotals' --output-on-failure --parallel $(nproc)` → PASS.

- [ ] **Step 5: Commit**

```bash
git add libs/reusex/include/core/survey_service.hpp libs/reusex/src/core/survey_service.cpp tests/unit/core/test_survey_service.cpp
git commit -m "feat(core): survey service — sample approval gate, quantity redistribution, totals"
```

---

### Task 5: `sync_survey` and `rux create survey`

**Files:**
- Modify: `libs/reusex/include/core/survey_service.hpp`, `libs/reusex/src/core/survey_service.cpp`
- Create: `apps/rux/include/create/survey.hpp`, `apps/rux/src/create/survey.cpp`
- Modify: `apps/rux/src/create.cpp` (register), `docs/CONTRACTS.md` (stage entry)
- Test: append to `tests/unit/core/test_survey_service.cpp`

**Interfaces:**
- Produces:

```cpp
namespace reusex::core {
struct SurveySyncOptions {
  std::string instances_cloud = "instances"; // mirrors `rux create materials`
  std::string semantic_cloud = "labels";
  std::string rooms_cloud = "rooms";
};
struct SurveySyncReport {
  std::size_t types_created = 0;
  std::size_t parts_created = 0;
  std::size_t parts_existing = 0;
  bool rooms_assigned = false;
};
SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts = {});
}
```

Rules: instances processed in ascending `instance_id`; one type per `semantic_class` among types whose `semantic_class` matches (first by id wins), created with name = semantic label definition, else `"Klasse <n>"`, or `"Uklassificeret"` for `-1`; existing parts untouched; room = `majority_room` over the instance and rooms label clouds, name = rooms label definition, else `"Rum <id>"`; no instance cloud → `std::runtime_error` whose message contains `rux create instances`; rooms cloud missing or size-mismatched → `warn` with both sizes and `rooms_assigned = false`.

- [ ] **Step 1: Failing tests** — append to `test_survey_service.cpp` (add `#include <pcl/point_types.h>` if `CloudL` needs it — follow how other core tests build a `CloudL`):

```cpp
namespace {
reusex::CloudL label_cloud(std::initializer_list<std::uint32_t> labels) {
  reusex::CloudL c;
  for (auto l : labels) {
    pcl::Label p;
    p.label = l;
    c.push_back(p);
  }
  return c;
}
/// Instances 1,2 (class 3 = window), 3 (class 5 = door); rooms 4 and 6.
void seed_scan(ProjectDB &db, bool with_rooms = true) {
  db.save_point_cloud("instances", label_cloud({1, 1, 2, 2, 3, 0}), "test", "{}");
  db.save_instances("instances", {{1, "g1", 3, 2}, {2, "g2", 3, 2}, {3, "g3", 5, 1}});
  db.save_label_definitions("labels", {{3, "window"}, {5, "door"}});
  if (with_rooms) {
    db.save_point_cloud("rooms", label_cloud({4, 4, 6, 6, 6, 0}), "test", "{}");
    db.save_label_definitions("rooms", {{4, "Office Zone"}});
  }
}
} // namespace

TEST_CASE("SyncSurvey_CreatesTypePerClass_PartPerInstance_WithRooms", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  const auto r = sync_survey(db);
  CHECK(r.types_created == 2);
  CHECK(r.parts_created == 3);
  CHECK(r.rooms_assigned);
  const auto types = db.survey_types();
  REQUIRE(types.size() == 2);
  CHECK(types[0].name == "window");
  CHECK(types[0].semantic_class == 3);
  const auto parts = db.survey_parts();
  REQUIRE(parts.size() == 3);
  CHECK(parts[0].code == "RX-001");
  CHECK(parts[0].instance_id == 1u);
  CHECK(parts[0].room_name == "Office Zone");
  CHECK(parts[1].room_id == 6u);
  CHECK(parts[1].room_name == "Rum 6");
  CHECK(parts[2].type_id == types[1].id);
}

TEST_CASE("SyncSurvey_Rerun_PreservesUserEdits", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db);
  sync_survey(db);
  const auto types = db.survey_types();
  ProjectDB::SurveyTypePatch rename;
  rename.name = "Vinduespartier, aluminium";
  db.update_survey_type(types[0].id, rename);
  ProjectDB::SurveyPartPatch move;
  move.type_id = types[1].id;
  move.quantity = 26;
  db.update_survey_part("RX-001", move);

  const auto r = sync_survey(db);
  CHECK(r.types_created == 0);
  CHECK(r.parts_created == 0);
  CHECK(r.parts_existing == 3);
  CHECK(db.survey_type(types[0].id)->name == "Vinduespartier, aluminium");
  CHECK(db.survey_part("RX-001")->type_id == types[1].id);
  CHECK(db.survey_part("RX-001")->quantity == 26);
}

TEST_CASE("SyncSurvey_NoInstanceCloud_ThrowsNamingTheStage", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  try {
    sync_survey(db);
    FAIL("expected a throw");
  } catch (const std::runtime_error &e) {
    CHECK(std::string(e.what()).find("rux create instances") != std::string::npos);
  }
}

TEST_CASE("SyncSurvey_WithoutRooms_StillCreatesParts", "[survey][sync]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  seed_scan(db, /*with_rooms=*/false);
  const auto r = sync_survey(db);
  CHECK(r.parts_created == 3);
  CHECK_FALSE(r.rooms_assigned);
  CHECK(db.survey_parts()[0].room_name.empty());
}
```

Confirm `save_label_definitions(std::string_view, const std::map<int,std::string>&)` (it exists at `ProjectDB.hpp:304`); label definitions may require the cloud to exist first — if so, `save_point_cloud("labels", label_cloud({3,3,3,3,5,0}), …)` before defining them, and keep the assertions.

- [ ] **Step 2: Build → FAIL** (`sync_survey` undeclared).

- [ ] **Step 3: Implement** — add the **Interfaces** block to `survey_service.hpp`, and to `survey_service.cpp` (add `#include "reusex/core/logging.hpp"`):

```cpp
namespace {
std::vector<std::uint32_t> labels_of(const ProjectDB &db, const std::string &name) {
  std::vector<std::uint32_t> out;
  if (const auto cloud = db.point_cloud_label(name))
    for (const auto &p : *cloud)
      out.push_back(p.label);
  return out;
}
} // namespace

SurveySyncReport sync_survey(ProjectDB &db, const SurveySyncOptions &opts) {
  if (!db.has_point_cloud(opts.instances_cloud))
    throw std::runtime_error("sync_survey: no instance cloud '" + opts.instances_cloud +
                             "' — run `rux create instances` first");
  SurveySyncReport report;

  // Room per instance, when a rooms cloud aligned with the instance cloud exists.
  std::map<std::uint32_t, std::uint32_t> room_of;
  std::map<int, std::string> room_names;
  if (db.has_point_cloud(opts.rooms_cloud)) {
    const auto inst = labels_of(db, opts.instances_cloud);
    const auto rooms = labels_of(db, opts.rooms_cloud);
    if (inst.size() == rooms.size()) {
      room_of = majority_room(inst, rooms);
      room_names = db.label_definitions(opts.rooms_cloud);
      report.rooms_assigned = true;
    } else {
      reusex::warn("sync_survey: '{}' has {} labels but '{}' has {} — clouds are out of sync; "
                   "parts get no room (re-run `rux create rooms`)",
                   opts.rooms_cloud, rooms.size(), opts.instances_cloud, inst.size());
    }
  } else {
    reusex::warn("sync_survey: no '{}' cloud; parts get no room (run `rux create rooms`)",
                 opts.rooms_cloud);
  }

  const auto class_names = db.has_point_cloud(opts.semantic_cloud)
                               ? db.label_definitions(opts.semantic_cloud)
                               : std::map<int, std::string>{};
  std::map<int, int64_t> type_for_class;
  for (const auto &t : db.survey_types())
    type_for_class.try_emplace(t.semantic_class, t.id);

  auto instances = db.instances(opts.instances_cloud);
  std::sort(instances.begin(), instances.end(),
            [](const auto &a, const auto &b) { return a.instance_id < b.instance_id; });
  int next = db.max_survey_part_number();
  for (const auto &inst : instances) {
    if (db.has_survey_part_for(opts.instances_cloud, inst.instance_id)) {
      ++report.parts_existing;
      continue;
    }
    auto it = type_for_class.find(inst.semantic_class);
    if (it == type_for_class.end()) {
      ProjectDB::SurveyTypeRecord t;
      const auto name = class_names.find(inst.semantic_class);
      t.name = inst.semantic_class < 0 ? "Uklassificeret"
               : name != class_names.end() ? name->second
                                           : "Klasse " + std::to_string(inst.semantic_class);
      t.semantic_class = inst.semantic_class;
      it = type_for_class.emplace(inst.semantic_class, db.add_survey_type(t).id).first;
      ++report.types_created;
    }
    ProjectDB::SurveyPartRecord part;
    part.code = part_code(++next);
    part.type_id = it->second;
    part.cloud_name = opts.instances_cloud;
    part.instance_id = inst.instance_id;
    if (const auto r = room_of.find(inst.instance_id); r != room_of.end()) {
      part.room_id = r->second;
      const auto n = room_names.find(static_cast<int>(r->second));
      part.room_name = n != room_names.end() ? n->second : "Rum " + std::to_string(r->second);
    }
    db.add_survey_part(part);
    ++report.parts_created;
  }
  if (instances.empty())
    reusex::warn("sync_survey: instance cloud '{}' has no instances; nothing to survey",
                 opts.instances_cloud);
  return report;
}
```

(Add `#include <algorithm>`. `InstanceRecord::instance_id` is `uint32_t`, `semantic_class` is `int`.)

- [ ] **Step 4: CLI** — `apps/rux/include/create/survey.hpp` declares `struct SubcommandCreateSurveyOptions { reusex::core::SurveySyncOptions sync; };` and `void setup_subcommand_create_survey(CLI::App &parent, std::shared_ptr<RuxOptions> global_opt);` (copy the include/guard style of `create/materials.hpp`). `apps/rux/src/create/survey.cpp`, modelled on `create/materials.cpp` (options, footer, callback that opens `ProjectDB(global_opt->project_path, false)` the same way materials does, calls `reusex::core::sync_survey`, logs `spdlog::info("Survey: {} type(s) and {} part(s) created, {} part(s) already present{}", …, report.rooms_assigned ? "" : " (no rooms assigned)")`, returns `exit_status` success; on `std::exception` logs `spdlog::error` and returns the failure status the other create commands use). Flags: `-i,--instances` → `sync.instances_cloud`, `--semantic` → `sync.semantic_cloud`, `--rooms` → `sync.rooms_cloud`, each with `->default_val(SurveySyncOptions{}.<field>)` (STANDARDS §4: defaults come from the library struct). Footer:

```
DESCRIPTION:
  Fills the Ressourcekortlægning (Kortlægning in 'rux gui'): one survey type
  per semantic class and one bygningsdel (RX-###) per instance, each placed in
  the room most of its points fall in. Idempotent — re-running only adds
  instances that have no part yet and never overwrites edits made in the GUI.

EXAMPLES:
  rux create survey
  rux -p scan.rux create survey --rooms rooms

WORKFLOW:
  1. rux create instances
  2. rux create rooms          # optional, for room assignment
  3. rux create survey
  4. rux gui                   # review in Kortlægning
```

Register with `setup_subcommand_create_survey(*sub, global_opt);` after the `materials` line in `apps/rux/src/create.cpp`, and add `survey` to the `create` row of the CLI table in `CLAUDE.md`.

- [ ] **Step 5: CONTRACTS.md** — add a `survey` entry in the stage list with: *Consumes* `instances` (Label) + `instances` table, optional `rooms` (Label) and `labels` definitions; *Produces* `survey_types`, `survey_parts`; *Idempotent* yes. Follow the format of the neighbouring `materials` entry.

- [ ] **Step 6: Build and run**

Run: `nix develop --command cmake --build build --parallel && ctest --test-dir build -R 'SyncSurvey' --output-on-failure --parallel $(nproc)` → PASS.
Smoke: `cp tests/fixtures/scans/office_corridor.rux /tmp/claude-survey.rux && ./build/apps/rux/rux -p /tmp/claude-survey.rux create survey` — either it reports counts, or (if the fixture has no instance cloud) it exits non-zero with the "run `rux create instances` first" message. Record which in the report; both are correct behaviour. Delete the copy afterwards.

- [ ] **Step 7: Commit**

```bash
git add libs/reusex/include/core/survey_service.hpp libs/reusex/src/core/survey_service.cpp \
  tests/unit/core/test_survey_service.cpp apps/rux/include/create/survey.hpp apps/rux/src/create/survey.cpp \
  apps/rux/src/create.cpp docs/CONTRACTS.md CLAUDE.md
git commit -m "feat(survey): sync_survey and \`rux create survey\`"
```

---

### Task 6: GUI read endpoints — survey, summary, fractions, samples

**Files:**
- Create: `apps/rux/include/gui/survey.hpp`, `apps/rux/src/gui/survey.cpp`
- Modify: `apps/rux/src/gui/api.cpp` (`endpoint_table`), `apps/rux/src/gui/Server.cpp` (`register_routes`)
- Modify: `tests/unit/rux_gui/test_gui_api.cpp` (route set), `docs/gui/openapi.yaml`
- Test: `tests/unit/rux_gui/test_gui_survey.cpp`

**Interfaces:**
- Consumes: Tasks 1–5.
- Produces (namespace `rux::gui`, all taking `const reusex::ProjectDB &db`):

```cpp
nlohmann::json survey_type_json(const reusex::ProjectDB::SurveyTypeRecord &t,
                                const std::vector<reusex::ProjectDB::SurveyPartRecord> &parts,
                                reusex::core::EnvironmentStatus env,
                                const std::vector<int64_t> &sample_ids);
nlohmann::json survey_part_json(const reusex::ProjectDB::SurveyPartRecord &p);
nlohmann::json sample_json(const reusex::ProjectDB::SampleRecord &s);
nlohmann::json survey_json(const reusex::ProjectDB &db);           // GET /survey
nlohmann::json survey_summary_json(const reusex::ProjectDB &db);   // GET /survey/summary
nlohmann::json survey_fractions_json(const reusex::ProjectDB &db); // GET /survey/fractions
nlohmann::json samples_json(const reusex::ProjectDB &db);          // GET /samples
```

JSON shapes (exact keys):

- **SurveyPart**: `code`, `type_id`, `cloud` (string|null), `instance_id` (int|null), `room_id` (int|null), `room_name`, `quantity`, `starred`, `note`, `material_guid` (string|null).
- **SurveyType**: `id`, `name`, `eak_code`, `eak_name`, `bim7aa_code`, `unit`, `treatment`, `review_status`, `confidence` (number|null), `mass_t` (number|null), `note`, `starred`, `semantic_class`, `environment_status`, `sample_ids` (int[]), `quantity` (sum of part quantities), `parts` (SurveyPart[] by code), `created_at`, `updated_at`.
- **GET /survey**: `{ "types": SurveyType[] (by id, rejected included), "counts": { "queue": n, "approved": n, "rejected": n, "all": queue+approved } }`.
- **GET /survey/summary**: `{ "counts": {…as above}, "circularity": { "bevaring": t, "genbrug": t, "genanvendelse": t, "nyttiggoerelse": t, "bortskaffelse": t }, "total_mass_t": t, "reuse_share": 0..1 | null (bevaring+genbrug over total; null when total is 0), "pending_samples": n (stage ≠ svar), "unlabeled_points": n | null (label 0 in the `instances` cloud; null without it), "rooms_without_parts": string[] (room names from `rooms` label definitions / "Rum <id>" for room ids present in the `rooms` cloud with no part; [] without it) }`.
- **GET /survey/fractions**: `{ "fractions": [{ "eak_code", "name", "treatment", "mass_t" }], "total_t", "blocking_types", "ready": blocking_types == 0 }`.
- **Sample**: `id`, `code`, `title`, `what`, `stage`, `result` (string|null), `type_ids`, `created_at`, `updated_at`. **GET /samples**: `{ "samples": Sample[] }`.

- [ ] **Step 1: Failing test** — `tests/unit/rux_gui/test_gui_survey.cpp` (direct handler calls, like `test_gui_edits.cpp`):

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Survey and sample JSON (Kortlægning): shapes, counts, derived miljøstatus,
// circularity and fractions as the frontend reads them.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "gui/survey.hpp"
#include "../../support/temp_path.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/survey_service.hpp>

using namespace rux::gui;
using reusex::ProjectDB;
namespace core = reusex::core;
using Catch::Approx;

namespace {
struct TempDB : reusex::test_support::TempPath {
  TempDB() : reusex::test_support::TempPath("test_gui_survey") {}
};
int64_t add_type(ProjectDB &db, const char *name, core::Treatment tr, double mass,
                 core::ReviewStatus st = core::ReviewStatus::queue, const char *eak = "17.01.01") {
  ProjectDB::SurveyTypeRecord t;
  t.name = name;
  t.treatment = tr;
  t.mass_t = mass;
  t.review_status = st;
  t.eak_code = eak;
  return db.add_survey_type(t).id;
}
} // namespace

TEST_CASE("SurveyJson_TypesWithParts_Counts_Environment", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  add_type(db, "Beton, fundament", core::Treatment::bevaring, 640, core::ReviewStatus::approved);
  add_type(db, "Fejl", core::Treatment::genbrug, 1, core::ReviewStatus::rejected);
  db.add_survey_part({"RX-002", a, std::nullopt, std::nullopt, 6, "Entrance", 6, false, "", {}});
  db.add_survey_part({"RX-001", a, std::nullopt, std::nullopt, 4, "Production Hall", 18, false, "", {}});
  const auto s = db.add_sample("PCB", "");
  db.set_sample_links(s.id, {a});

  const auto j = survey_json(db);
  CHECK(j.at("counts").at("queue") == 1);
  CHECK(j.at("counts").at("approved") == 1);
  CHECK(j.at("counts").at("rejected") == 1);
  CHECK(j.at("counts").at("all") == 2);
  const auto &t = j.at("types").at(0);
  CHECK(t.at("name") == "Betonsøjler");
  CHECK(t.at("treatment") == "genbrug");
  CHECK(t.at("environment_status") == "afventer");
  CHECK(t.at("sample_ids") == nlohmann::json::array({s.id}));
  CHECK(t.at("quantity").get<double>() == Approx(24));
  CHECK(t.at("eak_name") == "Beton");
  CHECK(t.at("confidence").is_null());
  REQUIRE(t.at("parts").size() == 2);
  CHECK(t.at("parts").at(0).at("code") == "RX-001");
  CHECK(t.at("parts").at(0).at("cloud").is_null());
  CHECK(t.at("parts").at(0).at("room_name") == "Production Hall");
}

TEST_CASE("SurveySummaryJson_Circularity_ReuseShare_PendingSamples", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::bevaring, 60);
  add_type(db, "b", core::Treatment::genanvendelse, 40);
  db.add_sample("pending", "");
  const auto j = survey_summary_json(db);
  CHECK(j.at("circularity").at("bevaring").get<double>() == Approx(60));
  CHECK(j.at("circularity").at("nyttiggoerelse").get<double>() == Approx(0));
  CHECK(j.at("total_mass_t").get<double>() == Approx(100));
  CHECK(j.at("reuse_share").get<double>() == Approx(0.6));
  CHECK(j.at("pending_samples") == 1);
  CHECK(j.at("unlabeled_points").is_null());       // no instance cloud
  CHECK(j.at("rooms_without_parts") == nlohmann::json::array());
}

TEST_CASE("SurveySummaryJson_EmptyProject_ReuseShareNull", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(survey_summary_json(db).at("reuse_share").is_null());
}

TEST_CASE("SurveyFractionsJson_ApprovedOnly_ReadyFlag", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  add_type(db, "a", core::Treatment::genanvendelse, 380, core::ReviewStatus::approved);
  add_type(db, "b", core::Treatment::genbrug, 58);
  auto j = survey_fractions_json(db);
  REQUIRE(j.at("fractions").size() == 1);
  CHECK(j.at("fractions").at(0).at("name") == "Beton");
  CHECK(j.at("fractions").at(0).at("treatment") == "genanvendelse");
  CHECK(j.at("blocking_types") == 1);
  CHECK(j.at("ready") == false);
}

TEST_CASE("SamplesJson_ResultNullWhenNone", "[gui][survey]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  db.add_sample("PCB i fugemasse", "Fugemasse");
  const auto j = samples_json(db);
  REQUIRE(j.at("samples").size() == 1);
  CHECK(j.at("samples").at(0).at("code") == "P-01");
  CHECK(j.at("samples").at(0).at("stage") == "planlagt");
  CHECK(j.at("samples").at(0).at("result").is_null());
}
```

Match the include style of `tests/unit/rux_gui/test_gui_edits.cpp` (it may include `<gui/edits.hpp>` or `"gui/edits.hpp"`, and `<core/ProjectDB.hpp>` vs `<reusex/core/ProjectDB.hpp>`); keep the assertions.

- [ ] **Step 2: Build → FAIL** (`gui/survey.hpp` missing).

- [ ] **Step 3: Implement `apps/rux/include/gui/survey.hpp`** (declarations from **Interfaces** plus Task 7's write handlers — add those in Task 7) **and `apps/rux/src/gui/survey.cpp`**:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/survey.hpp"

#include <reusex/core/survey.hpp>
#include <reusex/core/survey_service.hpp>

#include <map>
#include <set>

namespace rux::gui {
namespace {
using json = nlohmann::json;
namespace core = reusex::core;

template <typename T> json opt(const std::optional<T> &v) { return v ? json(*v) : json(nullptr); }

json counts_json(const std::vector<reusex::ProjectDB::SurveyTypeRecord> &types) {
  int queue = 0, approved = 0, rejected = 0;
  for (const auto &t : types) {
    if (t.review_status == core::ReviewStatus::queue) ++queue;
    else if (t.review_status == core::ReviewStatus::approved) ++approved;
    else ++rejected;
  }
  return {{"queue", queue}, {"approved", approved}, {"rejected", rejected}, {"all", queue + approved}};
}
} // namespace

json survey_part_json(const reusex::ProjectDB::SurveyPartRecord &p) {
  return {{"code", p.code},
          {"type_id", p.type_id},
          {"cloud", opt(p.cloud_name)},
          {"instance_id", opt(p.instance_id)},
          {"room_id", opt(p.room_id)},
          {"room_name", p.room_name},
          {"quantity", p.quantity},
          {"starred", p.starred},
          {"note", p.note},
          {"material_guid", opt(p.material_guid)}};
}

json survey_type_json(const reusex::ProjectDB::SurveyTypeRecord &t,
                      const std::vector<reusex::ProjectDB::SurveyPartRecord> &parts,
                      core::EnvironmentStatus env, const std::vector<int64_t> &sample_ids) {
  json part_list = json::array();
  double quantity = 0.0;
  for (const auto &p : parts) {
    part_list.push_back(survey_part_json(p));
    quantity += p.quantity;
  }
  return {{"id", t.id},
          {"name", t.name},
          {"eak_code", t.eak_code},
          {"eak_name", std::string(core::eak_fraction_name(t.eak_code))},
          {"bim7aa_code", t.bim7aa_code},
          {"unit", t.unit},
          {"treatment", std::string(core::to_string(t.treatment))},
          {"review_status", std::string(core::to_string(t.review_status))},
          {"confidence", opt(t.confidence)},
          {"mass_t", opt(t.mass_t)},
          {"note", t.note},
          {"starred", t.starred},
          {"semantic_class", t.semantic_class},
          {"environment_status", std::string(core::to_string(env))},
          {"sample_ids", sample_ids},
          {"quantity", quantity},
          {"parts", std::move(part_list)},
          {"created_at", t.created_at},
          {"updated_at", t.updated_at}};
}

json sample_json(const reusex::ProjectDB::SampleRecord &s) {
  return {{"id", s.id},
          {"code", s.code},
          {"title", s.title},
          {"what", s.what},
          {"stage", std::string(core::to_string(s.stage))},
          {"result", s.result == core::SampleResult::none ? json(nullptr)
                                                          : json(std::string(core::to_string(s.result)))},
          {"type_ids", s.type_ids},
          {"created_at", s.created_at},
          {"updated_at", s.updated_at}};
}

json survey_json(const reusex::ProjectDB &db) {
  const auto types = db.survey_types();
  const auto env = core::environment_statuses(db);
  std::map<int64_t, std::vector<reusex::ProjectDB::SurveyPartRecord>> parts_of;
  for (auto &p : db.survey_parts())
    parts_of[p.type_id].push_back(std::move(p));
  std::map<int64_t, std::vector<int64_t>> samples_of;
  for (const auto &s : db.samples())
    for (auto t : s.type_ids)
      samples_of[t].push_back(s.id);
  json list = json::array();
  for (const auto &t : types)
    list.push_back(survey_type_json(t, parts_of[t.id], env.at(t.id), samples_of[t.id]));
  return {{"types", std::move(list)}, {"counts", counts_json(types)}};
}

json survey_summary_json(const reusex::ProjectDB &db) {
  const auto types = db.survey_types();
  const auto totals = core::type_totals(db);
  const auto breakdown = core::circularity_breakdown(totals);
  json circ = json::object();
  double total = 0.0;
  for (std::size_t i = 0; i < core::kTreatmentCount; ++i) {
    circ[std::string(core::to_string(static_cast<core::Treatment>(i)))] = breakdown[i];
    total += breakdown[i];
  }
  const double reuse = breakdown[static_cast<std::size_t>(core::Treatment::bevaring)] +
                       breakdown[static_cast<std::size_t>(core::Treatment::genbrug)];
  int pending = 0;
  for (const auto &s : db.samples())
    if (s.stage != core::SampleStage::svar)
      ++pending;

  json unlabeled = nullptr;
  if (db.has_point_cloud("instances"))
    if (const auto cloud = db.point_cloud_label("instances")) {
      std::size_t n = 0;
      for (const auto &p : *cloud)
        if (p.label == 0)
          ++n;
      unlabeled = n;
    }

  json empty_rooms = json::array();
  if (db.has_point_cloud("rooms"))
    if (const auto cloud = db.point_cloud_label("rooms")) {
      std::set<std::uint32_t> present;
      for (const auto &p : *cloud)
        if (p.label != 0)
          present.insert(p.label);
      std::set<std::uint32_t> covered;
      for (const auto &p : db.survey_parts())
        if (p.room_id)
          covered.insert(*p.room_id);
      const auto names = db.label_definitions("rooms");
      for (auto r : present)
        if (!covered.contains(r)) {
          const auto n = names.find(static_cast<int>(r));
          empty_rooms.push_back(n != names.end() ? n->second : "Rum " + std::to_string(r));
        }
    }

  return {{"counts", counts_json(types)},
          {"circularity", std::move(circ)},
          {"total_mass_t", total},
          {"reuse_share", total > 0.0 ? json(reuse / total) : json(nullptr)},
          {"pending_samples", pending},
          {"unlabeled_points", std::move(unlabeled)},
          {"rooms_without_parts", std::move(empty_rooms)}};
}

json survey_fractions_json(const reusex::ProjectDB &db) {
  const auto report = core::fractions_by_eak(core::type_totals(db));
  json list = json::array();
  for (const auto &f : report.fractions)
    list.push_back({{"eak_code", f.eak_code},
                    {"name", f.name},
                    {"treatment", std::string(core::to_string(f.treatment))},
                    {"mass_t", f.mass_t}});
  return {{"fractions", std::move(list)},
          {"total_t", report.total_t},
          {"blocking_types", report.blocking_types},
          {"ready", report.blocking_types == 0}};
}

json samples_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &s : db.samples())
    list.push_back(sample_json(s));
  return {{"samples", std::move(list)}};
}

} // namespace rux::gui
```

- [ ] **Step 4: Routes** — in `endpoint_table()` (api.cpp), after the export-template rows:

```cpp
      {"GET", "/api/v1/survey", "Survey types with their parts, derived miljøstatus and counts"},
      {"GET", "/api/v1/survey/summary", "Survey KPIs: counts, circularity, reuse share, coverage"},
      {"GET", "/api/v1/survey/fractions", "Approved tonnes per EAK code for waste reporting"},
      {"GET", "/api/v1/samples", "Environmental samples with their linked survey types"},
```

In `Server.cpp::register_routes()`, next to the export-template routes:

```cpp
    get("/api/v1/survey")([this](const crow::request &) {
      return with_db([&](const reusex::ProjectDB &db) { return json_response(200, survey_json(db)); });
    });
    get("/api/v1/survey/summary")([this](const crow::request &) {
      return with_db([&](const reusex::ProjectDB &db) { return json_response(200, survey_summary_json(db)); });
    });
    get("/api/v1/survey/fractions")([this](const crow::request &) {
      return with_db([&](const reusex::ProjectDB &db) { return json_response(200, survey_fractions_json(db)); });
    });
```

(`GET /api/v1/samples` is registered in Task 7 together with its POST on one `route_dynamic` rule — one Crow rule per path. Add its `endpoint_table` row now so the table test is updated once per task; if the route-table test also checks registration, register a GET-only rule now and extend it to POST in Task 7.) Add `#include "gui/survey.hpp"` to `Server.cpp`.

In `test_gui_api.cpp`'s expected set add `"GET /api/v1/survey"`, `"GET /api/v1/survey/summary"`, `"GET /api/v1/survey/fractions"`, `"GET /api/v1/samples"`.

- [ ] **Step 5: openapi.yaml** — add paths (tag `survey`, add `- name: survey` to top-level `tags` if tags are declared there) and schemas. Paths:

```yaml
  /survey:
    get:
      tags: [survey]
      operationId: getSurvey
      summary: Survey types with their parts, derived miljøstatus and counts
      description: |
        Every survey type (Kortlægning group row) with its parts (RX-### bygningsdele).
        Rejected types are included with `review_status: rejected`; `counts.all` counts
        queue + approved only. `environment_status` is derived from linked samples, never stored.
      responses:
        "200":
          description: The survey
          content:
            application/json:
              schema:
                type: object
                required: [types, counts]
                properties:
                  types: { type: array, items: { $ref: "#/components/schemas/SurveyType" } }
                  counts: { $ref: "#/components/schemas/SurveyCounts" }
        default: { $ref: "#/components/responses/UnexpectedError" }
  /survey/summary:
    get:
      tags: [survey]
      operationId: getSurveySummary
      summary: "Survey KPIs: counts, circularity, reuse share, coverage"
      responses:
        "200":
          description: KPIs for Overblik and the Kortlægning coverage notice
          content:
            application/json:
              schema: { $ref: "#/components/schemas/SurveySummary" }
        default: { $ref: "#/components/responses/UnexpectedError" }
  /survey/fractions:
    get:
      tags: [survey]
      operationId: getSurveyFractions
      summary: Approved tonnes per EAK code for waste reporting
      description: |
        Only approved types count toward the fractions. `blocking_types` counts types still
        in the queue or awaiting a sample; the report is `ready` when it is 0.
      responses:
        "200":
          description: Waste fractions
          content:
            application/json:
              schema: { $ref: "#/components/schemas/SurveyFractions" }
        default: { $ref: "#/components/responses/UnexpectedError" }
  /samples:
    get:
      tags: [survey]
      operationId: listSamples
      summary: Environmental samples with their linked survey types
      responses:
        "200":
          description: Samples
          content:
            application/json:
              schema:
                type: object
                required: [samples]
                properties:
                  samples: { type: array, items: { $ref: "#/components/schemas/Sample" } }
        default: { $ref: "#/components/responses/UnexpectedError" }
```

Schemas (under `components.schemas`):

```yaml
    Treatment:
      type: string
      enum: [bevaring, genbrug, genanvendelse, nyttiggoerelse, bortskaffelse]
      description: Waste-hierarchy step (affaldshierarki), best first.
    ReviewStatus: { type: string, enum: [queue, approved, rejected] }
    EnvironmentStatus:
      type: string
      enum: [ren_screening, afventer, forurenet, ren_proevesvar]
      description: Derived from linked samples; `afventer` blocks approval.
    SurveyPart:
      type: object
      required: [code, type_id, cloud, instance_id, room_id, room_name, quantity, starred, note, material_guid]
      properties:
        code: { type: string, example: RX-008 }
        type_id: { type: integer }
        cloud: { type: string, nullable: true }
        instance_id: { type: integer, nullable: true }
        room_id: { type: integer, nullable: true }
        room_name: { type: string }
        quantity: { type: number }
        starred: { type: boolean }
        note: { type: string }
        material_guid: { type: string, nullable: true }
    SurveyType:
      type: object
      required: [id, name, eak_code, eak_name, bim7aa_code, unit, treatment, review_status, confidence, mass_t,
                 note, starred, semantic_class, environment_status, sample_ids, quantity, parts, created_at, updated_at]
      properties:
        id: { type: integer }
        name: { type: string }
        eak_code: { type: string, example: "17.04.02" }
        eak_name: { type: string, example: Aluminium }
        bim7aa_code: { type: string, example: 312 Udv. vinduer }
        unit: { type: string, example: stk }
        treatment: { $ref: "#/components/schemas/Treatment" }
        review_status: { $ref: "#/components/schemas/ReviewStatus" }
        confidence: { type: number, nullable: true, minimum: 0, maximum: 1 }
        mass_t: { type: number, nullable: true }
        note: { type: string }
        starred: { type: boolean }
        semantic_class: { type: integer }
        environment_status: { $ref: "#/components/schemas/EnvironmentStatus" }
        sample_ids: { type: array, items: { type: integer } }
        quantity: { type: number, description: Sum of the parts' quantities. }
        parts: { type: array, items: { $ref: "#/components/schemas/SurveyPart" } }
        created_at: { type: string }
        updated_at: { type: string }
    SurveyCounts:
      type: object
      required: [queue, approved, rejected, all]
      properties:
        queue: { type: integer }
        approved: { type: integer }
        rejected: { type: integer }
        all: { type: integer, description: queue + approved }
    SurveySummary:
      type: object
      required: [counts, circularity, total_mass_t, reuse_share, pending_samples, unlabeled_points, rooms_without_parts]
      properties:
        counts: { $ref: "#/components/schemas/SurveyCounts" }
        circularity:
          type: object
          description: Tonnes per treatment over non-rejected types.
          additionalProperties: { type: number }
        total_mass_t: { type: number }
        reuse_share: { type: number, nullable: true, description: (bevaring + genbrug) / total }
        pending_samples: { type: integer }
        unlabeled_points: { type: integer, nullable: true }
        rooms_without_parts: { type: array, items: { type: string } }
    SurveyFractions:
      type: object
      required: [fractions, total_t, blocking_types, ready]
      properties:
        fractions:
          type: array
          items:
            type: object
            required: [eak_code, name, treatment, mass_t]
            properties:
              eak_code: { type: string }
              name: { type: string }
              treatment: { $ref: "#/components/schemas/Treatment" }
              mass_t: { type: number }
        total_t: { type: number }
        blocking_types: { type: integer }
        ready: { type: boolean }
    Sample:
      type: object
      required: [id, code, title, what, stage, result, type_ids, created_at, updated_at]
      properties:
        id: { type: integer }
        code: { type: string, example: P-01 }
        title: { type: string }
        what: { type: string }
        stage: { type: string, enum: [planlagt, udtaget, sendt, svar] }
        result: { type: string, enum: [ren, forurenet], nullable: true }
        type_ids: { type: array, items: { type: integer } }
        created_at: { type: string }
        updated_at: { type: string }
```

- [ ] **Step 6: Build and run**

Run: `nix develop --command cmake --build build --parallel && ctest --test-dir build -R 'SurveyJson|SurveySummaryJson|SurveyFractionsJson|SamplesJson|EndpointTable|gui_api_contract_parses' --output-on-failure --parallel $(nproc)` → PASS.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/include/gui/survey.hpp apps/rux/src/gui/survey.cpp apps/rux/src/gui/api.cpp apps/rux/src/gui/Server.cpp \
  tests/unit/rux_gui/test_gui_survey.cpp tests/unit/rux_gui/test_gui_api.cpp docs/gui/openapi.yaml
git commit -m "feat(gui): survey, summary, fractions and samples read endpoints"
```

---

### Task 7: GUI write endpoints — sync, types, parts, samples

**Files:**
- Modify: `apps/rux/include/gui/survey.hpp`, `apps/rux/src/gui/survey.cpp`, `apps/rux/src/gui/api.cpp`, `apps/rux/src/gui/Server.cpp`
- Modify: `tests/unit/rux_gui/test_gui_api.cpp`, `docs/gui/openapi.yaml`
- Test: append to `tests/unit/rux_gui/test_gui_survey.cpp`

**Interfaces:**
- Consumes: Tasks 1–6, `HttpError` (api.hpp).
- Produces (all `reusex::ProjectDB &db`, body as `const std::string &`; each validates the whole body before writing anything):

```cpp
nlohmann::json sync_survey_json(reusex::ProjectDB &db, const std::string &body);                    // POST /survey/sync  -> 200
nlohmann::json create_survey_type_json(reusex::ProjectDB &db, const std::string &body);             // POST /survey/types -> 201
nlohmann::json patch_survey_type_json(reusex::ProjectDB &db, int64_t id, const std::string &body);  // PATCH /survey/types/<int>
nlohmann::json patch_survey_part_json(reusex::ProjectDB &db, const std::string &code, const std::string &body); // PATCH /survey/parts/<string>
nlohmann::json create_sample_json(reusex::ProjectDB &db, const std::string &body);                  // POST /samples -> 201
nlohmann::json patch_sample_json(reusex::ProjectDB &db, int64_t id, const std::string &body);       // PATCH /samples/<int>
void delete_sample(reusex::ProjectDB &db, int64_t id);                                              // DELETE /samples/<int> -> 204
nlohmann::json set_sample_links_json(reusex::ProjectDB &db, int64_t id, const std::string &body);   // PUT /samples/<int>/links
```

Bodies:
- sync: `{}` or `{ "instances_cloud"?, "semantic_cloud"?, "rooms_cloud"? }` → `{ "types_created", "parts_created", "parts_existing", "rooms_assigned" }`.
- create type: `{ "name" (required, non-empty), "eak_code"?, "bim7aa_code"?, "unit"?, "treatment"? }` → SurveyType.
- patch type: any of `name, eak_code, bim7aa_code, unit, note` (string), `treatment` (Treatment), `review_status` (ReviewStatus), `confidence, mass_t` (number|null), `starred` (bool), `quantity` (number ≥ 0 → `set_type_quantity`). `review_status` goes through `set_review_status` (gate). Order: validate all → apply field patch → quantity → review status → return the refreshed SurveyType (same builder as GET).
- patch part: any of `type_id` (int), `quantity` (number ≥ 0), `starred` (bool), `note`, `room_name` (string) → SurveyPart.
- create sample: `{ "title" (required), "what"?, "type_ids"? }` → Sample.
- patch sample: any of `title, what` (string), `stage`, `result` (string|null) → Sample (via `update_sample_checked`).
- links: `{ "type_ids": int[] }` → Sample.

Error mapping inside these handlers: malformed JSON / wrong types / unknown enum string / negative quantity → `HttpError(400)`; `std::out_of_range` → `HttpError(404)`; `core::SamplePendingError` → `HttpError(422)`; `std::invalid_argument` from the service (no parts, result before answer) → `HttpError(422)`; sync's missing instance cloud (`std::runtime_error` containing `rux create instances`) → `HttpError(422)`.

- [ ] **Step 1: Failing tests** — append to `test_gui_survey.cpp`:

```cpp
namespace {
int status_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const HttpError &e) {
    return e.status();
  }
  return 200;
}
} // namespace

TEST_CASE("PatchSurveyType_SparseFields_AndQuantityRedistribution", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Betonsøjler", core::Treatment::genbrug, 58);
  db.add_survey_part({"RX-001", id, std::nullopt, std::nullopt, std::nullopt, "", 18, false, "", {}});
  db.add_survey_part({"RX-002", id, std::nullopt, std::nullopt, std::nullopt, "", 6, false, "", {}});
  const auto j = patch_survey_type_json(db, id,
      R"({"treatment":"genanvendelse","mass_t":null,"starred":true,"quantity":30})");
  CHECK(j.at("treatment") == "genanvendelse");
  CHECK(j.at("mass_t").is_null());
  CHECK(j.at("starred") == true);
  CHECK(j.at("quantity").get<double>() == Approx(30));
  CHECK(j.at("parts").at(0).at("quantity").get<double>() == Approx(22.5));
}

TEST_CASE("PatchSurveyType_Errors_MapToStatuses", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto id = add_type(db, "Vinduer", core::Treatment::genbrug, 3.1);
  const auto s = db.add_sample("PCB", "");
  db.set_sample_links(s.id, {id});
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"review_status":"approved"})"); }) == 422);
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"treatment":"Genbrug"})"); }) == 400);
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"quantity":-1})"); }) == 400);
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"quantity":5})"); }) == 422); // no parts
  CHECK(status_of([&] { patch_survey_type_json(db, id + 99, R"({"starred":true})"); }) == 404);
  CHECK(status_of([&] { patch_survey_type_json(db, id, "not json"); }) == 400);
  // A rejected combination must not half-apply: nothing changed.
  CHECK(status_of([&] { patch_survey_type_json(db, id, R"({"starred":true,"review_status":"approved"})"); }) == 422);
  CHECK_FALSE(db.survey_type(id)->starred);
}

TEST_CASE("CreateSurveyType_RequiresName", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto j = create_survey_type_json(db, R"({"name":"Trapezplader, tag","eak_code":"17.04.05"})");
  CHECK(j.at("id").get<int64_t>() > 0);
  CHECK(j.at("review_status") == "queue");
  CHECK(status_of([&] { create_survey_type_json(db, R"({"name":""})"); }) == 400);
}

TEST_CASE("PatchSurveyPart_MoveAndErrors", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto a = add_type(db, "a", core::Treatment::genbrug, 1);
  const auto b = add_type(db, "b", core::Treatment::genbrug, 1);
  db.add_survey_part({"RX-001", a, std::nullopt, std::nullopt, std::nullopt, "", 1, false, "", {}});
  const auto j = patch_survey_part_json(db, "RX-001", R"({"type_id":)" + std::to_string(b) + R"(,"note":"flyttet"})");
  CHECK(j.at("type_id") == b);
  CHECK(j.at("note") == "flyttet");
  CHECK(status_of([&] { patch_survey_part_json(db, "RX-404", R"({"starred":true})"); }) == 404);
  CHECK(status_of([&] { patch_survey_part_json(db, "RX-001", R"({"type_id":9999})"); }) == 404);
}

TEST_CASE("SampleEndpoints_CreateLinkAnswerDelete", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  const auto t = add_type(db, "Vinduer", core::Treatment::genbrug, 3.1);
  auto s = create_sample_json(db, R"({"title":"PCB i fugemasse","type_ids":[)" + std::to_string(t) + "]}");
  const auto id = s.at("id").get<int64_t>();
  CHECK(s.at("code") == "P-01");
  CHECK(s.at("type_ids") == nlohmann::json::array({t}));
  CHECK(status_of([&] { patch_sample_json(db, id, R"({"result":"ren"})"); }) == 422);
  s = patch_sample_json(db, id, R"({"stage":"svar","result":"forurenet"})");
  CHECK(s.at("result") == "forurenet");
  CHECK(status_of([&] { patch_sample_json(db, id, R"({"stage":"lab"})"); }) == 400);
  s = set_sample_links_json(db, id, R"({"type_ids":[]})");
  CHECK(s.at("type_ids").empty());
  CHECK(status_of([&] { set_sample_links_json(db, id, R"({"type_ids":[9999]})"); }) == 404);
  delete_sample(db, id);
  CHECK(status_of([&] { delete_sample(db, id); }) == 404);
}

TEST_CASE("SyncSurveyJson_NoInstances_Is422", "[gui][survey][edits]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  CHECK(status_of([&] { sync_survey_json(db, "{}"); }) == 422);
}
```

(`#include <functional>` for `std::function`.)

- [ ] **Step 2: Build → FAIL** (handlers undeclared).

- [ ] **Step 3: Implement** — declarations in `gui/survey.hpp`; in `survey.cpp` add body parsing helpers and the handlers:

```cpp
namespace {
json parse_object(const std::string &body) {
  auto j = json::parse(body.empty() ? "{}" : body, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object())
    throw HttpError(400, "request body must be a JSON object");
  return j;
}
std::optional<std::string> opt_string(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end()) return std::nullopt;
  if (!it->is_string()) throw HttpError(400, std::string("'") + key + "' must be a string");
  return it->get<std::string>();
}
std::optional<bool> opt_bool(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end()) return std::nullopt;
  if (!it->is_boolean()) throw HttpError(400, std::string("'") + key + "' must be a boolean");
  return it->get<bool>();
}
/// Present-and-null clears; present-and-number sets; absent leaves alone.
std::optional<std::optional<double>> opt_nullable_number(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end()) return std::nullopt;
  if (it->is_null()) return std::optional<double>{};
  if (!it->is_number()) throw HttpError(400, std::string("'") + key + "' must be a number or null");
  return std::optional<double>{it->get<double>()};
}
std::optional<double> opt_quantity(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end()) return std::nullopt;
  if (!it->is_number() || it->get<double>() < 0.0)
    throw HttpError(400, std::string("'") + key + "' must be a number >= 0");
  return it->get<double>();
}
template <typename E>
std::optional<E> opt_enum(const json &j, const char *key, std::optional<E> (*parse)(std::string_view)) {
  const auto s = opt_string(j, key);
  if (!s) return std::nullopt;
  const auto v = parse(*s);
  if (!v) throw HttpError(400, std::string("'") + key + "' has no value '" + *s + "'");
  return v;
}
std::vector<int64_t> id_list(const json &j, const char *key) {
  auto it = j.find(key);
  if (it == j.end()) return {};
  if (!it->is_array()) throw HttpError(400, std::string("'") + key + "' must be an array of integers");
  std::vector<int64_t> out;
  for (const auto &v : *it) {
    if (!v.is_number_integer()) throw HttpError(400, std::string("'") + key + "' must be an array of integers");
    out.push_back(v.get<int64_t>());
  }
  return out;
}
/// Runs f, translating library exceptions into the documented statuses.
template <typename F> auto mapped(F &&f) -> decltype(f()) {
  try {
    return f();
  } catch (const HttpError &) {
    throw;
  } catch (const core::SamplePendingError &e) {
    throw HttpError(422, e.what());
  } catch (const std::out_of_range &e) {
    throw HttpError(404, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(422, e.what());
  }
}
json type_by_id(const reusex::ProjectDB &db, int64_t id) {
  for (auto &t : survey_json(db).at("types"))
    if (t.at("id") == id) return t;
  throw HttpError(404, "no survey type " + std::to_string(id));
}
} // namespace

json sync_survey_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  core::SurveySyncOptions opts;
  if (auto v = opt_string(j, "instances_cloud")) opts.instances_cloud = *v;
  if (auto v = opt_string(j, "semantic_cloud")) opts.semantic_cloud = *v;
  if (auto v = opt_string(j, "rooms_cloud")) opts.rooms_cloud = *v;
  try {
    const auto r = core::sync_survey(db, opts);
    return {{"types_created", r.types_created}, {"parts_created", r.parts_created},
            {"parts_existing", r.parts_existing}, {"rooms_assigned", r.rooms_assigned}};
  } catch (const std::runtime_error &e) {
    if (std::string(e.what()).find("rux create instances") != std::string::npos)
      throw HttpError(422, e.what());
    throw;
  }
}

json create_survey_type_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyTypeRecord t;
  const auto name = opt_string(j, "name");
  if (!name || name->empty()) throw HttpError(400, "'name' is required and must be non-empty");
  t.name = *name;
  if (auto v = opt_string(j, "eak_code")) t.eak_code = *v;
  if (auto v = opt_string(j, "bim7aa_code")) t.bim7aa_code = *v;
  if (auto v = opt_string(j, "unit")) t.unit = *v;
  if (auto v = opt_enum<core::Treatment>(j, "treatment", core::treatment_from_string)) t.treatment = *v;
  return type_by_id(db, db.add_survey_type(t).id);
}

json patch_survey_type_json(reusex::ProjectDB &db, int64_t id, const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyTypePatch p;
  p.name = opt_string(j, "name");
  p.eak_code = opt_string(j, "eak_code");
  p.bim7aa_code = opt_string(j, "bim7aa_code");
  p.unit = opt_string(j, "unit");
  p.note = opt_string(j, "note");
  p.treatment = opt_enum<core::Treatment>(j, "treatment", core::treatment_from_string);
  p.confidence = opt_nullable_number(j, "confidence");
  p.mass_t = opt_nullable_number(j, "mass_t");
  p.starred = opt_bool(j, "starred");
  const auto quantity = opt_quantity(j, "quantity");
  const auto status = opt_enum<core::ReviewStatus>(j, "review_status", core::review_status_from_string);
  return mapped([&] {
    if (!db.survey_type(id)) throw std::out_of_range("no survey type " + std::to_string(id));
    // Check every refusal before the first write, so a refused request changes nothing.
    if (status == core::ReviewStatus::approved &&
        core::environment_status_of(db, id) == core::EnvironmentStatus::afventer)
      core::set_review_status(db, id, *status); // throws SamplePendingError
    if (quantity) {
      bool has_parts = false;
      for (const auto &part : db.survey_parts()) has_parts = has_parts || part.type_id == id;
      if (!has_parts) throw std::invalid_argument("survey type has no parts to distribute a quantity over");
    }
    db.update_survey_type(id, p);
    if (quantity) core::set_type_quantity(db, id, *quantity);
    if (status) core::set_review_status(db, id, *status);
    return type_by_id(db, id);
  });
}

json patch_survey_part_json(reusex::ProjectDB &db, const std::string &code, const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SurveyPartPatch p;
  if (auto it = j.find("type_id"); it != j.end()) {
    if (!it->is_number_integer()) throw HttpError(400, "'type_id' must be an integer");
    p.type_id = it->get<int64_t>();
  }
  p.quantity = opt_quantity(j, "quantity");
  p.starred = opt_bool(j, "starred");
  p.note = opt_string(j, "note");
  p.room_name = opt_string(j, "room_name");
  return mapped([&] { return survey_part_json(db.update_survey_part(code, p)); });
}

json create_sample_json(reusex::ProjectDB &db, const std::string &body) {
  const auto j = parse_object(body);
  const auto title = opt_string(j, "title");
  if (!title || title->empty()) throw HttpError(400, "'title' is required and must be non-empty");
  const auto what = opt_string(j, "what").value_or("");
  const auto types = id_list(j, "type_ids");
  return mapped([&] {
    for (auto t : types)
      if (!db.survey_type(t)) throw std::out_of_range("no survey type " + std::to_string(t));
    const auto s = db.add_sample(*title, what);
    if (!types.empty()) db.set_sample_links(s.id, types);
    return sample_json(*db.sample(s.id));
  });
}

json patch_sample_json(reusex::ProjectDB &db, int64_t id, const std::string &body) {
  const auto j = parse_object(body);
  reusex::ProjectDB::SamplePatch p;
  p.title = opt_string(j, "title");
  p.what = opt_string(j, "what");
  p.stage = opt_enum<core::SampleStage>(j, "stage", core::sample_stage_from_string);
  if (auto it = j.find("result"); it != j.end()) {
    if (it->is_null()) p.result = core::SampleResult::none;
    else p.result = opt_enum<core::SampleResult>(j, "result", core::sample_result_from_string);
    if (p.result == core::SampleResult::none && !it->is_null())
      throw HttpError(400, "'result' must be 'ren', 'forurenet' or null");
  }
  return mapped([&] { return sample_json(core::update_sample_checked(db, id, p)); });
}

void delete_sample(reusex::ProjectDB &db, int64_t id) {
  if (!db.delete_sample(id)) throw HttpError(404, "no sample " + std::to_string(id));
}

json set_sample_links_json(reusex::ProjectDB &db, int64_t id, const std::string &body) {
  const auto j = parse_object(body);
  if (!j.contains("type_ids")) throw HttpError(400, "'type_ids' is required");
  const auto types = id_list(j, "type_ids");
  return mapped([&] {
    db.set_sample_links(id, types);
    return sample_json(*db.sample(id));
  });
}
```

(`gui/survey.hpp` must include `"gui/api.hpp"` for `HttpError`, and `<reusex/core/ProjectDB.hpp>`, `<nlohmann/json.hpp>`.)

- [ ] **Step 4: Routes** — endpoint table rows:

```cpp
      {"POST", "/api/v1/survey/sync", "Create survey parts for instances that have none"},
      {"POST", "/api/v1/survey/types", "Create a survey type"},
      {"PATCH", "/api/v1/survey/types/<int>", "Edit a survey type; approval is gated on samples"},
      {"PATCH", "/api/v1/survey/parts/<string>", "Edit or re-file a survey part"},
      {"POST", "/api/v1/samples", "Register an environmental sample"},
      {"PATCH", "/api/v1/samples/<int>", "Advance a sample's stage or record its result"},
      {"DELETE", "/api/v1/samples/<int>", "Delete a sample"},
      {"PUT", "/api/v1/samples/<int>/links", "Replace the survey types a sample covers"},
```

Registration in `Server.cpp` (one rule per path; a GET-only `/samples` from Task 6 becomes this combined rule):

```cpp
    app_.route_dynamic("/api/v1/survey/sync").methods(crow::HTTPMethod::POST)(
        [this](const crow::request &req) {
          return with_write([&](reusex::ProjectDB &db) { return json_response(200, sync_survey_json(db, req.body)); });
        });
    app_.route_dynamic("/api/v1/survey/types").methods(crow::HTTPMethod::POST)(
        [this](const crow::request &req) {
          return with_write([&](reusex::ProjectDB &db) { return json_response(201, create_survey_type_json(db, req.body)); });
        });
    app_.route_dynamic("/api/v1/survey/types/<int>").methods(crow::HTTPMethod::PATCH)(
        [this](const crow::request &req, int id) {
          return with_write([&](reusex::ProjectDB &db) { return json_response(200, patch_survey_type_json(db, id, req.body)); });
        });
    app_.route_dynamic("/api/v1/survey/parts/<string>").methods(crow::HTTPMethod::PATCH)(
        [this](const crow::request &req, std::string code) {
          return with_write([&](reusex::ProjectDB &db) { return json_response(200, patch_survey_part_json(db, code, req.body)); });
        });
    // One rule for both methods: registering the same path twice would create two competing Crow rules.
    app_.route_dynamic("/api/v1/samples").methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
        [this](const crow::request &req) {
          if (req.method == crow::HTTPMethod::GET)
            return with_db([&](const reusex::ProjectDB &db) { return json_response(200, samples_json(db)); });
          return with_write([&](reusex::ProjectDB &db) { return json_response(201, create_sample_json(db, req.body)); });
        });
    app_.route_dynamic("/api/v1/samples/<int>").methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
        [this](const crow::request &req, int id) {
          if (req.method == crow::HTTPMethod::PATCH)
            return with_write([&](reusex::ProjectDB &db) { return json_response(200, patch_sample_json(db, id, req.body)); });
          return with_write([&](reusex::ProjectDB &db) {
            delete_sample(db, id);
            return crow::response(204);
          });
        });
    app_.route_dynamic("/api/v1/samples/<int>/links").methods(crow::HTTPMethod::PUT)(
        [this](const crow::request &req, int id) {
          return with_write([&](reusex::ProjectDB &db) { return json_response(200, set_sample_links_json(db, id, req.body)); });
        });
```

If the server wraps mutating routes in a CSRF/content-type gate (it answers 415 without `Content-Type: application/json`), confirm these routes pass through the same gate as `/materials/<string>` PATCH — follow whatever the materials PATCH registration does beyond `with_write`.

Add the eight `"METHOD path"` strings to `test_gui_api.cpp`'s set.

- [ ] **Step 5: openapi.yaml** — add each path with `tags: [survey]`, an `operationId`, the `summary` string identical to the endpoint table, a `requestBody` schema matching **Bodies** above, responses `200`/`201`/`204` with the SurveyType / SurveyPart / Sample schema, and `"400"`, `"404"`, `"409": { $ref: "#/components/responses/WriteConflict" }`, `"503": { $ref: "#/components/responses/WriterBusy" }`, plus `"422"` described as "Approval blocked by a pending sample, a quantity with no parts, a sample result before the answer stage, or (sync) no instance cloud" on the routes that can return it (PATCH types, POST sync, PATCH samples). Add a `SurveyTypePatch`, `SurveyPartPatch`, `SamplePatch` schema mirroring **Bodies**.

- [ ] **Step 6: Build and run**

Run: `nix develop --command cmake --build build --parallel && ctest --test-dir build -R 'PatchSurvey|CreateSurveyType|SampleEndpoints|SyncSurveyJson|EndpointTable|gui_api_contract_parses' --output-on-failure --parallel $(nproc)` → PASS.
Smoke over HTTP (server from this build): `./build/apps/rux/rux -p /tmp/claude-survey.rux gui --port 8431 --no-browser &` then
`curl -s -X POST -H 'Content-Type: application/json' -d '{"name":"Test"}' localhost:8431/api/v1/survey/types` → 201 JSON with `"review_status":"queue"`; `curl -s localhost:8431/api/v1/survey | head -c 300`; kill the server, delete the temp project.

- [ ] **Step 7: Commit**

```bash
git add apps/rux/include/gui/survey.hpp apps/rux/src/gui/survey.cpp apps/rux/src/gui/api.cpp apps/rux/src/gui/Server.cpp \
  tests/unit/rux_gui/test_gui_survey.cpp tests/unit/rux_gui/test_gui_api.cpp docs/gui/openapi.yaml
git commit -m "feat(gui): survey and sample write endpoints with the 422 approval gate"
```

---

### Task 8: Evidence renders — instance highlight + `/renders`

**Files:**
- Create: `libs/reusex/include/visualize/highlight.hpp`, `libs/reusex/src/visualize/highlight.cpp`
- Modify: `libs/reusex/include/visualize/render_view.hpp`, `libs/reusex/src/visualize/render_view.cpp`
- Create: `apps/rux/include/gui/ViewRenderer.hpp`, `apps/rux/src/gui/render.cpp`
- Modify: `apps/rux/include/gui/Server.hpp`, `apps/rux/src/gui/Server.cpp`, `apps/rux/src/gui/api.cpp`, `apps/rux/src/gui.cpp`
- Modify: `tests/unit/rux_gui/test_gui_api.cpp`, `docs/gui/openapi.yaml`
- Test: `tests/unit/visualize/test_highlight.cpp`, append to `tests/unit/rux_gui/test_gui_survey.cpp`

**Interfaces:**
- Produces:

```cpp
// visualize/highlight.hpp — namespace reusex::visualize
inline constexpr std::array<unsigned char, 3> kHighlightRgb{255, 176, 64};
inline constexpr double kHighlightDimFactor = 0.3;
/// Paint points whose label == instance_id in kHighlightRgb and dim every other
/// point to kHighlightDimFactor of its colour. rgb is 3 bytes per point.
/// @return number of highlighted points. @throws std::invalid_argument on size mismatch.
std::size_t apply_instance_highlight(std::vector<unsigned char> &rgb,
                                     const std::vector<std::uint32_t> &labels,
                                     std::uint32_t instance_id);

// render_view.hpp — added to namespace and RenderOptions
struct InstanceHighlight {
  std::string cloud_name = "instances";
  std::uint32_t instance_id = 0;
};
// in RenderOptions:
  std::optional<InstanceHighlight> highlight;

// gui/ViewRenderer.hpp — namespace rux::gui
struct RenderRequest {
  std::string view = "plan";               // plan | top | front | orbit
  int orbit_index = 0;                     // with view == orbit, of 8
  std::vector<std::string> layers{"cloud"};
  std::optional<std::string> highlight_cloud;
  std::optional<std::uint32_t> highlight_instance;
  int width = 960;
  int height = 720;
};
class RenderUnavailable : public std::runtime_error { public: using std::runtime_error::runtime_error; };
class IViewRenderer {
public:
  virtual ~IViewRenderer() = default;
  /// PNG bytes. Throws RenderUnavailable (no GL), std::invalid_argument (bad
  /// request), std::runtime_error (the project lacks the data; message names
  /// the stage). Implementations must serialise internally.
  virtual std::vector<std::uint8_t> render_png(const reusex::ProjectDB &db, const RenderRequest &req) = 0;
};
RenderRequest render_request_from(const Params &params);       // 400 on bad values
Blob render_blob(const reusex::ProjectDB &db, IViewRenderer *renderer, const Params &params);
// Server: void set_view_renderer(IViewRenderer *renderer);
```

Query: `view`, `orbit_index` (0–7), `layers` (comma list), `highlight_cloud` (default `instances` when `highlight_instance` is given), `highlight_instance`, `width`/`height` (64–2048). Status: renderer null or `RenderUnavailable` → 503; `std::invalid_argument` or bad params → 400; other `std::runtime_error` → 422.

- [ ] **Step 1: Failing tests**

`tests/unit/visualize/test_highlight.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
// Instance highlight colouring for evidence renders (Kortlægning).

#include <catch2/catch_test_macros.hpp>
#include <visualize/highlight.hpp>

#include <stdexcept>

using namespace reusex::visualize;

TEST_CASE("ApplyInstanceHighlight_PaintsInstance_DimsRest", "[visualize][highlight]") {
  std::vector<unsigned char> rgb{100, 100, 100, 200, 200, 200, 50, 60, 70};
  const std::vector<std::uint32_t> labels{2, 7, 2};
  CHECK(apply_instance_highlight(rgb, labels, 2) == 2);
  CHECK(rgb[0] == kHighlightRgb[0]);
  CHECK(rgb[1] == kHighlightRgb[1]);
  CHECK(rgb[8] == kHighlightRgb[2]);
  CHECK(rgb[3] == 60); // 200 * 0.3
}

TEST_CASE("ApplyInstanceHighlight_AbsentInstance_ReturnsZero_AndDimsAll", "[visualize][highlight]") {
  std::vector<unsigned char> rgb{100, 100, 100};
  CHECK(apply_instance_highlight(rgb, {5}, 9) == 0);
  CHECK(rgb[0] == 30);
}

TEST_CASE("ApplyInstanceHighlight_SizeMismatch_Throws", "[visualize][highlight]") {
  std::vector<unsigned char> rgb{1, 2, 3};
  CHECK_THROWS_AS(apply_instance_highlight(rgb, {1, 2}, 1), std::invalid_argument);
}
```

Append to `test_gui_survey.cpp` (add `#include "gui/ViewRenderer.hpp"`):

```cpp
namespace {
struct FakeRenderer : IViewRenderer {
  RenderRequest last;
  enum class Mode { ok, no_gl, missing_data } mode = Mode::ok;
  std::vector<std::uint8_t> render_png(const reusex::ProjectDB &, const RenderRequest &req) override {
    last = req;
    if (mode == Mode::no_gl) throw RenderUnavailable("no EGL");
    if (mode == Mode::missing_data) throw std::runtime_error("needs the label cloud 'rooms' — run `rux create rooms` first");
    return {0x89, 'P', 'N', 'G'};
  }
};
Params params_of(std::initializer_list<std::pair<const char *, const char *>> kv) {
  Params p;
  for (auto [k, v] : kv) p.set(k, v);
  return p;
}
} // namespace

TEST_CASE("RenderBlob_ParsesQuery_DefaultsHighlightCloud", "[gui][render]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  FakeRenderer r;
  const auto blob = render_blob(db, &r, params_of({{"view", "orbit"}, {"orbit_index", "3"},
                                                   {"layers", "cloud,rooms"}, {"highlight_instance", "12"}}));
  CHECK(blob.content_type == "image/png");
  CHECK(blob.data.size() == 4);
  CHECK(r.last.view == "orbit");
  CHECK(r.last.orbit_index == 3);
  CHECK(r.last.layers == std::vector<std::string>{"cloud", "rooms"});
  CHECK(r.last.highlight_cloud == "instances");
  CHECK(r.last.highlight_instance == 12u);
}

TEST_CASE("RenderBlob_StatusMapping", "[gui][render]") {
  TempDB tmp;
  ProjectDB db(tmp.path);
  FakeRenderer r;
  CHECK(status_of([&] { render_blob(db, nullptr, {}); }) == 503);
  r.mode = FakeRenderer::Mode::no_gl;
  CHECK(status_of([&] { render_blob(db, &r, {}); }) == 503);
  r.mode = FakeRenderer::Mode::missing_data;
  CHECK(status_of([&] { render_blob(db, &r, {}); }) == 422);
  r.mode = FakeRenderer::Mode::ok;
  CHECK(status_of([&] { render_blob(db, &r, params_of({{"view", "sideways"}})); }) == 400);
  CHECK(status_of([&] { render_blob(db, &r, params_of({{"width", "5"}})); }) == 400);
  CHECK(status_of([&] { render_blob(db, &r, params_of({{"orbit_index", "8"}})); }) == 400);
}
```

- [ ] **Step 2: Build → FAIL** (headers missing).

- [ ] **Step 3: Highlight in the library**

`highlight.hpp`: declarations above, `#pragma once`, includes `<array> <cstddef> <cstdint> <vector>`. `highlight.cpp`:

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "reusex/visualize/highlight.hpp"

#include <stdexcept>
#include <string>

namespace reusex::visualize {

std::size_t apply_instance_highlight(std::vector<unsigned char> &rgb, const std::vector<std::uint32_t> &labels,
                                     std::uint32_t instance_id) {
  if (rgb.size() != labels.size() * 3)
    throw std::invalid_argument("apply_instance_highlight: " + std::to_string(rgb.size() / 3) +
                                " coloured points but " + std::to_string(labels.size()) + " labels");
  std::size_t hit = 0;
  for (std::size_t i = 0; i < labels.size(); ++i) {
    unsigned char *c = &rgb[3 * i];
    if (labels[i] == instance_id) {
      c[0] = kHighlightRgb[0];
      c[1] = kHighlightRgb[1];
      c[2] = kHighlightRgb[2];
      ++hit;
    } else {
      for (int k = 0; k < 3; ++k)
        c[k] = static_cast<unsigned char>(c[k] * kHighlightDimFactor);
    }
  }
  return hit;
}

} // namespace reusex::visualize
```

In `render_view.hpp` add `InstanceHighlight` before `RenderOptions` and the `highlight` member at the end of `RenderOptions` with doc comment "Paint one instance in kHighlightRgb and dim the rest (Kortlægning evidence). Applies to every point layer; needs the named Label cloud, index-aligned with cloud_name." In `render_view.cpp`, inside the render function, add a lazily loaded label vector and apply it in both point-layer branches right before `add_points_actor`:

```cpp
  std::optional<std::vector<std::uint32_t>> highlight_labels;
  const auto highlight = [&](std::vector<unsigned char> &colors, std::size_t points) {
    if (!opts.highlight)
      return;
    if (!highlight_labels) {
      const auto &h = *opts.highlight;
      if (!db.has_point_cloud(h.cloud_name))
        throw std::runtime_error("render: highlight needs the label cloud '" + h.cloud_name +
                                 "' — run `rux create instances` first");
      const CloudLPtr labels = db.point_cloud_label(h.cloud_name);
      highlight_labels.emplace();
      for (const auto &p : *labels)
        highlight_labels->push_back(p.label);
    }
    if (highlight_labels->size() != points)
      throw std::runtime_error("render: highlight cloud '" + opts.highlight->cloud_name + "' has " +
                               std::to_string(highlight_labels->size()) + " labels but '" + opts.cloud_name +
                               "' has " + std::to_string(points) + " points — the clouds are out of sync");
    if (apply_instance_highlight(colors, *highlight_labels, opts.highlight->instance_id) == 0)
      core::warn("render: instance {} has no points in '{}'; nothing is highlighted",
                 opts.highlight->instance_id, opts.highlight->cloud_name);
  };
```

and call `highlight(colors, cloud.size());` before each `add_points_actor(renderer, make_point_polydata(cloud, colors), …)` in the `Layer::cloud` and label-layer cases. Add `#include "reusex/visualize/highlight.hpp"`.

- [ ] **Step 4: Server side**

`apps/rux/include/gui/ViewRenderer.hpp`: the `RenderRequest`, `RenderUnavailable`, `IViewRenderer` declarations above with a LAYERING comment mirroring `FrameSegmenter.hpp` ("rux_gui_lib must not link reusex_visualize (VTK); the rux app layer registers a concrete renderer; without one the endpoint answers 503"). Includes `<reusex/core/ProjectDB.hpp>`, `<cstdint> <optional> <stdexcept> <string> <vector>`.

`apps/rux/src/gui/render.cpp` (in `rux_gui_lib`, VTK-free):

```cpp
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/ViewRenderer.hpp"
#include "gui/api.hpp"

#include <sstream>

namespace rux::gui {

RenderRequest render_request_from(const Params &params) {
  RenderRequest r;
  r.view = params.str("view", r.view);
  if (r.view != "plan" && r.view != "top" && r.view != "front" && r.view != "orbit")
    throw HttpError(400, "'view' must be plan, top, front or orbit, not '" + r.view + "'");
  r.orbit_index = static_cast<int>(params.integer("orbit_index", 0));
  if (r.orbit_index < 0 || r.orbit_index > 7)
    throw HttpError(400, "'orbit_index' must be 0..7");
  if (const auto layers = params.find("layers"); layers && !layers->empty()) {
    r.layers.clear();
    std::stringstream ss(*layers);
    for (std::string item; std::getline(ss, item, ',');)
      if (!item.empty())
        r.layers.push_back(item);
  }
  if (params.find("highlight_instance")) {
    const auto id = params.integer("highlight_instance", 0);
    if (id <= 0)
      throw HttpError(400, "'highlight_instance' must be a positive instance id");
    r.highlight_instance = static_cast<std::uint32_t>(id);
    r.highlight_cloud = params.str("highlight_cloud", "instances");
  }
  r.width = static_cast<int>(params.integer("width", r.width));
  r.height = static_cast<int>(params.integer("height", r.height));
  if (r.width < 64 || r.width > 2048 || r.height < 64 || r.height > 2048)
    throw HttpError(400, "'width' and 'height' must be 64..2048");
  return r;
}

Blob render_blob(const reusex::ProjectDB &db, IViewRenderer *renderer, const Params &params) {
  const auto req = render_request_from(params);
  if (!renderer)
    throw HttpError(503, "this server has no view renderer (built without the visualize module)");
  try {
    return Blob{"image/png", renderer->render_png(db, req)};
  } catch (const RenderUnavailable &e) {
    throw HttpError(503, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(400, e.what());
  } catch (const HttpError &) {
    throw;
  } catch (const std::runtime_error &e) {
    throw HttpError(422, e.what());
  }
}

} // namespace rux::gui
```

(Order matters: `RenderUnavailable` and `HttpError` derive from `std::runtime_error`, so they are caught first. If `HttpError` is not a `runtime_error` subclass in some build, the order is still correct.)

`Server.hpp`: add `void set_view_renderer(IViewRenderer *renderer);` with the same ownership comment as `set_segmenter`; forward-declare or include `gui/ViewRenderer.hpp`. `Server.cpp`: store `IViewRenderer *view_renderer_ = nullptr;` in Impl with a setter like `set_segmenter`, and register:

```cpp
    get("/api/v1/renders")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) { return blob_response(render_blob(db, view_renderer_, params)); });
    });
```

Endpoint table row: `{"GET", "/api/v1/renders", "Server-rendered view of the project, optionally highlighting one instance", true},` and `"GET /api/v1/renders"` in the test set.

- [ ] **Step 5: Concrete renderer in the rux app** — in `apps/rux/src/gui.cpp` (rux_lib, which links `reusex` incl. visualize), next to `DefaultFrameSegmenter`:

```cpp
/// Concrete evidence renderer (Kortlægning). render_view() drives VTK, which
/// is not safe to run concurrently in one process — hence the mutex.
class DefaultViewRenderer final : public rux::gui::IViewRenderer {
public:
  std::vector<std::uint8_t> render_png(const reusex::ProjectDB &db, const rux::gui::RenderRequest &req) override {
    namespace viz = reusex::visualize;
    viz::RenderOptions o;
    o.layers.clear();
    for (const auto &name : req.layers) {
      const auto layer = viz::layer_from_string(name);
      if (!layer)
        throw std::invalid_argument("unknown layer '" + name + "'");
      o.layers.push_back(*layer);
    }
    const auto view = viz::view_preset_from_string(req.view);
    if (!view)
      throw std::invalid_argument("unknown view '" + req.view + "'");
    o.view = *view;
    o.orbit_index = req.orbit_index;
    o.width = req.width;
    o.height = req.height;
    if (req.highlight_instance)
      o.highlight = viz::InstanceHighlight{req.highlight_cloud.value_or("instances"), *req.highlight_instance};
    std::lock_guard lock(mutex_);
    cv::Mat image;
    try {
      image = viz::render_view(db, o);
    } catch (const viz::OffscreenGlUnavailable &e) {
      throw rux::gui::RenderUnavailable(e.what());
    }
    std::vector<std::uint8_t> png;
    cv::imencode(".png", image, png);
    return png;
  }

private:
  std::mutex mutex_;
};
```

and register it where the segmenters are: `DefaultViewRenderer view_renderer; server.set_view_renderer(&view_renderer);`. Add includes `<reusex/visualize/render_view.hpp>`, `<opencv2/imgcodecs.hpp>`, `<mutex>`, `"gui/ViewRenderer.hpp"`. If `reusex_visualize` is conditional in the build, guard with the same `#if` / `if(TARGET reusex_visualize)` mechanism `apps/rux/src/render.cpp` relies on — check how `render.cpp` is compiled when visualize is absent, and follow it.

- [ ] **Step 6: openapi.yaml** — path `/renders` (GET, tag `survey`, the query parameters above, `200` `image/png` binary, `400`, `422`, `503`), description noting it backs the Plan / Punktsky / Rum-model evidence tabs and that the server may take a second or two.

- [ ] **Step 7: Build and run**

Run: `nix develop --command cmake --build build --parallel && ctest --test-dir build -R 'ApplyInstanceHighlight|RenderBlob|EndpointTable|gui_api_contract_parses|Render' --output-on-failure --parallel $(nproc)` → PASS (existing render tests too).
Real render smoke (needs GL; skip with a note if `OffscreenGlUnavailable`): on a project with `instances`, `./build/apps/rux/rux -p <proj> gui --port 8432 --no-browser &`, `curl -s -o /tmp/claude-render.png 'localhost:8432/api/v1/renders?view=plan&highlight_instance=1'`, open the PNG with Read — expect the plan with one orange instance on dimmed points. Kill the server.

- [ ] **Step 8: Commit**

```bash
git add libs/reusex/include/visualize/highlight.hpp libs/reusex/src/visualize/highlight.cpp \
  libs/reusex/include/visualize/render_view.hpp libs/reusex/src/visualize/render_view.cpp \
  apps/rux/include/gui/ViewRenderer.hpp apps/rux/src/gui/render.cpp apps/rux/include/gui/Server.hpp \
  apps/rux/src/gui/Server.cpp apps/rux/src/gui/api.cpp apps/rux/src/gui.cpp \
  tests/unit/visualize/test_highlight.cpp tests/unit/rux_gui/test_gui_survey.cpp tests/unit/rux_gui/test_gui_api.cpp \
  docs/gui/openapi.yaml
git commit -m "feat(gui): /renders evidence images with instance highlight via an injected renderer"
```

---

### Task 9: Frontend contract layer

**Files:**
- Modify: `apps/rux/frontend/src/api/types.ts`, `apps/rux/frontend/src/api/client.ts`
- Test: `apps/rux/frontend/src/test/survey.client.test.ts`

**Interfaces:**
- Produces (TypeScript, exact names Phase 3 imports):

```ts
export type Treatment = 'bevaring' | 'genbrug' | 'genanvendelse' | 'nyttiggoerelse' | 'bortskaffelse';
export const TREATMENTS: readonly Treatment[]; // hierarchy order
export type ReviewStatus = 'queue' | 'approved' | 'rejected';
export type EnvironmentStatus = 'ren_screening' | 'afventer' | 'forurenet' | 'ren_proevesvar';
export type SampleStage = 'planlagt' | 'udtaget' | 'sendt' | 'svar';
export type SampleResult = 'ren' | 'forurenet';
export interface SurveyPart { code: string; type_id: number; cloud: string | null; instance_id: number | null;
  room_id: number | null; room_name: string; quantity: number; starred: boolean; note: string; material_guid: string | null; }
export interface SurveyType { id: number; name: string; eak_code: string; eak_name: string; bim7aa_code: string;
  unit: string; treatment: Treatment; review_status: ReviewStatus; confidence: number | null; mass_t: number | null;
  note: string; starred: boolean; semantic_class: number; environment_status: EnvironmentStatus; sample_ids: number[];
  quantity: number; parts: SurveyPart[]; created_at: string; updated_at: string; }
export interface SurveyCounts { queue: number; approved: number; rejected: number; all: number; }
export interface Survey { types: SurveyType[]; counts: SurveyCounts; }
export interface SurveySummary { counts: SurveyCounts; circularity: Record<Treatment, number>; total_mass_t: number;
  reuse_share: number | null; pending_samples: number; unlabeled_points: number | null; rooms_without_parts: string[]; }
export interface SurveyFraction { eak_code: string; name: string; treatment: Treatment; mass_t: number; }
export interface SurveyFractions { fractions: SurveyFraction[]; total_t: number; blocking_types: number; ready: boolean; }
export interface Sample { id: number; code: string; title: string; what: string; stage: SampleStage;
  result: SampleResult | null; type_ids: number[]; created_at: string; updated_at: string; }
export interface SurveySyncReport { types_created: number; parts_created: number; parts_existing: number; rooms_assigned: boolean; }
export interface SurveyTypeCreate { name: string; eak_code?: string; bim7aa_code?: string; unit?: string; treatment?: Treatment; }
export interface SurveyTypePatch { name?: string; eak_code?: string; bim7aa_code?: string; unit?: string; note?: string;
  treatment?: Treatment; review_status?: ReviewStatus; confidence?: number | null; mass_t?: number | null;
  starred?: boolean; quantity?: number; }
export interface SurveyPartPatch { type_id?: number; quantity?: number; starred?: boolean; note?: string; room_name?: string; }
export interface SampleCreate { title: string; what?: string; type_ids?: number[]; }
export interface SamplePatch { title?: string; what?: string; stage?: SampleStage; result?: SampleResult | null; }
export type RenderView = 'plan' | 'top' | 'front' | 'orbit';
export interface RenderQuery { view?: RenderView; orbit_index?: number; layers?: string[];
  highlight_instance?: number; highlight_cloud?: string; width?: number; height?: number; }
```

Client methods on `RuxApiClient`: `survey(signal?)`, `surveySummary(signal?)`, `surveyFractions(signal?)`, `syncSurvey(body?)`, `createSurveyType(body)`, `patchSurveyType(id, patch)`, `patchSurveyPart(code, patch)`, `samples(signal?)` → `Sample[]`, `createSample(body)`, `patchSample(id, patch)`, `deleteSample(id)`, `setSampleLinks(id, typeIds)`, `renderUrl(query): string`. `ApiRequestError` gains `get isUnprocessable(): boolean` (status 422).

- [ ] **Step 1: Failing test** — `apps/rux/frontend/src/test/survey.client.test.ts`:

```ts
// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

/**
 * Survey / sample client methods: paths, verbs, bodies, and the 422 gate
 * surfacing as a typed error. Payloads mirror docs/gui/openapi.yaml; they are
 * hand-written because the endpoints are new (re-record into fixtures.ts once
 * a real server serves them).
 */

import { describe, expect, it } from 'vitest';

import { ApiRequestError, RuxApiClient, type FetchLike } from '../api/client';
import { TREATMENTS } from '../api/types';

function client(payload: unknown, status = 200) {
  const calls: { url: string; method?: string; body?: string }[] = [];
  const fetchLike: FetchLike = (url, init) => {
    calls.push({ url, method: init?.method, body: init?.body as string | undefined });
    return Promise.resolve(
      new Response(status === 204 ? null : JSON.stringify(payload), {
        status,
        headers: { 'Content-Type': 'application/json' },
      }),
    );
  };
  return { calls, api: new RuxApiClient({ baseUrl: '/api/v1', fetch: fetchLike }) };
}

describe('survey client', () => {
  it('lists treatments in waste-hierarchy order', () => {
    expect([...TREATMENTS]).toEqual(['bevaring', 'genbrug', 'genanvendelse', 'nyttiggoerelse', 'bortskaffelse']);
  });

  it('reads the survey, summary and fractions', async () => {
    const { calls, api } = client({ types: [], counts: { queue: 0, approved: 0, rejected: 0, all: 0 } });
    await api.survey();
    await api.surveySummary();
    await api.surveyFractions();
    expect(calls.map((c) => c.url)).toEqual(['/api/v1/survey', '/api/v1/survey/summary', '/api/v1/survey/fractions']);
  });

  it('patches a type sparsely with a JSON body', async () => {
    const { calls, api } = client({ id: 4 });
    await api.patchSurveyType(4, { review_status: 'approved', mass_t: null });
    expect(calls[0]).toMatchObject({ url: '/api/v1/survey/types/4', method: 'PATCH' });
    expect(JSON.parse(calls[0].body!)).toEqual({ review_status: 'approved', mass_t: null });
  });

  it('url-encodes part codes', async () => {
    const { calls, api } = client({ code: 'RX-001' });
    await api.patchSurveyPart('RX-001', { starred: true });
    expect(calls[0].url).toBe('/api/v1/survey/parts/RX-001');
  });

  it('surfaces the sample gate as a 422 ApiRequestError', async () => {
    const { api } = client({ error: 'cannot be approved while sample(s) P-01 await a lab answer' }, 422);
    const err = await api.patchSurveyType(1, { review_status: 'approved' }).catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ApiRequestError);
    expect((err as ApiRequestError).isUnprocessable).toBe(true);
    expect((err as ApiRequestError).message).toContain('P-01');
  });

  it('manages samples: list, create, patch, links, delete', async () => {
    const { calls, api } = client({ samples: [] });
    expect(await api.samples()).toEqual([]);
    await api.createSample({ title: 'PCB i fugemasse', type_ids: [6] });
    await api.patchSample(2, { stage: 'svar', result: 'ren' });
    await api.setSampleLinks(2, [6, 11]);
    expect(calls.map((c) => `${c.method} ${c.url}`)).toEqual([
      'GET /api/v1/samples',
      'POST /api/v1/samples',
      'PATCH /api/v1/samples/2',
      'PUT /api/v1/samples/2/links',
    ]);
    expect(JSON.parse(calls[3].body!)).toEqual({ type_ids: [6, 11] });
  });

  it('deletes a sample expecting 204', async () => {
    const { calls, api } = client(null, 204);
    await api.deleteSample(3);
    expect(calls[0]).toMatchObject({ url: '/api/v1/samples/3', method: 'DELETE' });
  });

  it('builds render URLs with a comma layer list', () => {
    const { api } = client({});
    expect(api.renderUrl({ view: 'plan', highlight_instance: 12, layers: ['cloud', 'rooms'] })).toBe(
      '/api/v1/renders?view=plan&highlight_instance=12&layers=cloud%2Crooms',
    );
  });
});
```

If `FetchLike` is not exported from `client.ts`, export it (it is a type-only change). If `buildQuery` orders keys differently from insertion order, adjust the expected render URL to the order `buildQuery` produces and note it in the report; `renderUrl` must pass keys in the order `view, orbit_index, highlight_instance, highlight_cloud, layers, width, height`, skipping undefined.

- [ ] **Step 2: Run → FAIL** (`npm --prefix apps/rux/frontend test -- survey.client`).

- [ ] **Step 3: Implement** — add the types to `types.ts` (with `export const TREATMENTS = ['bevaring', 'genbrug', 'genanvendelse', 'nyttiggoerelse', 'bortskaffelse'] as const satisfies readonly Treatment[];` and a doc comment per interface pointing at the openapi schema name). In `client.ts`, add to `ApiRequestError`:

```ts
  /** The server refused a well-formed request on a rule, e.g. approving while a sample is pending. */
  get isUnprocessable(): boolean {
    return this.status === 422;
  }
```

and a `// --------------------------------------------------------- survey ----` section:

```ts
  survey(signal?: AbortSignal): Promise<Survey> {
    return this.requestJson<Survey>('/survey', undefined, signal);
  }

  surveySummary(signal?: AbortSignal): Promise<SurveySummary> {
    return this.requestJson<SurveySummary>('/survey/summary', undefined, signal);
  }

  surveyFractions(signal?: AbortSignal): Promise<SurveyFractions> {
    return this.requestJson<SurveyFractions>('/survey/fractions', undefined, signal);
  }

  /** Create parts for instances that have none. Idempotent; never overwrites edits. */
  syncSurvey(body: { instances_cloud?: string; semantic_cloud?: string; rooms_cloud?: string } = {}): Promise<SurveySyncReport> {
    return this.postJson<SurveySyncReport>('/survey/sync', body);
  }

  createSurveyType(body: SurveyTypeCreate): Promise<SurveyType> {
    return this.postJson<SurveyType>('/survey/types', body);
  }

  /** Sparse edit. `review_status: 'approved'` is refused with a 422 while a sample is pending. */
  patchSurveyType(id: number, patch: SurveyTypePatch): Promise<SurveyType> {
    return this.patchJson<SurveyType>(`/survey/types/${id}`, patch);
  }

  patchSurveyPart(code: string, patch: SurveyPartPatch): Promise<SurveyPart> {
    return this.patchJson<SurveyPart>(`/survey/parts/${encodeURIComponent(code)}`, patch);
  }

  async samples(signal?: AbortSignal): Promise<Sample[]> {
    const body = await this.requestJson<{ samples: Sample[] }>('/samples', undefined, signal);
    return body.samples;
  }

  createSample(body: SampleCreate): Promise<Sample> {
    return this.postJson<Sample>('/samples', body);
  }

  patchSample(id: number, patch: SamplePatch): Promise<Sample> {
    return this.patchJson<Sample>(`/samples/${id}`, patch);
  }

  setSampleLinks(id: number, typeIds: number[]): Promise<Sample> {
    return this.putJson<Sample>(`/samples/${id}/links`, { type_ids: typeIds });
  }

  async deleteSample(id: number): Promise<void> {
    const url = this.url(`/samples/${id}`);
    const response = await this.doFetch(url, { method: 'DELETE' });
    if (!response.ok) {
      throw new ApiRequestError(response.status, await describeFailure(response), url);
    }
  }

  /** URL of a server-rendered evidence image, for use as an `<img src>`. */
  renderUrl(query: RenderQuery): string {
    return this.url('/renders', {
      view: query.view,
      orbit_index: query.orbit_index,
      highlight_instance: query.highlight_instance,
      highlight_cloud: query.highlight_cloud,
      layers: query.layers?.join(','),
      width: query.width,
      height: query.height,
    });
  }
```

(Check `Query`'s type accepts `number | string | undefined` values and that `buildQuery` skips `undefined`; if it doesn't, filter undefined before calling.)

- [ ] **Step 4: Run** `npm --prefix apps/rux/frontend test && npm --prefix apps/rux/frontend run typecheck` → PASS.

- [ ] **Step 5: Commit**

```bash
git add apps/rux/frontend/src/api/types.ts apps/rux/frontend/src/api/client.ts apps/rux/frontend/src/test/survey.client.test.ts
git commit -m "feat(gui): frontend contract for survey, samples and renders"
```

---

## Phase exit criteria

- Full C++ suite: `ctest --test-dir build --output-on-failure --parallel $(nproc)` green; frontend `test`, `typecheck`, `build` green; `reuse lint` compliant; `ctest -R gui_api_contract_parses` green.
- `rux create survey` on a project with instances creates types/parts; re-running changes nothing.
- Against a running `rux gui`: approving a type linked to an unanswered sample returns 422 with the sample code; answering the sample makes the same request succeed.
- `GET /api/v1/renders?view=plan&highlight_instance=<id>` returns a PNG with that instance highlighted (or 503 on a machine without GL).
- Follow-up issue filed: record `fixtures.ts` payloads for the new endpoints from a real server once Phase 3 needs them.
