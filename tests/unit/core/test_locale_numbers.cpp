// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Text -> number must not follow LC_NUMERIC. A QApplication calls
// setlocale(LC_ALL, ""), and under da_DK strtod read the stored intrinsics
// "799.8478" as 799 and the local_transform "-1.19e-07, 0, 1.0000001, …" as
// [-1, 0, 0, …] — the Qt client's ICP then aligned two lines (Q2 review).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <core/SensorIntrinsics.hpp>
#include <utils/parse_number.hpp>

#include <clocale>
#include <cstdlib>
#include <string>

using Catch::Approx;

namespace {

/// Switch LC_NUMERIC to a comma-decimal locale for the scope; restore after.
struct CommaLocale {
  std::string old;
  bool ok = false;
  CommaLocale() {
    if (const char *cur = std::setlocale(LC_NUMERIC, nullptr))
      old = cur;
    for (const char *name : {"da_DK.UTF-8", "da_DK.utf8", "de_DE.UTF-8",
                             "de_DE.utf8", "fr_FR.UTF-8", "nl_NL.UTF-8"})
      if (std::setlocale(LC_NUMERIC, name)) {
        ok = std::strtod("1.5", nullptr) == 1.0; // really comma-decimal
        if (ok)
          return;
      }
  }
  ~CommaLocale() {
    std::setlocale(LC_NUMERIC, old.empty() ? "C" : old.c_str());
  }
};

} // namespace

TEST_CASE("SensorIntrinsics_FromJson_IgnoresACommaDecimalLocale",
          "[core][locale]") {
  reusex::core::SensorIntrinsics k;
  k.fx = 795.7807006835938;
  k.fy = 799.8478;
  k.cx = 360.99725341796875;
  k.cy = 476.78765869140625;
  k.width = 720;
  k.height = 960;
  k.local_transform = {-1.1920928955078125e-07,
                       0,
                       1.0000001192092896,
                       0,
                       0,
                       -1.000000238418579,
                       0,
                       0,
                       1.0000001192092896,
                       0,
                       -1.1920928955078125e-07,
                       0,
                       0,
                       0,
                       0,
                       1};
  // The exact text the NewOffice project stores.
  const std::string stored =
      R"({"fx":795.7807006835938,"fy":799.8478,"cx":360.99725341796875,)"
      R"("cy":476.78765869140625,"width":720,"height":960,"local_transform":)"
      R"([-1.1920928955078125e-07,0,1.0000001192092896,0,0,-1.000000238418579,)"
      R"(0,0,1.0000001192092896,0,-1.1920928955078125e-07,0,0,0,0,1]})";

  CommaLocale comma;
  if (!comma.ok)
    SKIP("no comma-decimal locale installed (da_DK, de_DE, fr_FR, nl_NL)");

  for (const std::string &json : {stored, k.to_json()}) {
    const auto r = reusex::core::SensorIntrinsics::from_json(json);
    CHECK(r.fx == Approx(795.7807006835938));
    CHECK(r.fy == Approx(799.8478));
    CHECK(r.cx == Approx(360.99725341796875));
    CHECK(r.cy == Approx(476.78765869140625));
    CHECK(r.width == 720);
    CHECK(r.height == 960);
    for (int i = 0; i < 16; ++i)
      CHECK(r.local_transform[i] == Approx(k.local_transform[i]).margin(1e-9));
  }
}

TEST_CASE("ParseNumber_IsLocaleFreeAndStodLike", "[utils][locale]") {
  CommaLocale comma; // runs in either locale; the point is it never matters
  std::size_t used = 0;
  CHECK(reusex::utils::to_double("  +2.5e3xyz", &used) == 2500.0);
  CHECK(used == 8);
  CHECK(reusex::utils::to_double("-1.19e-07") == Approx(-1.19e-07));
  CHECK(reusex::utils::to_float("0.35") == Approx(0.35f));
  CHECK_THROWS_AS(reusex::utils::to_double("abc"), std::invalid_argument);
  CHECK_FALSE(reusex::utils::parse_number<double>("").has_value());
  CHECK(reusex::utils::parse_number<double>(",5", &used) == std::nullopt);
  CHECK(used == 0);
}
