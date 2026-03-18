// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

// Sample tests to verify the test infrastructure is working

TEST_CASE("Basic arithmetic", "[sample]") {
    REQUIRE(1 + 1 == 2);
    REQUIRE(2 * 3 == 6);
}

TEST_CASE("Floating point comparison", "[sample]") {
    using Catch::Approx;
    
    double a = 0.1 + 0.2;
    REQUIRE(a == Approx(0.3));
}

TEST_CASE("String operations", "[sample]") {
    std::string str = "ReUseX";
    
    REQUIRE(str.length() == 6);
    REQUIRE(str.substr(0, 2) == "Re");
    REQUIRE_FALSE(str.empty());
}

TEST_CASE("Vector operations", "[sample]") {
    std::vector<int> vec = {1, 2, 3, 4, 5};
    
    REQUIRE(vec.size() == 5);
    REQUIRE(vec[0] == 1);
    REQUIRE(vec.back() == 5);
    
    SECTION("Vector modification") {
        vec.push_back(6);
        REQUIRE(vec.size() == 6);
        REQUIRE(vec.back() == 6);
    }
    
    SECTION("Vector clearing") {
        vec.clear();
        REQUIRE(vec.empty());
    }
}
