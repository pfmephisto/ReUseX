<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX Unit Tests

This directory contains unit tests for the ReUseX library using Catch2 v3.

## Building and Running Tests

### Configure with Tests Enabled (default)

```bash
cmake -B build -DBUILD_TESTS=ON
```

### Build Tests

```bash
cmake --build build --target reusex_tests
```

### Run All Tests

```bash
cd build
ctest --output-on-failure
```

Or run the test executable directly:

```bash
./build/tests/reusex_tests
```

### Run Specific Tests

```bash
# Run tests matching a tag
./build/tests/reusex_tests "[sample]"

# Run tests matching a name
./build/tests/reusex_tests "Point cloud*"

# List all available tests
./build/tests/reusex_tests --list-tests

# List all tags
./build/tests/reusex_tests --list-tags
```

## Test Structure

- `test_sample.cpp` - Sample tests demonstrating Catch2 features
- `test_types.cpp` - Tests for ReUseX type definitions and basic operations

## Writing New Tests

Create a new `.cpp` file in this directory with your tests. The CMake configuration will automatically discover and include it.

Example test structure:

```cpp
#include <catch2/catch_test_macros.hpp>
#include <ReUseX/your_header.hpp>

TEST_CASE("Description of what is being tested", "[tag]") {
    // Setup
    YourClass obj;
    
    SECTION("Test scenario 1") {
        REQUIRE(obj.method() == expected_value);
    }
    
    SECTION("Test scenario 2") {
        REQUIRE_FALSE(obj.is_invalid());
    }
}
```

## Useful Catch2 Macros

- `REQUIRE(condition)` - Test fails if condition is false
- `REQUIRE_FALSE(condition)` - Test fails if condition is true
- `CHECK(condition)` - Like REQUIRE but continues after failure
- `REQUIRE_THROWS(expression)` - Test fails if expression doesn't throw
- `REQUIRE_NOTHROW(expression)` - Test fails if expression throws

For floating point comparisons:
```cpp
#include <catch2/catch_approx.hpp>
using Catch::Approx;

REQUIRE(value == Approx(expected).epsilon(0.01));
```

## Documentation

- [Catch2 Documentation](https://github.com/catchorg/Catch2/tree/devel/docs)
- [Catch2 Tutorial](https://github.com/catchorg/Catch2/blob/devel/docs/tutorial.md)
