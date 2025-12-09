# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Testing Guide for ReUseX

This document describes the testing infrastructure and procedures for the ReUseX project.

## Overview

ReUseX uses Catch2 v3 framework for C++ unit testing.

## C++ Tests

### Setup

C++ tests use Catch2 v3 framework and are located in the `tests/` directory. 

### Building and Running C++ Tests

Build the project with tests enabled:
```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
```

Run all C++ tests:
```bash
cd build
ctest --output-on-failure
```

Or run the test executable directly:
```bash
./build/tests/reusex_tests
```

Run specific test case:
```bash
./build/tests/reusex_tests "[math][remap]"
```

### C++ Test Coverage

The C++ test suite includes tests for:

1. **Math Utilities** (`test_math.cpp`) - 11 test cases
   - `remap()` function for value remapping
   - Tests for different data types (int, float, double)
   - Edge cases and extrapolation

2. **Geometry Utilities** (`test_geometry_utils.cpp`) - 12 test cases
   - `dist_plane_point()` function for point-to-plane distance
   - Tests with various plane orientations
   - Normalized and non-normalized normals

3. **Type Definitions** (`test_types.cpp`) - Existing tests
   - Point cloud creation and operations
   - Normal cloud operations
   - Indices operations
   - Eigen vector containers

4. **Sample Tests** (`test_sample.cpp`) - Existing tests
   - Basic arithmetic and string operations
   - Floating-point comparisons
   - Vector operations

### Test Structure

C++ tests follow Catch2's section-based structure:
```cpp
// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

TEST_CASE("Description", "[tag1][tag2]") {
    SECTION("Specific behavior") {
        // Arrange
        // Act
        // Assert
        REQUIRE(condition);
    }
}
```

## Configuration Files

### tests/CMakeLists.txt
Configuration for C++ tests:
- Test source discovery
- Catch2 integration
- Test executable setup

## Continuous Integration

Tests can be integrated into CI/CD pipelines:

### GitHub Actions Example
```yaml
- name: Run C++ Tests
  run: |
    cmake -B build -DBUILD_TESTS=ON
    cmake --build build
    cd build && ctest --output-on-failure
```

## Adding New Tests

### Adding C++ Tests

1. Create a new test file in `tests/` with prefix `test_`:
   ```bash
   touch tests/test_new_feature.cpp
   ```

2. Add test cases using Catch2 macros

3. Rebuild and run:
   ```bash
   cmake --build build
   ./build/tests/reusex_tests "[new_feature]"
   ```

## Coverage Goals

The testing strategy aims for:
- **Unit Test Coverage**: Test all utility functions and core algorithms
- **Edge Case Testing**: Cover boundary conditions and error cases
- **Integration Testing**: Verify component interactions (future work)
- **Regression Testing**: Prevent bugs from reappearing

## Current Test Statistics

- **C++ Tests**: 4 test files covering math utilities, geometry functions, and type operations
- All tests passing ✓

## Future Improvements

- [ ] Add integration tests for end-to-end workflows
- [ ] Increase coverage for complex geometry functions
- [ ] Add performance benchmarks
- [ ] Set up automated coverage reporting
- [ ] Add mock objects for testing components with heavy dependencies
- [ ] Create fixtures for common test data

## Troubleshooting

### C++ Build Failures
Check that all dependencies are available:
```bash
nix develop  # If using Nix
# OR
cmake -B build -DBUILD_TESTS=ON  # Check for missing dependencies
```

## Resources

- [Catch2 documentation](https://github.com/catchorg/Catch2)
- [CMake CTest documentation](https://cmake.org/cmake/help/latest/manual/ctest.1.html)
