# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Testing Guide for ReUseX

This document describes the testing infrastructure and procedures for the ReUseX project.

## Overview

ReUseX has both C++ and Python components, each with their own testing frameworks:
- **C++ Tests**: Using Catch2 v3 framework
- **Python Tests**: Using pytest framework

## Python Tests

### Setup

Python tests are located in `python/tests/` directory. The following dependencies are required:

```bash
pip install pytest pytest-cov numpy pandas
```

### Running Python Tests

Run all Python tests:
```bash
pytest python/tests/ -v
```

Run tests with coverage report:
```bash
pytest python/tests/ --cov=python/ReUseX --cov-report=term --cov-report=html
```

Run specific test file:
```bash
pytest python/tests/test_cli.py -v
```

Run specific test class or function:
```bash
pytest python/tests/test_cli.py::TestChunks::test_chunks_basic -v
```

### Python Test Coverage

The Python test suite includes **45 tests** covering:

1. **CLI Utilities** (`test_cli.py`) - 6 tests
   - `chunks()` function for dividing lists into chunks

2. **Export Functions** (`test_export.py`) - 5 tests
   - PTS file format export logic
   - Color conversion (BGR to RGB)
   - DataFrame structure validation

3. **Torch Utilities** (`test_torch_util.py`) - 6 tests
   - `_collate()` function for batching data

4. **Samplers** (`test_samplers.py`) - 11 tests
   - `StepSampler` class for step-based sampling
   - `ConsecutiveBatchSampler` class for consecutive pair sampling

5. **Dataset Utilities** (`test_dataset.py`) - 8 tests
   - `make_pairs()` function for creating unique pairs

6. **Pose Graph Utilities** (`test_pose_graph_utils.py`) - 9 tests
   - `to_numpy()` function for data conversion
   - `merge_corres()` function for correspondence merging

### Test Structure

Each test file follows this structure:
```python
# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Tests for <module> functions."""

import pytest

class Test<ClassName>:
    """Test suite for <ClassName>."""
    
    def test_<feature>(self):
        """Test <specific behavior>."""
        # Arrange
        # Act
        # Assert
```

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

### pytest.ini
Configuration for Python tests:
- Test discovery patterns
- Output verbosity
- Warning handling

### tests/CMakeLists.txt
Configuration for C++ tests:
- Test source discovery
- Catch2 integration
- Test executable setup

## Continuous Integration

Tests can be integrated into CI/CD pipelines:

### GitHub Actions Example
```yaml
- name: Run Python Tests
  run: |
    pip install pytest pytest-cov numpy pandas
    pytest python/tests/ -v --cov=python/ReUseX

- name: Run C++ Tests
  run: |
    cmake -B build -DBUILD_TESTS=ON
    cmake --build build
    cd build && ctest --output-on-failure
```

## Adding New Tests

### Adding Python Tests

1. Create a new test file in `python/tests/` with prefix `test_`:
   ```bash
   touch python/tests/test_new_module.py
   ```

2. Add test class and methods following the existing structure

3. Run the new tests:
   ```bash
   pytest python/tests/test_new_module.py -v
   ```

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

- **Python Tests**: 45 tests covering 6 modules
- **C++ Tests**: 4 test files with multiple test cases each
- All tests passing ✓

## Future Improvements

- [ ] Add integration tests for end-to-end workflows
- [ ] Increase coverage for complex geometry functions
- [ ] Add performance benchmarks
- [ ] Set up automated coverage reporting
- [ ] Add mock objects for testing components with heavy dependencies
- [ ] Create fixtures for common test data

## Troubleshooting

### Python Tests Not Found
Ensure you're running pytest from the repository root:
```bash
cd /path/to/ReUseX
pytest python/tests/
```

### C++ Build Failures
Check that all dependencies are available:
```bash
nix develop  # If using Nix
# OR
cmake -B build -DBUILD_TESTS=ON  # Check for missing dependencies
```

### Import Errors
Some modules depend on the compiled `_core` module. Tests for these use isolated function implementations to avoid dependency issues.

## Resources

- [pytest documentation](https://docs.pytest.org/)
- [Catch2 documentation](https://github.com/catchorg/Catch2)
- [CMake CTest documentation](https://cmake.org/cmake/help/latest/manual/ctest.1.html)
