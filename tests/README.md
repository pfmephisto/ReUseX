# ReUseX Testing Guide

This directory contains the test suite for the ReUseX library.

## Directory Structure

```
tests/
├── unit/                    # Unit tests organized by module
│   ├── core/               # Core functionality tests
│   ├── geometry/           # Geometry utilities tests
│   ├── io/                 # I/O and database tests
│   ├── utils/              # Utility function tests
│   ├── vision/             # Vision module tests
│   └── visualize/          # Visualization tests
├── integration/            # Integration and end-to-end tests
├── fixtures/               # Test data and sample files
└── README.md               # This file
```

## Running Tests

### Quick Run

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

### Verbose Output

```bash
cd build
ctest --verbose
```

### Run Specific Tests

```bash
# Run only unit tests
./build/reusex_unit_tests

# Run tests matching a pattern
./build/reusex_unit_tests "[geometry]"

# List all tests
./build/reusex_unit_tests --list-tests
```

## Code Coverage

### Generate Coverage Report

```bash
./tools/coverage/generate_coverage.sh
```

This script will:
1. Configure with coverage enabled
2. Build the project
3. Run all tests
4. Generate HTML coverage report in `build/coverage_html/`

### View Coverage Report

```bash
xdg-open build/coverage_html/index.html
```

### Requirements

Install lcov for coverage reporting:
```bash
sudo apt install lcov
```

## Current Test Coverage

**Overall Coverage**: ~5% (as of 2026-03-11)

### Coverage by Module

| Module       | Coverage | Notes                          |
|--------------|----------|--------------------------------|
| Core         | ~10%     | Basic type tests only          |
| Geometry     | ~8%      | Utilities tested               |
| IO           | 0%       | **Priority**: RTABMapDatabase  |
| Utils        | ~15%     | Math utilities covered         |
| Vision       | 0%       | **Priority**: Dataset, YOLO    |
| Visualize    | 0%       | Needs integration tests        |

### Priority Testing Gaps

1. **RTABMapDatabase** (io module)
   - Critical new functionality
   - Test getImage, getLabels, saveLabels
   - Verify rotation and encoding conventions

2. **Dataset Classes** (vision module)
   - IDataset, TorchDataset, TensorRTDataset
   - Test data loading and transformation

3. **YOLO Detection** (vision module)
   - Test inference pipeline
   - Verify detection output format

4. **Geometry Segmentation** (geometry module)
   - Test segmentation algorithms
   - Verify point cloud processing

## Writing Tests

### File Organization

Place tests in the appropriate module directory:
- `unit/core/` - Core types, logging
- `unit/geometry/` - Point clouds, segmentation
- `unit/io/` - Database access, file I/O
- `unit/utils/` - Math, string utilities
- `unit/vision/` - Datasets, models, object detection
- `unit/visualize/` - GUI components

### Test Template

```cpp
#include <catch2/catch_test_macros.hpp>
#include <ReUseX/module/component.hpp>

TEST_CASE("Component description", "[module][tag]") {
    SECTION("specific behavior") {
        // Arrange
        auto obj = ReUseX::module::Component();

        // Act
        auto result = obj.method();

        // Assert
        REQUIRE(result == expected);
    }
}
```

### Best Practices

1. **Use descriptive test names**: Explain what's being tested
2. **Tag tests appropriately**: Use `[module]` tags for filtering
3. **Test one thing per TEST_CASE**: Keep tests focused
4. **Use SECTION for variations**: Group related assertions
5. **Provide clear failure messages**: Use `INFO()` for context
6. **Test edge cases**: Zero, negative, null, empty, large values
7. **Mock external dependencies**: Isolate unit tests

### Fixtures

Place test data in `fixtures/` with a descriptive README:
- Sample images
- Test databases
- Point cloud samples
- Configuration files

## Contributing

When contributing code:
1. Write tests for new functionality
2. Ensure all tests pass: `ctest --output-on-failure`
3. Verify coverage hasn't decreased significantly
4. Update this README if adding new test categories
