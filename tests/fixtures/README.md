# Test Fixtures

This directory contains test data and sample files used by the test suite.

## Purpose

Fixtures provide consistent, reusable test data for:
- Unit tests that need sample inputs
- Integration tests requiring realistic data
- Performance benchmarks
- Documentation examples

## Organization

Organize fixtures by module or data type:

```
fixtures/
├── images/              # Sample images for vision tests
├── databases/           # Small RTABMap .db files for IO tests
├── pointclouds/         # Sample point cloud data
├── models/              # Small ONNX models for inference tests
└── config/              # Sample configuration files
```

## Guidelines

### Size Constraints

- Keep individual files under 1MB when possible
- Use Git LFS for files larger than 1MB
- Prefer small, focused samples over large realistic data

### File Naming

Use descriptive names that indicate the fixture purpose:
- `test_image_640x480_rgb.png` - Clear dimensions and format
- `rtabmap_minimal_3nodes.db` - Indicates content and size
- `pointcloud_room_corner.ply` - Describes scene content

## Current Fixtures

(None yet - add fixtures as needed for test development)

## Usage in Tests

```cpp
#include <filesystem>

TEST_CASE("Load test image", "[vision]") {
    namespace fs = std::filesystem;
    auto fixture_path = fs::path(__FILE__).parent_path() / "fixtures" / "images" / "test.png";

    cv::Mat img = cv::imread(fixture_path.string());
    REQUIRE(!img.empty());
}
```
