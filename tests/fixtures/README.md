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
├── scans/               # Trimmed real captures as .rux projects (see its README)
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

### Real capture data

Anything cut from a real scan needs, in [`scans/README.md`](scans/README.md) or
an equivalent alongside it:

1. **Provenance** — which capture, which frames, when.
2. **Licence** — and for third-party datasets, an explicit redistribution
   grant. ARKitScenes (CC BY-NC-ND 4.0) and MuSHRoom do not have one; they are
   fine to benchmark against locally but must not be vendored here. A capture
   made by the maintainer avoids the question entirely and is preferred.
3. **A privacy check** — every frame opened and inspected. No people, no
   readable documents or screens, no vehicle number plates.
4. **The exact commands** used to trim it, so it can be regenerated.

## Current Fixtures

| Fixture | Size | Contents | Used by |
|---|---|---|---|
| [`scans/office_corridor.rux`](scans/README.md) | 2.04 MiB (LFS) | 10 real iOS-LiDAR sensor frames (colour/depth/confidence/pose/intrinsics) | `tests/integration/test_real_scan_fixture.cpp` (`ctest -R fixture`) |

Binary fixtures are tracked in git-lfs (see `.gitattributes`). A clone without
`git lfs pull` gets pointer files; tests should detect that and skip with an
actionable message rather than failing on a corrupt read.

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
