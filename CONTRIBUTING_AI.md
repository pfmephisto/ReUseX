<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
SPDX-License-Identifier: GPL-3.0-or-later
-->

# AI Assistant Development Guide

This file provides specific guidance for AI coding assistants working on the ReUseX codebase.

## Quick Reference

**Before making changes:**
1. Enter Nix dev shell: `nix develop`
2. Read relevant source files completely
3. Check existing patterns in similar code
4. Verify SPDX headers are present

**After making changes:**
1. Build: `cmake -B build && cmake --build build`
2. Run tests: `cd build && ctest --output-on-failure`
3. Format: `clang-format -i <changed_files>`
4. Commit with proper attribution

## Code Patterns

### File Headers

Every source file must have SPDX headers:

```cpp
// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Your code here...
```

### Type System

Always use project type aliases from `types.hpp`:
- `PointT` not `pcl::PointXYZRGB`
- `Cloud` / `CloudPtr` not `pcl::PointCloud<pcl::PointXYZRGB>`
- `pcl::Indices` not `std::vector<int>`

### Error Handling

```cpp
// Use spdlog for logging
spdlog::info("Processing {} points", cloud->size());
spdlog::warn("No normals found, computing...");
spdlog::error("Failed to load file: {}", path);

// Throw for recoverable errors
throw std::runtime_error(fmt::format("Invalid parameter: {}", param));

// Use RAII for resources
auto db = std::make_shared<RTABMapDatabase>(path);
```

### Naming Conventions

- Classes: `PascalCase` (e.g., `CellComplex`, `RTABMapDatabase`)
- Functions: `snake_case` (e.g., `segment_planes`, `compute_normals`)
- Variables: `snake_case` (e.g., `point_cloud`, `node_id`)
- Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_ITERATIONS`)
- Private members: `snake_case` with trailing underscore (e.g., `impl_`)

### Include Order

```cpp
// 1. Corresponding header (for .cpp files)
#include "ReUseX/geometry/CellComplex.hpp"

// 2. Other ReUseX headers
#include "ReUseX/core/logging.hpp"
#include "ReUseX/types.hpp"

// 3. External library headers
#include <pcl/filters/voxel_grid.h>
#include <spdlog/spdlog.h>

// 4. Standard library headers
#include <algorithm>
#include <memory>
#include <vector>
```

## Common Pitfalls

### RTABMap API

```cpp
// ❌ WRONG - init() returns void, not bool
if (rtabmap.init()) { ... }

// ✅ CORRECT - use try-catch
try {
    rtabmap.init();
} catch (const std::exception& e) {
    spdlog::error("Failed to initialize: {}", e.what());
}
```

### Database Access

```cpp
// ❌ WRONG - labels need rotation and offset handling
cv::Mat labels = db->getLabels(nodeId);
cv::imwrite("labels.png", labels);

// ✅ CORRECT - use saveLabels which handles encoding
db->saveLabels(nodeId, labels);

// Note: getLabels() returns CV_32S (-1 for background)
// saveLabels() expects CV_32S and converts to CV_16U internally
```

### Multi-threading

```cpp
// ❌ WRONG - RTABMapDatabase is NOT thread-safe
auto db = std::make_shared<RTABMapDatabase>(path);
#pragma omp parallel for
for (int i = 0; i < nodeIds.size(); ++i) {
    auto image = db->getImage(nodeIds[i]); // RACE CONDITION
}

// ✅ CORRECT - one database instance per thread
#pragma omp parallel
{
    auto thread_db = std::make_shared<RTABMapDatabase>(path);
    #pragma omp for
    for (int i = 0; i < nodeIds.size(); ++i) {
        auto image = thread_db->getImage(nodeIds[i]);
    }
}
```

## Testing

### Writing Tests

Place tests in `tests/unit/<module>/` using Catch2:

```cpp
// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ReUseX/geometry/my_feature.hpp"

TEST_CASE("MyFeature basic functionality", "[geometry]") {
    SECTION("handles empty input") {
        Cloud cloud;
        REQUIRE(my_function(cloud).empty());
    }

    SECTION("processes valid data") {
        auto cloud = create_test_cloud();
        auto result = my_function(cloud);
        REQUIRE(result.size() > 0);
        REQUIRE_THAT(result[0],
            Catch::Matchers::WithinRel(1.0, 0.01));
    }
}
```

### Running Tests

```bash
# All tests
cd build && ctest --output-on-failure

# Specific test
ctest -R test_cell_complex

# Verbose output
ctest --verbose
```

## Build Considerations

### Adding Source Files

**No CMakeLists.txt updates needed!** The build system auto-detects:
- `.cpp` / `.cu` files in `libs/reusex/src/`
- `.hpp` files in `libs/reusex/include/ReUseX/`
- Test files in `tests/unit/`

Just create your files in the right location and rebuild.

### Adding Dependencies

For external dependencies, update:
- `libs/reusex/cmake/Dependencies.cmake` - find_package() calls
- `libs/reusex/cmake/ReUseXLibrary.cmake` - link dependencies
- `flake.nix` - Nix package dependencies

### Visualization Code

Code using PCL visualization or Qt should go in:
- `libs/reusex/src/visualize/` - automatically excluded from main target
- Links against `ReUseX_visualization` target
- Controlled by `-DBUILD_VISUALIZATION=ON/OFF`

## ML/Vision Code

### Backend Abstraction

When working with ML models:

```cpp
// ✅ Use factory pattern
auto backend = BackendFactory::create(
    BackendFactory::detect_backend(model_path)
);
auto model = backend->createModel(model_path);

// ❌ Don't hardcode backends
auto model = std::make_shared<TensorRTModel>(model_path);
```

### Dataset Pattern

```cpp
// For TensorRT
class MyDataset : public tensor_rt::Dataset {
    std::pair<cv::Mat, cv::Mat> forward(size_t idx) override {
        // Return (image, labels) pair
        return {getImage(idx), getLabels(idx)};
    }
};

// For PyTorch
class MyDataset : public libtorch::Dataset {
    torch::data::Example<> get(size_t index) override {
        // Return torch example
    }
};
```

## Documentation

### Doxygen Comments

```cpp
/**
 * @brief Brief one-line description
 *
 * Detailed description with multiple paragraphs if needed.
 * Explain the why, not just the what.
 *
 * @param cloud Input point cloud
 * @param threshold Segmentation threshold in meters
 * @return Segmented point cloud with labels
 *
 * @throws std::runtime_error if cloud is empty
 *
 * @note Thread-safe if each thread uses separate instances
 * @warning Modifies input cloud if in-place parameter is true
 */
CloudPtr segment_planes(CloudPtr cloud, float threshold);
```

### Updating CLAUDE.md

When adding significant new features:
1. Update relevant sections in CLAUDE.md
2. Add to "Common Tasks" if it's a new workflow
3. Update MEMORY.md for patterns/lessons learned

## Git Workflow

### Commits

```bash
# Stage specific files (not git add .)
git add src/geometry/new_feature.cpp include/ReUseX/geometry/new_feature.hpp

# Let Claude create commit message
# (Will auto-add Co-Authored-By trailer)
```

### Pull Requests

```bash
# Create PR (Claude will analyze full diff)
# PR title: Short (<70 chars)
# PR body: Detailed with test plan
```

## Resources

- Main docs: `CLAUDE.md` - High-level project guide
- Architecture: `ARCHITECTURE.md` - System design
- Humans: `CONTRIBUTING.md` - Contributing guide
- Memory: `.claude/memory/MEMORY.md` - Lessons learned
- API docs: Build with `cmake --build build --target doc`

## Questions?

When uncertain:
1. Check existing code for patterns
2. Look in MEMORY.md for known issues
3. Ask the user for clarification
4. Don't guess - especially for destructive operations

Remember: Read before modifying, test after changing, format before committing.
