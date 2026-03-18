# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ReUseX is a C++20/CUDA project for processing 3D point cloud scans of building interiors. It combines geometric processing, deep learning, and computational geometry to create semantic 3D models for building reuse and renovation projects.

**Key capabilities:**
- Point cloud processing from RTABMap SLAM databases
- Planar and room segmentation using CGAL and GraphBLAS
- Semantic segmentation via YOLO/SAM2 models (PyTorch/TensorRT)
- Cell complex 3D reconstruction
- Mesh generation with texture mapping
- Support for E57, PCD, OpenNURBS, and HDF5 formats

## Build System

### Development Environment

The project uses **Nix flakes** for reproducible builds. Always enter the dev shell first:

```bash
nix develop
```

### Building

```bash
# Standard build (Release mode, all features)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Debug build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build

# Build without visualization (reduces dependencies)
cmake -B build -DBUILD_VISUALIZATION=OFF
cmake --build build

# Build without tests
cmake -B build -DBUILD_TESTS=OFF
cmake --build build
```

**Important build options:**
- `-DBUILD_VISUALIZATION=ON/OFF` - Visualization library (default: ON)
- `-DBUILD_TESTS=ON/OFF` - Unit tests (default: ON)
- `-DBUILD_DOCUMENTATION=ON/OFF` - Doxygen docs (default: ON)
- `-DBUILD_PYTHON_BINDINGS=ON/OFF` - Python bindings (default: OFF, currently disabled)
- `-DGUI_ENABLED=ON/OFF` - CGAL GUI features (default: OFF)
- `-DENABLE_COVERAGE=ON/OFF` - Code coverage (default: OFF)

**CMake auto-detection:** The build system uses `GLOB_RECURSE` with `CONFIGURE_DEPENDS`, so new .cpp/.cu files are automatically detected. No need to manually update CMakeLists.txt when adding source files.

### Testing

```bash
# Build and run all tests
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure

# Run with verbose output
ctest --verbose

# Run specific test
ctest -R test_name_pattern
```

Tests are located in `tests/unit/` organized by module (core, geometry, io, vision, utils). Uses Catch2 v3 framework.

### Documentation

```bash
# Generate Doxygen documentation
cmake -B build -DBUILD_DOCUMENTATION=ON
cmake --build build --target doc

# View generated docs
xdg-open doc/html/index.html
```

## Architecture

### Project Structure

```
ReUseX/
├── libs/reusex/               # Core C++ library
│   ├── include/ReUseX/        # Public headers
│   │   ├── core/              # Logging, version
│   │   ├── geometry/          # Point cloud processing, segmentation
│   │   ├── io/                # RTABMap, E57, Rhino I/O
│   │   ├── vision/            # ML models, backends, datasets
│   │   ├── visualize/         # PCL visualization (optional)
│   │   ├── utils/             # Math, formatters
│   │   └── types.hpp          # Core PCL type aliases
│   ├── src/                   # Implementation files
│   └── extern/                # Vendored dependencies
├── apps/rux/                  # CLI application
│   └── src/                   # Subcommand implementations
├── tests/                     # Unit and integration tests
│   ├── unit/                  # Per-module unit tests
│   └── fixtures/              # Test data
├── python/                    # Python utilities (standalone scripts)
└── models/                    # Pre-trained models (not in repo)
```

### Core Libraries and Two-Target Design

**ReUseX Library** (`libs/reusex/`):
- **Main target** (`ReUseX`): Core library excluding visualization components
- **Visualization target** (`ReUseX_visualization`): Optional PCL/Qt visualization features
- Visualization is excluded to reduce dependencies when only core processing is needed
- Both targets export headers from `include/ReUseX/`

**RUX CLI** (`apps/rux/`):
- Command-line interface with subcommands
- Links against both `ReUseX` and optionally `ReUseX_visualization`
- Uses CLI11 for argument parsing, spdlog for logging

### Type System (types.hpp)

Core PCL type aliases used throughout the codebase:
- `PointT` = `pcl::PointXYZRGB` - Point cloud points with color
- `NormalT` = `pcl::Normal` - Surface normals
- `LabelT` = `pcl::Label` - Segmentation labels
- `Cloud` / `CloudPtr` - Point cloud containers
- `CloudN` / `CloudL` / `CloudLoc` - Normals, labels, locations

### Geometry Module

**CellComplex** (`geometry/CellComplex.hpp`):
- Graph-based 3D spatial representation using Boost.Graph
- Node types: Cell (room), Face (wall/floor/ceiling), Vertex (corner)
- Uses `boost::adjacency_list` with custom vertex/edge properties
- Core abstraction for room segmentation and 3D reconstruction

**Segmentation algorithms:**
- `segment_planes.hpp`: Planar surface detection (RANSAC-based)
- `segment_rooms.hpp`: Room partitioning via GraphBLAS/LAGraph
- `Solidifier.hpp`: Converts segmented planes to watertight meshes
- `regularization.hpp`: Geometric constraint enforcement

**SceneGraph and Registry:**
- Manage spatial relationships and object hierarchies
- Used for tracking plane assignments and room boundaries

### Vision/ML Module

**Backend abstraction** (vision/IMLBackend.hpp, BackendFactory.hpp):
- Pluggable backend system for ML inference
- Currently supports: **TensorRT** (primary), libTorch (planned), ONNX (planned)
- Backend detection based on file extension (.engine, .pt, .onnx)
- Factory pattern: `BackendFactory::detect_backend()` → `BackendFactory::create()`

**Model interfaces:**
- `IModel`: Base interface for all ML models
- `Yolo`: YOLO object detection/segmentation
- `Sam3`: SAM2 segmentation (TensorRT implementation)
- Models loaded from filesystem paths, backend auto-detected

**Dataset interfaces:**
- `IDataset`: Base dataset interface
- `libtorch::Dataset`: PyTorch-compatible dataset
- `tensor_rt::Dataset`: TensorRT-optimized dataset
- All datasets provide `(image, label)` pairs via `forward()`

**Key vision components:**
- `annotate.hpp`: Semantic annotation pipeline
- `project.hpp`: 3D-to-2D projection for annotation
- `Dataloader.hpp`: Batch data loading
- `osd/`: On-screen display for visualization

### I/O Module

**RTABMapDatabase** (`io/RTABMapDatabase.hpp`):
- Unified wrapper for RTABMap SLAM database access
- Uses Pimpl idiom to hide RTABMap dependencies from public headers
- Manages both RTABMap core tables (Node, Data, Link) and custom Segmentation table
- **Important conventions:**
  - Images are rotated 90° clockwise on read (RTABMap convention)
  - Labels stored as CV_16U with +1 offset (0 = background)
  - API returns CV_32S labels with -1 for background, 0+ for classes
  - NOT thread-safe (sqlite3 limitation) - create per-thread instances if needed

**Other I/O:**
- `rtabmap.hpp`: RTABMap integration utilities
- `rhino.hpp`: OpenNURBS (.3dm) import/export
- `reusex.hpp`: Custom HDF5-based format

### CLI Subcommands (apps/rux/src/)

Organized by operation:
- `import/rtabmap.cpp`: Import RTABMap SLAM databases
- `segment.cpp`: Plane and room segmentation
- `mesh.cpp`: 3D mesh generation
- `texture.cpp`: Texture mapping
- `annotate.cpp`: Semantic annotation with ML models
- `view.cpp`: Point cloud visualization (requires BUILD_VISUALIZATION)
- `export.cpp`: Export to various formats
- `assemble.cpp`: Multi-scan assembly
- `project.cpp`: 3D-to-2D projection

## Development Patterns

### Adding New Source Files

**No CMakeLists.txt updates required.** The build system automatically detects:
- C++ sources: `libs/reusex/src/**/*.cpp`
- CUDA sources: `libs/reusex/src/**/*.cu`
- Headers: `libs/reusex/include/ReUseX/**/*.hpp`
- Tests: `tests/unit/**/*.cpp`

Just add your file in the correct location and rebuild.

### Creating New ML Backends

1. Inherit from `IMLBackend` interface
2. Implement `createModel()` and `createDataset()`
3. Register file extension in `BackendFactory::detect_backend_from_file()`
4. Add backend enum to `BackendFactory::Backend`

### Adding New CLI Subcommands

1. Create `apps/rux/src/your_command.cpp`
2. Add subcommand setup in `apps/rux/src/rux.cpp`
3. Use existing commands as templates (e.g., `segment.cpp`)

### Working with Point Clouds

- Always use type aliases from `types.hpp` (PointT, Cloud, CloudPtr)
- Point clouds use `pcl::PointXYZRGB` (XYZ + RGB color)
- Use `pcl::Indices` for index vectors, not `std::vector<int>`

### Error Handling

- Use `spdlog` for logging (trace, debug, info, warn, error, critical)
- Throw `std::runtime_error` for recoverable errors
- Use RAII for resource management (smart pointers, custom destructors)

### RTABMap API Gotchas (from MEMORY.md)

- `rtabmap::Rtabmap::init()` returns void (use try-catch for errors)
- `std::multimap` is in `<map>` header, not `<multimap>`
- Segmentation table foreign key references `Node(id)` (singular)

## TODO Comment Conventions

This project uses structured TODO comments compatible with the [tdg (TODO Generator)](https://gitlab.com/ribtoks/tdg) tool. A GitHub Action automatically parses these comments and can sync them with issue trackers.

### Format Specification

All TODO-style comments must follow this format:

```cpp
// [MARKER]: Brief imperative title (50-70 chars max)
// category=ModuleName estimate=Xh [issue=N] [author=alias]
// Multi-line description providing context and rationale.
// Additional details about current behavior, impact, or solution approach.
```

**Key formatting rules:**
- First line: Marker keyword followed by colon, then brief imperative title
- Second line: Metadata fields as `key=value` pairs, space-separated
- Subsequent lines: Detailed description (no metadata, just explanation)
- All lines must start with `//` (C++ line comment style)

### Supported Markers

Use the appropriate marker based on the task type:

- **TODO:** Future improvements, missing features, enhancements, or refactoring opportunities
  - Example: Adding new functionality, improving performance, enhancing UX

- **FIXME:** Known issues that need fixing, incorrect behavior requiring correction
  - Example: Logic errors, edge cases not handled, suboptimal implementations

- **BUG:** Active bugs, crashes, segfaults, or critical defects
  - Example: Segmentation faults, memory leaks, race conditions

- **HACK:** Temporary workarounds or shortcuts needing proper refactoring
  - Example: Quick fixes, technical debt, code that bypasses proper patterns

### Required Metadata Fields

Every TODO comment **must** include these fields on the second line:

1. **`category=X`** - Module or area of codebase (see Categories below)
2. **`estimate=Xh`** - Time estimate using standard suffixes:
   - `30m`, `1h`, `2h`, `4h` - Hours (use 'm' for minutes, 'h' for hours)
   - `1d`, `2d` - Days
   - `1w`, `2w` - Weeks

**Example:**
```cpp
// TODO: Add input validation for mesh generation parameters
// category=CLI estimate=30m
// Currently accepts any values which can lead to crashes with extreme inputs.
```

### Optional Metadata Fields

- **`issue=N`** - GitHub issue number (usually auto-assigned by tdg GitHub Action)
  - Don't add manually unless linking to existing issue
  - Example: `issue=123`

- **`author=alias`** - Creator's alias (use for handoffs; git blame is preferred otherwise)
  - Example: `author=pfils`

### Time Estimate Guidelines

Choose estimates based on complexity and scope:

| Estimate | Complexity | Description | Example |
|----------|-----------|-------------|---------|
| `30m-1h` | TRIVIAL | Quick config change, single-line fix, simple parameter addition | Adding a CLI flag, fixing typo logic |
| `1h-4h` | EASY | Single-session task, isolated change, well-understood problem | Input validation, error message improvement |
| `1d-2d` | MEDIUM | Multi-session work, moderate complexity, some research needed | Refactoring a module, adding medium feature |
| `3d-1w` | HARD | Significant refactor, architectural changes, complex debugging | Redesigning subsystem, fixing deep bugs |
| `>1w` | VERY_HARD | Major feature, fundamental redesign, multi-component changes | New backend support, SLAM integration |

**Estimation tips:**
- Consider implementation + testing + documentation time
- Account for unknowns and edge cases
- When uncertain, estimate higher (easier to complete early than explain delays)

### Categories

Map your TODO to the appropriate module:

| Category | Scope | Examples |
|----------|-------|----------|
| `CLI` | Command-line interface, argument parsing, rux subcommands | Argument validation, help text, new subcommand |
| `I/O` | File I/O, database access, format conversions | RTABMap database, E57 import, Rhino export, HDF5 |
| `Geometry` | Point cloud processing, segmentation, mesh generation | Plane detection, room segmentation, CGAL algorithms |
| `Vision` | ML models, inference backends, semantic annotation | YOLO integration, TensorRT optimization, SAM2 |
| `Visualization` | PCL visualization, on-screen display, rendering | Point cloud viewer, debug overlays, Qt widgets |
| `Documentation` | Code comments, Doxygen, guides, examples | API docs, tutorials, inline comments |

**Category selection tips:**
- Choose the primary module affected (even if change touches multiple areas)
- Use `CLI` for user-facing interface changes
- Use module name for internal implementation changes
- When truly cross-cutting, pick the most impacted area

### Best Practices

**Writing good titles:**
- Use imperative mood: "Add validation" not "Validation needed"
- Be specific: "Validate mesh vertex count" not "Fix validation"
- Keep under 70 characters
- Focus on the *what*, not the *why* (save that for description)

**Writing good descriptions:**
- **Line 1**: Describe current behavior or problem
- **Line 2**: Explain why change is needed (impact, rationale)
- **Line 3+**: Outline solution approach or steps
- **Include references**: Related TODOs, docs, issue numbers

**Example structure:**
```cpp
// TODO: Add comprehensive input size validation with detailed error messages
// category=CLI estimate=30m
// Current validation only checks a subset of input files. Should validate all:
// 1. Check cloud, rooms, normals, plane_labels all have same size
// 2. Verify plane_normals and plane_centroids have expected dimensions
// 3. Provide specific error message showing actual vs expected sizes
// 4. Add early validation before heavy processing to fail fast
```

### Real Examples from Codebase

**Simple TODO (mesh.cpp:160-166):**
```cpp
// TODO: Add comprehensive input size validation with detailed error messages
// category=CLI estimate=30m
// Current validation only checks a subset of input files. Should validate all:
// 1. Check cloud, rooms, normals, plane_labels all have same size
// 2. Verify plane_normals and plane_centroids have expected dimensions
// 3. Provide specific error message showing actual vs expected sizes
// 4. Add early validation before heavy processing to fail fast
```

**Critical BUG (rtabmap.cpp:516-523):**
```cpp
// BUG: Segfault during KdTree initialization with empty/small clouds
// category=I/O estimate=1d
// Occurs when setInputCloud() is called on an empty or very small point cloud
// after voxel downsampling. Need to add validation:
// 1. Check if cloud->empty() before KdTree operations
// 2. Check if cloud->size() < minimum threshold (e.g., 10 points)
// 3. Handle gracefully by returning nullptr labels or logging error
// Reproduction: Small scans with aggressive voxel grid settings
```

**FIXME with cross-reference (rooms.cpp:160-168):**
```cpp
// FIXME: Label encoding offset may cause indexing errors for unlabeled points
// category=Geometry estimate=2h
// Current code uses label-1 as index into plane_normals vector (line 169).
// If unlabeled points (label < 1) exist and are not skipped properly, this
// could cause off-by-one indexing errors. Need to verify that:
// 1. All unlabeled points have label < 1 (currently checking this)
// 2. Plane labels start at 1 (not 0) consistently throughout pipeline
// 3. plane_normals vector size matches max(label) not unique label count
// See MEMORY.md for label encoding conventions: -1 for background in API
```

### Workflow Integration

**GitHub Action (`.github/workflows/todo-action.yml`):**
- Automatically runs on push to scan for TODO comments
- Generates `TODO.json` with all parsed comments
- Can create/update GitHub issues based on TODOs
- Tracks completion when TODOs are removed from code

**Local development:**
```bash
# Find all TODOs in the codebase
grep -r "// TODO:" libs/ apps/ --include="*.cpp" --include="*.hpp"

# Find TODOs by category
grep -r "category=Geometry" libs/ apps/ --include="*.cpp" --include="*.hpp"

# Find high-estimate TODOs (1d+)
grep -r "estimate=[0-9]*[dw]" libs/ apps/ --include="*.cpp" --include="*.hpp"
```

**Using tdg locally:**
Install tdg from https://gitlab.com/ribtoks/tdg and run:
```bash
tdg --path libs/ --path apps/ --output TODO.json
```

### When to Use Each Marker

**Use TODO when:**
- Planning future enhancements or features
- Identifying optimization opportunities
- Noting missing functionality in initial implementations
- Suggesting refactoring that isn't urgent

**Use FIXME when:**
- Code works but has known limitations
- Edge cases aren't fully handled
- Implementation is suboptimal but functional
- Workarounds are in place

**Use BUG when:**
- Code crashes or produces incorrect results
- Memory leaks or resource leaks are present
- Race conditions or concurrency issues exist
- Critical functionality is broken

**Use HACK when:**
- Using temporary workarounds to unblock development
- Bypassing proper abstractions for quick fixes
- Duplicating code that should be refactored
- Violating design patterns knowingly

### See Also

- Full tdg documentation: https://gitlab.com/ribtoks/tdg
- GitHub Action config: `.github/workflows/todo-action.yml`
- Existing TODOs: Run `grep -r "// TODO:" libs/ apps/`

## Key Dependencies

**Geometry & Processing:**
- PCL (Point Cloud Library) - point cloud operations
- CGAL - computational geometry algorithms
- Eigen3 - linear algebra
- Embree - ray tracing

**Deep Learning:**
- PyTorch (LibTorch) 2.9.0+ - neural network inference
- TensorRT - optimized GPU inference

**Graph Processing:**
- GraphBLAS, LAGraph - sparse graph algorithms for room segmentation

**Optimization:**
- HiGHS - Mixed Integer Programming (MIP) solver
  - ✅ Built with CUDA/GPU support (`CUPDLP_GPU=ON` in `overlays/highs.nix`)
  - ⚠️  GPU acceleration (PDLP solver) not used by Solidifier class
  - **Reason**: Solidifier uses MIP with binary variables; PDLP only works for continuous LP
  - See `docs/HIGHS_GPU_ACCELERATION.md` for details and alternatives
- SCIP solver - constraint optimization (alternative to HiGHS)
- TBB - parallel processing

**I/O:**
- RTABMap - SLAM database access
- E57Format - point cloud exchange format
- OpenNURBS - Rhino 3D (.3dm) files
- OpenCV - image processing and computer vision

**Utilities:**
- CLI11 - command-line parsing
- spdlog - fast logging
- fmt - string formatting
- range-v3 - modern C++ ranges

## Pre-trained Models

Models should be placed in the project root or specified via CLI:
- **YOLO11**: `yolo11n.pt`, `yolo11l.pt`, `yolo11x.pt` (object detection)
- **YOLO11-seg**: `yolo11n-seg.pt`, `yolo11l-seg.pt` (segmentation)
- **SAM2**: `sam2.1_s.pt`, `sam2.1_b.pt`, `sam2_hiera_large.pt`
- **SuperPoint**: `superpoint.pt` (feature detection)
- **TensorRT engines**: `.engine` files for optimized inference

## Common Tasks

### Import and process a scan

```bash
# Import RTABMap database
rux import rtabmap path/to/database.db

# Segment planes
rux segment planes <options>

# Segment rooms
rux segment rooms <options>

# Generate mesh
rux mesh <options>

# View results
rux view <options>
```

### Run semantic annotation

```bash
# Annotate with YOLO
rux annotate --model yolo11l.pt --input scan.db

# View annotated results
rux view --labels <options>
```

### Debug with verbosity

```bash
# Use -v, -vv, or -vvv for increasing detail
rux -vvv segment planes <options>
```

## Python Integration

**Status:** Python bindings are currently DISABLED and being refactored.

The `python/` directory contains standalone Python utilities for:
- Pose graph manipulation (`python/ReUseX/pose_graph/`)
- PyTorch model interfaces (`python/ReUseX/torch/`)

These are NOT integrated with the C++ library yet. Do not attempt to build with `-DBUILD_PYTHON_BINDINGS=ON`.

## License and Copyright

- License: GPL-3.0-or-later
- Copyright: 2025 Povl Filip Sonne-Frederiksen
- Follows REUSE specification for license compliance
- All source files must have SPDX headers
