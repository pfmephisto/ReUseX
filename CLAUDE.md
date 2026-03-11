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
- SCIP solver - constraint optimization
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
