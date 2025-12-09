<!--
SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX

ReUseX is a comprehensive tool for processing 3D point cloud scans of building interiors, designed to support building reuse and renovation projects. The project processes LiDAR scans to create semantic 3D models with advanced segmentation capabilities for architectural elements.

## Features

- **Point Cloud Processing**: Import and process 3D point cloud data from RTABMap SLAM databases
- **Planar Segmentation**: Extract and segment planar surfaces (walls, floors, ceilings) using advanced geometric algorithms
- **Room Segmentation**: Automatically partition point clouds into individual rooms using graph-based methods (GraphBLAS/LAGraph)
- **Semantic Segmentation**: Deep learning-based identification of architectural elements using YOLO and SAM2 models
- **3D Reconstruction**: Create cell complex representations and simplified 3D surface models
- **Mesh Generation**: Generate textured 3D meshes from segmented point clouds
- **Multiple I/O Formats**: Support for E57, PCD, OpenNURBS (.3dm), and HDF5 formats
- **GPU Acceleration**: CUDA-accelerated processing with PyTorch for neural network inference

## Architecture

The project consists of:
- **ReUseX Library**: C++ library with core point cloud processing and geometric algorithms
- **rux CLI**: Command-line interface with subcommands for various operations

## Getting Started

### Prerequisites

- **Nix with Flakes** (recommended): For reproducible builds with all dependencies
- **CUDA-capable GPU** (optional): For GPU-accelerated deep learning inference
- **C++20 compatible compiler**: GCC 10+ or Clang 12+
- **CMake 3.17+**

This project uses Nix flakes for reproducible dependency management. Install [Nix](https://nixos.org/download.html) with flakes enabled for the easiest setup.

### Development Shell

Launch a development environment with all dependencies:

```shell
nix develop
```

### Build Instructions

#### Using Nix (Recommended)

```shell
git clone https://github.com/pfmephisto/ReUseX
cd ReUseX
nix build
```

#### Using CMake

```shell
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

**Build Options:**
- `-DBUILD_VISUALIZATION=ON/OFF` - Enable/disable visualization library (default: ON)
- `-DBUILD_TESTS=ON/OFF` - Enable/disable unit tests (default: ON)
- `-DBUILD_DOCUMENTATION=ON/OFF` - Enable/disable documentation generation (default: ON)
- `-DGUI_ENABLED=ON/OFF` - Enable/disable CGAL GUI features (default: OFF)

**Note on Visualization:** The visualization functionality (including `rux view` and `rux mesh` commands) can be optionally disabled by setting `-DBUILD_VISUALIZATION=OFF`. This reduces dependencies and build time if you only need the core processing functionality.

### Building API Documentation

Generate comprehensive API documentation with Doxygen:

```shell
# Configure the build with documentation enabled (default)
cmake -B build -DBUILD_DOCUMENTATION=ON

# Generate the documentation
cmake --build build --target doc

# View the documentation
xdg-open doc/html/index.html  # Linux
open doc/html/index.html       # macOS
```

**Requirements:** Doxygen and optionally Graphviz (for diagrams)

The documentation will be generated in the `doc/` folder, covering:
- Complete API reference for all C++ classes and functions
- Module and namespace organization
- Class hierarchies and collaboration diagrams
- Source code browsing

### Running Tests

ReUseX has comprehensive C++ unit test coverage. See [TESTING.md](TESTING.md) for detailed testing documentation.

Quick start:
```shell
# Run all tests
./run_tests.sh

# Or run C++ tests directly (after building)
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

Test statistics:
- **C++ tests** for math utilities, geometry functions, and type operations
- All tests passing ✓

## Usage

### Command-Line Interface

The `rux` executable provides several subcommands for a complete point cloud processing pipeline:

```shell
# Show version and help
rux --version
rux --help

# Verbosity control (use -v, -vv, or -vvv for increasing detail)
rux -vv <subcommand>

# Import scan data from various sources
rux import rtabmap <path>      # Import from RTABMap database

# Segment point cloud into components
rux segment planes <options>   # Detect and segment planar surfaces
rux segment rooms <options>    # Segment into rooms

# Generate 3D mesh from point cloud
rux mesh <options>

# Apply textures to mesh
rux texture <options>

# Export results in various formats
rux export <options>

# Visualize point clouds and results
rux view <options>

# Annotate point clouds with semantic information
rux annotate <options>

# Assemble multiple scans into unified model
rux assemble <options>
```

> **Note:** Python bindings are currently disabled and being refactored. They will be reintroduced in a future release.

## Dependencies

The project relies on an extensive set of libraries (see `flake.nix` for complete list):

**Core Libraries:**
- PCL (Point Cloud Library)
- Eigen3 - Linear algebra
- CGAL - Computational geometry
- OpenCV - Computer vision
- Boost, TBB - System utilities
- Qt6 - GUI components

**Deep Learning:**
- PyTorch (LibTorch) 2.9.0+ with CUDA support
- RTABMap - SLAM and 3D mapping

**Optimization:**
- SCIP solver
- Embree - Ray tracing
- GraphBLAS, LAGraph - Graph algorithms

**I/O Formats:**
- E57Format - Point cloud exchange
- OpenNURBS - 3D modeling
- HDF5 - Data storage

**Development Tools:**
- CMake (3.17+) with C++20 support
- CLI11 - Command-line parsing
- spdlog - Fast logging
- fmt - String formatting
- range-v3 - Modern C++ ranges

## Pre-trained Models

The project supports various pre-trained PyTorch models for semantic segmentation and feature detection:

- **YOLO11**: Object detection for architectural elements
  - `yolo11n.pt` - Nano variant (fastest)
  - `yolo11l.pt` - Large variant (balanced)
  - `yolo11x.pt` - Extra-large variant (most accurate)
  - Segmentation variants: `yolo11n-seg.pt`, `yolo11l-seg.pt`
- **SAM2** (Segment Anything Model 2): Universal image segmentation
  - `sam2.1_s.pt` - Small variant
  - `sam2.1_b.pt` - Base variant
  - `sam2_hiera_large.pt` - Large hierarchical variant
- **SuperPoint** (`superpoint.pt`): Feature point detection and description
- **YOLOv8**: Legacy support for older YOLO models

Models should be placed in the project root directory or specified via command-line arguments.

## License

This project is licensed under the GNU General Public License v3.0 or later - see the [LICENSE.md](LICENSE.md) file for details.

## Contributing

Contributions are welcome! This project follows the REUSE specification for license compliance.

## Roadmap

- [ ] Re-enable Python bindings with updated API
- [ ] Enhanced texture mapping capabilities
- [ ] Support for additional scan sources beyond RTABMap
- [ ] Improved visualization tools
- [ ] BIM model export (IFC format)
- [ ] Real-time processing pipeline

## Author

**Povl Filip Sonne-Frederiksen**  
Link Arkitektur  
Email: pfs@linkarkitektur.dk

## Acknowledgments

This project builds upon numerous open-source libraries and tools from the computer vision, geometry processing, and deep learning communities.
