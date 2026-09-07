<!-- SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen SPDX-License-Identifier: GPL-3.0-or-later
-->

# ReUseX

⚠️ **Warning: This project is in active development and will regularly be introducing breaking changes**

![](assets/banner.png)

ReUseX is a comprehensive tool for processing 3D point cloud scans of building interiors, designed to support building reuse and renovation projects. The project processes LiDAR scans to create semantic 3D models with advanced segmentation capabilities for architectural elements.

## Features

- **Point Cloud Processing**: Import sensor frames from RTABMap SLAM databases, MuSHRoom captures, E57/PLY clouds, 360° panoramas and survey photos
- **Pose Refinement**: Plane-landmark pose-graph optimization and joint pairwise registration (GTSAM)
- **Planar Segmentation**: Extract and segment planar surfaces (walls, floors, ceilings) via noise-adaptive region growing
- **Room Segmentation**: Automatically partition point clouds into individual rooms with Leiden community detection over the plane graph (igraph)
- **Semantic Segmentation**: Deep learning-based identification of architectural elements using YOLO and SAM3 models
- **3D Reconstruction**: Cell complex representations solidified into simplified 3D surface models via a MIP solve (HiGHS on CPU, cuOpt on GPU)
- **Mesh Generation**: Generate textured 3D meshes from segmented point clouds; dense MVS clouds via OpenMVS
- **Multiple I/O Formats**: E57, PLY, OpenNURBS (.3dm), COLMAP, Speckle, CSV, MaterialEPAS
- **GPU Acceleration**: CUDA-accelerated processing with TensorRT / LibTorch / ONNX Runtime for neural network inference

## Architecture

The project consists of:
- **ReUseX library** (`libs/reusex/`): one CMake target per module
  (`reusex_core`, `reusex_segmentation`, `reusex_reconstruction`, `reusex_slam`,
  `reusex_io`, `reusex_vision`, …) with a link-enforced layer graph
- **rux CLI** (`apps/rux/`): command-line interface with subcommands for the
  whole pipeline
- **ruxd** (`apps/ruxd/`): HTTP service worker

For details see [ARCHITECTURE.md](ARCHITECTURE.md), the engineering standards in
[docs/STANDARDS.md](docs/STANDARDS.md), and the pipeline-stage data contracts in
[docs/CONTRACTS.md](docs/CONTRACTS.md).

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
- `-DWITH_CUDA=ON/OFF` - CUDA / NVIDIA GPU support; also gates the TensorRT backend and the cuOpt solver (default: ON)
- `-DBUILD_TESTS=ON/OFF` - Enable/disable unit tests (default: ON)
- `-DBUILD_DOCUMENTATION=ON/OFF` - Enable/disable documentation generation (default: ON)
- `-DBUILD_PYTHON_BINDINGS=ON/OFF` - Build the pybind11 Python bindings (default: ON)
- `-DGUI_ENABLED=ON/OFF` - Enable/disable CGAL Qt6 GUI features (default: OFF)
- `-DUSE_CCACHE=ON/OFF` - Use ccache when available (default: ON)
- `-DENABLE_COVERAGE=ON/OFF` - Code coverage instrumentation (default: OFF)
- `-DML_BACKENDS=AUTO|NONE|<list>` - Which ML backends to enable, e.g. `-DML_BACKENDS="TensorRT;LibTorch"` (default: `AUTO`)

**Note on Visualization:** the `visualize` module is built automatically when
`libs/reusex/src/visualize/` has sources; there is no `BUILD_VISUALIZATION`
switch. Use `-DGUI_ENABLED=OFF` (the default) to skip CGAL's Qt6 GUI
components.

### Building API Documentation

Generate comprehensive API documentation with Doxygen:

```shell
# Configure the build with documentation enabled (default)
cmake -B build -DBUILD_DOCUMENTATION=ON

# Generate the documentation (target is named `docs`)
cmake --build build --target docs

# View the documentation
xdg-open docs/api/html/index.html  # Linux
open docs/api/html/index.html      # macOS
```

**Requirements:** Doxygen and optionally Graphviz (for diagrams)

The documentation will be generated in `docs/api/`, covering:
- Complete API reference for all C++ classes and functions
- Module and namespace organization
- Class hierarchies and collaboration diagrams
- Source code browsing

### Running Tests

See [docs/guides/TESTING.md](docs/guides/TESTING.md) and
[tests/README.md](tests/README.md) for detailed testing documentation.

Quick start:
```shell
# Build and run C++ tests
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure
```

The suite (Catch2 v3) is organized as `tests/unit/<module>/`,
`tests/integration/`, `tests/benchmarks/`, with shared helpers in
`tests/support/` and data in `tests/fixtures/`.

## Usage

### Command-Line Interface

The `rux` executable provides several subcommands for a complete point cloud processing pipeline:

All commands operate on a single `.rux` project database, selected with the
global `-p/--project` flag (default `./project.rux`).

```shell
# Show version, license and help
rux --version
rux --license
rux --help

# Verbosity control (use -v, -vv, or -vvv for increasing detail)
rux -vv <subcommand>

# Import scan data from various sources
rux import rtabmap <path>              # RTABMap SLAM database
rux import mushroom <path>             # MuSHRoom RGB-D benchmark capture
rux import e57|ply <path>              # Point cloud files
rux import 360 <path>                  # 360° panoramic images
rux import photos <path>               # Manual survey photos
rux import csv|materialepas <path>     # Element / material passport data

# Refine the stored per-frame sensor poses
rux optimize                           # Plane-landmark pose graph
rux register                           # Joint pairwise registration

# Create derived data products (all creation operations)
rux create clouds                      # Back-project depth frames into a cloud
rux create dense                       # Dense cloud via OpenMVS MVS
rux create annotate -n <model>         # ML inference on sensor frames
rux create project                     # Project 2D labels onto the 3D cloud
rux create planes                      # Detect and segment planar surfaces
rux create rooms                       # Segment into rooms (Leiden clustering)
rux create instances                   # Split labels into spatial instances
rux create mesh                        # Watertight mesh from planes
rux create texture                     # Apply textures to mesh
rux create windows                     # Window building components
rux create material|materials          # Material passports

# Edit stored clouds
rux edit downsample                    # Voxel-grid downsample

# Inspect and validate
rux info                               # Project database summary
rux log                                # Pipeline execution history
rux validate [--stage <name>] [--json] # Referential integrity / stage inputs
rux analyze quality|accuracy           # Reconstruction quality metrics (JSON)

# Path-based database access
rux get <path> | rux set <path> <value> | rux del <path>

# Export results in various formats
rux export ply|e57|rhino|colmap|speckle|csv|materialepas|semantic-images

# Visualize point clouds and results
rux view

# Assemble multiple scans into unified model
rux assemble <paths...> -o <out.rux>
```

Run `rux <command> --help` for the full flag list; the pipeline-stage
prerequisites are documented in [docs/CONTRACTS.md](docs/CONTRACTS.md).

`ruxd` is a separate HTTP service worker binary (`ruxd --help`).

> **Note:** the Python bindings in `bindings/python/` are built by default
> (`BUILD_PYTHON_BINDINGS=ON`) and currently expose **read-only** `.rux`
> inspection (`reusex.ProjectDB` and the summary value types).

## Dependencies

The project relies on an extensive set of libraries. The authoritative lists are
the `find_package` calls in `libs/reusex/cmake/Dependencies.cmake` and the
`buildInputs` in `default.nix`.

**Core Libraries:**
- PCL (Point Cloud Library)
- Eigen3 - Linear algebra
- CGAL - Computational geometry
- OpenCV - Computer vision
- Boost (incl. Boost.Graph), TBB - System utilities and parallelism
- Qt6 - GUI components

**Deep Learning:**
- LibTorch, ONNX Runtime, TensorRT (CUDA-only) - inference backends
- RTABMap - SLAM database import

**Optimization:**
- HiGHS - MIP solver (CPU path for the cell-complex solve)
- cuOpt - optional NVIDIA GPU MIP backend
- GTSAM - factor-graph pose optimization
- Embree - Ray tracing
- igraph - Graph algorithms (Leiden room clustering)
- OpenMVS (+ nanoflann, jsoncpp) - Dense multi-view stereo

**I/O Formats:**
- SQLite3 - the `.rux` project database
- E57Format - Point cloud exchange
- OpenNURBS - Rhino 3D modeling
- exiv2 - Photo EXIF metadata
- CURL / OpenSSL / nlohmann_json - Speckle and HTTP transport

**Development Tools:**
- CMake (3.17+) with C++20 support
- CLI11 - Command-line parsing
- spdlog - Fast logging sink for the CLI apps
- fmt - String formatting
- range-v3 - Modern C++ ranges
- Catch2 v3 - Unit and integration tests

## Pre-trained Models

Models are not distributed with the repository. Pass a path with
`rux create annotate -n/--net <path>` — either a single model file or a
directory of sub-models. The model family and inference backend are detected
from the path (`libs/reusex/include/vision/BackendFactory.hpp`):

- **SAM3 / SAM2** (Segment Anything): any path whose name contains `sam3` or
  `sam2`, or a directory containing a `vision-encoder.*` file
- **YOLO**: anything else, e.g. `yolo11l.pt` or `yolo11l-seg.pt`
- Backend by extension/layout: `.engine` → TensorRT, `.onnx` → ONNX Runtime,
  `.pt` → LibTorch

## License

This project is licensed under the GNU General Public License v3.0 or later - see the [LICENSE.md](LICENSE.md) file for details.

## Contributing

Contributions are welcome! This project follows the REUSE specification for license compliance.

- [CONTRIBUTING.md](CONTRIBUTING.md) - workflow
- [CONTRIBUTING_AI.md](CONTRIBUTING_AI.md) - guidance for AI coding assistants
- [CLAUDE.md](CLAUDE.md) - naming conventions, build/CLI orientation, TODO format
- [docs/STANDARDS.md](docs/STANDARDS.md) - the objective bar every change must
  meet (module boundaries, label contract, Definition of Done)

## Roadmap

- [ ] Extend the Python bindings beyond read-only project inspection
- [ ] Enhanced texture mapping capabilities
- [ ] Support for additional scan sources beyond RTABMap and MuSHRoom
- [ ] Improved visualization tools
- [ ] BIM model export (IFC format)
- [ ] Real-time processing pipeline

## Author

**Povl Filip Sonne-Frederiksen**  
Link Arkitektur  
Email: pfs@linkarkitektur.dk

## Acknowledgments

This project builds upon numerous open-source libraries and tools from the computer vision, geometry processing, and deep learning communities.
