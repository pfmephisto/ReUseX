# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Direction

Project heading, priorities, and active workstreams live in
[`docs/DIRECTION.md`](docs/DIRECTION.md). Read it before planning, reviewing, or
filing issues. When work changes the project's direction, update it in the same
PR with a dated changelog entry.

## Agent Model Selection

The main session model is for **orchestration, guidance, review, and
synthesis only** — delegate substantial work to subagents, and always set
the `model:` parameter explicitly (never let an agent inherit the expensive
session default):

- **`model: "sonnet"`** — simple/mechanical execution: downloads, running
  scripts/benchmarks/test suites, file conversions, data collection,
  formatting fixes, report extraction.
- **`model: "opus"`** — reasoning-heavy work: debugging, root-cause
  analysis, design, code review, non-trivial implementation.

Pick by task complexity; when unsure whether a task needs judgment,
prefer sonnet for gathering and opus for deciding.

## Normative Documents

This file is the orientation/guidance layer. The **normative** rules live in:

- [`docs/STANDARDS.md`](docs/STANDARDS.md) — module boundaries (§1), header
  hygiene (§2), label & identity contract (§3), parameters (§4), error handling
  (§5), determinism (§6), testing (§7), performance (§8), Definition of Done (§9).
- [`docs/CONTRACTS.md`](docs/CONTRACTS.md) — what each pipeline stage consumes
  and produces in a `.rux` project database, enforced by `rux validate --stage`.

If this file and those two disagree, **they win** — and the disagreement is a bug
worth fixing here.

## Project Overview

ReUseX is a C++20/CUDA project for processing 3D point cloud scans of building interiors. It combines geometric processing, deep learning, and computational geometry to create semantic 3D models for building reuse and renovation projects.

**Key capabilities:**
- Sensor-frame import from RTABMap SLAM databases, MuSHRoom and ARKitScenes
  captures, E57/PLY clouds, 360° panoramas and survey photos
- Pose refinement: plane-landmark pose-graph optimization (GTSAM) and joint
  pairwise registration
- Planar segmentation (noise-adaptive region growing) and room segmentation
  (Leiden clustering over a plane graph, via igraph)
- Semantic segmentation via YOLO / SAM3 models (TensorRT, ONNX Runtime, LibTorch),
  including SAM3 on 360° panoramas via perspective tiling
- Cell complex 3D reconstruction with a MIP solve (HiGHS CPU / cuOpt GPU)
- Mesh generation with texture mapping; dense MVS clouds via OpenMVS
- Export to PLY, E57, OpenNURBS (.3dm), COLMAP, Speckle, CSV, MaterialEPAS

## Naming Conventions

- **Functions/methods**: snake_case — `save_point_cloud()`, `is_open()`
- **Getters**: No `get_` prefix — `path()`, `schema_version()`, `mesh()`
- **Boolean queries**: `is_`/`has_` prefix — `is_open()`, `has_mesh()`
- **Setters**: `set_` prefix — `set_log_level()`, `set_database_path()`
- **Classes/structs**: PascalCase — `CellComplex`, `ProjectDB`, `DetectionBox`
- **Enum classes**: PascalCase name, snake_case values — `Stage::mesh_generation`
- **Namespaces**: snake_case, root namespace is `reusex` (lowercase — `ReUseX::`
  appears nowhere in the source) — `reusex::core`, `reusex::segmentation`,
  `reusex::vision::tensor_rt`
- **Type aliases**: PascalCase — `CloudPtr`, `CloudNConstPtr`
- **Member variables**: snake_case with trailing `_` — `impl_`, `cloud_`
- **File naming**: PascalCase for class files, snake_case for function/algorithm files
- **Header guards**: `#pragma once`

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

# CPU-only build (no CUDA, no TensorRT)
cmake -B build -DWITH_CUDA=OFF
cmake --build build

# Build without tests
cmake -B build -DBUILD_TESTS=OFF
cmake --build build
```

**Build options that actually exist** (grep `option(` in `CMakeLists.txt`,
`cmake/`, `libs/reusex/cmake/`):

| Option | Default | Defined in | Effect |
|---|---|---|---|
| `WITH_CUDA` | `ON` | `CMakeLists.txt:25` | CUDA / NVIDIA GPU support. Also gates the TensorRT backend search and `cuOpt`. |
| `USE_CCACHE` | `ON` | `CMakeLists.txt:66` | Use ccache when available |
| `ENABLE_COVERAGE` | `OFF` | `CMakeLists.txt:103` | Code coverage instrumentation |
| `BUILD_PYTHON_BINDINGS` | `ON` | `CMakeLists.txt:129` | Adds `bindings/python` |
| `BUILD_TESTS` | `ON` | `CMakeLists.txt:137` | Adds `tests/` and `enable_testing()` |
| `BUILD_DOCUMENTATION` | `ON` | `CMakeLists.txt:146`, `cmake/Documentation.cmake:9` | Defines the `docs` target |
| `GUI_ENABLED` | `OFF` | `libs/reusex/cmake/Dependencies.cmake:207` | CGAL Qt6 GUI components |
| `ML_BACKENDS` | `AUTO` | `libs/reusex/cmake/Dependencies.cmake:49` | Cache string, not a bool: `AUTO`, `NONE`, or a list like `TensorRT;LibTorch;ONNX;OpenVINO` |
| `LIN_ENABLE_ASAN` / `MSAN` / `UBSAN` / `TSAN` | `OFF` | `libs/reusex/cmake/CompilerOptions.cmake` | Sanitizers |
| `LIN_ENABLE_WERROR` | `OFF` | `libs/reusex/cmake/CompilerOptions.cmake:23` | `-Werror` |

There is **no** `BUILD_VISUALIZATION` option. The `visualize` module is built
whenever `libs/reusex/src/visualize/**` has sources (see
`libs/reusex/cmake/reusexLibrary.cmake`), and the umbrella `reusex` target picks
it up only `if(TARGET reusex_visualize)`.

**CMake auto-detection:** The build system uses `GLOB_RECURSE` with `CONFIGURE_DEPENDS`, so new .cpp/.cu files are automatically detected. No need to manually update CMakeLists.txt when adding source files.

### Testing

```bash
# Build and run all tests
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest --output-on-failure --parallel $(nproc)

# Run with verbose output
ctest --verbose

# Run specific test
ctest -R test_name_pattern
```

**Always pass `--parallel`.** Nearly the entire suite's wall time is
per-process dynamic-loader overhead, not test work, so it scales almost
linearly with cores: serial is ~12 min, `-j8` about 1/6 of that.
`scripts/check.sh` already does this. Parallel ctest used to be flaky
because temp-file helpers derived names from object addresses; that was
fixed in #262 by `tests/support/temp_path.hpp`, which every test must use
for temp paths.

Tests live in `tests/`: `unit/` (per-module: `core`, `geometry`, `io`, `ruxd`,
`utils`, `vision`, `visualize`), `integration/`, `benchmarks/`, `support/`,
`fixtures/`. Catch2 v3.

Unit tests link into **two** executables (`tests/CMakeLists.txt`):
`reusex_unit_tests` for most modules, and `reusex_unit_tests_vision` for
`unit/vision`, `unit/ruxd` and `unit/visualize`, which need libtorch /
TensorRT / `ruxd_lib` / the PCL-Qt viewer. Test names and `ctest -R` are
unaffected by the split. Put a new test in a heavy directory only if it
really needs those dependencies (#268).

Benchmarks: `scripts/bench.sh` produces an XML report and
`scripts/bench-compare.py` diffs a baseline against a candidate, failing on a
regression beyond `--threshold` percent. The required before/after workflow is
[`docs/STANDARDS.md` §8.1](docs/STANDARDS.md#81-baseline-vs-candidate-workflow);
`scripts/bench-arkitscenes.sh` and `scripts/bench-mushroom.sh` drive
dataset-level runs.

### Documentation

The Doxygen target is named `docs` (not `doc`) and writes to `docs/api`
(`OUTPUT_DIRECTORY` in `docs/Doxyfile`).

```bash
# Generate Doxygen documentation
cmake -B build -DBUILD_DOCUMENTATION=ON
cmake --build build --target docs

# View generated docs
xdg-open docs/api/html/index.html
```

## Architecture

### Project Structure

```
ReUseX/
├── libs/reusex/                    # The library (one CMake target per module)
│   ├── include/                    # Public headers; consumers use <reusex/...>
│   │   ├── core/                   # ProjectDB, logging, stages, validate,
│   │   │                           #   MaterialPassport, guid, label_semantics
│   │   ├── geometry/               # geometry_common (Layer 1½); #222 forwarding
│   │   │                           #   shims removed in #248
│   │   ├── segmentation/           # planes, rooms, instances, reconstruct,
│   │   │                           #   depth_filters, downsample, surfels
│   │   ├── reconstruction/         # CellComplex, Solidifier, mesh, texture,
│   │   │                           #   regularization, SceneGraph, Registry
│   │   ├── slam/                   # PlaneGraphOptimizer, JointPairwiseRegistration
│   │   ├── io/                     # rtabmap, e57, ply, rhino, colmap, speckle,
│   │   │                           #   mushroom, exif, export_scene
│   │   ├── vision/                 # ML models, backends, datasets (tensor_rt/, onnx/,
│   │   │                           #   libtorch/, osd/, common/)
│   │   ├── visualize/              # PCL/Qt visualization (optional)
│   │   ├── utils/                  # math, cv, tolerances, fmt_formatter
│   │   ├── types/                  # point_types.hpp, eigen_types.hpp
│   │   └── types.hpp               # Umbrella re-exporting types/*
│   ├── src/                        # Implementation, mirrors include/
│   ├── cmake/                      # reusexLibrary.cmake, Dependencies.cmake, ...
│   └── extern/                     # Vendored headers
├── apps/rux/                       # CLI application
│   ├── include/ + src/             # Subcommands, grouped in subdirs
│   └── cmake/RuxExecutable.cmake
├── apps/ruxd/                      # HTTP service worker (ruxd)
├── apps/blender/reusex_panel/      # Blender add-on
├── bindings/python/                # pybind11 bindings (read-only ProjectDB access)
├── python/                         # reusex_sam3: SAM 3.1 -> ONNX -> TensorRT export
├── models/                         # Model weights (gitignored; see models/README.md)
├── tests/                          # unit/ integration/ benchmarks/ support/ fixtures/
├── docs/                           # STANDARDS.md, CONTRACTS.md, guides/, design/, research/
├── cmake/                          # Shared CMake utilities
├── overlays/ pkgs/ devshells/      # Nix packaging
└── tools/ scripts/ completions/    # Dev tooling (bench.sh, bench-compare.py, check.sh)
```

Three distinct Python trees, easy to confuse:
`bindings/python/` (C++ bindings, built by CMake), `python/` (the standalone
`reusex_sam3` model-export pipeline, run in its own venv — **not** built by
CMake), and `apps/blender/reusex_panel/` (Blender add-on).

### Module targets

`libs/reusex/` builds **one static library per module**, named
`reusex_<module>`, wired in `libs/reusex/cmake/reusexLibrary.cmake`. The layer
graph is **link-enforced** — an illegal dependency is a link error, not a
convention. See [`docs/STANDARDS.md` §1](docs/STANDARDS.md#1-module-boundaries)
for the authoritative diagram and the documented exceptions.

Targets: `reusex_utils`, `reusex_geometry_common`, `reusex_core`, `reusex_io`,
`reusex_vision`, `reusex_segmentation`, `reusex_reconstruction`, `reusex_slam`,
`reusex_visualize` (conditional), plus two interface targets
(`reusex_common` = public deps, `reusex_private_deps`) and the umbrella
INTERFACE target `reusex` that links everything for backward compatibility.

The old `ReUseX` / `ReUseX_visualization` target names no longer exist.

**Executables:** `rux` (`apps/rux`), `ruxd` (`apps/ruxd`, HTTP service worker).
Both use CLI11 for argument parsing and spdlog as the log sink.

### Type System (types.hpp)

Aliases are split so a TU pulls in only what it needs (STANDARDS §2):
`types/point_types.hpp` (PCL) and `types/eigen_types.hpp` (Eigen).
`types.hpp` is a backward-compatible umbrella over both — prefer the narrower
header in new code.

- `PointT` = `pcl::PointXYZRGB` - Point cloud points with color
- `NormalT` = `pcl::Normal` - Surface normals
- `LabelT` = `pcl::Label` - Segmentation labels
- `Cloud` / `CloudPtr` - Point cloud containers
- `CloudN` / `CloudL` / `CloudLoc` - Normals, labels, locations

### Segmentation / Reconstruction / SLAM modules

The former single `geometry` module was split by pipeline stage in #222.
`include/geometry/*.hpp` held one-line forwarding shims (e.g.
`geometry/CellComplex.hpp` including `reusex/reconstruction/CellComplex.hpp`)
so consumers didn't need to move at the same time; #248 migrated every
consumer to the new `<reusex/{segmentation,reconstruction,slam}/...>` paths
and deleted the shims. `include/geometry/` now holds only the real
`geometry_common` headers (next section) — there is nothing left to forward.

**segmentation** (`include/segmentation/`):
- `reconstruct.hpp`: pinhole back-projection of depth frames into clouds
- `segment_planes.hpp`: planar detection (noise-adaptive region growing;
  `noise_estimate.hpp` supplies the adaptive thresholds)
- `segment_rooms.hpp`: room partitioning via Leiden clustering (igraph)
- `segment_instances.hpp` / `reconcile_instances.hpp`: semantic → spatial instances
- `depth_filters.hpp`, `downsample.hpp`, `sync_downsample.hpp`, `densify.hpp`
- `Surfel.hpp` / `surfel_extraction.hpp`: surfels shared with `slam`

**reconstruction** (`include/reconstruction/`):
- `CellComplex.hpp`: Boost.Graph-based 3D spatial representation
  (Cell / Face / Vertex nodes); recenters to its bbox centroid internally to
  keep CGAL's inexact kernel usable on georeferenced input (STANDARDS §4)
- `Solidifier.hpp`: cell selection via MIP (HiGHS CPU / cuOpt GPU) → watertight mesh
- `mesh.hpp`, `texture_mesh.hpp`, `regularization.hpp`, `create_windows.hpp`
- `SceneGraph.hpp` / `Registry.hpp`: spatial relationships and object hierarchies
- `quality_metrics.hpp` / `accuracy_metrics.hpp`: backing `rux analyze`

**slam** (`include/slam/`):
- `PlaneGraphOptimizer.hpp`: plane-landmark pose graph (GTSAM), `rux optimize`.
  Also hosts the optional wide-baseline loop-closure front-end (ORB + depth →
  RANSAC), enabled with `--loop-closure`; run `rux optimize --help` for the
  current flag list rather than trusting a doc.
- `JointPairwiseRegistration.hpp`: `rux register`
- `PanoramaAlignment.hpp`: content-based 360 pose refinement, `rux align 360`

**geometry_common** (`include/geometry/`, layer 1½): the real (non-shim) headers
there — `utils.hpp`, `cgal_utils.hpp`, `transform_utils.hpp`,
`CoplanarPolygon.hpp`, `BuildingComponent.hpp`, `unweld.hpp`,
`EquirectProjection.hpp` — are the CGAL/PCL/OpenCV primitives shared by the
peers above and by `core`. (`EquirectProjection` sits here because both `slam`
and `vision` need equirect↔perspective reprojection and peers may not link each
other.) Sources are listed **explicitly** (not globbed) in
`reusexLibrary.cmake`, so a new `src/geometry/*.cpp` **does** need a CMake edit;
every other module is globbed.

### Vision/ML Module

**Backend abstraction** (`vision/IMLBackend.hpp`, `vision/BackendFactory.hpp`):
- Pluggable backend system for ML inference. Which backends are compiled in is
  decided at configure time by `ML_BACKENDS` (`AUTO` probes for
  TensorRT / LibTorch / ONNX / OpenVINO; TensorRT is skipped unless
  `WITH_CUDA=ON`). See `libs/reusex/cmake/MLBackendConfig.cmake` — it also
  *excludes* the source files of disabled backends from the glob.
- Implemented today: `vision/tensor_rt/` and `vision/onnx/` (Yolo + Sam3),
  `vision/libtorch/` (dataset side).
- `BackendFactory::detect_model()` sniffs the path — `sam3`/`sam2` in the name,
  or a directory containing `vision-encoder.*`, means `Model::sam3`; otherwise
  `Model::yolo`. `BackendFactory::detect_backend()` picks the backend from the
  extension / directory contents.
- **Construct models via `reusex::vision::create_model_from_path()`**
  (`vision/model_factory.hpp`), not by reaching for a backend class directly:
  the `REUSEX_USE_*` backend defines are attached to `reusex_vision` by
  `configure_ml_backends()`, so they are not visible from the app layer.
- `enum class Backend { opencv, tensor_rt, libtorch, dnn, onnx_runtime,
  openvino, unknown }`.

**Model interfaces:**
- `IModel`: Base interface for all ML models
- `Yolo`: YOLO object detection/segmentation
- `Sam3`: SAM3 segmentation (`tensor_rt/Sam3.hpp`, `onnx/Sam3.hpp`)
- Models loaded from filesystem paths, backend auto-detected

**Dataset interfaces:**
- `IDataset`: Base dataset interface
- `LibTorchDataset` (`vision/libtorch/Dataset.hpp`): PyTorch-compatible dataset
- `TensorRTDataset` (`vision/tensor_rt/Dataset.hpp`): TensorRT-optimized dataset
- `ONNXSam3Dataset` (`vision/onnx/Sam3Dataset.hpp`): SAM3 under ONNX Runtime
- Datasets read frames from `ProjectDB`, not from an RTABMap database

**Key vision components:**
- `annotate.hpp`: Semantic annotation pipeline (`rux create annotate`)
- `project.hpp`: 2D-label → 3D-cloud projection (`rux create project`)
- `Dataloader.hpp`: Batch data loading
- `osd/`: On-screen display for visualization

### Core Module — ProjectDB

`core/ProjectDB.hpp` is the single project store (`*.rux`, sqlite3). It
**replaced** the old `RTABMapDatabase`, which no longer exists in the source
tree — if a doc mentions `RTABMapDatabase`, that doc is stale.

- Pimpl idiom keeps sqlite3 out of the public header; `cv::Mat` and the PCL mesh
  types are forward-declared, not included (STANDARDS §2)
- Stores point clouds (chunked), meshes + texture blobs, sensor frames
  (color/depth/confidence/pose/intrinsics), panoramic images,
  `segmentation_images`, building components, material passports,
  instance↔material links, and the pipeline log
- Migrating schema; `LATEST_SCHEMA_VERSION` is defined in
  `src/core/ProjectDB.cpp` — read it there rather than trusting a doc (it moves
  most releases)
- **NOT thread-safe** (sqlite3): create a per-thread instance if needed
- **No image rotation.** Images and labels are stored in their original
  orientation; the 90°-clockwise rotation the old RTABMap reader applied is gone
- Label encoding: `CV_16U` + 1 offset in storage (0 = unlabeled), `CV_32S` with
  `-1` in the API. The full contract, including the in-memory `CloudL`
  convention, is [`docs/STANDARDS.md` §3](docs/STANDARDS.md#3-label--identity-contract);
  helpers live in `core/label_semantics.hpp`

Other core pieces: `logging.hpp`, `stages.hpp` (`Stage` enum),
`validate.hpp` (`check_stage_inputs`, backing `rux validate --stage`),
`MaterialPassport.hpp` + `materialepas_*`, `guid.hpp`, `SensorIntrinsics.hpp`,
`processing_observer.hpp` / `visual_observer.hpp`, `filter_expression.hpp`.

### I/O Module

External-format adapters only — project state itself lives in `core/ProjectDB`.

- `rtabmap.hpp`: RTABMap SLAM database import (the only RTABMap consumer)
- `mushroom.hpp`: MuSHRoom RGB-D benchmark captures
- `e57.hpp`, `ply.hpp`: point cloud exchange
- `rhino.hpp`: OpenNURBS (.3dm) import/export
- `colmap.hpp`: COLMAP sparse model export
- `speckle.hpp`: Speckle export
- `exif.hpp`: photo metadata (exiv2)
- `export_scene.hpp`: serializes a reconstructed scene
- `reusex.hpp`: legacy custom format

There is no HDF5 dependency anywhere in the build.

### CLI Subcommands (apps/rux/src/)

Top-level commands, as registered in `apps/rux/src/rux.cpp`:

| Command | Sub-commands | Source |
|---|---|---|
| `import` | `rtabmap`, `mushroom`, `arkitscenes`, `e57`, `ply`, `materialepas`, `csv`, `360`, `photos` | `src/import/` |
| `create` | `clouds`, `dense`, `annotate`, `annotate-360`, `material`, `project`, `planes`, `rooms`, `instances`, `materials`, `mesh`, `texture`, `windows` | `src/create/` |
| `export` | `ply`, `e57`, `materialepas`, `csv`, `rhino`, `semantic-images`, `speckle`, `colmap` | `src/export/` |
| `align` | `360` (content-based panorama pose refinement) | `src/align/` |
| `edit` | `downsample` | `src/edit/` |
| `analyze` | `quality`, `accuracy` | `src/analyze/` |
| `optimize` | — (plane-landmark pose graph) | `src/optimize.cpp` |
| `register` | — (joint pairwise registration) | `src/register.cpp` |
| `validate` | — (`--stage`, `--json`) | `src/validate.cpp` |
| `get` / `set` / `del` | path-based DB access (`src/database/*_router.cpp`) | `src/get.cpp`, `src/set.cpp`, `src/del.cpp` |
| `info` | — (project summary) | `src/info.cpp` |
| `log` | — (pipeline execution history) | `src/log.cpp` |
| `view` | — (viewer) | `src/view/` |
| `assemble` | — (multi-scan assembly) | `src/assemble.cpp` |

`create`, `import`, `export`, `edit`, `analyze`, `align` all
`require_subcommand(1)`.
Global flags: `-v/-vv/-vvv`, `-V/--version`, `-L/--license`, `-D/--visualize`,
`-p/--project <path.rux>` (defaults to `./project.rux`).

`ruxd` (`apps/ruxd/`) is a separate HTTP service worker binary with its own
flags (`--port`, `--threads`, `--pg-url`, `--redis-url`, `--s3-*`,
`--auth-token`).

## Development Patterns

### Adding New Source Files

**Usually no CMakeLists.txt update is required.** `reusexLibrary.cmake` globs
per module with `CONFIGURE_DEPENDS`:
- C++ sources: `libs/reusex/src/<module>/**/*.cpp` — where `<module>` is
  `utils`, `core`, `io`, `vision`, `segmentation`, `reconstruction`, `slam`,
  `visualize`
- CUDA sources: `libs/reusex/src/vision/**/*.cu`
- Headers: `libs/reusex/include/**/*.hpp`
- Tests: `tests/unit/**/*.cpp`

**Exception:** `src/geometry/` (the `geometry_common` module) uses an explicit
source list, so adding a `.cpp` there needs a `reusexLibrary.cmake` edit.

Put the file in the module that matches its pipeline stage, and check
[`STANDARDS.md` §1](docs/STANDARDS.md#1-module-boundaries) first — a
cross-peer include will fail at link time.

### Creating New ML Backends

1. Inherit from `IMLBackend` (`vision/IMLBackend.hpp`)
2. Implement the model/dataset creation entry points
3. Teach `BackendFactory::detect_backend()` about the new extension/layout
4. Add the value to `reusex::vision::Backend`
5. Register the backend's packages and source-exclusion rules in
   `libs/reusex/cmake/Dependencies.cmake` (`ML_BACKEND_<name>_PACKAGES`) and
   `libs/reusex/cmake/MLBackendConfig.cmake`

### Adding New CLI Subcommands

1. Create `apps/rux/src/<group>/<name>.cpp` with a
   `setup_subcommand_<group>_<name>(CLI::App &parent, std::shared_ptr<RuxOptions>)`
2. Call it from the group's `setup_subcommand_<group>()` (e.g. `src/create.cpp`),
   or from `apps/rux/src/rux.cpp` for a new top-level command
3. Keep it thin: parse, validate, call one library entry point, report
   (STANDARDS §1). Templates: `src/create/planes.cpp`, `src/export/ply.cpp`
4. CLI flag defaults must mirror the library options struct, never redefine it
   (STANDARDS §4)

### Working with Point Clouds

- Always use the type aliases (PointT, Cloud, CloudPtr) — include
  `reusex/types/point_types.hpp` in new code, not the `types.hpp` umbrella
- Point clouds use `pcl::PointXYZRGB` (XYZ + RGB color)
- Use `pcl::Indices` for index vectors, not `std::vector<int>`
- `cloud` / `normals` / `planes` / `rooms` / `instances` / `labels` for one scan
  are index-aligned and must be filtered/reordered together — see
  [`docs/CONTRACTS.md`](docs/CONTRACTS.md) and STANDARDS §3.2
- Labels: `0` means unlabeled in a `CloudL`; never index with `label - 1`
  without checking `label >= 1`

### Logging

**Library code** uses the ReUseX logging API in
`libs/reusex/include/core/logging.hpp`:
- **Syntax**: `reusex::debug()`, `reusex::info()`, `reusex::warn()`,
  `reusex::error()`, `reusex::trace()`, `reusex::critical()`
- **Levels**: `reusex::core::LogLevel` — `trace, debug, info, warn, error,
  critical, off`
- **Formatting**: `fmt` compile-time format strings — `reusex::debug("Value: {}", x)`.
  Each function also has a plain `std::string_view` overload.
- **Filtering**: `should_log()` is checked before formatting, so a suppressed
  message costs nothing
- **Timing**: `reusex::stopwatch` (a `spdlog::stopwatch` replacement,
  `elapsed()` → seconds as `double`; has an `fmt` formatter)

**CLI applications** (`apps/rux`, `apps/ruxd`) use `spdlog` directly:
- Syntax: `spdlog::debug()`, `spdlog::info()`, etc.
- `rux` installs spdlog as the library's log handler and keeps both levels in
  sync from `-v` (see `apps/rux/src/rux.cpp`)

**Implementation details**:
- Logging functions are defined in namespace `reusex::core`
- Explicit `using` declarations promote them into `reusex` for convenience
  (`critical, debug, error, info, log, LogLevel, stopwatch, trace, warn`)
- This avoids namespace conflicts with external libraries (e.g., CGAL's `debug` parameter)
- Handler configured via `reusex::core::set_log_handler()` (defaults to no-op in
  the library); `reset_log_handler()`, `set_log_level()`, `get_log_level()`
  are only reachable as `reusex::core::…`, not via the promoted names

### Error Handling

- Throw `std::runtime_error` for recoverable errors with descriptive messages
- Use RAII for resource management (smart pointers, custom destructors)
- Validate inputs early and fail fast with clear error messages
- **No silent failure**: a stage that produces empty/degenerate output must log
  at `warn` or above *with the reason and the numbers*. Full rules:
  [`docs/STANDARDS.md` §5](docs/STANDARDS.md#5-error-handling--diagnostics)

### RTABMap API Gotchas

RTABMap is now confined to `libs/reusex/src/io/rtabmap.cpp` (import only).

- `rtabmap::Rtabmap::init()` returns void (use try-catch for errors)
- `std::multimap` is in `<map>` header, not `<multimap>`
- Frames read out of RTABMap keep their original orientation — do **not**
  reintroduce the old 90° rotation on the way into `ProjectDB`

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
| `I/O` | File I/O, database access, format conversions | ProjectDB schema, RTABMap import, E57 import, Rhino/Speckle export |
| `Geometry` | Point cloud processing, segmentation, reconstruction, SLAM, mesh generation — the `geometry_common` / `segmentation` / `reconstruction` / `slam` modules | Plane detection, room segmentation, CGAL algorithms, pose-graph optimization |
| `Vision` | ML models, inference backends, semantic annotation | YOLO integration, TensorRT optimization, SAM3 |
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

**Simple TODO with a reference (`libs/reusex/include/vision/nms.hpp:22`):**
```cpp
// TODO: Replace custom NMS with torchvision library implementation
// category=Vision estimate=4h
// Current implementation is custom-written. Consider using official torchvision
// NMS: Reference:
// https://github.com/pytorch/vision/blob/main/torchvision/csrc/ops/cpu/nms_kernel.cpp
```

**Large TODO with an `issue=` link (`libs/reusex/src/core/ProjectDB.cpp:13`):**
```cpp
// TODO: Remove core -> geometry dependency in BuildingComponent persistence
// category=I/O estimate=1d issue=222
// ProjectDB (Layer 2, core) persists geometry::BuildingComponent and calls
// geometry::CoplanarPolygon (de)serialization — a Layer-3 type. This is a
// documented layering exception enforced via an explicit reusex_core ->
// ...
```

**HACK documenting a workaround (`libs/reusex/src/io/speckle.cpp:883`):**
```cpp
// HACK: Wrap material InstanceProxies in a "Cameras" sub-collection and
// rename each one "Camera N" to match the legacy Grasshopper layout.
// category=I/O estimate=2h
// The reuse-x webapp's loadImages composable hardcodes
//     speckleRoot.elements.find(c => c.name === 'Cameras')
// so without this wrapper our material tags never get discovered.
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

Authoritative sources: `find_package` calls in
`libs/reusex/cmake/Dependencies.cmake`, and the `buildInputs` in `default.nix`.
Anything not found there is not a dependency.

**Geometry & Processing:**
- PCL (Point Cloud Library) - point cloud operations
- CGAL (`Core`, optionally `Qt6` under `GUI_ENABLED`) - computational geometry
- Eigen3 - linear algebra
- Embree - ray tracing
- OpenMVS (+ nanoflann, jsoncpp, Boost iostreams/program_options/serialization)
  - dense multi-view stereo for `rux create dense`
- TBB - parallel processing
- Boost (incl. Boost.Graph for `CellComplex`)

**Deep Learning:**
- LibTorch - neural network inference
- ONNX Runtime - portable inference
- TensorRT (via `trtsam3` + `tokenizers_cpp`) - optimized GPU inference; only
  searched for when `WITH_CUDA=ON`
- Protobuf / absl / utf8_range - pulled in by the above

**Graph Processing:**
- igraph - Leiden community detection for room segmentation
  (there is **no** GraphBLAS or LAGraph in this build)

**Optimization:**
- HiGHS - Mixed Integer Programming (MIP) solver, the CPU path for `Solidifier`
  - CUDA/GPU support is **disabled** (`CUPDLP_GPU=OFF` in `overlays/highs.nix`)
  - Reason: `Solidifier` solves a MIP with binary variables; HiGHS's GPU PDLP
    solver only handles continuous LP, so it would never be used
- cuOpt - optional NVIDIA GPU MIP backend (`find_package(cuOpt)` under
  `WITH_CUDA`); select with `rux create mesh --solver auto|cuopt|highs`
  (`auto` falls back to HiGHS on GPU error/OOM)
- GTSAM - factor-graph optimization for `slam` (pose-graph / registration)
- There is **no** SCIP dependency (`pkgs/cuOpt/package.nix` only references
  the `scipopt` GitHub org as an upstream source)

**I/O:**
- SQLite3 - the `.rux` project database backing `ProjectDB`
- RTABMap - SLAM database import
- E57Format - point cloud exchange format
- OpenNURBS - Rhino 3D (.3dm) files
- OpenCV (`core`, `imgproc`, `highgui`) - image processing
- exiv2 - photo EXIF metadata
- CURL + OpenSSL + nlohmann_json - Speckle / HTTP transport
- No HDF5

**Utilities:**
- CLI11 - command-line parsing
- spdlog - logging sink used by the `rux`/`ruxd` apps
- fmt - string formatting (the library's logging API is built on it)
- range-v3 - modern C++ ranges
- nlohmann_json - JSON
- Qt6 (`Core Widgets Gui OpenGL`) - GUI components
- Catch2 v3 - tests

## Pre-trained Models

Weights are **never committed**. `models/` is gitignored except for its README
— [`models/README.md`](models/README.md) is the reference for the expected
layout and where each file comes from.

Pass a path with `rux create annotate -n/--net`; it accepts either a single file
or a directory of sub-models. `BackendFactory::detect_model()` decides YOLO vs
SAM3 from the path:

- **SAM3/SAM2** — any path whose stem contains `sam3` or `sam2`, or a directory
  containing a `vision-encoder.*` file
- **YOLO** — everything else (e.g. `yolo11l.pt`, `yolo11l-seg.pt`)
- **TensorRT engines**: `.engine` files (or a directory of them) for optimized
  inference; `.onnx` selects the ONNX Runtime backend, `.pt` LibTorch

**SAM 3.1 engines** are produced by the `reusex_sam3` pipeline in `python/`
(eight engines + `tokenizer.json` + `tracker-meta.json` in one directory):
- [`python/README.md`](python/README.md) — run order / quickstart
- [`docs/sam3.1-tensorrt.md`](docs/sam3.1-tensorrt.md) — full write-up of the
  export, the engine-I/O contract, and how the C++ tracker consumes the engines
- [`docs/sam3.1-export-guide.md`](docs/sam3.1-export-guide.md) — export guide

## Common Tasks

### Import and process a scan

Every command reads/writes one `.rux` project; select it with the global
`-p/--project` flag (default `./project.rux`). The named clouds and tables each
stage consumes and produces are specified in
[`docs/CONTRACTS.md`](docs/CONTRACTS.md).

```bash
# 1. Import raw sensor frames from an RTABMap SLAM database
rux -p scan.rux import rtabmap path/to/database.db

# 2. (optional) Refine the stored per-frame poses
rux -p scan.rux optimize                   # plane-landmark pose graph
rux -p scan.rux register                   # joint pairwise registration

# 3. Back-project depth frames into a fused cloud (-g = voxel size in m)
rux -p scan.rux create clouds -g 0.05

# 4. Geometry pipeline
rux -p scan.rux create planes
rux -p scan.rux create rooms
rux -p scan.rux create mesh --solver auto

# Check a stage's inputs before running it
rux -p scan.rux validate --stage mesh

# Inspect / view
rux -p scan.rux info
rux -p scan.rux log
rux -p scan.rux view
```

### Run semantic annotation

```bash
# ML inference over the stored sensor frames
rux -p scan.rux create annotate -n models/sam3 --cuda

# Project the 2D labels onto the 3D cloud, then split into instances
rux -p scan.rux create project
rux -p scan.rux create instances
```

### Debug with verbosity

```bash
# Use -v, -vv, or -vvv for increasing detail (max 3)
rux -vvv -p scan.rux create planes
```

## Python Integration

Python bindings live in `bindings/python/` (pybind11 + scikit-build-core,
package name `reusex`). `BUILD_PYTHON_BINDINGS` defaults to **ON** in
`CMakeLists.txt:129`, so they are part of a default build — they are **not**
disabled.

Current scope is **read-only `.rux` inspection**, implemented in
`bindings/python/src/bindings.cpp` (~490 lines): `ProjectDB` plus the
`ProjectSummary` / `ProjectInfo` / `CloudInfo` / `MeshInfo` /
`SensorFrameInfo` / `PanoramicInfo` / `ComponentInfo` / `MaterialInfo` /
`PipelineLogEntry` value types. The native module is `reusex._reusex`;
`reusex/__init__.py` re-exports it and degrades to `__status__ = "Native module
not available: …"` if it is missing.

Writing/pipeline APIs are not exposed.

The top-level `python/` directory is **unrelated** to these bindings: it is the
standalone `reusex_sam3` package that exports Meta's SAM 3.1 checkpoint to ONNX
and then to TensorRT engines. It is not built by CMake — run it in its own venv
with prebuilt wheels (see [`python/README.md`](python/README.md); do not
nix-compile torch). `apps/blender/reusex_panel/` is a separate, standalone
Blender add-on.

## License and Copyright

- License: GPL-3.0-or-later
- Copyright: 2025 Povl Filip Sonne-Frederiksen
- Follows REUSE specification for license compliance
- All source files must have SPDX headers
