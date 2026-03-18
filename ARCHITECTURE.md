# ReUseX Architecture Overview

This document provides a quick reference to the ReUseX repository structure and architecture.

For detailed architectural documentation, see `docs/design/`.

## Repository Structure

```
ReUseX/
├── libs/                       # Library subprojects
│   └── reusex/                # Core C++ library
│       ├── include/ReUseX/    # Public API headers
│       ├── src/ReUseX/        # Implementation
│       └── cmake/             # Library-specific CMake
├── apps/                      # Application subprojects
│   └── rux/                   # CLI tool
│       ├── include/rux/       # CLI headers
│       └── src/rux/           # CLI implementation
├── bindings/                  # Language bindings
│   └── python/                # Python bindings (future)
├── tests/                     # Test suite
│   ├── unit/                  # Unit tests by module
│   ├── integration/           # Integration tests
│   └── fixtures/              # Test data
├── docs/                      # Documentation
│   ├── api/                   # Doxygen output
│   ├── guides/                # User guides
│   └── design/                # Architecture docs
├── cmake/                     # Shared CMake utilities
└── tools/                     # Development tools
```

## Core Modules

### libs/reusex - Core Library

Six main modules:

- **core**: Fundamental types, logging, constants
- **geometry**: Point cloud processing, CGAL/PCL integration
- **io**: Database access (RTABMapDatabase), file I/O
- **utils**: Math, string utilities, helpers
- **vision**: Deep learning (YOLO, SAM), datasets, TensorRT
- **visualize**: GUI components (optional)

### apps/rux - CLI Tool

Command-line interface for:
- Database import and management
- Segmentation and inference
- Visualization (if enabled)

### bindings/python - Python Bindings

Status: Not yet implemented (see `bindings/python/README.md`)

## Key Design Patterns

### Subproject Isolation
Following LLVM/Boost conventions:
- Each component has own CMakeLists.txt
- Clear dependency graph: apps/bindings → libs
- Independent build configuration

See: `docs/design/subproject-structure.md`

### Pimpl Idiom (RTABMapDatabase)
Hide implementation details:
- Public headers don't expose RTABMap
- Faster compilation
- ABI stability

See: `docs/design/database-design.md`

### Dataset Composition
Different patterns for different needs:
- IDataset: Lightweight wrapper (delegates to RTABMapDatabase)
- TorchDataset: Composition (PyTorch interface requirements)
- TensorRTDataset: Inheritance (extends IDataset)

## Data Flow

```
RTABMapDatabase (Storage)
    │
    ├─→ IDataset (Interface)
    │       └─→ TensorRTDataset → Inference
    │
    └─→ TorchDataset → Training
```

## Build System

### Quick Start
```bash
cmake -B build
cmake --build build
```

### Key Options
- `BUILD_VISUALIZATION`: Enable GUI (default: ON)
- `BUILD_TESTS`: Build test suite (default: ON)
- `BUILD_PYTHON_BINDINGS`: Build Python bindings (default: OFF)
- `ENABLE_COVERAGE`: Code coverage (default: OFF)

### Subproject Targets
- `ReUseX`: Core library
- `ReUseX_visualization`: Visualization library
- `rux`: CLI executable
- `reusex_unit_tests`: Unit test executable

## Version Management

### Single Source of Truth
Version is defined **once** in the root `CMakeLists.txt`:

```cmake
project(ReUseX VERSION 0.0.1 ...)
```

All subprojects and bindings inherit this version automatically:
- **C++ library**: Uses `PROJECT_VERSION` in CMake
- **CLI app**: Inherits from parent project
- **Python bindings**: CMake generates `_version.py` from template

### Version Propagation

**C++ (libs/reusex)**:
- `include/ReUseX/core/version.hpp.in` template
- CMake substitutes `@PROJECT_VERSION@`
- Accessible via `ReUseX::version::VERSION`

**Python (bindings/python)**:
- `_version.py.in` template configured by CMake
- Generates `reusex/_version.py`
- Imported in `__init__.py` as `__version__`
- `pyproject.toml` uses `dynamic = ["version"]`

### Package Structure

**Python bindings** live in `bindings/python/` with their own `pyproject.toml`:
- Package name: `reusex` (lowercase, Python convention)
- Built via: `pip install bindings/python/`
- scikit-build-core bridges Python packaging with CMake
- Treats Python as optional binding to C++ library (not primary interface)

## Conventions

### Label Encoding
- **Storage**: CV_16U with +1 offset (0 = unlabeled)
- **API**: CV_32S with -1 for unlabeled
- **Rotation**: 90° clockwise on read

### Image Rotation
All images and labels rotated 90° clockwise from RTABMap storage

### Namespaces
- Maximum 3 levels: `ReUseX::vision::object`
- No redundant layers (no `common`)
- Internal utilities: `detail` namespace

## Documentation

- **User Guides**: `docs/guides/`
- **Design Docs**: `docs/design/`
- **API Reference**: `docs/api/` (Doxygen)
- **Contributing**: `CONTRIBUTING.md`

## Getting Started

1. **Using the library**: See `docs/guides/getting-started.md`
2. **Contributing**: See `CONTRIBUTING.md`
3. **Testing**: See `tests/README.md`
4. **Architecture deep dive**: See `docs/design/architecture.md`

## Questions?

- **Build issues**: Check `docs/design/subproject-structure.md`
- **Database questions**: See `docs/design/database-design.md`
- **General architecture**: See `docs/design/architecture.md`
- **Testing**: See `tests/README.md`
- **Report issues**: https://github.com/anthropics/ReUseX/issues
