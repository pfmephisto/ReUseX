# Subproject Structure

This document explains the organization of ReUseX into separate subprojects following LLVM/Boost conventions.

## Overview

ReUseX is organized into three main categories:

```
ReUseX/
├── libs/        # Library subprojects
├── apps/        # Application subprojects
├── bindings/    # Language bindings
```

This structure provides:
- **Clear component boundaries**: Each subproject has its own scope
- **Independent configuration**: Separate CMakeLists.txt for each component
- **Explicit dependencies**: Apps and bindings depend on libs
- **Easier contribution**: Contributors can focus on specific components

## Subprojects

### libs/reusex - Core Library

**Location**: `libs/reusex/`

The core C++ library providing all ReUseX functionality.

**Structure**:
```
libs/reusex/
├── CMakeLists.txt           # Library build configuration
├── cmake/                   # Library-specific CMake modules
│   ├── CompilerOptions.cmake
│   ├── CUDAOptions.cmake
│   ├── Dependencies.cmake
│   └── ReUseXLibrary.cmake
├── include/ReUseX/          # Public API headers
│   ├── core/
│   ├── geometry/
│   ├── io/
│   ├── utils/
│   ├── vision/
│   └── visualize/
└── src/ReUseX/              # Implementation
```

**Build Configuration**:
- Defines `ReUseX` target (core library)
- Optionally defines `ReUseX_visualization` target
- Handles all dependencies (OpenCV, CUDA, RTABMap, etc.)
- Configures compiler and CUDA options

**Options**:
- `BUILD_VISUALIZATION`: Enable/disable visualization library (default: ON)

### apps/rux - CLI Tool

**Location**: `apps/rux/`

Command-line interface for ReUseX operations.

**Structure**:
```
apps/rux/
├── CMakeLists.txt           # CLI build configuration
├── cmake/                   # CLI-specific CMake (if needed)
├── include/rux/             # CLI-specific headers
└── src/rux/                 # CLI implementation
    ├── rux.cpp              # Main entry point
    ├── import/              # Import commands
    └── segment/             # Segmentation commands
```

**Dependencies**:
- ReUseX library (from libs/reusex)
- CLI11 (command-line parsing)
- spdlog (logging)

**Build Configuration**:
- Creates `rux` executable
- Links against `ReUseX` target
- Conditionally links `ReUseX_visualization` if available

### bindings/python - Python Bindings

**Location**: `bindings/python/`

Future Python bindings using pybind11.

**Structure**:
```
bindings/python/
├── CMakeLists.txt           # Python binding build (stub)
├── README.md                # Status and roadmap
├── src/                     # Future pybind11 code
└── reusex/                  # Future Python package
    └── __init__.py
```

**Status**: Not yet implemented

**Dependencies** (future):
- ReUseX library (from libs/reusex)
- pybind11
- Python 3.8+

## Root CMakeLists.txt - Orchestrator

The root `CMakeLists.txt` acts as an orchestrator that:
1. Sets global project settings
2. Adds each subproject with `add_subdirectory()`
3. Provides build options for optional components
4. Configures tests and documentation

**Simplified structure** (~100 lines vs. previous ~110 with inline logic):
```cmake
# Global settings
set(CMAKE_CXX_STANDARD 20)
# ...

# Add subprojects
add_subdirectory(libs/reusex)
add_subdirectory(apps/rux)

# Optional components
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(bindings/python)
endif()

# Tests
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
```

## Dependency Graph

```
┌─────────────────┐
│  libs/reusex    │  Core library
│  (ReUseX)       │
└────────┬────────┘
         │
         ├──────────────┐
         │              │
    ┌────▼────┐    ┌────▼────────────┐
    │apps/rux │    │bindings/python  │
    │  (CLI)  │    │  (future)       │
    └─────────┘    └─────────────────┘
```

- **apps/rux** depends on **libs/reusex**
- **bindings/python** (future) will depend on **libs/reusex**
- Subprojects are independent of each other (only depend on libs)

## Build System Benefits

### 1. Clear Separation of Concerns

Each subproject:
- Has its own `CMakeLists.txt` with focused scope
- Manages only its own source files
- Declares its dependencies explicitly

### 2. Independent Development

Contributors can:
- Build only the library: `cmake --build build --target ReUseX`
- Build only the CLI: `cmake --build build --target rux`
- Work on one component without rebuilding others

### 3. Flexible Configuration

Users can:
- Build without CLI if only library needed
- Skip Python bindings if not required
- Enable/disable visualization independently

### 4. Future Extensibility

Easy to add:
- Additional CLI tools in `apps/`
- Other language bindings in `bindings/` (Rust, Julia, etc.)
- Additional libraries in `libs/` (plugins, extensions)

## Build Examples

### Full build (all components)
```bash
cmake -B build
cmake --build build
```

### Library only (no CLI)
```bash
cmake -B build
cmake --build build --target ReUseX
```

### Library + Python bindings (future)
```bash
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build
```

### Development build with tests and coverage
```bash
cmake -B build -DBUILD_TESTS=ON -DENABLE_COVERAGE=ON
cmake --build build
```

## Migration Notes

### From Monolithic to Subproject

**Before** (monolithic):
- Single CMakeLists.txt with all logic inline
- All includes in `include/`
- All source in `src/`
- Hard to see component boundaries

**After** (subprojects):
- Root CMakeLists.txt orchestrates
- Each component has own CMakeLists.txt
- Clear directory structure: `libs/`, `apps/`, `bindings/`
- Explicit dependencies via `target_link_libraries()`

### Include Path Changes

**Code remains the same**:
```cpp
#include <ReUseX/io/RTABMapDatabase.hpp>  // Still works!
#include <rux/commands.hpp>                // Still works!
```

**CMake needs updates**:
- Tests now include from `${CMAKE_SOURCE_DIR}/libs/reusex/include`
- External projects linking ReUseX use exported targets (unchanged)

## Comparison with Other Projects

This structure follows patterns from:

- **LLVM**: `llvm/`, `clang/`, `lld/` as separate subprojects
- **Boost**: Each library in `libs/` directory
- **OpenCV**: Modules in `modules/`, apps in `apps/`
- **Qt**: Separate repos for qtbase, qtdeclarative, etc.

## Questions?

For questions about the subproject structure or build system, please:
1. Check this document and `ARCHITECTURE.md`
2. Review the CMakeLists.txt in each subproject
3. Open an issue with the `build-system` label
