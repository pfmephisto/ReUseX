# ReUseX Architecture

This document provides an architectural overview of the ReUseX library and its components.

## High-Level Overview

ReUseX is a C++20 library for processing 3D point cloud data from interior lidar scans of buildings. It combines SLAM (RTABMap), deep learning (YOLO, SAM), and point cloud processing (CGAL, PCL) to enable automated semantic segmentation and object detection for building reuse applications.

## Module Organization

The library is organized into six main modules:

```
ReUseX/
├── core/           # Core types and logging
├── geometry/       # Point cloud and geometric operations
├── io/             # Database and file I/O
├── utils/          # Utility functions
├── vision/         # Deep learning and computer vision
└── visualize/      # GUI components (optional)
```

### core - Core Functionality

**Purpose**: Fundamental types and utilities used throughout the library

**Key Components**:
- `types.hpp`: Core type definitions
- Logging infrastructure (spdlog integration)
- Common constants and enumerations

**Dependencies**: Minimal (spdlog, fmt)

### geometry - Point Cloud Processing

**Purpose**: 3D geometric operations and point cloud manipulation

**Key Components**:
- Point cloud utilities
- Segmentation algorithms
- Geometric transformations
- CGAL and PCL integration

**Dependencies**: CGAL, PCL, Eigen

### io - Input/Output

**Purpose**: Database access and file I/O operations

**Key Components**:
- **RTABMapDatabase**: Unified wrapper for RTABMap database + segmentation labels
- SQLite integration via RTABMap
- Image and label persistence

**Architecture** (RTABMapDatabase):
- Pimpl idiom to hide RTABMap dependencies
- Automatic image rotation (90° clockwise on read)
- Label encoding: CV_32S API (-1 for background), CV_16U storage (+1 offset)
- Cached node IDs for performance

**Dependencies**: RTABMap, SQLite, OpenCV

See `docs/design/database-design.md` for detailed RTABMapDatabase architecture.

### utils - Utilities

**Purpose**: General-purpose utility functions

**Key Components**:
- Math utilities
- String processing
- File system helpers
- Timing and profiling

**Dependencies**: Standard library, fmt

### vision - Computer Vision

**Purpose**: Deep learning models and computer vision operations

**Key Submodules**:

#### Dataset API
- `IDataset`: Lightweight interface for dataset access
- `TorchDataset`: PyTorch-compatible dataset for training
- `TensorRTDataset`: TensorRT-optimized dataset for inference

**Architecture**:
- IDataset delegates to RTABMapDatabase via shared_ptr
- TorchDataset uses RTABMapDatabase via composition (PyTorch interface compatibility)
- TensorRTDataset extends IDataset

#### Object Detection (`vision/object`)
- Detection primitives: DetectionBox, Mask, Keypoint
- YOLO integration for object detection

#### Segmentation Models
- SAM (Segment Anything Model) integration via TensorRT
- YOLO-based segmentation

#### TensorRT Backend (`vision/tensor_rt`)
- TensorRT model loading and inference
- CUDA-accelerated preprocessing
- Memory management utilities (in `detail` namespace)

#### On-Screen Display (`vision/osd`)
- Visualization of detection results
- Bounding box and label rendering

**Dependencies**: OpenCV, PyTorch (optional), TensorRT, CUDA

### visualize - Visualization (Optional)

**Purpose**: GUI components for interactive visualization

**Key Components**:
- 3D point cloud viewer
- Image display widgets
- Annotation tools

**Build Option**: `BUILD_VISUALIZATION=ON/OFF`

**Dependencies**: VTK, Qt (or similar GUI framework)

## Design Patterns

### Pimpl Idiom (RTABMapDatabase)

Used to hide RTABMap dependencies from public headers:

```cpp
// Header: Clean interface
class RTABMapDatabase {
public:
    cv::Mat getImage(int nodeId);
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

// Implementation: RTABMap details
class RTABMapDatabase::Impl {
    rtabmap::Rtabmap rtabmap_;
    // ...
};
```

**Benefits**:
- Faster compilation (no RTABMap headers in public API)
- ABI stability
- Clear separation of interface and implementation

### Composition over Inheritance (Dataset)

TorchDataset uses composition instead of extending IDataset:

```cpp
class TorchDataset : public torch::data::Dataset<TorchDataset> {
    std::shared_ptr<RTABMapDatabase> db_;  // Composition
    // PyTorch interface methods
};
```

**Rationale**: PyTorch requires specific inheritance; can't also extend IDataset

### Resource Management

- Smart pointers for ownership (unique_ptr, shared_ptr)
- RAII for resource cleanup
- Explicit database lifetime management

## Data Flow

### Training Pipeline

```
RTABMapDatabase → TorchDataset → PyTorch Training
     │                │
     │                ├─ getImage(nodeId)
     │                ├─ getLabels(nodeId)
     │                └─ Image augmentation
     │
     └─ Stores: Images (rotated) + Labels (CV_16U)
```

### Inference Pipeline

```
RTABMapDatabase → IDataset → TensorRTDataset → TensorRT Inference
     │              │              │
     │              │              ├─ Preprocessor (CUDA)
     │              │              └─ TensorRT Engine
     │              │
     │              └─ Provides: Image + Label access
     │
     └─ Optional: Store predictions via saveLabels()
```

### CLI Workflow (rux)

```
User Input → CLI Parser (CLI11) → Command Handlers
                                        │
                                        ├─ Import: DB creation
                                        └─ Segment: Run inference
                                             │
                                             ├─ Load model (TensorRT)
                                             ├─ Process dataset
                                             └─ Save results
```

## Build System Architecture

See `docs/design/subproject-structure.md` for detailed build system documentation.

**Key Points**:
- Subproject isolation: `libs/`, `apps/`, `bindings/`
- CMake orchestrator pattern
- Explicit dependencies between components

## Threading and Concurrency

- Dataset loading: Thread-safe via SQLite
- CUDA operations: Single stream per inference session
- PyTorch: Follows PyTorch DataLoader threading model
- Future: Parallel point cloud processing

## Memory Management

### CUDA Memory
- TensorRT manages GPU memory via engine
- CUDA preprocessing uses device memory pools
- Automatic cleanup on object destruction

### Host Memory
- OpenCV Mat for images (reference counting)
- Torch Tensors (reference counting)
- Point clouds: PCL smart pointers

## Error Handling

- Exceptions for unrecoverable errors (database open failure)
- Return values for expected failures (node not found)
- Logging for diagnostics (spdlog)
- CUDA error checking via macros

## Future Architecture Considerations

### Planned Improvements
1. Plugin system for custom segmentation algorithms
2. Parallel point cloud processing pipeline
3. Streaming dataset support for large databases
4. Python bindings (pybind11)

### Scalability
- Database caching strategies for large datasets
- Memory-mapped file support
- Distributed processing for multi-node deployments

## Related Documentation

- `docs/design/database-design.md`: RTABMapDatabase detailed design
- `docs/design/module-overview.md`: Per-module component details
- `docs/design/subproject-structure.md`: Build system and subprojects
- `CONTRIBUTING.md`: Code standards and development practices

## Questions?

For architectural questions:
1. Review this document and related design docs
2. Check module-specific documentation
3. Open an issue with the `architecture` label
