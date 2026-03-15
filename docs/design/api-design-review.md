# ReUseX Library API/Architecture Design Review

## 1. Executive Summary

The public API is functional but architecturally uneven: naming and construction patterns vary across modules, third-party types leak heavily into headers, and high-level components (especially `vision`) are tightly coupled to concrete I/O and backend implementations. The strongest design work today is in `io::RTABMapDatabase` (RAII + Pimpl). The main risks are long-term API lock-in to PCL/OpenCV/PyTorch/TensorRT and avoidable compile-time/deployment coupling. The recommendations below focus on minimal, incremental changes that improve coherence without rewriting the library.

## 2. API Consistency Issues

| Location | Problem | Recommended fix (+ sketch) |
|---|---|---|
| `libs/reusex/include/ReUseX/vision/IDataset.hpp` (`IDataset`) vs `.../vision/libtorch/Dataset.hpp` (`TorchDataset`) vs `.../vision/tensor_rt/Dataset.hpp` (`TensorRTDataset`) | Inconsistent naming for the same abstraction family (`IDataset`, `TorchDataset`, `TensorRTDataset`) and mixed responsibility (dataset + DB access). | Keep interface prefix (`IDataset`) and standardize backend-specific suffixes in backend namespaces (`vision::libtorch::DatasetAdapter`, `vision::tensor_rt::DatasetAdapter`). See sketch below. |
| `libs/reusex/include/ReUseX/vision/BackendFactory.hpp` (`BackendFactory::create`) and `libs/reusex/include/ReUseX/vision/IModel.hpp` (`IModel::create`) | Mixed creation paradigm (factory class + static interface factory) with no documented rule. | Adopt one construction rule for public API by consolidating creation behind a single factory (shown below as `ModelFactory`, a proposed replacement for split `BackendFactory`/`IModel::create`). Keep static `create` only for concrete implementation internals. |
| `libs/reusex/include/ReUseX/vision/project.hpp` (`project(...)`) | Free-function entry point in `vision` while related model/dataset APIs are class-based; rationale is not explicit. | Introduce a small service façade (`ProjectionService`) and keep free function as deprecated forwarding shim to preserve compatibility. |
| `libs/reusex/include/ReUseX/vision/common/object.hpp` (`Box::center_x`, `center_y`) | `snake_case` method names in an API where many symbols follow Camel/Pascal variants (`TensorRTDataset`, `BackendFactory`). | Pick one method naming style for public members (recommended: `snake_case` for functions/methods/parameters in this codebase) and apply uniformly in touched API areas. |
| `libs/reusex/include/ReUseX/vision/BackendFactory.hpp` (`Backend::libTorch`) | Mixed enum casing (`libTorch` among `OpenCV`, `TensorRT`, `DNN`). | Normalize enum values (`LibTorch`) and keep compatibility alias during migration (`[[deprecated]]`). |

```cpp
namespace ReUseX::vision {
class IDataset { /* interface */ };

class ModelFactory {
public:
  // ModelSpec = lightweight model path + backend/options descriptor.
  static std::unique_ptr<IModel> create(const ModelSpec& spec);
};
} // namespace ReUseX::vision

namespace ReUseX::vision::libtorch {
// Backend-internal adapter type; do not expose via generic vision headers.
class DatasetAdapter final
    : public torch::data::datasets::Dataset<DatasetAdapter> { /* ... */ };
} // namespace ReUseX::vision::libtorch
```

## 3. Reusability Issues

| Location | Problem | Recommended fix (+ sketch) |
|---|---|---|
| `libs/reusex/include/ReUseX/vision/IDataset.hpp` | `IDataset` requires `io::RTABMapDatabase` in constructors and protected API. Dataset abstraction is not reusable without RTABMap DB. | Split data access from dataset behavior: introduce `IFrameStore` (or `ISampleStore`) in `vision` and adapt `RTABMapDatabase` behind it. |
| `libs/reusex/include/ReUseX/types.hpp` and geometry headers using `Cloud*` aliases | Core types are direct PCL aliases; geometry API cannot be reused with alternate point containers. | Introduce boundary-level view types (`PointSpan`, `NormalSpan`) in public API and convert internally to PCL when needed. Start with overloads (e.g., `segment_rooms(...)`, `regularize_planes(...)`) instead of replacement to reduce churn. |
| `libs/reusex/include/ReUseX/vision/libtorch/Dataset.hpp` | Public inheritance from `torch::data::datasets::Dataset<>` hard-couples API to PyTorch. | Keep this in backend adapter namespace only, and avoid exposing it in generic `vision` headers. |
| `libs/reusex/include/ReUseX/geometry/regularization.hpp` | Templated algorithm includes CGAL+PCL headers directly in public interface. Reuse and build times suffer for clients not needing this algorithm. | Move heavy-template implementation to detail header and expose a narrow compiled façade in `.cpp` where possible (or explicit instantiations for supported types). |

## 4. Third-Party Dependency Leakage

### OpenCV
- **Where it leaks:** `vision/IDataset.hpp` (`cv::Mat`), `io/RTABMapDatabase.hpp` (`cv::Mat` in public methods), `vision/common/object.hpp` (`cv::Mat` fields).
- **Abstraction strategy:** Introduce lightweight `ImageView`/`LabelImage` wrappers in `ReUseX::vision` and keep OpenCV adapters in `io/adapters/opencv`.

```cpp
namespace ReUseX::vision {
struct ImageView { int width{}, height{}, channels{}; std::span<const std::byte> data; };
struct MutableImageView { int width{}, height{}, channels{}; std::span<std::byte> data; };
}
```

### PCL
- **Where it leaks:** `types.hpp` aliases (`PointT`, `Cloud`, `CloudPtr`, etc.) are part of top-level public API.
- **Abstraction strategy:** Keep PCL aliases in `ReUseX::adapters::pcl` and expose generic geometry-facing data structures in `ReUseX::geometry::types`.

```cpp
namespace ReUseX::geometry {
struct Point3f { float x, y, z; };
using PointSpan = std::span<const Point3f>;
}
```

### CGAL
- **Where it leaks:** `geometry/regularization.hpp` includes CGAL types in public header.
- **Abstraction strategy:** Define algorithm options/result types independent of CGAL and hide CGAL implementation behind pimpl/compiled unit.

```cpp
struct PlaneRegularizationOptions { double angle_deg{25.0}; double distance{0.01}; };
// PlaneVector<Scalar> refers to the existing alias in geometry/regularization.hpp.
template <typename Scalar>
auto regularize_planes(PlaneVector<Scalar>& planes,
                       PointSpan points,
                       PlaneRegularizationOptions opts) -> void;
```

### PyTorch
- **Where it leaks:** `vision/libtorch/Dataset.hpp` includes `<torch/torch.h>` and public inheritance from torch dataset.
- **Abstraction strategy:** Treat as backend plugin module (`vision/backends/libtorch`) not part of core `vision` API surface.

### TensorRT/CUDA
- **Where it leaks:** `vision/tensor_rt/Sam3.hpp` includes TensorRT/CUDA-heavy headers publicly.
- **Abstraction strategy:** Keep `IModel` pure and move TensorRT model classes behind factory + pimpl in compiled module.

### RTABMap
- **Where it leaks:** `io/RTABMapDatabase.hpp` uses Pimpl but still exposes `rtabmap::Transform`, `rtabmap::Link`, `rtabmap::Signature` in the public `getGraph` signature.
- **Abstraction strategy:** Return ReUseX-owned graph DTOs from public API and add conversion adapter for RTABMap-specific consumers.

```cpp
namespace ReUseX::io {
struct PoseNode { int id; std::array<float, 16> transform; };
struct Edge { int from; int to; /* typed relation */ };
}
```

## 5. Recommended Codebase Structure

```text
libs/reusex/include/ReUseX/
  core/                 # logging, version, error model
  api/                  # stable public façades (model, dataset, projection)
  geometry/
    types/              # dependency-light geometric DTOs/views
    algorithms/         # public algorithm façades
  io/
    stores/             # public data store interfaces (IFrameStore)
    formats/            # format-specific APIs (reusex, rhino)
  adapters/             # third-party adapters (pcl, opencv, rtabmap)
  vision/
    types/              # model-agnostic vision DTOs
    interfaces/         # IModel, IData, Dataset abstractions
    backends/           # tensor_rt/, libtorch/ implementation-only headers
```

**Header-only vs compiled**
- **Header-only:** small value types, traits, simple non-heavy utility templates.
- **Compiled:** anything pulling CGAL/PCL/OpenCV/TensorRT/PyTorch/RTABMap headers, and all factory implementations.

**Decoupling priorities**
1. `vision::interfaces` must not depend on `io::RTABMapDatabase`.
2. `geometry` public API should not require direct PCL/CGAL includes.
3. backend-specific directories should not leak into top-level `vision` headers.

## 6. Design Pattern Recommendations (Prioritized)

1. **Adapter + Facade (highest priority)**
   - **Apply to:** OpenCV/PCL/RTABMap/TensorRT/PyTorch boundary.
   - **Why:** Contains dependency leakage without full rewrites.
   - **Example:**
   ```cpp
   class FrameStoreFacade {
   public:
     virtual ~FrameStoreFacade() = default;
     virtual ImageView image(std::size_t frame_index) const = 0;
   };
   class RTABMapFrameStoreAdapter final : public FrameStoreFacade { /* ... */ };
   ```

2. **Factory unification**
   - **Apply to:** `IModel::create`, `BackendFactory::create`.
   - **Why:** Single creation entry point reduces API ambiguity and centralizes policy.

3. **Pimpl for heavy backends**
   - **Apply to:** TensorRT model public headers (`TensorRTSam3`), potentially libtorch wrappers.
   - **Why:** Better compile times and binary compatibility; consistent with `RTABMapDatabase`.

4. **Strategy for backend selection**
   - **Apply to:** backend auto-detection and runtime backend switching.
   - **Why:** Replace extension-based conditionals spread across APIs with explicit strategy object.

5. **Rule-of-Zero enforcement**
   - **Apply to:** value-like structs in `vision/common/object.hpp` and similar DTOs.
   - **Why:** Avoid custom special members unless ownership is real (e.g., `SegmentMap` is an exception because it owns CUDA pinned memory).

6. **Unified error model**
   - **Apply to:** factory/detection code paths that currently mix logs + throws.
   - **Why:** one public contract (exceptions now, or migrate to `std::expected` when toolchain permits) improves predictability.

## 7. Quick Wins vs Long-Term Refactors

### Quick wins (low effort, high impact)
- Add API style guide section in `CONTRIBUTING.md` (naming + construction rules).
- Normalize enum names in `Backend` from `libTorch` to `LibTorch` with deprecation aliases.
- Deprecate direct public use of free `project(...)` in favor of a service façade.
- Introduce thin wrapper types (`ImageView`, `Point3f`) alongside existing `cv::Mat`/PCL APIs (non-breaking additive change).
- Move backend-heavy includes out of top-level `vision` headers where possible.

### Strategic refactors (planned, high impact)
- Introduce `IFrameStore` and remove `RTABMapDatabase` from generic `vision` interfaces.
- Split public API into dependency-light interfaces and backend adapters.
- Replace RTABMap types in public IO signatures with ReUseX DTOs.
- Build a stable `api/` façade layer that shields consumers from third-party churn.
- Gradually isolate CGAL/PCL-heavy algorithms behind compiled boundaries and explicit instantiations.
