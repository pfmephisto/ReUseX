# ReUseX Python Bindings

pybind11 bindings that give Python read access to a ReUseX project database
(`*.rux`). Implemented in `src/bindings.cpp`; the native module is
`reusex._reusex`, re-exported by `reusex/__init__.py`.

## Status

**Implemented, read-oriented.** `ProjectDB` opens `read_only=True` by default
and the exposed surface is inspection plus geometry retrieval — there are no
write or pipeline-driving APIs yet.

`reusex.__status__` is `"Active"` when the native module imported successfully,
otherwise `"Native module not available: <reason>"`.

## API

`reusex.ProjectDB(path, read_only=True)`:

| Method | Returns |
|---|---|
| `is_open()`, `path()`, `schema_version()` | basic state |
| `project_summary()` | `ProjectSummary` |
| `list_point_clouds()`, `list_meshes()` | names of stored clouds / meshes |
| `list_building_components()`, `building_component_count()` | building components |
| `label_definitions()` | semantic label definitions |
| `pipeline_log(limit=0)` | `list[PipelineLogEntry]` |
| `point_cloud_xyzrgb(name)` | `dict` with `positions` (N,3) float32 and `colors` (N,3) uint8 |
| `point_cloud_xyz(name)`, `point_cloud_label(name)` | numpy arrays |
| `sensor_frame_ids()`, `sensor_frame_pose(id)`, `sensor_frame_intrinsics(id)` | frame metadata |
| `has_sensor_frame_pose(id)` | is the stored pose usable? `sensor_frame_pose()` returns identity for a poseless frame and an all-zero/NaN transform verbatim, so ask this before trusting one |
| `reconstruct_frame(...)`, `reconstruct_frames_parallel(...)` | back-projected frame geometry |

Value types re-exported from the package: `ProjectSummary`, `ProjectInfo`,
`CloudInfo`, `MeshInfo`, `SensorFrameInfo`, `PanoramicInfo`, `ComponentInfo`,
`MaterialInfo`, `PipelineLogEntry`.

## Requirements

- Python 3.9+ (`requires-python = ">=3.9"`)
- NumPy (runtime dependency)
- pybind11 2.11+ and scikit-build-core (build dependencies)
- `torch` for the optional `ml` extra

## Building

`BUILD_PYTHON_BINDINGS` defaults to **ON** in the root `CMakeLists.txt`, so a
normal project build configures this module.

```bash
# Editable install
pip install -e bindings/python/

# Or build a wheel
pip install bindings/python/
```

The version is not declared in `pyproject.toml` (`dynamic = ["version"]`); CMake
configures `_version.py.in` from `PROJECT_VERSION` and it surfaces as
`reusex.__version__`.

## Previous Implementation

An older, unrelated Python package lived in a top-level `python/` directory and
was removed (broken imports, outdated API). Last commit containing it:

```
24b1dab00508722a5d1c0b4d416b41c09ff9608e Add docstrings to nested helper functions in pose graph (#110)
```

```bash
git show 24b1dab00508722a5d1c0b4d416b41c09ff9608e:python/
```

## Possible Extensions

Not implemented — filed here as ideas, not commitments:

- Mesh retrieval as numpy arrays
- Material passport read/write
- Dataset API mirroring `reusex::vision::IDataset`
- Write paths (saving clouds / labels back into the project)

## Contributing

- The C++ API being wrapped lives in `libs/reusex/include/` (notably
  `core/ProjectDB.hpp`); consume it as `<reusex/core/ProjectDB.hpp>`
- Read [`docs/STANDARDS.md`](../../docs/STANDARDS.md) before adding surface area
- Follow pybind11 best practices for modern C++ binding

Questions: open an issue at https://github.com/pfmephisto/ReUseX/issues
