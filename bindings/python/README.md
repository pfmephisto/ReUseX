# ReUseX Python Bindings

## Status

**Not yet implemented**

The Python bindings are currently under development. This directory provides a stub structure for future pybind11-based bindings.

## Previous Implementation

A previous Python package existed but has been removed due to broken imports and outdated API. The last commit containing the old Python code was:

```
24b1dab00508722a5d1c0b4d416b41c09ff9608e Add docstrings to nested helper functions in pose graph (#110)
```

To retrieve the old Python code for reference:
```bash
git show 24b1dab00508722a5d1c0b4d416b41c09ff9608e:python/
```

## Planned Roadmap

The new Python bindings will focus on core functionality:

1. **Database Access** (`reusex.io`)
   - RTABMapDatabase wrapper
   - Image and label retrieval
   - Graph access

2. **Dataset API** (`reusex.vision`)
   - IDataset interface
   - PyTorch integration (Dataset class)
   - TensorRT integration

3. **Vision Utilities** (`reusex.vision`)
   - Object detection types (DetectionBox, etc.)
   - Image preprocessing

4. **Geometry** (`reusex.geometry`)
   - Point cloud utilities
   - Segmentation algorithms

## Requirements

When implemented, the bindings will require:
- Python 3.8+
- pybind11 2.10+
- NumPy
- OpenCV (cv2)
- PyTorch (optional, for Dataset)

## Building (Future)

When pybind11 bindings are implemented, you can install the Python package using:

```bash
# Install in development mode (editable)
pip install -e bindings/python/

# Or build and install a wheel
pip install bindings/python/
```

Alternatively, build the entire project with Python bindings:

```bash
cmake -B build -DBUILD_PYTHON_BINDINGS=ON
cmake --build build
pip install build/bindings/python/
```

## Contributing

If you're interested in contributing to Python bindings development, please:
1. Check existing issues tagged with `python-bindings`
2. Review the C++ API in `libs/reusex/include/ReUseX/`
3. Follow pybind11 best practices for modern C++ binding

## Questions?

For questions about Python binding development, please open an issue on the main repository.
