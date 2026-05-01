"""
ReUseX Python Bindings

Read-only access to .rux project databases via pybind11 bindings
to the ReUseX C++ library.
"""

from ._version import __version__

try:
    from ._reusex import (
        CloudInfo,
        ComponentInfo,
        MaterialInfo,
        MeshInfo,
        PanoramicInfo,
        PipelineLogEntry,
        ProjectDB,
        ProjectInfo,
        ProjectSummary,
        SensorFrameInfo,
    )

    __status__ = "Active"
except ImportError as exc:
    __status__ = f"Native module not available: {exc}"

__all__ = ["__version__", "__status__"]
