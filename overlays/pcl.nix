# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
_: _final: prev: {
  pcl = prev.pcl.override {vtk = prev.vtkWithQt6;};
}
