# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  pcl = prev.pcl.override {vtk = prev.vtkWithQt6;};
}
