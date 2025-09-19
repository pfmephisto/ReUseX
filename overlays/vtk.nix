# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  vtk = prev.vtk.override {tbb = prev.tbb_2022;};
}
