# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  opencv = prev.opencv.override {
    enableGtk2 = true;
    enableGtk3 = true;
    enableVtk = true;
    enableTbb = true;
    # tbb = prev.tbb_2022;
    enableCudnn = true;
    enablePython = true;
    enableUnfree = true;
  };
}
