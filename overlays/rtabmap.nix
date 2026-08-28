# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
_: _final: prev: {
  rtabmap = prev.rtabmap.overrideAttrs (old: {
    buildInputs =
      (old.buildInputs or [])
      ++ (with prev.pkgs; [
        gtsam
        onetbb
      ]);

    cmakeFlags =
      (old.cmakeFlags or [])
      ++ [
        "-DWITH_TORCH=ON"
        "-DWITH_GTSAM=ON"
        "-DWITH_CERES=ON"
      ];
  });
}
