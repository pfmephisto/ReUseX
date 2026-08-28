# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# NOTE: CUDA support disabled - HiGHS GPU acceleration (PDLP solver) is not used
# by Solidifier which requires MIP solver. See MEMORY.md for details.
_: _final: prev: {
  highs = prev.highs.overrideAttrs (_self: super: {
    cmakeFlags =
      (super.cmakeFlags or [])
      ++ [
        (prev.lib.cmakeFeature "CUPDLP_GPU" "OFF")
      ];
  });
}
