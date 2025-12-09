# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: {
  python3 = prev.python3.override {
    packageOverrides = pyfinal: pyprev: {
      specklepy = prev.pkgs.specklepy;
      spdlog = prev.pkgs.spdlog-python;
    };
  };
}
