# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
_: _final: prev: rec {
  python3 = prev.python3.override {
    packageOverrides = _pyfinal: pyprev: {
      onnx = pyprev.onnx.override {
        onnx = prev.onnx.overrideAttrs (old: {
          env =
            old.env
            // {
              ONNX_ML = "1";
            };
        });
      };
    };
  };
  python3Packages = python3.pkgs;
}
