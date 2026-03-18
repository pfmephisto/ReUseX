# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: let
in
  final: prev: rec {
    python3 = prev.python3.override {
      packageOverrides = pyfinal: pyprev: {
        specklepy = prev.pkgs.specklepy;
        spdlog = prev.pkgs.spdlog-python;

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
