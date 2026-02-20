# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}:
pkgs.mkShell {
  inputsFrom = [self.packages.${system}.default];

  packages = with pkgs; [
    (pkgs.python3.withPackages (
      ps:
        with ps; [
          numpy
          ruamel-yaml
          opencv-python
          # from skvideo import io
          pillow
          pandas
          scipy
          ultralytics
          clip
          onnx
          timm
          tensorrt
          onnx2torch
          sam3

          notebook
          ipywidgets
        ]
    ))
  ];

  shellHook = ''
    echo "Entering python shell"
    export VIRTUAL_ENV_PROMPT="ReUseX Python Environment"
  '';
}
