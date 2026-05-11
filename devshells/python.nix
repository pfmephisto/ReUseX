# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}: let
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │        ReUseX  •  Python / ML  shell        │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  jupyter notebook   start Jupyter notebook server"
    echo "  jupyter lab        start JupyterLab server"
    echo "  yolo               Ultralytics YOLO CLI"
    echo "  python             Python with numpy, torch, cv2, SAM3, ..."
    echo "  menu               show this message"
    echo ""
  '';
in
  pkgs.mkShell {
    inputsFrom = [self.packages.${system}.default];

    packages = [
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
            transformers
            notebook
            ipywidgets
          ]
      ))
      # Top-level Python packages not in python3Packages
      pkgs.sam3
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = ''
      export VIRTUAL_ENV_PROMPT="ReUseX Python"
      ${motd}
    '';
  }
