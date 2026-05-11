# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}: let
  pythonEnv = pkgs.python3.withPackages (
    ps:
      with ps; [
        numpy
        pillow
        reportlab
      ]
  );
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │         ReUseX  •  AprilTag  shell          │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  python tools/april/generate_apriltags.py --help"
    echo "  python tools/april/create_apriltag_pdf.py --help"
    echo "  menu   show this message"
    echo ""
  '';
in
  pkgs.mkShell {
    packages = with pkgs; [
      pythonEnv
      imagemagickBig
      img2pdf
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = motd;
  }
