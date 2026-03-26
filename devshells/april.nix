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
in
  pkgs.mkShell {
    packages = with pkgs; [
      pythonEnv
      imagemagickBig
      img2pdf
    ];

    shellHook = ''
      echo "AprilTag tooling environment"
      echo "  Generate tags:  python tools/april/generate_apriltags.py --help"
      echo "  Create PDF:     python tools/april/create_apriltag_pdf.py --help"
      echo "  ImageMagick:    magick --version"
    '';
  }
