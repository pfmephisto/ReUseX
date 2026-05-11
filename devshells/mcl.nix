# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}: let
  mcl = pkgs.python3Packages.buildPythonPackage rec {
    pname = "mcl";
    version = "0.0.6";

    src = pkgs.fetchFromGitHub {
      owner = "GuyAllard";
      repo = "markov_clustering";
      rev = "${version}";
      sha256 = "sha256-mKBAa/CDtyKQgDTa5tUG9VfhCIOrebx2x8onmKvysSE=";
    };

    pyproject = true;

    build-system = [pkgs.python3Packages.setuptools];

    propagatedBuildInputs = with pkgs.python3Packages; [
      numpy
      scipy
      scikit-learn

      networkx
      matplotlib
    ];
  };

  pythonEnv = pkgs.python3.withPackages (
    ps:
      with ps; [
        # Add your Python packages here
        numpy
        pandas
        matplotlib
        mcl
      ]
  );
  motd = ''
    echo ""
    echo "  ┌─────────────────────────────────────────────┐"
    echo "  │          ReUseX  •  MCL  shell              │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  python   MCL clustering available (import markov_clustering)"
    echo "  menu     show this message"
    echo ""
  '';
in
  pkgs.mkShell {
    packages = [
      pythonEnv
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = motd;
  }
