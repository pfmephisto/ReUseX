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
  buildInputs = self.checks.${system}.pre-commit-check.enabledPackages;

  packages = with pkgs; [
    # Documentation tools
    help2man # For generating man pages from --help output
    pandoc
    sphinx

    # Degugging and analysis tools
    gdb
    valgrind
    kdePackages.kcachegrind

    # Development tools
    libnotify # Send noctification when build finishes
    sqlite
    ffmpeg
    mcl

    #qt6.full
    #qtcreator

    # DevOps tools
    nix-update
    sqlitebrowser
    github-copilot-cli
    doxygen
    tree
  ];

  shellHook =
    ''
      echo "Entering dev shell"
      export VIRTUAL_ENV_PROMPT="ReUseX Environment"
      #export QT_STYLE_OVERRIDE="fusion"
      #export REPO_ROOT=$(pwd)
      #export CUDA_PATH=${pkgs.cudaPackages.cudatoolkit}
      # Required for qmlls to find the correct type declarations
      export QMLLS_BUILD_DIRS=${pkgs.qt6.qtdeclarative}/lib/qt-6/qml/:${pkgs.quickshell}/lib/qt-6/qml/
      export QML_IMPORT_PATH=$PWD/src
      # ./tmux_session
    ''
    + self.checks.${system}.pre-commit-check.shellHook;
}
