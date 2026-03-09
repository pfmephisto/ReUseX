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

    openusd

    #qt6.full
    #qtcreator

    # DevOps tools
    nix-update
    sqlitebrowser
    hugin
    github-copilot-cli
    claude-code
    doxygen
    tree
  ];

  shellHook =
    ''
      echo "Entering dev shell"
      export VIRTUAL_ENV_PROMPT="ReUseX Environment"
      # ./tmux_session
    ''
    + self.checks.${system}.pre-commit-check.shellHook;
}
