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
    echo "  │       ReUseX  •  Completions  shell         │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  generate-completions   generate fish completions from rux binary"
    echo "  menu                   show this message"
    echo ""
    echo "  Build rux first (from default shell): build --target rux"
    echo ""
  '';
  generate-completions = pkgs.writeShellScriptBin "generate-completions" ''
    python3 "$PWD/scripts/generate_fish_completions.py" \
      -o "$PWD/completions/rux.fish" \
      --rux-binary "$PWD/build/apps/rux/rux"
    echo "Written: completions/rux.fish"
  '';
in
  pkgs.mkShell {
    inputsFrom = [self.packages.${system}.default];

    packages = [
      pkgs.python3
      pkgs.fish
      generate-completions
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = motd;
  }
