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
    python3
    fish
  ];

  shellHook = ''
    echo "Shell completions generation environment"
    echo "  Generate completions:  generate-completions"
    echo ""

    # Create wrapper function for completion generation
    generate-completions() {
      python3 scripts/generate_fish_completions.py \
        -o completions/rux.fish \
        --rux-binary "build/apps/rux/rux"
    }
    export -f generate-completions
  '';
}
