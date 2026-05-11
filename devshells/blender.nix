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
    echo "  │         ReUseX  •  Blender  shell           │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  rux-blender [file]   Launch Blender with ReUseX extension"
    echo "  rux-blender          (builds bindings first if needed)"
    echo "  menu                 show this message"
    echo ""
    echo "  Build bindings (from default shell):"
    echo "    cmake -B build -DBUILD_PYTHON_BINDINGS=ON"
    echo "    cmake --build build --target _reusex"
    echo ""
  '';
  rux-blender = pkgs.writeShellScriptBin "rux-blender" ''
    _bld="$PWD/build"
    if [ ! -f "$_bld/libs/reusex/libreusex.so" ]; then
      echo "Error: libreusex.so not found. Build Python bindings first:"
      echo "  cmake -B build -DBUILD_PYTHON_BINDINGS=ON && cmake --build build --target _reusex"
      exit 1
    fi
    # LD_PRELOAD libreusex.so so its 500+ transitive deps get static
    # TLS allocation at startup, avoiding the glibc dlopen TLS limit.
    LD_PRELOAD="$_bld/libs/reusex/libreusex.so" blender "$@"
  '';
in
  pkgs.mkShell {
    inputsFrom = [self.packages.${system}.default];

    packages = [
      pkgs.blender
      rux-blender
      (pkgs.writeShellScriptBin "menu" motd)
    ];

    shellHook = ''
      _res="$PWD/.blender-dev"
      _bld="$PWD/build"
      _ext="$_res/extensions/user_default"
      _mod="$_res/scripts/modules/reusex"
      mkdir -p "$_ext" "$_mod"

      ln -sfn "$PWD/apps/blender/reusex_panel" "$_ext/reusex_panel"
      ln -sfn "$PWD/bindings/python/reusex/__init__.py" "$_mod/__init__.py"
      if [ -f "$_bld/bindings/python/reusex/_version.py" ]; then
        ln -sfn "$_bld/bindings/python/reusex/_version.py" "$_mod/_version.py"
      fi
      for so in "$_bld"/bindings/python/_reusex*.so; do
        [ -e "$so" ] && ln -sfn "$so" "$_mod/$(basename "$so")"
      done

      export BLENDER_USER_RESOURCES="$_res"
      export BLENDER_USER_SCRIPTS="$_res/scripts"

      ${motd}
    '';
  }
