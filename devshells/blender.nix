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
    blender
  ];

  shellHook = ''
    _res="$PWD/.blender-dev"
    _bld="$PWD/build"
    _ext="$_res/extensions/user_default"
    _mod="$_res/scripts/modules/reusex"
    mkdir -p "$_ext" "$_mod"

    # Symlink the Blender extension
    ln -sfn "$PWD/apps/blender/reusex_panel" "$_ext/reusex_panel"

    # Assemble reusex Python package in modules dir from build artifacts
    # NOTE: Build must be done separately with -DBUILD_PYTHON_BINDINGS=ON
    ln -sfn "$PWD/bindings/python/reusex/__init__.py" "$_mod/__init__.py"
    if [ -f "$_bld/bindings/python/reusex/_version.py" ]; then
      ln -sfn "$_bld/bindings/python/reusex/_version.py" "$_mod/_version.py"
    fi
    for so in "$_bld"/bindings/python/_reusex*.so; do
      [ -e "$so" ] && ln -sfn "$so" "$_mod/$(basename "$so")"
    done

    export BLENDER_USER_RESOURCES="$_res"
    export BLENDER_USER_SCRIPTS="$_res/scripts"

    rux-blender() {
      if [ ! -f "$_bld/bindings/python/_reusex"*.so ]; then
        echo "Error: Python bindings not built. Run from default shell:"
        echo "  cmake -B build -DBUILD_PYTHON_BINDINGS=ON"
        echo "  cmake --build build --target _reusex"
        return 1
      fi

      # LD_PRELOAD libreusex.so so its 500+ transitive deps get static
      # TLS allocation at startup, avoiding the glibc dlopen TLS limit.
      LD_PRELOAD="$_bld/libs/reusex/libreusex.so" blender "$@"
    }
    export -f rux-blender

    echo ""
    echo "Blender development shell"
    echo "  Blender:  $(blender --version 2>/dev/null | head -1)"
    echo "  Python:   $(blender --background --python-expr 'import sys; print(sys.version.split()[0])' 2>/dev/null | tail -1)"
    echo ""
    echo "Build Python bindings first (from default shell):"
    echo "  cmake -B build -DBUILD_PYTHON_BINDINGS=ON"
    echo "  cmake --build build --target _reusex"
    echo ""
    echo "Then launch Blender:"
    echo "  rux-blender              Launch Blender with extension"
    echo "  rux-blender file.blend   Open specific file"
    echo ""
  '';
}
