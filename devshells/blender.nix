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
    _dev="$PWD/.blender-dev/scripts"
    _mod="$_dev/modules/reusex"
    mkdir -p "$_dev/addons" "$_mod"

    # Symlink the Blender addon
    ln -sfn "$PWD/apps/blender/reusex_panel" "$_dev/addons/reusex_panel"

    # Assemble reusex Python package in modules dir
    ln -sfn "$PWD/bindings/python/reusex/__init__.py" "$_mod/__init__.py"
    if [ -f "$PWD/build/bindings/python/reusex/_version.py" ]; then
      ln -sfn "$PWD/build/bindings/python/reusex/_version.py" "$_mod/_version.py"
    fi
    for so in "$PWD"/build/bindings/python/_reusex*.so; do
      [ -e "$so" ] && ln -sfn "$so" "$_mod/$(basename "$so")"
    done

    export BLENDER_USER_SCRIPTS="$_dev"

    _rux_refresh_module() {
      ln -sfn "$PWD/build/bindings/python/reusex/_version.py" "$_mod/_version.py"
      for so in "$PWD"/build/bindings/python/_reusex*.so; do
        [ -e "$so" ] && ln -sfn "$so" "$_mod/$(basename "$so")"
      done
    }

    rux-blender() {
      cmake -B build -DBUILD_PYTHON_BINDINGS=ON -DBUILD_TESTS=OFF \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYBIND11_FINDPYTHON=ON \
            -DPython3_EXECUTABLE="$(which python3)" 2>&1 | tail -1
      cmake --build build --target _reusex -- -j"$(nproc)" || return 1

      _rux_refresh_module

      # LD_PRELOAD libreusex.so so its 500+ transitive deps get static
      # TLS allocation at startup, avoiding the glibc dlopen TLS limit.
      LD_PRELOAD="$PWD/build/libs/reusex/libreusex.so" blender "$@"
    }
    export -f rux-blender _rux_refresh_module

    echo ""
    echo "Blender development shell"
    echo "  Blender:  $(blender --version 2>/dev/null | head -1)"
    echo "  Python:   $(blender --background --python-expr 'import sys; print(sys.version.split()[0])' 2>/dev/null | tail -1)"
    echo ""
    echo "  rux-blender          Build _reusex module + launch Blender"
    echo "  rux-blender file.blend   ... with a specific file"
    echo ""
  '';
}
