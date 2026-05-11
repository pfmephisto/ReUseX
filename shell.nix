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
    echo "  │         ReUseX  development shell           │"
    echo "  └─────────────────────────────────────────────┘"
    echo "  configure [Release|Debug]   cmake + compile_commands.json"
    echo "  build                       cmake --build --parallel"
    echo "  run-tests                   build, then ctest"
    echo "  clean                       remove build/"
    echo "  format                      clang-format all C++ sources"
    echo "  lint                        cppcheck static analysis"
    echo "  docs                        build Doxygen docs"
    echo "  menu                        show this menu"
    echo ""
  '';

  # Convenience scripts available in any shell (bash, fish, zsh).
  scripts = pkgs.lib.attrValues {
    menu = pkgs.writeShellScriptBin "menu" motd;
    configure = pkgs.writeShellScriptBin "configure" ''
      cmake -B "$PWD/build" -GNinja \
        -DCMAKE_BUILD_TYPE=''${1:-Release} \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        "''${@:2}"
      ln -sf "$PWD/build/compile_commands.json" "$PWD/compile_commands.json"
    '';
    build = pkgs.writeShellScriptBin "build" ''
      cmake --build "$PWD/build" --parallel "$@"
    '';
    run-tests = pkgs.writeShellScriptBin "run-tests" ''
      cmake --build "$PWD/build" --parallel \
        && ctest --test-dir "$PWD/build" --output-on-failure --parallel "$@"
    '';
    clean = pkgs.writeShellScriptBin "clean" ''
      rm -rf "$PWD/build" && echo "Build directory removed."
    '';
    format = pkgs.writeShellScriptBin "format" ''
      find "$PWD/libs" "$PWD/apps" \( -name '*.cpp' -o -name '*.hpp' \) \
        | xargs clang-format -i && echo "Done."
    '';
    lint = pkgs.writeShellScriptBin "lint" ''
      cppcheck --enable=all --suppress=missingIncludeSystem \
        -I "$PWD/libs/reusex/include" "$PWD/libs/" "$PWD/apps/"
    '';
    docs = pkgs.writeShellScriptBin "docs" ''
      cmake --build "$PWD/build" --target doc "$@"
    '';
  };
in
  pkgs.mkShell {
    inputsFrom = [self.packages.${system}.default];
    buildInputs = self.checks.${system}.pre-commit-check.enabledPackages;

    packages =
      scripts
      ++ (with pkgs; [
        # Python 3.11 (matches Blender) - must come first to take precedence
        blender.pythonPackages.python

        # Documentation tools
        help2man # For generating man pages from --help output
        pandoc
        sphinx
        graphviz # For Doxygen diagrams and visualization

        # Debugging and analysis tools
        gdb
        valgrind
        kdePackages.kcachegrind
        heaptrack # Memory profiler with GUI

        # Build tools
        cmake-format # Format CMakeLists.txt files
        ccache # Cache C++ compilation to speed up rebuilds
        ninja # Faster build system alternative to Make
        bear # Generate compile_commands.json for LSP/clangd

        # C++ development tools
        clang-tools # Includes clang-format, clang-tidy, clang-rename
        cppcheck # Static analysis for C++
        include-what-you-use # Check #include dependencies

        # Performance profiling
        linuxPackages.perf # Performance profiling
        hotspot # GUI for perf data visualization

        # Development tools
        libnotify # Send notification when build finishes
        sqlite
        ffmpeg
        openusd
        jq # JSON processor for scripts
        ripgrep # Fast code search (faster than grep)
        fd # Fast file finder (faster than find)
        hyperfine # Command-line benchmarking

        # Version control tools
        tig # Text-mode interface for git
        git-filter-repo # Advanced git history rewriting
        gitui # Terminal UI for git (alternative)

        #qt6.full
        #qtcreator

        # DevOps tools
        nix-update
        sqlitebrowser
        hugin
        github-copilot-cli
        claude-code
        gh
        doxygen
        tree
        git-lfs

        # Code coverage
        lcov
        gcovr # Alternative coverage report generator

        # Python development
        python3Packages.black # Python code formatter
        python3Packages.pytest # Testing framework
        python3Packages.mypy # Type checking
      ]);

    shellHook =
      ''
        export VIRTUAL_ENV_PROMPT="ReUseX"

        ${motd}
      ''
      + self.checks.${system}.pre-commit-check.shellHook;
  }
