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
  ];

  shellHook =
    ''
      echo "Entering dev shell"
      export VIRTUAL_ENV_PROMPT="ReUseX Environment"
      # ./tmux_session
    ''
    + self.checks.${system}.pre-commit-check.shellHook;
}
