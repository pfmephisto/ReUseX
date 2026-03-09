# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
{
  pkgs,
  self,
  system,
  ...
}: let
  #!/usr/bin/env python
  script = ''
    #!/usr/bin/env python

    import argparse
    from os import path
    import subprocess

    lfs_script = 'git lfs %s "$@"'

    parser = argparse.ArgumentParser(
        description="A simple script that installs git lfs hook"
    )
    parser.add_argument(
        "--stage", help="Specify the stage where to add the git lfs part", required=True
    )

    args = parser.parse_args()

    git_command = subprocess.run(
        ["git", "rev-parse", "--path-format=absolute", "--git-common-dir"],
        check=True,
        capture_output=True,
        text=True,
    )
    git_common_dir = git_command.stdout.strip()

    stage_location = path.join(git_common_dir, "hooks", args.stage)

    if path.exists(stage_location):
        # check if the script was already installed
        with open(stage_location, "r") as file:
            for line in file:
                if line.startswith('git lfs %s "$@"' % args.stage):
                    exit(0)  # nothing else to do

        with open(stage_location, "a") as file:
            file.write(lfs_script % args.stage)
    else:
        with open(stage_location, "w") as file:
            file.write("#!/usr/bin/env bash")
            file.write(lfs_script % args.stage)
  '';
in
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
      git-lfs
      (pkgs.writeScriptBin "project-git-lfs-hook-installer" script)
    ];

    shellHook =
      ''
        echo "Entering dev shell"

        echo "Injecting Git LFS hooks..."
        for hook in pre-push post-checkout post-commit post-merge; do
           project-git-lfs-hook-installer --stage $hook
        done
        export VIRTUAL_ENV_PROMPT="ReUseX Environment"
        # ./tmux_session
      ''
      + self.checks.${system}.pre-commit-check.shellHook;
  }
