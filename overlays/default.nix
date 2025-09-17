{lib, ...}: let
  allFiles = builtins.readDir ./.;

  valid = file: allFiles.${file} != "directory" && file != "default.nix" && lib.hasSuffix ".nix" file;

  overlayFiles = builtins.filter valid (builtins.attrNames allFiles);

  overlaysList = builtins.map (file: import (builtins.toPath ./. + "/" + file) {}) overlayFiles;
in
  overlaysList
