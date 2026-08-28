# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
_: final: prev: (prev.lib.packagesFromDirectoryRecursive {
  callPackage = prev.lib.callPackageWith final;
  directory = ../pkgs;
})
