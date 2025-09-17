{...}: final: prev: (prev.lib.packagesFromDirectoryRecursive {
  callPackage = prev.lib.callPackageWith final;
  directory = ../pkgs;
})
