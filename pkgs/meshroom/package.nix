# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  stdenvNoCC,
  python3,
  fetchurl,
  e2fsprogs,
  glib,
  krb5,
  zlib,
  postgresql,
  unixODBC,
  cups,
  speechd,
  libsForQt5,
  gtk3,
  atk,
  gdk-pixbuf,
  cairo,
  pango,
  libdrm,
  libGLU,
  libglvnd,
  ocl-icd,
  xorg,
  autoPatchelfHook,
  autoAddDriverRunpath,
  makeWrapper,
  ...
}:
stdenvNoCC.mkDerivation rec {
  pname = "meshroom-bin";
  version = "2025.1.0";
  src = fetchurl {
    url = "https://github.com/alicevision/meshroom/releases/download/v${version}/Meshroom-${version}-linux-cuda10.tar.gz";
    hash = "sha256-3pTrJktbMNVbNjKcR2FZa8xrTEQ7CWsLFayWOJLXnuQ=";
  };

  nativeBuildInputs = [
    autoPatchelfHook
    autoAddDriverRunpath
    libsForQt5.wrapQtAppsHook
    makeWrapper
  ];

  buildInputs = [
    e2fsprogs
    glib
    krb5
    zlib
    postgresql
    unixODBC
    cups
    speechd
    python3
    libsForQt5.qtwebengine
    libsForQt5.qtwebview
    libsForQt5.qtwebsockets
    libsForQt5.qttools
    gtk3
    atk
    gdk-pixbuf
    cairo
    pango
    libdrm
    libGLU
    libglvnd
    libglvnd
    ocl-icd
    xorg.libX11
    xorg.libXfixes
    xorg.libXi
    xorg.libXrender
    xorg.libXxf86vm
  ];

  installPhase = ''
    mkdir -p $out/opt/meshroom
    cp -r \
      Meshroom \
      meshroom_batch \
      meshroom_compute \
      qtPlugins \
      lib \
      aliceVision \
      $out/opt/meshroom
    mkdir -p $out/bin
    makeWrapper $out/opt/meshroom/Meshroom $out/bin/Meshroom --chdir $out/opt/meshroom
    makeWrapper $out/opt/meshroom/meshroom_batch $out/bin/meshroom_batch --chdir $out/opt/meshroom
    makeWrapper $out/opt/meshroom/meshroom_compute $out/bin/meshroom_compute --chdir $out/opt/meshroom
  '';

  postFixup = ''
    patchelf --debug --add-needed libpython${lib.versions.major python3.pythonVersion}.so \
      "$out/opt/meshroom/Meshroom"
  '';

  meta = {
    mainProgram = "meshroom";
    # TODO: Fix Qt5 package dependencies for nixpkgs compatibility
    # category=I/O estimate=2h
    # Needs libsForQt5.callPackage or proper Qt5 scope handling
    broken = true;
  };
}
