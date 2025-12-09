# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  fetchFromGitHub,
  lib,
  stdenv,
  cmake,
  ...
}: let
  opennurbsConfig = ./opennurbsConfig.cmake;
in
  stdenv.mkDerivation rec {
    pname = "opennurbs";
    version = "8.24.25281.15001";

    src = fetchFromGitHub {
      owner = "mcneel";
      repo = "${pname}";
      rev = "v${version}";
      fetchSubmodules = true;
      sha256 = "sha256-G8bYWY4qQQ7DEo6RLhf80KtAVZuGMyUcomy2c9Kr41Q=";
    };

    cmakeFlags = [
      (lib.cmakeFeature "CMAKE_POLICY_VERSION_MINIMUM" "3.5")
    ];

    nativeBuildInputs = [
      cmake
    ];

    fixupPhase = ''
      mkdir -p $out/share/cmake/opennurbs
      cp ${opennurbsConfig} $out/share/cmake/opennurbs/opennurbsConfig.cmake
      cp ../*.h $out/include/
      # cp ../opennurbs_public.h $out/include/opennurbs_public.h
      # cp ../opennurbs_public.h $out/include/OpenNURBS/opennurbs_public.h
      # cp ../opennurbs_public.h $out/include/opennurbsStatic/opennurbs_public.h

    '';

    meta = with lib; {
      description = "OpenNURBS libraries allow anyone to read and write the 3DM file format without the need for Rhino. ";
      license = licenses.mit;
      maintainers = with maintainers; [];
    };
  }
