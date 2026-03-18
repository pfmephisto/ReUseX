# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{
  lib,
  stdenv,
  cmake,
  fetchFromGitHub,
  ninja,
  doxygen,
  cargo,
  rustPlatform,
  rustc,
  gcc14Stdenv,
}:
gcc14Stdenv.mkDerivation rec {
  pname = "tokenizers-cpp";
  version = "0.1.1";

  src = fetchFromGitHub {
    owner = "mlc-ai";
    repo = "${pname}";
    #rev = "v${version}";
    rev = "acbdc5a27ae01ba74cda756f94da698d40f11dfe";
    fetchSubmodules = true;
    # sha256 = "sha256-7RCcbTNaLAZEYns64yFl8xurnzkfJZI7bXOeJcmiIVc=";
    sha256 = "sha256-/Y9FphwL0zs9hXyfvEbDbaDKAzy/hJ9qlSpUzViuDo8=";
  };

  nativeBuildInputs =
    [
      cmake
      ninja
      cargo
      rustc
    ]
    ++ (with rustPlatform; [
      cargoSetupHook
    ]);

  cargoRoot = "rust";
  # cargoVendorDir = "rust";
  #dontUnpack = true;

  #cargoDeps = rustPlatform.importCargoLock {
  #  lockFile = ./Cargo.lock;
  #  #outputHashes = {
  #  #  "rand-0.8.3" = "0ya2hia3cn31qa8894s3av2s8j5bjwb6yq92k0jsnlx7jid0jwqa";
  #  #};
  #};

  # cargoDeps = rustPlatform.fetchCargoVendor {
  #   src = src + "/rust";
  #   # sourceRoot = "src";
  #   name = "${pname}-${version}";
  #   hash = lib.fakeHash;
  # };

  cargoDeps = rustPlatform.importCargoLock {lockFile = ./Cargo.lock;};

  # cargoLock = ./Cargo.lock;
  # useFetchCargoVendor = false;
  # cargoHash = lib.fakeHash;

  patches = [
    ./Cargo_lock.patch
  ];

  buildInputs = [
    doxygen
  ];

  cmakeFlags = [
    (lib.cmakeFeature "CMAKE_POLICY_VERSION_MINIMUM" "3.10")
    #(lib.cmakeFeature "CMAKE_CXX_STANDARD" "17")
    #(lib.cmakeFeature "CMAKE_CXX_STANDARD_REQUIRED" "ON")
    #(lib.cmakeFeature "CMAKE_CXX_EXTENSIONS" "OFF")
  ];

  propagatedBuildInputs = [
  ];
}
