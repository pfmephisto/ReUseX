{
    fetchFromGitHub,
    pkgs,
    lib,
    stdenv,
    ...
}:
let
    opennurbsConfig = ./opennurbsConfig.cmake;
in
stdenv.mkDerivation rec {

    pname = "opennurbs";
    version = "8.13.24317.13001";

    src = fetchFromGitHub {
        owner = "mcneel";
        repo = "${pname}";
        rev = "v${version}";
        fetchSubmodules = true;
        sha256 = "sha256-Q+ExlsJqsjUXQs8le/bjp8nw6I10W0YWJUNgAjKTNXg=";
    };

    nativeBuildInputs = with pkgs; [
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
        maintainers = with maintainers; [  ];
    };
}