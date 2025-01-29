{
    mkDerivation,
    fetchFromGitHub,
    pkgs,
    ...
}:
mkDerivation {

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
        cat > $out/share/cmake/opennurbs/opennurbsConfig.cmake << 'EOF'

        set(polyscope_DIR $${opennurbs_INCLUDE_DIR} $out)

        if($${opennurbs_INCLUDE_DIR} STREQUAL opennurbs-NOTFOUND)
            message("can't found opennurbs")
            set(opennurbs_FOUND False)
        else()
            message("found opennurbs at path:" $${opennurbs_INCLUDE_DIR})
            
            set(opennurbs_FOUND True)
            set(opennurbs_INCLUDE_DIRS $out/lnclude/OpenNURBS
                                        $out/lnclude/opennurbsStatic
            )
            set(opennurbs_LIBRARIES $out/lib
            )

        endif()
        EOF
    '';

    meta = with lib; {
        description = "OpenNURBS libraries allow anyone to read and write the 3DM file format without the need for Rhino. ";
        license = licenses.mit;
        maintainers = with maintainers; [  ];
    };
}