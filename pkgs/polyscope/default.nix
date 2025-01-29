{
    mkDerivation,
    pkgs,
    fetchFromGitHub,
    ...
}:
mkDerivation {

    pname = "polyscope";
    version = "2.3.0";

    src = fetchFromGitHub {
        owner = "nmwsharp";
        repo = "polyscope";
        rev = "v${version}";
        fetchSubmodules = true;
        sha256 = "sha256-pViqQ/7F0+7LfVVNkb/Yb/iOdOm1puY8QEoNip0LsYk=";
    };

    nativeBuildInputs = with pkgs; [
        cmake
    ];

    buildInputs = with pkgs; [
        xorg.libX11
        xorg.libXrandr
        xorg.libXinerama
        xorg.libXcursor
        xorg.libxcb
        xorg.libXi
        libGL
    ];

    fixupPhase = ''
        mkdir -p $out/share/cmake/polyscope
        cat > $out/share/cmake/polyscope/polyscopeConfig.cmake << 'EOF'
        # Findpolyscope.cmake
        #[============[
        Findpolyscope
        ----------
        find polyscope library.
        Set 'polyscope_DIR' to the path of this file.

        Result Variables
        ^^^^^^^^^^^^^^^^
        ``polyscope_FOUND``
        ``polyscope_INCLUDE_DIRS``
        ``polyscope_LIBRARIES``

        Cache Variables
        ^^^^^^^^^^^^^^^
        ``polyscope_INCLUDE_DIR``
            the path include the polyscope.h.

        ]============]

        # use find_path to get a path anchor
        find_path(polyscope_INCLUDE_DIR
            NAMES polyscope.h
            PATHS $out/include/polyscope
            PATH_SUFFIXES polyscope)
        set(polyscope_DIR $out)

        if($${polyscope_INCLUDE_DIR} STREQUAL polyscope-NOTFOUND)
            message("can't found polyscope")
            set(polyscope_FOUND False)
        else()
            message("found polyscope at path:" $${polyscope_INCLUDE_DIR})
            
            set(polyscope_FOUND True)
            set(polyscope_INCLUDE_DIRS $${polyscope_DIR}/include
                                    $${polyscope_DIR}/include/render
                                    $${polyscope_DIR}/include/render/mock_opengl
                                    $${polyscope_DIR}/include/render/opengl
                                    $${polyscope_DIR}/include/render/opengl/shaders
            )
            set(polyscope_LIBRARIES $${polyscope_DIR}/lib

            )
        endif()
        EOF
    '';


    meta = with pkgs.lib; {
        description = "Polyscope is a C++/Python viewer and user interface for 3D data such as meshes and point clouds.";
        homepage = "https://polyscope.run/";
        license = licenses.mit;
        maintainers = with maintainers; [  ];
    };
}