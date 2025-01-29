{
    buildPythonPackage,
    fetchFromGitHub,
    pkgs,
    ...
}:
let
    g2o-pymem = pkgs.g2o.overrideAttrs (old: {

        version = "pymem";

        patches = [
            ./pybind_11.patch
        ];

        src = fetchFromGitHub {
            owner = "RainerKuemmerle";
            repo = "g2o";
            rev = "c203321596a38502cb3008a3883805cb91d3535a";
            sha256 = "sha256-oGOzQpU0BW0KDjUZPK0pYjknio2rC2dQoDVLWrIb+SI=";
        };

        # nativeBuildInputs = (old.nativeBuildInputs or []) ++ (with pkgs; [
        #     git
        # ]);

        buildInputs = (old.buildInputs or []) ++ (with pkgs; [
            python3Packages.pybind11
            nlohmann_json
        ]);

        cmakeFlags = (old.cmakeFlags or []) ++ [
            "-DG2O_BUILD_PYTHON=ON"
        ];

    });

in
    buildPythonPackage rec {

    stdenv = if pkgs.config.cudaSupport then pkgs.cudaPackages.backendStdenv else pkgs.stdenv;

    pname = "g2o";
    version = "0.0.1";

    dontUnpack = true;
    pyproject = false;

    propagatedBuildInputs = [
        python
    ];

    installPhase = ''
        echo "Installing the package"2

        mkdir -p "$out/${python.sitePackages}/g2o"
        export PYTHONPATH="$out/${python.sitePackages}:$PYTHONPATH"

        cp -r ${g2o-pymem}/g2o/* $out/${python.sitePackages}/g2o
        # touch $out/${g2o-pymem}/g2o/__init__.py

        echo "Finished installing the package"
    '';
    
    }