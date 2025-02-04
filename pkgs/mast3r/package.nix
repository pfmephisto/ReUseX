{
    fetchFromGitHub,
    lib,
    pkgs,
    ...
}:

pkgs.python3Packages.buildPythonPackage rec {

    pname = "mast3r";
    version = "e06b009";

    format = "pyproject";

    src = fetchFromGitHub {
        owner = "naver";
        repo = "${pname}";
        fetchSubmodules = true;
        rev = "e06b0093ddacfd8267cdafe5387954a650af0d3b";
        # sha256 = lib.fakeSha256;
        sha256 = "sha256-xkLHkaQ3YrYETKB16EoUiiz51A9IaAXTihA1QVVg7T8=";
    };

    patches = [
        ./add_pyproject.patch
    ];


    buildInputs = with pkgs; [
        python3Packages.setuptools
    ];


    # Metadata (optional)
    meta = with lib; {
        description = "Grounding Image Matching in 3D with MASt3R ";
        license = licenses.cc-by-nc-sa-40;
        homepage = "https://github.com/naver/mast3r";
    };

}