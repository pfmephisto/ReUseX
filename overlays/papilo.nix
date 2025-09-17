{...}: final: prev: {
  papilo = prev.papilo.overrideAttrs (old: {
    version = "2.4.0";
    src = prev.pkgs.fetchFromGitHub {
      owner = "scipopt";
      repo = "papilo";
      rev = "v2.4.0";
      sha256 = "sha256-WMw9v57nuP6MHj9Ft4l5FxdIF5VUWCRm/909tbz7VD4=";
    };
    propagatedBuildInputs = with prev.pkgs; [
      tbb_2022
    ];
  });
}
