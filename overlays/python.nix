# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
{...}: final: prev: rec {
  python3 = prev.python3.override {
    packageOverrides = pyfinal: pyprev: {
      specklepy = prev.pkgs.specklepy;
      spdlog = prev.pkgs.spdlog-python;

      # Required for custom sam3 package
      regex = pyprev.regex.overridePythonAttrs (old: rec {
        version = "2025.11.3";
        src = prev.fetchFromGitHub {
          owner = "mrabarnett";
          repo = "mrab-regex";
          tag = "2025.11.3";
          hash = "sha256-KEn+8DoAAq2OBqnl7vluqn1UPBpIfmO1v4wxKUZrcyA=";
        };

        checkPhase = '''';

        pythonImportsCheck = [];
      });

      # Includes SAM3 WIP
      ultralytics = pyprev.ultralytics.overridePythonAttrs (old: {
        version = "414c048";

        src = prev.fetchFromGitHub {
          owner = "ultralytics";
          repo = "ultralytics";
          rev = "414c0482c20fb3f7c9df283abccf548f5efaa235";
          hash = "sha256-eRuuMU1YqpvQOxWzWDnuhc9gC+OGvCO+2HG3zlqnb10=";
        };

        disabledTests = [
          # also remove the individual tests that require internet
          "test_all_model_yamls"
          "test_data_annotator"
          "test_labels_and_crops"
          "test_model_embeddings"
          "test_model_methods"
          "test_predict_callback_and_setup"
          "test_predict_grey_and_4ch"
          "test_predict_img"
          "test_predict_txt"
          "test_predict_visualize"
          "test_results"
          "test_train_pretrained"
          "test_train_scratch"
          "test_utils_torchutils"
          "test_val"
          "test_workflow"
          "test_yolo_world"
          "test_yolov10"
          "test_yoloe"
          "test_multichannel"
          "test_grayscale"
          "test_predict_gray_and_4ch"
        ];
      });
    };
  };
  python3Packages = python3.pkgs;
}
