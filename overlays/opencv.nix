# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# Slimmed OpenCV. Only the modules something in the closure actually links are
# built (BUILD_LIST); OpenCV resolves each listed module's internal deps itself.
#
#   ReUseX       core imgproc imgcodecs highgui features2d calib3d stitching
#   rtabmap      + photo video videoio objdetect aruco xfeatures2d flann
#                  (+ cudafeatures2d cudaoptflow cudaimgproc when CUDA is on)
#   trtsam3      core imgcodecs imgproc videoio
#   openvino     gapi (NPU protopipe tool)
#   OpenMVS      links a blanket ${OpenCV_LIBS}; its only contrib use
#                (ximgproc in SemiGlobalMatcher.cpp) is behind the disabled
#                _USE_FILTER_DEMO define.
#
# Add a module here (not a whole feature flag) if a consumer starts needing one.
_: _final: prev: let
  cudaSupport = prev.config.cudaSupport or false;
in {
  opencv =
    (prev.opencv.override {
      # enableGtk2 was dropped upstream (nixpkgs removed the arg).
      # GTK3 is off: highgui builds headless. The only imshow/waitKey calls in
      # ReUseX are under #ifndef NDEBUG (vision/annotate.cpp, IDataset.cpp) and
      # would throw "The function is not implemented" in a Debug build. To get
      # preview windows back for local debugging, set enableGtk3 = true.
      enableGtk3 = false;
      # VTK off: drops OpenCV's viz module (unused) and the second VTK build it
      # pulled into the closure; PCL brings its own VTK.
      enableVtk = false;
      enableTbb = true;
      # tbb = prev.tbb_2022;
      # enableCudnn is left at the nixpkgs default (false): cuDNN only feeds
      # cv::dnn, which nothing here builds or uses.
      # Kept so the opencv python bindings (cv2) stay available.
      enablePython = true;
      # Needed for rtabmap's SURF/SIFT via xfeatures2d nonfree.
      enableUnfree = true;
      # Contrib stays on (aruco, xfeatures2d, cuda*) but is restricted by
      # enabledModules below.
      enableContrib = true;
      # The accuracy-test binaries (package_tests output) are never run by us
      # and are a large share of the build; they also need the `ts` module.
      runAccuracyTests = false;
      enabledModules =
        [
          "core"
          "imgproc"
          "imgcodecs"
          "highgui"
          "calib3d"
          "features2d"
          "flann"
          "stitching"
          "photo"
          "video"
          "videoio"
          "objdetect"
          "aruco"
          "xfeatures2d"
          # openvino (a transitive dep via the ML backends) builds its NPU
          # `protopipe` tool against opencv_gapi. Its find_package(OpenCV
          # COMPONENTS gapi) is QUIET and only checks OpenCV_VERSION, so a
          # missing gapi is a compile error rather than a skipped tool.
          "gapi"
          # cv2 python bindings
          "python3"
          "python_bindings_generator"
        ]
        ++ prev.lib.optionals cudaSupport [
          # core refuses to configure with WITH_CUDA unless cudev is built, and
          # BUILD_LIST does not pull it in on its own.
          "cudev"
          "cudafeatures2d"
          "cudaoptflow"
          "cudaimgproc"
        ];
    })
    .overrideAttrs (_old: {
      # Upstream also imports cv2.sfm, which BUILD_LIST no longer builds.
      pythonImportsCheck = ["cv2"];
    });
}
