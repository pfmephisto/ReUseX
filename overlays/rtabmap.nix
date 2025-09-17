{...}: findal: prev: {
  rtabmap = prev.rtabmap.overrideAttrs (old: {
    buildInputs =
      (old.buildInputs or [])
      ++ (with prev.pkgs; [
        tbb_2022
        gtsam
      ]);

    cmakeFlags =
      (prev.rtabmap.cmakeFlags or [])
      ++ [
        "-DWITH_TORCH=ON"
        "-DWITH_GTSAM=ON"
        #"-DWITH_PYTHON=ON"
        #"-DWITH_PYTHON_THREADING=ON"
        "-DWITH_LIBLAS=ON"
        "-DWITH_CERES=ON"
        "-DWITH_CVSBA=ON"
        "-DWITH_CCCORELIB=ON"
        "-DWITH_LOAM=ON"
        "-DWITH_FLOAM=ON"
        "-DWITH_DEPTHAI=ON"
        "-DWITH_XVSDK=ON"
        "-DWITH_GRIDMAP=ON"
        "-DWITH_CPUTSDF=ON"
        "-DWITH_OPENCHISEL=ON"
        "-DWITH_ALICE_VISION=ON"
        "-DWITH_FOVIS=ON"
        "-DWITH_VISO2=ON"
        "-DWITH_DVO=ON"
        "-DWITH_ORB_SLAM=ON"
        "-DWITH_OKVIS=ON"
        "-DWITH_MSCKF_VIO=ON"
        "-DWITH_VINS=ON"
        "-DWITH_OPENVINS=ON"
      ];
  });
}
