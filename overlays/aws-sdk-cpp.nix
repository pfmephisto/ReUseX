# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: MIT
#
# S3-only build of the AWS SDK for C++ used by the ruxd service worker. The
# upstream package builds ~300 service clients; restricting `apis` to s3 keeps
# the closure and build small while still speaking plain S3 against any
# S3-compatible endpoint (MinIO/Ceph/etc.) via an endpoint override.
{...}: final: prev: {
  aws-sdk-cpp-s3 = prev.aws-sdk-cpp.override {
    apis = ["s3"];
  };
}
