// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSCredentials.h>
#include <aws/s3/S3Client.h>
#include <aws/s3/S3ClientConfiguration.h>
#include <aws/s3/model/ListBucketsRequest.h>

namespace ruxd {

// --- AwsApiGuard -----------------------------------------------------------

struct AwsApiGuard::Impl {
  Aws::SDKOptions options;
};

AwsApiGuard::AwsApiGuard() : impl_(std::make_unique<Impl>()) {
  Aws::InitAPI(impl_->options);
}

AwsApiGuard::~AwsApiGuard() { Aws::ShutdownAPI(impl_->options); }

// --- S3Client --------------------------------------------------------------

struct S3Client::Impl {
  bool configured = false;
  std::unique_ptr<Aws::S3::S3Client> client;
};

S3Client::S3Client(const Config &cfg) : impl_(std::make_unique<Impl>()) {
  // Treat S3 as configured once an endpoint or credentials are supplied.
  impl_->configured = !cfg.s3_endpoint.empty() || !cfg.s3_access_key.empty();
  if (!impl_->configured) {
    return;
  }

  Aws::S3::S3ClientConfiguration client_cfg;
  client_cfg.region = cfg.s3_region;
  if (!cfg.s3_endpoint.empty()) {
    client_cfg.endpointOverride = cfg.s3_endpoint;
  }
  // Path-style addressing (http://host/bucket/key) is required by MinIO/Ceph;
  // virtual-hosted style (http://bucket.host/key) is the AWS default.
  client_cfg.useVirtualAddressing = !cfg.s3_path_style;

  Aws::Auth::AWSCredentials creds(cfg.s3_access_key, cfg.s3_secret_key);
  impl_->client = std::make_unique<Aws::S3::S3Client>(
      creds, /*endpointProvider=*/nullptr, client_cfg);
}

S3Client::~S3Client() = default;

bool S3Client::is_configured() const { return impl_->configured; }

PingResult S3Client::ping() const {
  if (!impl_->configured) {
    return {false, "not configured"};
  }
  const auto outcome = impl_->client->ListBuckets();
  if (outcome.IsSuccess()) {
    return {true, "ok"};
  }
  return {false, outcome.GetError().GetMessage()};
}

} // namespace ruxd
