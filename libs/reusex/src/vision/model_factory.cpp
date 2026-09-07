// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "vision/model_factory.hpp"

#include "vision/BackendFactory.hpp"

namespace reusex::vision {

std::unique_ptr<IModel>
create_model_from_path(const std::filesystem::path &modelPath, bool use_cuda) {
  // Mirrors the creation sequence in vision::annotate; runs here (inside the
  // vision module) so the backend compile-defines are in scope.
  const auto backend_type = BackendFactory::detect_backend(modelPath);
  auto backend = BackendFactory::create(backend_type);
  const auto model_type = BackendFactory::detect_model(modelPath);
  return backend->create_model(model_type, modelPath, use_cuda);
}

} // namespace reusex::vision
