// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Backend/model creation from a filesystem path, compiled INSIDE the vision
// module. BackendFactory gates each backend behind a compile define
// (REUSEX_USE_TENSORRT, ...) that is PRIVATE to the vision library, so callers
// outside the library (apps, tests) must not invoke BackendFactory directly —
// they would see every backend as "not compiled". This entry point does the
// creation where the defines are set and returns a ready IModel.

#pragma once

#include "reusex/vision/IModel.hpp"

#include <filesystem>
#include <memory>

namespace reusex::vision {

/// Detect the backend + model kind from @p modelPath and build the model.
/// @throws std::runtime_error if the required backend is not compiled in.
std::unique_ptr<IModel>
create_model_from_path(const std::filesystem::path &modelPath, bool use_cuda);

} // namespace reusex::vision
