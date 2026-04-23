// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <filesystem>

namespace reusex::io {

/// Read EXIF DateTimeOriginal from a JPEG file.
/// Returns epoch seconds (double), or -1.0 if not available.
double read_exif_timestamp(const std::filesystem::path &jpeg_path);

} // namespace reusex::io
