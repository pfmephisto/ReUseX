// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/exif.hpp"
#include "core/logging.hpp"

#include <exiv2/exiv2.hpp>

#include <ctime>
#include <sstream>

namespace reusex::io {

double read_exif_timestamp(const std::filesystem::path &jpeg_path) {
  try {
    auto image = Exiv2::ImageFactory::open(jpeg_path.string());
    if (!image)
      return -1.0;

    image->readMetadata();
    const auto &exifData = image->exifData();

    if (exifData.empty())
      return -1.0;

    // Try DateTimeOriginal first, then DateTimeDigitized, then DateTime
    const char *keys[] = {"Exif.Photo.DateTimeOriginal",
                          "Exif.Photo.DateTimeDigitized",
                          "Exif.Image.DateTime"};

    for (const auto *key : keys) {
      auto it = exifData.findKey(Exiv2::ExifKey(key));
      if (it == exifData.end())
        continue;

      std::string dateStr = it->toString();
      if (dateStr.empty() || dateStr.size() < 19)
        continue;

      // Parse "YYYY:MM:DD HH:MM:SS" format
      std::tm tm = {};
      std::istringstream ss(dateStr);
      ss >> std::get_time(&tm, "%Y:%m:%d %H:%M:%S");
      if (ss.fail())
        continue;

      // Convert to epoch seconds using mktime (local time)
      tm.tm_isdst = -1; // Let mktime determine DST
      std::time_t epoch = std::mktime(&tm);
      if (epoch == -1)
        continue;

      core::debug("EXIF timestamp from {}: {} -> {:.0f}", jpeg_path.filename().string(),
                  dateStr, static_cast<double>(epoch));
      return static_cast<double>(epoch);
    }

    core::debug("No EXIF timestamp found in {}", jpeg_path.filename().string());
    return -1.0;

  } catch (const Exiv2::Error &e) {
    core::warn("Failed to read EXIF from {}: {}", jpeg_path.string(),
               e.what());
    return -1.0;
  }
}

} // namespace reusex::io
