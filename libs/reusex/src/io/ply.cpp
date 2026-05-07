// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/ply.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

#include <algorithm>
#include <fmt/format.h>
#include <stdexcept>
#include <string>

namespace reusex::io {

static bool has_field(const pcl::PCLPointCloud2 &cloud2, const std::string &name) {
    return std::any_of(cloud2.fields.begin(), cloud2.fields.end(),
        [&](const auto &f) { return f.name == name; });
}

void import_ply(ProjectDB &db, const std::filesystem::path &ply_path) {
    core::info("Importing PLY file: {}", ply_path.string());

    int logId = db.log_pipeline_start("import_ply",
        fmt::format(R"({{"source":"{}"}})", ply_path.string()));

    try {
        pcl::PCLPointCloud2 cloud2;
        if (pcl::io::loadPLYFile(ply_path.string(), cloud2) < 0) {
            throw std::runtime_error("Failed to load PLY file: " + ply_path.string());
        }

        if (cloud2.width == 0 || cloud2.height == 0) {
            throw std::runtime_error("PLY file contains no points: " + ply_path.string());
        }

        size_t pointCount = static_cast<size_t>(cloud2.width) * cloud2.height;
        core::info("PLY loaded: {} points", pointCount);

        // Log which fields were found
        std::string fieldList;
        for (const auto &f : cloud2.fields) {
            if (!fieldList.empty()) fieldList += ", ";
            fieldList += f.name;
        }
        core::debug("PLY fields: {}", fieldList);

        bool hasXYZ   = has_field(cloud2, "x") && has_field(cloud2, "y") && has_field(cloud2, "z");
        bool hasRgb   = has_field(cloud2, "rgb") || has_field(cloud2, "rgba");
        bool hasRgbSep = has_field(cloud2, "r") && has_field(cloud2, "g") && has_field(cloud2, "b");
        bool hasColor = hasRgb || hasRgbSep;
        bool hasNormals = (has_field(cloud2, "normal_x") || has_field(cloud2, "nx")) &&
                          (has_field(cloud2, "normal_y") || has_field(cloud2, "ny")) &&
                          (has_field(cloud2, "normal_z") || has_field(cloud2, "nz"));
        bool hasLabel = has_field(cloud2, "label");
        bool hasIntensity = has_field(cloud2, "intensity");

        if (!hasXYZ) {
            throw std::runtime_error("PLY file has no XYZ position fields: " + ply_path.string());
        }

        std::string cloudName = ply_path.stem().string();
        std::string params = fmt::format(R"({{"source":"{}"}})", ply_path.string());

        // ── XYZRGB cloud (always saved) ──────────────────────────────────────
        Cloud cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        // If no color but intensity exists, map intensity to grayscale RGB
        if (!hasColor && hasIntensity) {
            pcl::PointCloud<pcl::PointXYZI> xyzi_cloud;
            pcl::fromPCLPointCloud2(cloud2, xyzi_cloud);

            // Find intensity range for normalization
            float iMin = std::numeric_limits<float>::max();
            float iMax = std::numeric_limits<float>::lowest();
            for (const auto &pt : xyzi_cloud) {
                if (std::isfinite(pt.intensity)) {
                    iMin = std::min(iMin, pt.intensity);
                    iMax = std::max(iMax, pt.intensity);
                }
            }
            float iRange = (iMax > iMin) ? (iMax - iMin) : 1.0f;

            for (size_t i = 0; i < cloud.size(); ++i) {
                if (i < xyzi_cloud.size() && std::isfinite(xyzi_cloud[i].intensity)) {
                    auto gray = static_cast<uint8_t>(
                        std::clamp((xyzi_cloud[i].intensity - iMin) / iRange * 255.0f, 0.0f, 255.0f));
                    cloud[i].r = cloud[i].g = cloud[i].b = gray;
                } else {
                    cloud[i].r = cloud[i].g = cloud[i].b = 128;
                }
            }
        }

        core::info("Saving XYZRGB cloud '{}' ({} points)", cloudName, cloud.size());
        db.save_point_cloud(cloudName, cloud, "import_ply", params);

        // ── Normals cloud ─────────────────────────────────────────────────────
        if (hasNormals) {
            CloudN normals;
            pcl::fromPCLPointCloud2(cloud2, normals);

            // Verify normals are non-trivial (not all zero)
            bool hasData = std::any_of(normals.begin(), normals.end(), [](const NormalT &n) {
                return std::isfinite(n.normal_x) &&
                       (n.normal_x != 0.0f || n.normal_y != 0.0f || n.normal_z != 0.0f);
            });

            if (hasData) {
                std::string normalsName = cloudName + "_normals";
                core::info("Saving normals cloud '{}' ({} normals)", normalsName, normals.size());
                db.save_point_cloud(normalsName, normals, "import_ply", params);
            }
        }

        // ── Label cloud ───────────────────────────────────────────────────────
        if (hasLabel) {
            CloudL labels;
            pcl::fromPCLPointCloud2(cloud2, labels);

            // Check if any non-zero labels exist
            bool hasData = std::any_of(labels.begin(), labels.end(),
                [](const LabelT &l) { return l.label != 0; });

            if (hasData) {
                std::string labelsName = cloudName + "_labels";
                core::info("Saving label cloud '{}' ({} labels)", labelsName, labels.size());
                db.save_point_cloud(labelsName, labels, "import_ply", params);
            }
        }

        db.log_pipeline_end(logId, true);
    } catch (...) {
        db.log_pipeline_end(logId, false, "import_ply failed");
        throw;
    }

    core::info("PLY import complete");
}

// ── Export ────────────────────────────────────────────────────────────────────

void export_ply(const std::filesystem::path &path,
                const Cloud &cloud,
                const CloudN *normals) {
    core::info("Exporting PLY to: {} ({} points)", path.string(), cloud.size());

    int result = 0;

    if (normals == nullptr) {
        // No normals: write XYZRGB cloud directly
        result = pcl::io::savePLYFileBinary(path.string(), cloud);
    } else {
        if (normals->size() != cloud.size()) {
            throw std::runtime_error(
                fmt::format("export_ply: normals size ({}) != cloud size ({})",
                            normals->size(), cloud.size()));
        }
        // Merge into PointXYZRGBNormal for a single-pass write
        pcl::PointCloud<pcl::PointXYZRGBNormal> combined;
        combined.resize(cloud.size());
        combined.width  = cloud.width;
        combined.height = cloud.height;
        for (size_t i = 0; i < cloud.size(); ++i) {
            combined[i].x       = cloud[i].x;
            combined[i].y       = cloud[i].y;
            combined[i].z       = cloud[i].z;
            combined[i].r       = cloud[i].r;
            combined[i].g       = cloud[i].g;
            combined[i].b       = cloud[i].b;
            combined[i].a       = cloud[i].a;
            combined[i].normal_x = (*normals)[i].normal_x;
            combined[i].normal_y = (*normals)[i].normal_y;
            combined[i].normal_z = (*normals)[i].normal_z;
            combined[i].curvature = (*normals)[i].curvature;
        }
        result = pcl::io::savePLYFileBinary(path.string(), combined);
    }

    if (result < 0) {
        throw std::runtime_error("Failed to write PLY file: " + path.string());
    }

    core::info("PLY export complete: {}", path.string());
}

} // namespace reusex::io
