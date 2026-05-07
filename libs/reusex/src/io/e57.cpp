// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/e57.hpp"
#include "core/ProjectDB.hpp"
#include "core/logging.hpp"

#include <E57Format/E57SimpleData.h>
#include <E57Format/E57SimpleReader.h>
#include <E57Format/E57SimpleWriter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fmt/format.h>
#include <stdexcept>
#include <vector>

namespace reusex::io {

static constexpr size_t CHUNK_SIZE = 65536;

static std::array<double, 12> pose_to_matrix(const e57::RigidBodyTransform &t) {
    double w = t.rotation.w, x = t.rotation.x, y = t.rotation.y, z = t.rotation.z;
    return {
        1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),     t.translation.x,
        2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),     t.translation.y,
        2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y), t.translation.z,
    };
}

static void apply_pose(const std::array<double, 12> &T, float &px, float &py, float &pz) {
    float tx = static_cast<float>(T[0]*px + T[1]*py + T[2]*pz + T[3]);
    float ty = static_cast<float>(T[4]*px + T[5]*py + T[6]*pz + T[7]);
    float tz = static_cast<float>(T[8]*px + T[9]*py + T[10]*pz + T[11]);
    px = tx; py = ty; pz = tz;
}

static bool is_identity_pose(const e57::RigidBodyTransform &t) {
    return t.rotation.w == 1.0 && t.rotation.x == 0.0 &&
           t.rotation.y == 0.0 && t.rotation.z == 0.0 &&
           t.translation.x == 0.0 && t.translation.y == 0.0 &&
           t.translation.z == 0.0;
}

void import_e57(ProjectDB &db, const std::filesystem::path &e57_path) {
    core::info("Importing E57 file: {}", e57_path.string());

    int logId = db.log_pipeline_start("import_e57",
        fmt::format(R"({{"source":"{}"}})", e57_path.string()));

    try {
        e57::Reader reader(e57_path.string(), {});

        int64_t scanCount = reader.GetData3DCount();
        core::info("E57 file contains {} scan(s)", scanCount);

        std::string stem = e57_path.stem().string();

        for (int64_t scanIdx = 0; scanIdx < scanCount; ++scanIdx) {
            e57::Data3D header;
            if (!reader.ReadData3D(scanIdx, header)) {
                core::warn("Scan {}: failed to read header, skipping", scanIdx);
                continue;
            }

            const auto &fields = header.pointFields;
            if (!fields.cartesianXField || !fields.cartesianYField || !fields.cartesianZField) {
                core::warn("Scan {}: no Cartesian XYZ fields, skipping", scanIdx);
                continue;
            }

            int64_t rowMax = 0, colMax = 0, pointsSize = 0, groupsSize = 0, countSize = 0;
            bool colIndex = false;
            reader.GetData3DSizes(scanIdx, rowMax, colMax, pointsSize, groupsSize, countSize, colIndex);

            if (pointsSize == 0) {
                core::warn("Scan {}: 0 points, skipping", scanIdx);
                continue;
            }

            bool hasColor = fields.colorRedField && fields.colorGreenField && fields.colorBlueField;
            bool hasNormals = fields.normalXField && fields.normalYField && fields.normalZField;
            bool hasIntensity = fields.intensityField;

            // Derive cloud name from E57 header or file stem
            std::string cloudName;
            if (!header.name.empty()) {
                cloudName = header.name;
            } else if (scanCount == 1) {
                cloudName = stem;
            } else {
                cloudName = fmt::format("{}_scan_{}", stem, scanIdx);
            }

            core::info("Scan {} '{}': {} points  color={} normals={} intensity={}",
                scanIdx, cloudName, pointsSize, hasColor, hasNormals, hasIntensity);

            // Color normalization scales
            double scaleR = 1.0, scaleG = 1.0, scaleB = 1.0;
            if (hasColor) {
                double maxR = header.colorLimits.colorRedMaximum > 0
                    ? header.colorLimits.colorRedMaximum : 255.0;
                double maxG = header.colorLimits.colorGreenMaximum > 0
                    ? header.colorLimits.colorGreenMaximum : 255.0;
                double maxB = header.colorLimits.colorBlueMaximum > 0
                    ? header.colorLimits.colorBlueMaximum : 255.0;
                scaleR = 255.0 / maxR;
                scaleG = 255.0 / maxG;
                scaleB = 255.0 / maxB;
            }
            double intensityScale = 1.0;
            if (hasIntensity && header.intensityLimits.intensityMaximum > 0) {
                intensityScale = 255.0 / header.intensityLimits.intensityMaximum;
            }

            // Pose transform
            auto poseMat = pose_to_matrix(header.pose);
            bool applyPose = !is_identity_pose(header.pose);

            // Per-chunk buffers
            size_t chunkSize = std::min(static_cast<int64_t>(CHUNK_SIZE), pointsSize);

            std::vector<float> xs(chunkSize), ys(chunkSize), zs(chunkSize);
            std::vector<int8_t> invalid(chunkSize, 0);
            std::vector<uint16_t> rs(chunkSize), gs(chunkSize), bs(chunkSize);
            std::vector<double> intensities(chunkSize);
            std::vector<float> nxs(chunkSize), nys(chunkSize), nzs(chunkSize);

            e57::Data3DPointsFloat bufs;
            bufs.cartesianX = xs.data();
            bufs.cartesianY = ys.data();
            bufs.cartesianZ = zs.data();
            bufs.cartesianInvalidState = invalid.data();
            if (hasColor) {
                bufs.colorRed   = rs.data();
                bufs.colorGreen = gs.data();
                bufs.colorBlue  = bs.data();
            }
            if (hasIntensity) {
                bufs.intensity = intensities.data();
            }
            if (hasNormals) {
                bufs.normalX = nxs.data();
                bufs.normalY = nys.data();
                bufs.normalZ = nzs.data();
            }

            Cloud cloud;
            cloud.reserve(static_cast<size_t>(pointsSize));
            CloudN normals_cloud;
            if (hasNormals)
                normals_cloud.reserve(static_cast<size_t>(pointsSize));

            auto cvReader = reader.SetUpData3DPointsData(scanIdx, chunkSize, bufs);
            unsigned nRead = 0;
            while ((nRead = cvReader.read()) > 0) {
                for (unsigned i = 0; i < nRead; ++i) {
                    if (invalid[i] != 0)
                        continue;

                    float px = xs[i], py = ys[i], pz = zs[i];
                    if (applyPose)
                        apply_pose(poseMat, px, py, pz);

                    PointT pt;
                    pt.x = px; pt.y = py; pt.z = pz;

                    if (hasColor) {
                        pt.r = static_cast<uint8_t>(std::clamp(rs[i] * scaleR, 0.0, 255.0));
                        pt.g = static_cast<uint8_t>(std::clamp(gs[i] * scaleG, 0.0, 255.0));
                        pt.b = static_cast<uint8_t>(std::clamp(bs[i] * scaleB, 0.0, 255.0));
                    } else if (hasIntensity) {
                        auto gray = static_cast<uint8_t>(
                            std::clamp(intensities[i] * intensityScale, 0.0, 255.0));
                        pt.r = pt.g = pt.b = gray;
                    } else {
                        pt.r = pt.g = pt.b = 128;
                    }

                    cloud.push_back(pt);

                    if (hasNormals) {
                        NormalT n;
                        n.normal_x = nxs[i];
                        n.normal_y = nys[i];
                        n.normal_z = nzs[i];
                        normals_cloud.push_back(n);
                    }
                }
            }
            cvReader.close();

            core::info("Scan {}: {} valid points stored as '{}'", scanIdx, cloud.size(), cloudName);

            std::string params = fmt::format(
                R"({{"source":"{}","scan_index":{}}})", e57_path.string(), scanIdx);

            db.save_point_cloud(cloudName, cloud, "import_e57", params);

            if (hasNormals && !normals_cloud.empty()) {
                db.save_point_cloud(cloudName + "_normals", normals_cloud, "import_e57", params);
            }
        }

        db.log_pipeline_end(logId, true);
    } catch (...) {
        db.log_pipeline_end(logId, false, "import_e57 failed");
        throw;
    }

    core::info("E57 import complete");
}

// ── Export ────────────────────────────────────────────────────────────────────

void export_e57(const std::filesystem::path &path,
                const std::vector<E57ScanExport> &scans) {
    if (scans.empty()) {
        throw std::runtime_error("export_e57: no scans provided");
    }

    core::info("Exporting {} scan(s) to E57: {}", scans.size(), path.string());

    e57::Writer writer(path.string(), e57::WriterOptions{});

    for (const auto &scan : scans) {
        if (!scan.cloud || scan.cloud->empty()) {
            core::warn("Scan '{}' is empty, skipping", scan.name);
            continue;
        }

        const size_t N = scan.cloud->size();
        bool hasNormals = (scan.normals != nullptr && scan.normals->size() == N);

        core::info("Writing scan '{}': {} points  normals={}", scan.name, N, hasNormals);

        // ── Deinterleave cloud into separate arrays ───────────────────────────
        std::vector<float> xs(N), ys(N), zs(N);
        std::vector<uint16_t> rs(N), gs(N), bs(N);
        std::vector<float> nxs, nys, nzs;
        if (hasNormals) {
            nxs.resize(N); nys.resize(N); nzs.resize(N);
        }

        for (size_t i = 0; i < N; ++i) {
            const auto &pt = (*scan.cloud)[i];
            xs[i] = pt.x; ys[i] = pt.y; zs[i] = pt.z;
            // Store RGB as uint16 in [0, 255] range
            rs[i] = pt.r; gs[i] = pt.g; bs[i] = pt.b;
        }
        if (hasNormals) {
            for (size_t i = 0; i < N; ++i) {
                const auto &n = (*scan.normals)[i];
                nxs[i] = n.normal_x; nys[i] = n.normal_y; nzs[i] = n.normal_z;
            }
        }

        // ── Build Data3D header ───────────────────────────────────────────────
        e57::Data3D header;
        header.name = scan.name;
        header.pointCount = static_cast<size_t>(N);

        header.pointFields.cartesianXField = true;
        header.pointFields.cartesianYField = true;
        header.pointFields.cartesianZField = true;
        header.pointFields.colorRedField   = true;
        header.pointFields.colorGreenField = true;
        header.pointFields.colorBlueField  = true;
        header.colorLimits.colorRedMinimum   = 0;
        header.colorLimits.colorRedMaximum   = 255;
        header.colorLimits.colorGreenMinimum = 0;
        header.colorLimits.colorGreenMaximum = 255;
        header.colorLimits.colorBlueMinimum  = 0;
        header.colorLimits.colorBlueMaximum  = 255;

        if (hasNormals) {
            header.pointFields.normalXField = true;
            header.pointFields.normalYField = true;
            header.pointFields.normalZField = true;
        }

        // ── Fill buffers and write ────────────────────────────────────────────
        e57::Data3DPointsFloat bufs;
        bufs.cartesianX = xs.data();
        bufs.cartesianY = ys.data();
        bufs.cartesianZ = zs.data();
        bufs.colorRed   = rs.data();
        bufs.colorGreen = gs.data();
        bufs.colorBlue  = bs.data();
        if (hasNormals) {
            bufs.normalX = nxs.data();
            bufs.normalY = nys.data();
            bufs.normalZ = nzs.data();
        }

        writer.WriteData3DData(header, bufs);
    }

    writer.Close();
    core::info("E57 export complete: {}", path.string());
}

} // namespace reusex::io
