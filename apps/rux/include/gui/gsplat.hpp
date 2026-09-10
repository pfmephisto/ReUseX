// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Serving the Gaussian splats stored in a project (#322).
//
// `rux create gsplat` stores its trained model in the `.rux` project (schema
// v12), exactly as `rux create mesh` stores a mesh, so these routes are
// ordinary ProjectDB reads and follow the mesh routes shape for shape:
// a listing, one record, and the blob.
//
// Framework-free like the rest of gui/api.hpp: these functions take a
// ProjectDB and return JSON or bytes, and throw HttpError to signal failure.

#include "gui/api.hpp"

#include <string>
#include <string_view>

namespace reusex {
class ProjectDB;
}

namespace rux::gui {

/// Upper bound on a splat blob this server will serve in one response.
///
/// The body is materialized in memory (as the mesh-blob route already is), so
/// this is a real ceiling rather than a formality: 1 GiB is roughly four
/// million Gaussians at spherical-harmonic degree 3. Past it the honest answer
/// is 413 rather than an out-of-memory server.
inline constexpr uint64_t kMaxGsplatBytes = 1024ull * 1024ull * 1024ull;

/// `GET /api/v1/gsplats` — every splat stored in the project, paged.
///
/// Metadata only. The `byte_size` in each row is what makes the client's
/// decision informed: unlike a cloud, which streams in pages, a splat is one
/// response of hundreds of megabytes, so the viewport does not fetch it until
/// the user switches the layer on.
nlohmann::json gsplats_json(const reusex::ProjectDB &db, const Params &params);

/// `GET /api/v1/gsplats/{name}` — one splat's metadata.
/// @throws HttpError(404) when no splat is stored under @p name.
nlohmann::json gsplat_json(const reusex::ProjectDB &db, std::string_view name);

/// `GET /api/v1/gsplats/{name}/data` — the INRIA `.ply` bytes, verbatim.
///
/// @throws HttpError(404) when no splat is stored under @p name.
/// @throws HttpError(413) when it exceeds kMaxGsplatBytes.
Blob gsplat_blob(const reusex::ProjectDB &db, std::string_view name);

} // namespace rux::gui
