// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/gsplat.hpp"

#include <reusex/core/ProjectDB.hpp>

#include <string>
#include <utility>

namespace rux::gui {
namespace {

using json = nlohmann::json;

json metadata_json(const reusex::ProjectDB::GaussianSplatMetadata &metadata) {
  return json{{"name", metadata.name},
              {"format", metadata.format},
              {"gaussian_count", metadata.gaussian_count},
              {"sh_degree", metadata.sh_degree},
              {"byte_size", metadata.byte_size},
              {"stage", metadata.stage},
              {"parameters", metadata.parameters},
              {"created_at", metadata.created_at}};
}

} // namespace

json gsplats_json(const reusex::ProjectDB &db, const Params &) {
  // Deliberately unpaged, unlike /clouds and /meshes. A project holds a
  // handful of splats — one per training run someone chose to keep — not a
  // number that grows with the size of the scan, so paging would be ceremony
  // with no bound to enforce. The same reasoning as /meshes/{name}/textures.
  // The `Params` argument stays for symmetry with the other collections and so
  // that adding a filter later is not a signature change.
  json list = json::array();
  for (const auto &name : db.list_gaussian_splats())
    list.push_back(metadata_json(db.gaussian_splat_metadata(name)));
  return json{{"gsplats", std::move(list)}};
}

json gsplat_json(const reusex::ProjectDB &db, std::string_view name) {
  const std::string key(name);
  if (!db.has_gaussian_splat(key))
    throw HttpError(404, "no such gsplat '" + key +
                             "'; train one with `rux create gsplat`, or bring "
                             "an existing .ply in with `rux import gsplat`");
  return metadata_json(db.gaussian_splat_metadata(key));
}

Blob gsplat_blob(const reusex::ProjectDB &db, std::string_view name) {
  const std::string key(name);
  if (!db.has_gaussian_splat(key))
    throw HttpError(404, "no such gsplat '" + key + "'");

  // Checked from the metadata rather than after loading: refusing a 2 GB splat
  // only once it is already in memory is the failure the limit exists to
  // prevent.
  const auto metadata = db.gaussian_splat_metadata(key);
  if (metadata.byte_size > kMaxGsplatBytes)
    throw HttpError(413, "gsplat '" + key + "' exceeds the " +
                             std::to_string(kMaxGsplatBytes / (1024 * 1024)) +
                             " MiB response limit; retrain with a smaller "
                             "Gaussian budget (--mcmc-cap / --max-points)");

  Blob blob;
  blob.data = db.gaussian_splat_blob(key);
  // Not "application/ply": there is no registered media type for it, and the
  // renderer is told the format explicitly by the client rather than sniffing
  // this header.
  blob.content_type = "application/octet-stream";
  return blob;
}

} // namespace rux::gui
