// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/api.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/component_record.hpp>
#include <reusex/core/stages.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/core/version.hpp>
#include <reusex/types/point_types.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cstring>
#include <exception>
#include <utility>

namespace rux::gui {
namespace {

using json = nlohmann::json;
namespace pipeline = reusex::pipeline;

/// Re-parse a stored parameter blob so `parameters` is a JSON *object* on the
/// wire rather than an escaped string. An unparseable blob degrades to {}
/// rather than corrupting the response.
json parameters_object(const std::string &raw) {
  if (raw.empty())
    return json::object();
  auto parsed = json::parse(raw, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    return json::object();
  return parsed;
}

/// Machine-readable token for a progress phase.
///
/// core::to_string(Stage) is a *display* string ("Region Growing") intended for
/// a terminal progress bar. Putting that on the wire would force every client
/// to string-match prose, so it is slugified here and the display form is
/// shipped alongside it as `stage_label`.
std::string stage_token(reusex::core::Stage stage) {
  std::string token(reusex::core::to_string(stage));
  for (char &c : token) {
    if (c == ' ' || c == '-')
      c = '_';
    else
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return token;
}

[[noreturn]] void not_found(std::string_view what, std::string_view which) {
  throw HttpError(
      404,
      std::string("no such ").append(what).append(" '").append(which) + "'");
}

json pose_array(const std::array<double, 16> &pose) {
  json out = json::array();
  for (double value : pose)
    out.push_back(value);
  return out;
}

json intrinsics_json(const reusex::core::SensorIntrinsics &k) {
  return json{{"fx", k.fx},
              {"fy", k.fy},
              {"cx", k.cx},
              {"cy", k.cy},
              {"width", k.width},
              {"height", k.height},
              {"local_transform", pose_array(k.local_transform)}};
}

json cloud_info_json(const reusex::ProjectDB::ProjectSummary::CloudInfo &info) {
  json out{{"name", info.name},
           {"type", info.type},
           {"point_count", info.point_count},
           {"width", info.width},
           {"height", info.height},
           {"organized", info.organized}};
  if (!info.labels.empty()) {
    json labels = json::object();
    for (const auto &[id, name] : info.labels)
      labels[std::to_string(id)] = name;
    out["labels"] = std::move(labels);
  }
  return out;
}

json material_info_json(
    const reusex::ProjectDB::ProjectSummary::MaterialInfo &info) {
  return json{{"id", info.id},
              {"guid", info.guid},
              {"property_count", info.property_count},
              {"created_at", info.created_at},
              {"version_number", info.version_number}};
}

json project_info_json(
    const reusex::ProjectDB::ProjectSummary::ProjectInfo &info) {
  return json{{"id", info.id},
              {"name", info.name},
              {"building_address", info.building_address},
              {"year_of_construction", info.year_of_construction},
              {"survey_date", info.survey_date},
              {"survey_organisation", info.survey_organisation},
              {"notes", info.notes}};
}

/// PNG-encode a matrix, normalizing the types PNG cannot carry.
///
/// Depth arrives as CV_32F metres; PNG has no float channel, so it is scaled
/// to millimetres in CV_16U. That is lossy and the client must know the scale —
/// which is why the contract spells it out (docs/gui/openapi.yaml).
Blob encode_png(const cv::Mat &image, std::string_view what) {
  if (image.empty())
    throw HttpError(404, std::string("no ").append(what) + " stored");

  cv::Mat encodable = image;
  if (image.depth() == CV_32F || image.depth() == CV_64F)
    image.convertTo(encodable, CV_16U, 1000.0);
  else if (image.depth() == CV_32S)
    image.convertTo(encodable, CV_16U);

  Blob blob;
  blob.content_type = "image/png";
  if (!cv::imencode(".png", encodable, blob.data))
    throw HttpError(500, std::string("could not PNG-encode ").append(what));
  return blob;
}

/// The stage catalogue surfaced by GET /stages. `runnable` mirrors whether a
/// library job runner exists (pipeline::JobStage); `ready` is the independent
/// question of whether this project satisfies the stage's input contract.
struct CatalogueEntry {
  std::string_view name;
  reusex::core::PipelineStage contract;
  std::optional<pipeline::JobStage> job_stage;
};

const std::vector<CatalogueEntry> &stage_catalogue() {
  static const std::vector<CatalogueEntry> catalogue{
      {"clouds", reusex::core::PipelineStage::clouds,
       pipeline::JobStage::clouds},
      {"planes", reusex::core::PipelineStage::planes,
       pipeline::JobStage::planes},
      {"rooms", reusex::core::PipelineStage::rooms, pipeline::JobStage::rooms},
      {"instances", reusex::core::PipelineStage::instances,
       pipeline::JobStage::instances},
      // No runner yet — the cell-complex/MIP plumbing is Phase 3 (#265). It is
      // still listed so the UI can show its readiness and grey out "Run".
      {"mesh", reusex::core::PipelineStage::mesh, std::nullopt},
  };
  return catalogue;
}

} // namespace

// ===========================================================================
// Params
// ===========================================================================

void Params::set(std::string key, std::string value) {
  values_[std::move(key)] = std::move(value);
}

std::optional<std::string> Params::find(std::string_view key) const {
  auto it = values_.find(key);
  if (it == values_.end())
    return std::nullopt;
  return it->second;
}

std::string Params::str(std::string_view key, std::string fallback) const {
  auto value = find(key);
  if (!value || value->empty())
    return fallback;
  return *value;
}

long long Params::integer(std::string_view key, long long fallback) const {
  auto value = find(key);
  if (!value || value->empty())
    return fallback;
  long long parsed = 0;
  const char *first = value->data();
  const char *last = first + value->size();
  auto [ptr, ec] = std::from_chars(first, last, parsed);
  if (ec != std::errc{} || ptr != last)
    throw HttpError(400, std::string("query parameter '")
                                 .append(key)
                                 .append("' must be an integer, got '") +
                             *value + "'");
  return parsed;
}

// ===========================================================================
// Route table
// ===========================================================================

const std::vector<Endpoint> &endpoint_table() {
  // Keep in lockstep with docs/gui/openapi.yaml. tests/unit/rux_gui asserts the
  // exact set, so adding a route without documenting it fails the build.
  static const std::vector<Endpoint> table{
      {"GET", "/api/v1/health", "Liveness probe and version handshake"},
      {"GET", "/api/v1/endpoints",
       "The route table this server actually serves"},
      {"GET", "/api/v1/project",
       "Everything the dashboard needs in one request"},
      {"GET", "/api/v1/projects",
       "Project metadata records stored in this database"},
      {"GET", "/api/v1/clouds",
       "All named point clouds with their type and point count"},
      {"GET", "/api/v1/clouds/<string>",
       "One cloud's metadata, including its label definitions"},
      {"GET", "/api/v1/clouds/<string>/points", "A page of point data"},
      {"GET", "/api/v1/meshes", "All stored meshes"},
      {"GET", "/api/v1/meshes/<string>", "One mesh's metadata"},
      {"GET", "/api/v1/meshes/<string>/data",
       "The mesh geometry blob, verbatim", true},
      {"GET", "/api/v1/meshes/<string>/textures",
       "Texture metadata for a mesh (no image bytes)"},
      {"GET", "/api/v1/meshes/<string>/textures/<string>", "One texture image",
       true},
      {"GET", "/api/v1/frames", "Sensor frame ids and aggregate counts"},
      {"GET", "/api/v1/frames/<int>",
       "One sensor frame's pose, intrinsics and availability flags"},
      {"GET", "/api/v1/frames/<int>/image",
       "An encoded image for one sensor frame", true},
      {"GET", "/api/v1/panoramas", "All 360 panoramas with pose provenance"},
      {"GET", "/api/v1/panoramas/<int>", "One panorama's metadata"},
      {"GET", "/api/v1/panoramas/<int>/image", "The equirectangular image",
       true},
      {"GET", "/api/v1/components",
       "Building components, optionally filtered by type"},
      {"GET", "/api/v1/components/<string>",
       "One component, including its boundary polygon"},
      {"GET", "/api/v1/materials", "Material passports stored in this project"},
      {"GET", "/api/v1/materials/<string>", "One passport's stored properties"},
      {"GET", "/api/v1/instances/<string>",
       "Instance rows of an instance-label cloud, with material links"},
      {"GET", "/api/v1/stages",
       "The stage catalogue, with readiness for this project"},
      {"GET", "/api/v1/pipeline-log", "Persisted stage execution history"},
      {"GET", "/api/v1/jobs", "Jobs known to this server, most recent first"},
      {"POST", "/api/v1/jobs", "Submit a stage run"},
      {"GET", "/api/v1/jobs/<string>", "One job's current status and progress"},
      {"POST", "/api/v1/jobs/<string>/cancel", "Request cancellation"},
      {"GET", "/api/v1/events", "WebSocket progress channel (upgrade)", true},
  };
  return table;
}

// ===========================================================================
// meta
// ===========================================================================

json error_json(int status, std::string_view message) {
  return json{{"error", std::string(message)}, {"status", status}};
}

json health_json(const reusex::ProjectDB *db,
                 const std::filesystem::path &project) {
  json out{{"status", "ok"},
           {"api_version", std::string(kApiVersion)},
           {"version", reusex::core::VERSION},
           {"implementation", std::string(kImplementation)}};
  // Only the file name: the contract must stay meaningful for a remote
  // implementation, which has no business disclosing server paths.
  json info{{"name", project.filename().string()}, {"open", db != nullptr}};
  if (db != nullptr)
    info["schema_version"] = db->schema_version();
  out["project"] = std::move(info);
  return out;
}

json endpoints_json() {
  json list = json::array();
  for (const auto &endpoint : endpoint_table()) {
    json entry{{"method", endpoint.method},
               {"path", endpoint.path},
               {"summary", endpoint.summary}};
    if (endpoint.binary)
      entry["binary"] = true;
    list.push_back(std::move(entry));
  }
  return json{{"endpoints", std::move(list)}};
}

// ===========================================================================
// project
// ===========================================================================

json project_summary_json(const reusex::ProjectDB &db) {
  const auto summary = db.project_summary();

  json projects = json::array();
  for (const auto &project : summary.projects)
    projects.push_back(project_info_json(project));

  json clouds = json::array();
  for (const auto &cloud : summary.clouds)
    clouds.push_back(cloud_info_json(cloud));

  json meshes = json::array();
  for (const auto &mesh : summary.meshes)
    meshes.push_back(json{{"name", mesh.name},
                          {"vertex_count", mesh.vertex_count},
                          {"face_count", mesh.face_count}});

  json materials = json::array();
  for (const auto &material : summary.materials)
    materials.push_back(material_info_json(material));

  json by_type = json::object();
  for (const auto &[type, count] : summary.components.count_by_type)
    by_type[type] = count;

  return json{{"path", summary.path.filename().string()},
              {"schema_version", summary.schema_version},
              {"projects", std::move(projects)},
              {"clouds", std::move(clouds)},
              {"meshes", std::move(meshes)},
              {"sensor_frames",
               {{"total_count", summary.sensor_frames.total_count},
                {"segmented_count", summary.sensor_frames.segmented_count},
                {"width", summary.sensor_frames.width},
                {"height", summary.sensor_frames.height}}},
              {"panoramic_images",
               {{"total_count", summary.panoramic_images.total_count},
                {"matched_count", summary.panoramic_images.matched_count}}},
              {"components",
               {{"total_count", summary.components.total_count},
                {"count_by_type", std::move(by_type)}}},
              {"materials", std::move(materials)}};
}

json projects_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &id : db.list_project_ids()) {
    const auto metadata = db.get_project_metadata(id);
    list.push_back(json{{"id", metadata.id},
                        {"name", metadata.name},
                        {"building_address", metadata.building_address},
                        {"year_of_construction", metadata.year_of_construction},
                        {"survey_date", metadata.survey_date},
                        {"survey_organisation", metadata.survey_organisation},
                        {"notes", metadata.notes}});
  }
  return json{{"projects", std::move(list)}};
}

// ===========================================================================
// clouds
// ===========================================================================

json clouds_json(const reusex::ProjectDB &db) {
  const auto summary = db.project_summary();
  json list = json::array();
  for (const auto &cloud : summary.clouds)
    list.push_back(cloud_info_json(cloud));
  return json{{"clouds", std::move(list)}};
}

json cloud_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);
  const auto summary = db.project_summary();
  for (const auto &cloud : summary.clouds)
    if (cloud.name == name)
      return cloud_info_json(cloud);
  not_found("cloud", name);
}

json cloud_points_json(const reusex::ProjectDB &db, const std::string &name,
                       const Params &params) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  const std::string format = params.str("format", "json");
  if (format == "binary")
    throw HttpError(501, "binary point transport is not implemented yet; it is "
                         "Phase 2/5 of issue #265. Use format=json.");
  if (format != "json")
    throw HttpError(400,
                    "format must be 'json' or 'binary', got '" + format + "'");

  const long long raw_offset = params.integer("offset", 0);
  const long long raw_limit =
      params.integer("limit", static_cast<long long>(kDefaultPointsPerPage));
  if (raw_offset < 0)
    throw HttpError(400, "offset must be >= 0");
  if (raw_limit < 1)
    throw HttpError(400, "limit must be >= 1");

  const size_t offset = static_cast<size_t>(raw_offset);
  const size_t limit =
      std::min<size_t>(static_cast<size_t>(raw_limit), kMaxPointsPerPage);

  const std::string type = db.point_cloud_type(name);

  // NOTE: the whole cloud is materialized to serve one page. Acceptable for a
  // localhost single-user server and for the small clouds Phase 1 targets;
  // Phase 2's binary transport is where chunk-level reads and voxel LOD land
  // (the v7 chunked point_cloud_data scheme already stores it in chunks).
  json fields = json::array();
  json points = json::array();
  size_t total = 0;

  auto emit_range = [&](size_t size, const auto &writer) {
    total = size;
    const size_t begin = std::min(offset, size);
    const size_t end = std::min(begin + limit, size);
    for (size_t i = begin; i < end; ++i)
      points.push_back(writer(i));
  };

  if (type == "PointXYZRGB") {
    fields = json::array({"x", "y", "z", "r", "g", "b"});
    auto cloud = db.point_cloud_xyzrgb(name);
    emit_range(cloud ? cloud->size() : 0, [&](size_t i) {
      const auto &p = cloud->points[i];
      return json::array({p.x, p.y, p.z, p.r, p.g, p.b});
    });
  } else if (type == "PointXYZ") {
    fields = json::array({"x", "y", "z"});
    auto cloud = db.point_cloud_xyz(name);
    emit_range(cloud ? cloud->size() : 0, [&](size_t i) {
      const auto &p = cloud->points[i];
      return json::array({p.x, p.y, p.z});
    });
  } else if (type == "Normal") {
    fields = json::array({"nx", "ny", "nz"});
    auto cloud = db.point_cloud_normal(name);
    emit_range(cloud ? cloud->size() : 0, [&](size_t i) {
      const auto &p = cloud->points[i];
      return json::array({p.normal_x, p.normal_y, p.normal_z});
    });
  } else if (type == "Label") {
    fields = json::array({"label"});
    auto cloud = db.point_cloud_label(name);
    emit_range(cloud ? cloud->size() : 0,
               [&](size_t i) { return json::array({cloud->points[i].label}); });
  } else {
    throw HttpError(500, "unsupported cloud type '" + type + "'");
  }

  return json{{"name", name},
              {"type", type},
              {"offset", offset},
              {"count", points.size()},
              {"total", total},
              {"fields", std::move(fields)},
              {"points", std::move(points)}};
}

// ===========================================================================
// meshes
// ===========================================================================

json meshes_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &name : db.list_meshes())
    list.push_back(mesh_json(db, name));
  return json{{"meshes", std::move(list)}};
}

json mesh_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_mesh(name))
    not_found("mesh", name);
  const auto metadata = db.mesh_metadata(name);
  return json{{"name", metadata.name},
              {"format", metadata.format},
              {"vertex_count", metadata.vertex_count},
              {"face_count", metadata.face_count},
              {"stage", metadata.stage},
              {"parameters", metadata.parameters},
              {"created_at", metadata.created_at},
              {"texture_count", db.mesh_texture_metadata(name).size()}};
}

json mesh_textures_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_mesh(name))
    not_found("mesh", name);
  json list = json::array();
  for (const auto &texture : db.mesh_texture_metadata(name))
    list.push_back(json{{"tex_name", texture.tex_name},
                        {"format", texture.format},
                        {"width", texture.width},
                        {"height", texture.height}});
  return json{{"textures", std::move(list)}};
}

Blob mesh_data_blob(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_mesh(name))
    not_found("mesh", name);
  Blob blob;
  blob.data = db.mesh_data_blob(name);
  const std::string format = db.mesh_format(name);
  blob.content_type =
      format == "obj" ? "model/obj" : "application/octet-stream";
  return blob;
}

Blob mesh_texture_blob(const reusex::ProjectDB &db, const std::string &name,
                       const std::string &texture) {
  if (!db.has_mesh(name))
    not_found("mesh", name);
  for (auto &stored : db.mesh_texture_blobs(name)) {
    if (stored.tex_name != texture)
      continue;
    Blob blob;
    blob.data = std::move(stored.image_data);
    blob.content_type = stored.format == "jpg" || stored.format == "jpeg"
                            ? "image/jpeg"
                            : "image/png";
    return blob;
  }
  not_found("texture", texture);
}

// ===========================================================================
// sensor frames
// ===========================================================================

json frames_json(const reusex::ProjectDB &db) {
  const auto summary = db.project_summary();
  json ids = json::array();
  for (int id : db.sensor_frame_ids())
    ids.push_back(id);
  return json{{"ids", std::move(ids)},
              {"total_count", summary.sensor_frames.total_count},
              {"segmented_count", summary.sensor_frames.segmented_count},
              {"width", summary.sensor_frames.width},
              {"height", summary.sensor_frames.height}};
}

json frame_json(const reusex::ProjectDB &db, int id) {
  if (!db.has_sensor_frame(id))
    not_found("sensor frame", std::to_string(id));

  json out{{"id", id},
           {"timestamp", db.sensor_frame_timestamp(id)},
           {"pose", pose_array(db.sensor_frame_pose(id))},
           {"intrinsics", intrinsics_json(db.sensor_frame_intrinsics(id))},
           {"has_segmentation", db.has_segmentation_image(id)}};
  // Availability is probed rather than assumed: an import may have stored the
  // color frame only.
  out["has_depth"] = !db.sensor_frame_depth(id).empty();
  out["has_confidence"] = !db.sensor_frame_confidence(id).empty();
  return out;
}

Blob frame_image_blob(const reusex::ProjectDB &db, int id,
                      const std::string &kind) {
  if (!db.has_sensor_frame(id) && kind != "segmentation")
    not_found("sensor frame", std::to_string(id));

  if (kind == "color")
    return encode_png(db.sensor_frame_image(id), "color image");
  if (kind == "depth")
    return encode_png(db.sensor_frame_depth(id), "depth image");
  if (kind == "confidence")
    return encode_png(db.sensor_frame_confidence(id), "confidence image");
  if (kind == "segmentation") {
    if (!db.has_segmentation_image(id))
      not_found("segmentation image for frame", std::to_string(id));
    return encode_png(db.segmentation_image(id), "segmentation image");
  }
  throw HttpError(400, "kind must be color|depth|confidence|segmentation, got "
                       "'" +
                           kind + "'");
}

// ===========================================================================
// panoramas
// ===========================================================================

namespace {
json panorama_entry_json(const reusex::ProjectDB::PanoramicImage &pano) {
  return json{{"id", pano.id},
              {"filename", pano.filename},
              {"timestamp", pano.timestamp},
              {"node_id", pano.node_id},
              {"has_pose", pano.has_pose},
              {"pose", pose_array(pano.pose)},
              {"pose_source", pano.pose_source},
              {"align_inliers", pano.align_inliers},
              {"align_rms", pano.align_rms}};
}
} // namespace

json panoramas_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &pano : db.list_panoramic_images())
    list.push_back(panorama_entry_json(pano));
  return json{{"panoramas", std::move(list)}};
}

json panorama_json(const reusex::ProjectDB &db, int id) {
  for (const auto &pano : db.list_panoramic_images())
    if (pano.id == id)
      return panorama_entry_json(pano);
  not_found("panorama", std::to_string(id));
}

Blob panorama_image_blob(const reusex::ProjectDB &db, int id) {
  const cv::Mat image = db.panoramic_image(id);
  if (image.empty())
    not_found("panorama", std::to_string(id));

  Blob blob;
  blob.content_type = "image/jpeg";
  if (!cv::imencode(".jpg", image, blob.data))
    throw HttpError(500, "could not JPEG-encode panorama");
  return blob;
}

// ===========================================================================
// components / materials / instances
// ===========================================================================

namespace {
json component_summary_json(const reusex::core::ComponentRecord &record) {
  constexpr size_t kBytesPerVertex = 3 * sizeof(double);
  return json{{"name", record.name},
              {"guid", record.guid},
              {"type", record.type},
              {"parent_id", record.parent_id},
              {"confidence", record.confidence},
              {"vertex_count", record.vertex_data.size() / kBytesPerVertex}};
}
} // namespace

json components_json(const reusex::ProjectDB &db, const Params &params) {
  const std::string type = params.str("type", "");
  const auto names = type.empty() ? db.list_building_components()
                                  : db.list_building_components(type);

  json list = json::array();
  for (const auto &name : names)
    list.push_back(component_summary_json(db.component_record(name)));
  return json{{"components", std::move(list)}};
}

json component_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_building_component(name))
    not_found("component", name);

  const auto record = db.component_record(name);
  json out = component_summary_json(record);
  out["plane"] = json::array(
      {record.plane[0], record.plane[1], record.plane[2], record.plane[3]});

  // vertex_data is packed little-endian float64 xyz triples (see
  // core/component_record.hpp). Unpack via memcpy rather than reinterpreting
  // the buffer, so alignment is never assumed.
  constexpr size_t kBytesPerVertex = 3 * sizeof(double);
  json vertices = json::array();
  const size_t count = record.vertex_data.size() / kBytesPerVertex;
  for (size_t i = 0; i < count; ++i) {
    std::array<double, 3> xyz{};
    std::memcpy(xyz.data(), record.vertex_data.data() + i * kBytesPerVertex,
                kBytesPerVertex);
    vertices.push_back(json::array({xyz[0], xyz[1], xyz[2]}));
  }
  out["vertices"] = std::move(vertices);
  out["metadata"] = record.metadata;
  out["notes"] = record.notes;
  return out;
}

json materials_json(const reusex::ProjectDB &db) {
  const auto summary = db.project_summary();
  json list = json::array();
  for (const auto &material : summary.materials)
    list.push_back(material_info_json(material));
  return json{{"materials", std::move(list)}};
}

json material_json(const reusex::ProjectDB &db, const std::string &guid) {
  const auto guids = db.list_passport_guids();
  if (std::find(guids.begin(), guids.end(), guid) == guids.end())
    not_found("material passport", guid);

  const auto properties = db.passport_stored_properties(guid);
  json props = json::object();
  for (const auto &[key, value] : properties)
    props[key] = value;

  json out{{"guid", guid},
           {"property_count", properties.size()},
           {"properties", std::move(props)}};

  // Enrich from the summary when the passport is listed there (it carries the
  // human-facing id / created_at / version that the property table does not).
  for (const auto &material : db.project_summary().materials) {
    if (material.guid != guid)
      continue;
    out["id"] = material.id;
    out["created_at"] = material.created_at;
    out["version_number"] = material.version_number;
    break;
  }

  if (auto node_id = db.passport_linked_node_id(guid))
    out["linked_node_id"] = *node_id;
  return out;
}

json instances_json(const reusex::ProjectDB &db, const std::string &cloud) {
  if (!db.has_point_cloud(cloud))
    not_found("cloud", cloud);

  const auto links = db.instance_materials(cloud);
  json list = json::array();
  for (const auto &record : db.instances(cloud)) {
    json entry{{"instance_id", record.instance_id},
               {"guid", record.guid},
               {"semantic_class", record.semantic_class},
               {"point_count", record.point_count}};
    auto link = links.find(static_cast<int>(record.instance_id));
    entry["material_guid"] =
        link == links.end() ? json(nullptr) : json(link->second);
    list.push_back(std::move(entry));
  }
  return json{{"cloud", cloud}, {"instances", std::move(list)}};
}

// ===========================================================================
// pipeline
// ===========================================================================

json stages_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &entry : stage_catalogue()) {
    std::vector<reusex::core::ValidationIssue> issues;
    reusex::core::check_stage_inputs(db, entry.contract, issues);

    json blockers = json::array();
    for (const auto &issue : issues)
      if (issue.severity == reusex::core::ValidationSeverity::error)
        blockers.push_back(issue.check + ": " + issue.message);

    list.push_back(
        json{{"stage", std::string(entry.name)},
             {"runnable", entry.job_stage.has_value()},
             {"cancellable",
              entry.job_stage &&
                  pipeline::stage_supports_cancellation(*entry.job_stage)},
             {"ready", blockers.empty()},
             {"blockers", std::move(blockers)}});
  }
  return json{{"stages", std::move(list)}};
}

json pipeline_log_json(const reusex::ProjectDB &db, const Params &params) {
  const long long limit = params.integer("limit", 100);
  if (limit < 0)
    throw HttpError(400, "limit must be >= 0");

  json list = json::array();
  for (const auto &entry : db.pipeline_log(static_cast<int>(limit)))
    list.push_back(json{{"id", entry.id},
                        {"stage", entry.stage},
                        {"status", entry.status},
                        {"started_at", entry.started_at},
                        {"finished_at", entry.finished_at},
                        {"parameters", entry.parameters},
                        {"error_msg", entry.error_msg}});
  return json{{"entries", std::move(list)}};
}

// ===========================================================================
// jobs
// ===========================================================================

json job_json(const pipeline::JobRecord &record) {
  json progress{{"stage", stage_token(record.progress_stage)},
                {"stage_label",
                 std::string(reusex::core::to_string(record.progress_stage))},
                {"current", record.progress_current},
                {"total", record.progress_total}};
  progress["fraction"] =
      record.progress_total > 0
          ? json(static_cast<double>(record.progress_current) /
                 static_cast<double>(record.progress_total))
          : json(nullptr);

  return json{{"id", record.id},
              {"stage", std::string(pipeline::to_string(record.stage))},
              {"status", std::string(pipeline::to_string(record.status))},
              {"parameters", parameters_object(record.parameters)},
              {"error", record.error},
              {"submitted_at", record.submitted_at},
              {"started_at", record.started_at},
              {"finished_at", record.finished_at},
              {"cancel_requested", record.cancel_requested},
              {"progress", std::move(progress)}};
}

json jobs_json(const std::vector<pipeline::JobRecord> &jobs) {
  json list = json::array();
  for (const auto &record : jobs)
    list.push_back(job_json(record));
  return json{{"jobs", std::move(list)}};
}

json job_event_json(const pipeline::JobEvent &event) {
  return json{{"type", std::string(pipeline::to_string(event.type))},
              {"timestamp", event.timestamp},
              {"job", job_json(event.job)}};
}

json hello_json(const std::vector<pipeline::JobRecord> &jobs,
                const std::filesystem::path &project) {
  return json{{"type", "hello"},
              {"timestamp", pipeline::iso8601_utc_now()},
              {"api_version", std::string(kApiVersion)},
              {"implementation", std::string(kImplementation)},
              {"project", project.filename().string()},
              {"jobs", jobs_json(jobs).at("jobs")}};
}

JobSubmission parse_job_request(std::string_view body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  auto stage_it = parsed.find("stage");
  if (stage_it == parsed.end() || !stage_it->is_string())
    throw HttpError(400, "'stage' is required and must be a string");

  const auto stage_name = stage_it->get<std::string>();
  auto stage = pipeline::parse_job_stage(stage_name);
  if (!stage) {
    std::string known;
    for (const auto &name : pipeline::job_stage_names())
      known += (known.empty() ? "" : ", ") + name;
    throw HttpError(400, "unknown or non-runnable stage '" + stage_name +
                             "'; runnable stages are: " + known);
  }

  JobSubmission submission;
  submission.stage = *stage;

  auto params_it = parsed.find("parameters");
  if (params_it != parsed.end() && !params_it->is_null()) {
    if (!params_it->is_object())
      throw HttpError(400, "'parameters' must be a JSON object");
    submission.parameters = params_it->dump();
  }
  return submission;
}

std::optional<json> handle_ws_message(
    std::string_view body,
    const std::function<void(std::optional<std::string>)> &subscribe) {
  auto reply_error = [](std::string message) {
    return json{{"type", "error"},
                {"timestamp", pipeline::iso8601_utc_now()},
                {"error", std::move(message)}};
  };

  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    return reply_error("message must be a JSON object");

  auto type_it = parsed.find("type");
  if (type_it == parsed.end() || !type_it->is_string())
    return reply_error("message is missing a string 'type'");

  const auto type = type_it->get<std::string>();
  if (type == "ping")
    return json{{"type", "pong"}, {"timestamp", pipeline::iso8601_utc_now()}};

  if (type == "subscribe") {
    auto job_it = parsed.find("job_id");
    if (job_it == parsed.end())
      return reply_error("'subscribe' requires a 'job_id' (null to clear)");
    if (job_it->is_null()) {
      subscribe(std::nullopt);
      return std::nullopt;
    }
    if (!job_it->is_string())
      return reply_error("'job_id' must be a string or null");
    subscribe(job_it->get<std::string>());
    return std::nullopt;
  }

  return reply_error("unknown message type '" + type + "'");
}

bool event_matches_subscription(
    const pipeline::JobEvent &event,
    const std::optional<std::string> &subscription) {
  return !subscription.has_value() || *subscription == event.job.id;
}

} // namespace rux::gui
