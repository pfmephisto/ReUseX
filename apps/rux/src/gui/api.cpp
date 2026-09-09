// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/api.hpp"

#include "gui/binary_points.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/component_record.hpp>
#include <reusex/core/stages.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/core/version.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/types/point_types.hpp>

#include <reusex/utils/cv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstring>
#include <exception>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>

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

/// A validated, clamped `offset`/`limit` pair, shared by both wire formats so
/// the two are interchangeable page for page.
struct PageRequest {
  uint64_t offset = 0;
  uint64_t limit = 0;
};

PageRequest parse_page_request(const Params &params) {
  const long long raw_offset = params.integer("offset", 0);
  const long long raw_limit =
      params.integer("limit", static_cast<long long>(kDefaultPointsPerPage));
  if (raw_offset < 0)
    throw HttpError(400, "offset must be >= 0");
  if (raw_limit < 1)
    throw HttpError(400, "limit must be >= 1");

  return {static_cast<uint64_t>(raw_offset),
          std::min<uint64_t>(static_cast<uint64_t>(raw_limit),
                             static_cast<uint64_t>(kMaxPointsPerPage))};
}

/// Read one little-endian value out of a stored point record.
float read_f32(const uint8_t *at) {
  float value = 0.0F;
  std::memcpy(&value, at, sizeof(value));
  return value;
}

uint32_t read_u32(const uint8_t *at) {
  uint32_t value = 0;
  std::memcpy(&value, at, sizeof(value));
  return value;
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

/// Validated `max_size`: 0 means "no limit".
int max_size_param(const Params &params) {
  const long long requested = params.integer("max_size", 0);
  if (requested == 0)
    return 0;
  if (requested < 1 || requested > kMaxImageSize)
    throw HttpError(400, "max_size must be between 1 and " +
                             std::to_string(kMaxImageSize) + ", got " +
                             std::to_string(requested));
  return static_cast<int>(requested);
}

/// Downscale so the longest edge is at most @p max_size. Never upscales.
///
/// Label images must be resampled with nearest-neighbour: interpolating
/// between class 3 and class 5 would silently manufacture class 4, a label no
/// point in the scan carries.
cv::Mat resized_to(const cv::Mat &image, int max_size, bool is_label) {
  if (max_size <= 0 || image.empty())
    return image;

  const int longest = std::max(image.cols, image.rows);
  if (longest <= max_size)
    return image;

  const double scale = static_cast<double>(max_size) / longest;
  cv::Mat out;
  cv::resize(image, out, cv::Size(), scale, scale,
             is_label ? cv::INTER_NEAREST : cv::INTER_AREA);
  return out;
}

/// Turn a stored measurement into something a browser can actually show.
///
/// This is a *rendering*, not a conversion: the result carries no scale, which
/// is why the observed range travels back to the caller in @p range so a UI can
/// label what it is looking at. See the `normalize` parameter in
/// docs/gui/openapi.yaml.
cv::Mat displayable(const cv::Mat &image, const std::string &kind,
                    ValueRange &range) {
  if (kind == "segmentation") {
    // Colourise with the same categorical palette the viewport uses, so a mask
    // and the 3D labels of the same scan are recognisably the same classes.
    const cv::Mat &lut = reusex::utils::get_glasbey_lut();
    cv::Mat rgb(image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < image.rows; ++y) {
      const auto *row = image.ptr<int32_t>(y);
      auto *out = rgb.ptr<cv::Vec3b>(y);
      for (int x = 0; x < image.cols; ++x) {
        // The API encoding is CV_32S with -1 for background
        // (core/label_semantics.hpp); leave both it and 0 black.
        if (row[x] <= 0)
          continue;
        out[x] = lut.at<cv::Vec3b>(0, row[x] % lut.cols);
      }
    }
    return rgb;
  }

  // Depth and confidence: stretch the range of *valid* pixels to 8-bit grey.
  // Zero is "no return" in both, and including it would peg the low end of
  // every frame to a pixel that carries no measurement, flattening the range
  // the picture is supposed to show.
  cv::Mat values;
  image.convertTo(values, CV_64F);
  const cv::Mat valid = values > 0.0;

  double lo = 0.0;
  double hi = 0.0;
  cv::minMaxLoc(values, &lo, &hi, nullptr, nullptr, valid);
  range.valid = cv::countNonZero(valid) > 0;
  range.min = lo;
  range.max = hi;

  cv::Mat grey(image.size(), CV_8UC1, cv::Scalar(0));
  if (!range.valid || hi <= lo) {
    // A constant (or entirely empty) image has no range to stretch. Returning
    // flat black is honest; scaling by 1/0 would not be.
    return grey;
  }

  values.convertTo(grey, CV_8UC1, 255.0 / (hi - lo), -255.0 * lo / (hi - lo));
  grey.setTo(0, ~valid);
  return grey;
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

/// Every validation finding for a stage, verbatim — including the derived
/// resolution `hint` (#295), which is the whole point of surfacing issues
/// separately from `blockers`. `blockers` is a flat list of strings a client
/// can print; `issues` is what a UI needs to say "run `rux create clouds`
/// first" without re-deriving the pipeline order itself.
json issues_json(const std::vector<reusex::core::ValidationIssue> &issues) {
  json list = json::array();
  for (const auto &issue : issues)
    list.push_back(json{
        {"check", issue.check},
        {"message", issue.message},
        {"severity", issue.severity == reusex::core::ValidationSeverity::error
                         ? "error"
                         : "warning"},
        {"hint", issue.hint},
        {"artifact", issue.artifact},
        {"commands", issue.commands}});
  return list;
}

/// The parameter descriptors of a runnable stage, straight from
/// pipeline::stage_parameters() — which reads the library option structs, so
/// no default is ever re-typed here (STANDARDS §4).
json stage_parameters_json(pipeline::JobStage stage) {
  json list = json::array();
  for (const auto &parameter : pipeline::stage_parameters(stage)) {
    json entry{{"key", parameter.key},
               {"type", std::string(pipeline::to_string(parameter.type))},
               {"label", parameter.label},
               {"description", parameter.description},
               {"presence_sensitive", parameter.presence_sensitive}};

    // `null` rather than an invented zero: these parameters genuinely have no
    // neutral value, and a form that pre-filled one would change behaviour.
    std::visit(
        [&entry](const auto &value) {
          using T = std::decay_t<decltype(value)>;
          if constexpr (std::is_same_v<T, std::monostate>)
            entry["default"] = nullptr;
          else
            entry["default"] = value;
        },
        parameter.default_value);

    entry["minimum"] =
        parameter.minimum ? json(*parameter.minimum) : json(nullptr);
    entry["maximum"] =
        parameter.maximum ? json(*parameter.maximum) : json(nullptr);
    list.push_back(std::move(entry));
  }
  return list;
}

/// One stage's full record: identity, runnability, readiness and knobs.
json stage_entry_json(const reusex::ProjectDB &db,
                      const CatalogueEntry &entry) {
  std::vector<reusex::core::ValidationIssue> issues;
  reusex::core::check_stage_inputs(db, entry.contract, issues);

  json blockers = json::array();
  for (const auto &issue : issues)
    if (issue.severity == reusex::core::ValidationSeverity::error)
      blockers.push_back(issue.check + ": " + issue.message);

  const auto &contract = reusex::core::stage_contract(entry.contract);
  json outputs = json::array();
  for (const auto &output : contract.outputs)
    outputs.push_back(std::string(output));

  return json{
      {"stage", std::string(entry.name)},
      // The name this stage writes into `pipeline_log.stage`, which is NOT the
      // wire token — the CLI has been writing `segment_planes` into that
      // column since before the GUI existed. Without this a client cannot join
      // durable history to a stage card without hard-coding the mapping.
      {"log_name", entry.job_stage ? std::string(pipeline::pipeline_log_name(
                                         *entry.job_stage))
                                   : std::string()},
      {"summary", std::string(contract.summary)},
      {"command", std::string(contract.command)},
      {"runnable", entry.job_stage.has_value()},
      {"cancellable", entry.job_stage && pipeline::stage_supports_cancellation(
                                             *entry.job_stage)},
      {"ready", blockers.empty()},
      {"outputs", std::move(outputs)},
      {"blockers", std::move(blockers)},
      {"issues", issues_json(issues)},
      {"parameters", entry.job_stage ? stage_parameters_json(*entry.job_stage)
                                     : json::array()}};
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

std::optional<bool> Params::boolean(std::string_view key) const {
  auto value = find(key);
  if (!value || value->empty())
    return std::nullopt;

  std::string lowered = *value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  if (lowered == "true" || lowered == "1" || lowered == "yes")
    return true;
  if (lowered == "false" || lowered == "0" || lowered == "no")
    return false;

  throw HttpError(400, std::string("query parameter '")
                               .append(key)
                               .append("' must be a boolean, got '") +
                           *value + "'");
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
      {"GET", "/api/v1/clouds/<string>/labels",
       "The label legend of one Label cloud"},
      {"PATCH", "/api/v1/clouds/<string>/labels",
       "Rename label classes of one Label cloud"},
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
      {"PATCH", "/api/v1/materials/<string>",
       "Add, change or clear passport properties"},
      {"GET", "/api/v1/instances/<string>",
       "Instance rows of an instance-label cloud, with material links"},
      {"GET", "/api/v1/stages",
       "The stage catalogue, with readiness for this project"},
      {"GET", "/api/v1/stages/<string>/validation",
       "Input-contract validation for one stage"},
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

json cloud_labels_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  json labels = json::object();
  for (const auto &[id, label_name] : db.label_definitions(name)) {
    // 0 is unlabeled and has no name to show (STANDARDS §3). A stray row for it
    // is stale data, not a class, so it is dropped rather than surfaced.
    if (id <= 0)
      continue;
    labels[std::to_string(id)] = label_name;
  }
  return json{{"labels", std::move(labels)}};
}

json cloud_points_json(const reusex::ProjectDB &db, const std::string &name,
                       const Params &params) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  const auto request = parse_page_request(params);

  // Only the bytes this page occupies are read: point_cloud_page() maps the
  // window onto a byte range in the chunked point_cloud_data store and pulls
  // it with SQLite incremental blob I/O, so peak memory is O(page), not
  // O(cloud). The records come back in storage layout and are decoded inline
  // here — inflating a pcl::PointCloud would cost 32 B/point of SSE-padded
  // memory to produce a JSON array.
  const auto page = db.point_cloud_page(name, request.offset, request.limit);
  const auto *record = page.data.data();
  const size_t step = page.point_step;
  const size_t count = static_cast<size_t>(page.count);

  json fields = json::array();
  json points = json::array();

  if (page.point_type == "PointXYZRGB") {
    fields = json::array({"x", "y", "z", "r", "g", "b"});
    for (size_t i = 0; i < count; ++i) {
      const auto *p = record + i * step;
      // Bytes 12..15 are pcl::PointXYZRGB::rgba, whose union is declared over
      // { b, g, r, a } — so r is at +14, g at +13, b at +12 on a
      // little-endian host. Same swizzle as the RUXP path.
      points.push_back(json::array({read_f32(p), read_f32(p + 4),
                                    read_f32(p + 8), p[14], p[13], p[12]}));
    }
  } else if (page.point_type == "PointXYZ") {
    fields = json::array({"x", "y", "z"});
    for (size_t i = 0; i < count; ++i) {
      const auto *p = record + i * step;
      points.push_back(
          json::array({read_f32(p), read_f32(p + 4), read_f32(p + 8)}));
    }
  } else if (page.point_type == "Normal") {
    // The stored record also carries curvature at +12; neither wire format
    // exposes it.
    fields = json::array({"nx", "ny", "nz"});
    for (size_t i = 0; i < count; ++i) {
      const auto *p = record + i * step;
      points.push_back(
          json::array({read_f32(p), read_f32(p + 4), read_f32(p + 8)}));
    }
  } else if (page.point_type == "Label") {
    fields = json::array({"label"});
    for (size_t i = 0; i < count; ++i)
      points.push_back(json::array({read_u32(record + i * step)}));
  } else {
    throw HttpError(500, "unsupported cloud type '" + page.point_type + "'");
  }

  return json{{"name", name},
              {"type", page.point_type},
              {"offset", page.offset},
              {"count", points.size()},
              {"total", page.total},
              {"fields", std::move(fields)},
              {"points", std::move(points)}};
}

PointsResponse cloud_points(const reusex::ProjectDB &db,
                            const std::string &name, const Params &params) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  const std::string format = params.str("format", "json");
  if (format == "json") {
    PointsResponse response;
    response.body = cloud_points_json(db, name, params);
    return response;
  }
  if (format != "binary")
    throw HttpError(400,
                    "format must be 'json' or 'binary', got '" + format + "'");

  const auto request = parse_page_request(params);
  const auto page = db.point_cloud_page(name, request.offset, request.limit);
  if (!ruxp_supports(page.point_type))
    throw HttpError(500, "unsupported cloud type '" + page.point_type + "'");

  PointsResponse response;
  response.blob = Blob{"application/octet-stream", encode_ruxp(page)};
  // Mirrors of the body header, for curl-level debugging only.
  response.headers = {{"X-Ruxp-Version", std::to_string(kRuxpVersion)},
                      {"X-Ruxp-Type", page.point_type},
                      {"X-Ruxp-Offset", std::to_string(page.offset)},
                      {"X-Ruxp-Count", std::to_string(page.count)},
                      {"X-Ruxp-Total", std::to_string(page.total)}};
  return response;
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

json frames_json(const reusex::ProjectDB &db, const Params &params) {
  const auto summary = db.project_summary();
  const auto segmented = params.boolean("segmented");

  // One query for the whole set rather than has_segmentation_image() per frame:
  // the filter exists precisely because a browser must not pay per-frame costs
  // to draw a list.
  std::set<int> with_masks;
  if (segmented) {
    const auto ids = db.segmentation_image_ids();
    with_masks.insert(ids.begin(), ids.end());
  }

  json ids = json::array();
  for (int id : db.sensor_frame_ids()) {
    if (segmented && with_masks.count(id) != static_cast<size_t>(*segmented))
      continue;
    ids.push_back(id);
  }

  // The counts describe the whole scan even when `ids` is filtered — a browser
  // showing "12 of 380 segmented" needs both numbers, and making it issue a
  // second unfiltered request for the denominator would be gratuitous.
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

ImageResponse frame_image(const reusex::ProjectDB &db, int id,
                          const Params &params) {
  const std::string kind = params.str("kind", "color");
  if (kind != "color" && kind != "depth" && kind != "confidence" &&
      kind != "segmentation")
    throw HttpError(400,
                    "kind must be color|depth|confidence|segmentation, got '" +
                        kind + "'");

  if (!db.has_sensor_frame(id) && kind != "segmentation")
    not_found("sensor frame", std::to_string(id));

  cv::Mat image;
  std::string what;
  if (kind == "color") {
    image = db.sensor_frame_image(id);
    what = "color image";
  } else if (kind == "depth") {
    image = db.sensor_frame_depth(id);
    what = "depth image";
  } else if (kind == "confidence") {
    image = db.sensor_frame_confidence(id);
    what = "confidence image";
  } else {
    if (!db.has_segmentation_image(id))
      not_found("segmentation image for frame", std::to_string(id));
    image = db.segmentation_image(id);
    what = "segmentation image";
  }

  if (image.empty())
    throw HttpError(404, "no " + what + " stored");

  // Resize before colourising: cheaper, and for a label image it keeps
  // nearest-neighbour operating on ids rather than on colours, where blending
  // two neighbouring classes would produce a third class's colour.
  image = resized_to(image, max_size_param(params), kind == "segmentation");

  ImageResponse response;
  if (params.boolean("normalize").value_or(false) && kind != "color") {
    ValueRange range;
    response.blob = encode_png(displayable(image, kind, range), what);
    response.range = range;
  } else {
    response.blob = encode_png(image, what);
  }
  return response;
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
/// Unpack the packed little-endian float64 xyz triples of a boundary.
///
/// memcpy rather than a reinterpret_cast: the blob carries no alignment
/// guarantee (core/component_record.hpp).
std::vector<std::array<double, 3>>
component_vertices(const reusex::core::ComponentRecord &record) {
  constexpr size_t kBytesPerVertex = 3 * sizeof(double);
  const size_t count = record.vertex_data.size() / kBytesPerVertex;

  std::vector<std::array<double, 3>> vertices(count);
  for (size_t i = 0; i < count; ++i)
    std::memcpy(vertices[i].data(),
                record.vertex_data.data() + i * kBytesPerVertex,
                kBytesPerVertex);
  return vertices;
}

/// Area of a planar polygon by Newell's method, in m².
///
/// Derived on read rather than stored: `building_components` has no area
/// column, and a cached one would be free to disagree with the geometry it
/// claims to describe.
double polygon_area(const std::vector<std::array<double, 3>> &vertices) {
  if (vertices.size() < 3)
    return 0.0;

  std::array<double, 3> normal{0.0, 0.0, 0.0};
  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto &a = vertices[i];
    const auto &b = vertices[(i + 1) % vertices.size()];
    normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
    normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
    normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
  }
  return 0.5 * std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                         normal[2] * normal[2]);
}

/// The provenance link (#211) out of the opaque geometry-owned metadata JSON.
///
/// `core` deliberately never parses this column, so the read is defensive:
/// unparseable metadata means "no link", not a failed request. A component
/// whose metadata is malformed is still worth listing.
std::string source_instance_guid(const reusex::core::ComponentRecord &record) {
  if (record.metadata.empty())
    return {};
  const auto parsed =
      json::parse(record.metadata, nullptr, /*allow_throw=*/false);
  if (!parsed.is_object())
    return {};
  const auto it = parsed.find("source_instance_guid");
  if (it == parsed.end() || !it->is_string())
    return {};
  return it->get<std::string>();
}

json component_summary_json(const reusex::core::ComponentRecord &record) {
  const auto vertices = component_vertices(record);

  json out{{"name", record.name},
           {"guid", record.guid},
           {"type", record.type},
           {"parent_id", record.parent_id},
           {"confidence", record.confidence},
           {"vertex_count", vertices.size()}};

  if (vertices.size() >= 3)
    out["area"] = polygon_area(vertices);

  const std::string source = source_instance_guid(record);
  if (!source.empty())
    out["source_instance_guid"] = source;

  return out;
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

  json vertices = json::array();
  for (const auto &xyz : component_vertices(record))
    vertices.push_back(json::array({xyz[0], xyz[1], xyz[2]}));
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
  for (const auto &entry : stage_catalogue())
    list.push_back(stage_entry_json(db, entry));
  return json{{"stages", std::move(list)}};
}

json stage_validation_json(const reusex::ProjectDB &db,
                           const std::string &stage) {
  const auto &catalogue = stage_catalogue();
  const auto it = std::find_if(
      catalogue.begin(), catalogue.end(),
      [&](const CatalogueEntry &entry) { return entry.name == stage; });
  if (it == catalogue.end())
    not_found("stage", stage);

  // Deliberately the same body as one element of /stages rather than a
  // narrower one: a client that re-checks a single stage after a run must not
  // have to merge two differently-shaped records to refresh its card.
  return stage_entry_json(db, *it);
}

json pipeline_log_json(const reusex::ProjectDB &db, const Params &params) {
  const long long requested = params.integer("limit", kDefaultLogEntries);
  if (requested < 0)
    throw HttpError(400, "limit must be >= 0");

  // 0 means "give me as many as you will", not "unbounded" — ProjectDB treats
  // 0 as no limit, which would let one query materialize a project's entire
  // history. Clamp both ends, exactly as the points endpoint does.
  const int limit = (requested == 0 || requested > kMaxLogEntries)
                        ? kMaxLogEntries
                        : static_cast<int>(requested);

  json list = json::array();
  for (const auto &entry : db.pipeline_log(limit))
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

json job_json(const pipeline::JobRecord &record, std::string_view project) {
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
              {"project", std::string(project)},
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

json jobs_json(const std::vector<pipeline::JobRecord> &jobs,
               std::string_view project) {
  json list = json::array();
  for (const auto &record : jobs)
    list.push_back(job_json(record, project));
  return json{{"jobs", std::move(list)}};
}

json job_event_json(const pipeline::JobEvent &event, std::string_view project) {
  return json{{"type", std::string(pipeline::to_string(event.type))},
              {"seq", event.sequence},
              {"timestamp", event.timestamp},
              {"project", std::string(project)},
              {"job", job_json(event.job, project)}};
}

json hello_json(const std::vector<pipeline::JobRecord> &jobs,
                const std::filesystem::path &project) {
  const auto name = project.filename().string();
  return json{{"type", "hello"},
              {"timestamp", pipeline::iso8601_utc_now()},
              {"api_version", std::string(kApiVersion)},
              {"implementation", std::string(kImplementation)},
              {"project", name},
              {"jobs", jobs_json(jobs, name).at("jobs")}};
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

  auto project_it = parsed.find("project");
  if (project_it != parsed.end() && !project_it->is_null()) {
    if (!project_it->is_string())
      throw HttpError(400, "'project' must be a string");
    submission.project = project_it->get<std::string>();
  }
  return submission;
}

void check_job_project(const JobSubmission &submission,
                       std::string_view open_project) {
  if (!submission.project || *submission.project == open_project)
    return;
  throw HttpError(409, "this server has '" + std::string(open_project) +
                           "' open, not '" + *submission.project +
                           "'; a job submitted here would run against the "
                           "wrong project");
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
