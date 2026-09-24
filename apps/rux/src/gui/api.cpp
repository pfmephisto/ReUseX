// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/api.hpp"

#include "gui/binary_points.hpp"
#include "gui/point_lod.hpp"

#include <reusex/core/MaterialPassport.hpp>
#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/SensorIntrinsics.hpp>
#include <reusex/core/component_record.hpp>
#include <reusex/core/frame_visibility.hpp>
#include <reusex/core/guid.hpp>
#include <reusex/core/stages.hpp>
#include <reusex/core/validate.hpp>
#include <reusex/core/version.hpp>
#include <reusex/pipeline/stage_parameters.hpp>
#include <reusex/types/point_types.hpp>

#include <reusex/utils/cv.hpp>

#include <Eigen/Core>

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
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>

namespace rux::gui {
namespace {

using json = nlohmann::json;
namespace pipeline = reusex::pipeline;

/// Classify a Label cloud as geometry (structural segmentation) or semantic
/// (object-class annotation).
///
/// `planes` and `rooms` are the outputs of `rux create planes` / `rux create
/// rooms` — structural segmentation whose labels are plane/room ids. Everything
/// else (including `instances`, annotation-derived clouds, and user-defined
/// ones) carries per-object semantic classes and is therefore semantic.
///
/// This is the single, authoritative server-side definition of `label_kind`.
/// The frontend must not re-derive it from the name.
std::string_view label_kind_of(std::string_view name) noexcept {
  if (name == "planes" || name == "rooms")
    return "geometry";
  return "semantic";
}

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

/// One page of an already-materialized vector, wrapped in the shared envelope.
///
/// @param source    The whole collection, in the order the contract documents.
/// @param key       Response key the items go under, e.g. "clouds".
/// @param serialize Called once per item that is actually in the page — items
///                  outside it are never touched, which is the point: several
///                  of these serializers issue a database query per item.
/// @param max_limit     Server maximum for this collection; `limit=0` asks for
///                      exactly it.
/// @param default_limit `limit` when the parameter is absent. 0 means "the
///                      maximum", which is right for every collection except a
///                      history view, where a first screenful is wanted.
template <typename T, typename F>
json paged_collection(const std::vector<T> &source, std::string_view key,
                      const Params &params, F &&serialize,
                      long long max_limit = kMaxCollectionItems,
                      long long default_limit = 0) {
  const auto window = page_window(
      parse_page_request(params, default_limit, max_limit), source.size());

  json items = json::array();
  for (size_t i = window.first; i < window.last; ++i)
    items.push_back(serialize(source[i]));

  json out{{std::string(key), std::move(items)}};
  add_page_envelope(out, window);
  return out;
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
  if (info.type == "Label") {
    out["label_kind"] = std::string(label_kind_of(info.name));
    if (!info.labels.empty()) {
      json labels = json::object();
      for (const auto &[id, name] : info.labels)
        labels[std::to_string(id)] = name;
      out["labels"] = std::move(labels);
    }
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
      {"mesh", reusex::core::PipelineStage::mesh, pipeline::JobStage::mesh},
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

// --- point/frame visibility (#453) -----------------------------------------

/// Parse a finite double query parameter, or 400 if absent/malformed. Used by
/// the visibility endpoints, whose coordinates are not integers.
double require_double(const Params &params, std::string_view key) {
  const auto value = params.find(key);
  if (!value || value->empty())
    throw HttpError(400, std::string("query parameter '") + std::string(key) +
                             "' is required");
  try {
    std::size_t pos = 0;
    const double parsed = std::stod(*value, &pos);
    if (pos != value->size() || !std::isfinite(parsed))
      throw std::invalid_argument("");
    return parsed;
  } catch (const std::exception &) {
    throw HttpError(400, std::string("query parameter '") + std::string(key) +
                             "' must be a finite number, got '" + *value + "'");
  }
}

/// Build the shared visibility query (currently just an optional `max_depth`)
/// from request parameters.
reusex::core::VisibilityQuery visibility_query_of(const Params &params) {
  reusex::core::VisibilityQuery query;
  if (const auto value = params.find("max_depth"); value && !value->empty()) {
    try {
      std::size_t pos = 0;
      const double parsed = std::stod(*value, &pos);
      if (pos != value->size() || !std::isfinite(parsed) || parsed < 0.0)
        throw std::invalid_argument("");
      query.max_depth = parsed;
    } catch (const std::exception &) {
      throw HttpError(400, "query parameter 'max_depth' must be a "
                           "non-negative number, got '" +
                               *value + "'");
    }
  }
  return query;
}

/// Serialise a ranked visibility result. `limit` bounds how many frames the
/// body carries (`total` always reports the full count).
json visibility_json(const std::array<double, 3> &point,
                     const std::vector<reusex::core::FrameVisibility> &frames,
                     const Params &params) {
  const long long raw_limit = params.integer("limit", 0);
  if (raw_limit < 0)
    throw HttpError(400, "limit must be >= 0");
  const std::size_t count =
      raw_limit == 0 ? frames.size()
                     : std::min<std::size_t>(
                           static_cast<std::size_t>(raw_limit), frames.size());

  json arr = json::array();
  for (std::size_t i = 0; i < count; ++i) {
    const auto &f = frames[i];
    // `centrality` is 0 at the principal point; `score` is its higher-is-better
    // complement so a client can sort/threshold either way without inverting.
    arr.push_back(json{{"frame_id", f.frame_id},
                       {"centrality", f.centrality},
                       {"score", 1.0 - f.centrality},
                       {"depth", f.depth},
                       {"u", f.u},
                       {"v", f.v}});
  }

  return json{{"point", {point[0], point[1], point[2]}},
              {"frames", std::move(arr)},
              {"count", count},
              {"total", frames.size()}};
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
// Paging
// ===========================================================================

PageRequest parse_page_request(const Params &params, long long default_limit,
                               long long max_limit) {
  const long long raw_offset = params.integer("offset", 0);
  const long long raw_limit = params.integer("limit", default_limit);
  if (raw_offset < 0)
    throw HttpError(400, "offset must be >= 0");
  if (raw_limit < 0)
    throw HttpError(400, "limit must be >= 0");

  // 0 means "as many as the server will give", which is the maximum — not
  // "unbounded". Anything larger is clamped rather than refused: a client
  // asking for more than it can get is not making a mistake, and a 400 there
  // would only teach it to hard-code our cap.
  const long long limit =
      (raw_limit == 0) ? max_limit : std::min(raw_limit, max_limit);

  return {static_cast<uint64_t>(raw_offset), static_cast<uint64_t>(limit)};
}

PageWindow page_window(const PageRequest &page, size_t total) {
  PageWindow window;
  window.total = total;
  window.first = std::min<size_t>(static_cast<size_t>(page.offset), total);
  // Saturating add: offset+limit can overflow size_t for a large offset paired
  // with a large limit, and wrapping would turn "past the end" into "the whole
  // collection" — an empty page is the honest answer.
  const size_t remaining = total - window.first;
  window.last = window.first +
                std::min<size_t>(static_cast<size_t>(page.limit), remaining);
  return window;
}

void add_page_envelope(json &object, const PageWindow &window) {
  object["offset"] = window.first;
  object["count"] = window.count();
  object["total"] = window.total;
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
      {"PATCH", "/api/v1/projects/<string>",
       "Sparsely update (or create) a project metadata record"},
      {"GET", "/api/v1/clouds",
       "All named point clouds with their type and point count"},
      {"GET", "/api/v1/clouds/<string>",
       "One cloud's metadata, including its label definitions"},
      {"GET", "/api/v1/clouds/<string>/points", "A page of point data"},
      {"GET", "/api/v1/clouds/<string>/labels",
       "The label legend of one Label cloud"},
      {"PATCH", "/api/v1/clouds/<string>/labels",
       "Rename label classes of one Label cloud"},
      {"GET", "/api/v1/clouds/<string>/tiles",
       "Spatial tile index for frustum-culled streaming"},
      {"GET", "/api/v1/meshes", "All stored meshes"},
      {"GET", "/api/v1/meshes/<string>", "One mesh's metadata"},
      {"GET", "/api/v1/meshes/<string>/data",
       "The mesh geometry blob, verbatim", true},
      {"GET", "/api/v1/meshes/<string>/textures",
       "Texture metadata for a mesh (no image bytes)"},
      {"GET", "/api/v1/meshes/<string>/textures/<string>", "One texture image",
       true},
      {"GET", "/api/v1/gsplats", "Gaussian splats stored in this project"},
      {"GET", "/api/v1/gsplats/<string>", "One Gaussian splat's metadata"},
      {"GET", "/api/v1/gsplats/<string>/data",
       "The INRIA-format splat PLY, verbatim", true},
      {"GET", "/api/v1/frames", "Sensor frame ids and aggregate counts"},
      {"GET", "/api/v1/frames/visibility",
       "Sensor frames that see a world point, ranked by centrality"},
      {"GET", "/api/v1/frames/<int>",
       "One sensor frame's pose, intrinsics and availability flags"},
      {"GET", "/api/v1/frames/<int>/image",
       "An encoded image for one sensor frame", true},
      {"POST", "/api/v1/frames/<int>/segment",
       "Run SAM3 on one frame and store the label mask"},
      {"GET", "/api/v1/panoramas", "All 360 panoramas with pose provenance"},
      {"GET", "/api/v1/panoramas/<int>", "One panorama's metadata"},
      {"GET", "/api/v1/panoramas/<int>/image", "The equirectangular image",
       true},
      {"GET", "/api/v1/components",
       "Building components, optionally filtered by type"},
      {"GET", "/api/v1/components/<string>",
       "One component, including its boundary polygon"},
      {"GET", "/api/v1/materials", "Material passports stored in this project"},
      {"POST", "/api/v1/materials", "Create a new blank material passport"},
      {"GET", "/api/v1/materials/<string>", "One passport's stored properties"},
      {"PATCH", "/api/v1/materials/<string>",
       "Add, change or clear passport properties"},
      {"DELETE", "/api/v1/materials/<string>", "Delete a material passport"},
      {"GET", "/api/v1/materials/<string>/thumbnail",
       "A material's thumbnail image", true},
      {"PUT", "/api/v1/materials/<string>/thumbnail",
       "Upload or replace a material's thumbnail image"},
      {"GET", "/api/v1/material-columns",
       "User-defined material column definitions"},
      {"POST", "/api/v1/material-columns",
       "Create a material column definition"},
      {"PATCH", "/api/v1/material-columns/<string>",
       "Update a material column definition"},
      {"DELETE", "/api/v1/material-columns/<string>",
       "Delete a material column definition"},
      {"GET", "/api/v1/instances/<string>",
       "Instance rows of an instance-label cloud, with material links"},
      {"GET", "/api/v1/instances/<string>/<int>/frames",
       "Sensor frames that see an instance's centroid, ranked by centrality"},
      {"PUT", "/api/v1/instances/<string>/<int>/material",
       "Link a material passport to an instance (upsert)"},
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
      {"GET", "/api/v1/posegraph",
       "Pose-graph nodes (frame poses) and edges (with post-solve residuals)"},
      {"DELETE", "/api/v1/posegraph/edges/<int>/<int>",
       "Delete a pose-graph edge (by from/to node ids)"},
      {"POST", "/api/v1/posegraph/edges", "Add a manual pose-graph edge"},
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

json projects_json(const reusex::ProjectDB &db, const Params &params) {
  // The id list is paged before the metadata is read, so an out-of-page record
  // costs no query at all.
  return paged_collection(
      db.list_project_ids(), "projects", params, [&db](const auto &id) {
        const auto metadata = db.get_project_metadata(id);
        return json{{"id", metadata.id},
                    {"name", metadata.name},
                    {"building_address", metadata.building_address},
                    {"year_of_construction", metadata.year_of_construction},
                    {"survey_date", metadata.survey_date},
                    {"survey_organisation", metadata.survey_organisation},
                    {"notes", metadata.notes}};
      });
}

// ===========================================================================
// clouds
// ===========================================================================

json clouds_json(const reusex::ProjectDB &db, const Params &params) {
  const auto summary = db.project_summary();
  return paged_collection(
      summary.clouds, "clouds", params,
      [](const auto &cloud) { return cloud_info_json(cloud); });
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

json cloud_tiles_json(const reusex::ProjectDB &db, const std::string &name) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  const auto blob = db.tile_index(name);
  if (blob.empty())
    throw HttpError(404, "no tile index for cloud '" + name + "'");

  const auto [hdr, tiles] = parse_tile_index(blob);
  const uint32_t K = 1u << hdr.tile_bits;

  json tile_array = json::array();
  for (uint32_t k = 0; k < K; ++k) {
    const auto &t = tiles[k];
    tile_array.push_back(
        json{{"id", k},
             {"count", t.count},
             {"min", json::array({t.min[0], t.min[1], t.min[2]})},
             {"max", json::array({t.max[0], t.max[1], t.max[2]})}});
  }
  return json{{"name", name},
              {"tile_count", K},
              {"tile_bits", hdr.tile_bits},
              {"point_count", hdr.point_count},
              {"tiles", std::move(tile_array)}};
}

namespace {

/// A validated `max_points` / `lod_source` pair (#320).
struct LodRequest {
  bool active = false;
  uint64_t max_points = 0;
  /// Cloud whose positions drive the selection. Empty means "this one".
  std::string source;
};

/// Parse the LOD half of the points query.
///
/// `max_points` and `offset`/`limit` are refused together rather than
/// reconciled. They answer different questions — "all of it, coarsely" versus
/// "this window of it" — and any rule that made them coexist would have to
/// invent a meaning for `offset` into a set the client cannot enumerate. A
/// progressive on-disk ordering is what would collapse the two (see
/// docs/gui/binary-points.md § "Level of detail"); until then, refusing is the
/// honest answer.
LodRequest parse_lod_request(const Params &params) {
  // An empty value (`?offset=`) is treated as absent throughout this API, so
  // it must not be what makes max_points a 400.
  const auto supplied = [&params](std::string_view key) {
    const auto value = params.find(key);
    return value.has_value() && !value->empty();
  };

  LodRequest request;
  if (!supplied("max_points")) {
    if (supplied("lod_source"))
      throw HttpError(400, "lod_source is only meaningful with max_points");
    return request;
  }
  if (supplied("offset") || supplied("limit"))
    throw HttpError(400, "max_points selects points from the whole cloud and "
                         "cannot be combined with offset or limit");

  const long long value = params.integer("max_points", 0);
  if (value < 1)
    throw HttpError(400, "max_points must be at least 1, got " +
                             std::to_string(value));

  request.active = true;
  // Clamped rather than rejected, like `limit`: a client may always ask for
  // more than the server is willing to serve.
  request.max_points =
      std::min<uint64_t>(static_cast<uint64_t>(value), kMaxPointsPerPage);
  request.source = params.str("lod_source", "");
  return request;
}

/// A LOD page plus what the wire needs to say about it.
struct LodPage {
  reusex::ProjectDB::CloudPage page;
  /// False when the cloud fit the budget: the page is then an ordinary,
  /// complete, storage-ordered read and must not claim LOD on the wire.
  bool subsampled = false;
  double voxel_size = 0.0;
};

LodPage resolve_lod(const reusex::ProjectDB &db, const std::string &name,
                    const LodRequest &request) {
  const std::string source = request.source.empty() ? name : request.source;
  if (!db.has_point_cloud(source))
    not_found("cloud", source);
  if (!lod_supports(db.point_cloud_type(source)))
    throw HttpError(400,
                    "cloud '" + source +
                        "' carries no positions to voxelise; pass lod_source "
                        "naming an index-aligned geometry cloud");

  auto selection = voxel_lod(db, source, request.max_points);
  if (source == name)
    return {std::move(selection.page), selection.subsampled,
            selection.voxel_size};

  // A sibling: sample it at the *same* storage indices, which is the only way
  // a Label cloud can still be zipped positionally against a subsampled
  // geometry cloud (docs/CONTRACTS.md, STANDARDS §3.2).
  const auto sibling = db.point_cloud_page(name, 0, 0);
  if (sibling.total != selection.page.total)
    throw HttpError(400, "cloud '" + name + "' holds " +
                             std::to_string(sibling.total) + " points and '" +
                             source + "' holds " +
                             std::to_string(selection.page.total) +
                             "; they are not index-aligned");

  return {gather_points(db, name, selection.indices), selection.subsampled,
          selection.voxel_size};
}

/// A validated single-tile request (#395 / #396).
struct TileRequest {
  bool active = false;
  uint32_t tile_id = 0;
  /// Geometry cloud whose tile index drives the scan, for a sibling that has
  /// no positions of its own (a Label cloud). Empty = serve this cloud
  /// directly.
  std::string lod_source;
  /// Within-tile offset and limit for multi-level LOD (#396). Both are 0 for
  /// a full-resolution tile fetch. When limit > 0, the response carries the
  /// LOD flag so the client knows more data is available for this tile.
  uint64_t tile_offset = 0;
  uint64_t tile_limit = 0;
};

/// Parse the `tile` half of the points query.
///
/// Mutually exclusive with `max_points` only: `offset`/`limit` serve as
/// within-tile slice parameters (#396) when `tile` is present, enabling
/// multi-level LOD without schema changes.
///
/// `lod_source` may accompany `tile`: it names the geometry cloud whose tile
/// membership is used to gather the position-free sibling (BLOCKER 2 fix).
TileRequest parse_tile_request(const Params &params) {
  const auto supplied = [&params](std::string_view key) {
    const auto value = params.find(key);
    return value.has_value() && !value->empty();
  };

  TileRequest request;
  if (!supplied("tile"))
    return request;
  if (supplied("max_points"))
    throw HttpError(400, "tile cannot be combined with max_points");

  const long long value = params.integer("tile", -1);
  if (value < 0)
    throw HttpError(400, "tile must be >= 0, got " + std::to_string(value));
  request.active = true;
  request.tile_id = static_cast<uint32_t>(value);
  request.lod_source = params.str("lod_source", "");

  // Within-tile slice for multi-level LOD. offset and limit are validated but
  // not applied to the global page: they address the tile's point sequence.
  if (supplied("offset")) {
    const long long off = params.integer("offset", 0);
    if (off < 0)
      throw HttpError(400, "offset must be >= 0");
    request.tile_offset = static_cast<uint64_t>(off);
  }
  if (supplied("limit")) {
    const long long lim = params.integer("limit", 0);
    if (lim < 0)
      throw HttpError(400, "limit must be >= 0");
    request.tile_limit = static_cast<uint64_t>(lim);
  }
  return request;
}

/// Read one spatial tile by scanning for points with matching sort_key.
///
/// When `request.lod_source` is set the cloud being served has no positions
/// (e.g. a Label cloud); tile membership is determined from the source cloud's
/// tile index and the matching storage indices are used to gather the sibling.
/// `request.tile_offset`/`tile_limit` select a within-tile slice (#396).
reusex::ProjectDB::CloudPage resolve_tile(const reusex::ProjectDB &db,
                                          const std::string &name,
                                          const TileRequest &request) {
  if (!request.lod_source.empty()) {
    // Companion fetch: gather the label (or other position-free) cloud at the
    // same storage indices as the geometry tile — the only way to keep it
    // index-aligned without positions of its own.
    const auto src_blob = db.tile_index(request.lod_source);
    if (src_blob.empty())
      throw HttpError(404, "no tile index for source cloud '" +
                               request.lod_source + "'");
    const auto [src_hdr, src_tiles] = parse_tile_index(src_blob);
    const uint32_t K = 1u << src_hdr.tile_bits;
    if (request.tile_id >= K)
      throw HttpError(400, "tile " + std::to_string(request.tile_id) +
                               " out of range; source cloud '" +
                               request.lod_source + "' has " +
                               std::to_string(K) + " tiles");

    const auto probe = db.point_cloud_page(name, 0, 0);
    if (probe.total != src_hdr.point_count)
      throw HttpError(400, "cloud '" + name + "' has " +
                               std::to_string(probe.total) +
                               " points but source '" + request.lod_source +
                               "' tile index was built for " +
                               std::to_string(src_hdr.point_count) +
                               " points — re-run create clouds");

    // Skip/limit must match geometry so label and geometry remain
    // index-aligned.
    const auto indices =
        gather_tile_indices(db, request.lod_source, src_hdr, request.tile_id,
                            request.tile_offset, request.tile_limit);
    return gather_points(db, name, indices);
  }

  const auto blob = db.tile_index(name);
  if (blob.empty())
    throw HttpError(404, "no tile index for cloud '" + name + "'");

  const auto [hdr, tiles] = parse_tile_index(blob);

  // Stale-index guard: a cloud rewrite clears tile_index (ProjectDB upsert),
  // but a stale blob from before schema v16 could slip through. 404 so the
  // client falls back to sequential paging rather than serving wrong tiles.
  const auto probe = db.point_cloud_page(name, 0, 0);
  if (probe.total != hdr.point_count)
    throw HttpError(404, "tile index for cloud '" + name + "' was built for " +
                             std::to_string(hdr.point_count) +
                             " points but the cloud now has " +
                             std::to_string(probe.total) +
                             " — re-run create clouds");

  const uint32_t K = 1u << hdr.tile_bits;
  if (request.tile_id >= K)
    throw HttpError(400, "tile " + std::to_string(request.tile_id) +
                             " out of range; cloud '" + name + "' has " +
                             std::to_string(K) + " tiles");

  return gather_tile_points(db, name, hdr, request.tile_id, request.tile_offset,
                            request.tile_limit);
}

/// Serialize one already-read page as the JSON `CloudPointsPage` object.
json points_json_from_page(const std::string &name,
                           const reusex::ProjectDB::CloudPage &page) {
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

} // namespace

json cloud_points_json(const reusex::ProjectDB &db, const std::string &name,
                       const Params &params) {
  if (!db.has_point_cloud(name))
    not_found("cloud", name);

  const auto tile = parse_tile_request(params);
  if (tile.active) {
    auto body = points_json_from_page(name, resolve_tile(db, name, tile));
    // A tile slice (limit > 0) is partial — more data is available for this
    // tile. A full-resolution tile fetch (limit == 0) is complete.
    body["lod"] = tile.tile_limit > 0;
    return body;
  }

  const auto lod = parse_lod_request(params);
  if (lod.active) {
    const auto result = resolve_lod(db, name, lod);
    auto body = points_json_from_page(name, result.page);
    // Always present when max_points was asked for, so a client can tell "the
    // whole cloud fit" from "here is a coarse view of it" without comparing
    // counts. The RUXP path says the same thing with a flag bit.
    body["lod"] = result.subsampled;
    if (result.subsampled)
      body["voxel_size"] = result.voxel_size;
    return body;
  }

  const auto request =
      parse_page_request(params, static_cast<long long>(kDefaultPointsPerPage),
                         static_cast<long long>(kMaxPointsPerPage));

  // Only the bytes this page occupies are read: point_cloud_page() maps the
  // window onto a byte range in the chunked point_cloud_data store and pulls
  // it with SQLite incremental blob I/O, so peak memory is O(page), not
  // O(cloud). The records come back in storage layout and are decoded inline
  // by points_json_from_page() — inflating a pcl::PointCloud would cost
  // 32 B/point of SSE-padded memory to produce a JSON array.
  return points_json_from_page(
      name, db.point_cloud_page(name, request.offset, request.limit));
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

  const auto tile = parse_tile_request(params);
  const auto lod = parse_lod_request(params);

  reusex::ProjectDB::CloudPage page;
  uint32_t flags = 0;
  if (tile.active) {
    page = resolve_tile(db, name, tile);
    // A tile slice (limit > 0) is partial: signal that more data is available.
    if (tile.tile_limit > 0)
      flags |= kRuxpFlagLod;
  } else if (lod.active) {
    auto result = resolve_lod(db, name, lod);
    // The flag is set only when points were actually dropped. A cloud that fit
    // the budget came back whole and in storage order, and claiming LOD would
    // tell the client to distrust an offset that is perfectly good.
    if (result.subsampled)
      flags |= kRuxpFlagLod;
    page = std::move(result.page);
  } else {
    const auto request = parse_page_request(
        params, static_cast<long long>(kDefaultPointsPerPage),
        static_cast<long long>(kMaxPointsPerPage));
    page = db.point_cloud_page(name, request.offset, request.limit);
  }

  if (!ruxp_supports(page.point_type))
    throw HttpError(500, "unsupported cloud type '" + page.point_type + "'");

  PointsResponse response;
  response.blob = Blob{"application/octet-stream", encode_ruxp(page, flags)};
  // Mirrors of the body header, for curl-level debugging only.
  response.headers = {{"X-Ruxp-Version", std::to_string(kRuxpVersion)},
                      {"X-Ruxp-Type", page.point_type},
                      {"X-Ruxp-Flags", std::to_string(flags)},
                      {"X-Ruxp-Offset", std::to_string(page.offset)},
                      {"X-Ruxp-Count", std::to_string(page.count)},
                      {"X-Ruxp-Total", std::to_string(page.total)}};
  return response;
}

// ===========================================================================
// meshes
// ===========================================================================

json meshes_json(const reusex::ProjectDB &db, const Params &params) {
  // mesh_json() costs two queries per mesh (metadata + texture list), so the
  // page is applied to the name list first and only the page is inflated.
  return paged_collection(
      db.list_meshes(), "meshes", params,
      [&db](const std::string &name) { return mesh_json(db, name); });
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

  std::vector<int> matching;
  for (int id : db.sensor_frame_ids()) {
    if (segmented && with_masks.count(id) != static_cast<size_t>(*segmented))
      continue;
    matching.push_back(id);
  }

  json out = paged_collection(
      matching, "ids", params, [](int id) { return json(id); }, kMaxFrameIds);

  // TWO DIFFERENT TOTALS, deliberately. The envelope's `total` (added above)
  // counts the ids matching `segmented` — what the client is paging through.
  // These two describe the WHOLE scan and ignore both filter and page, because
  // a browser showing "12 of 380 segmented" needs the denominator and should
  // not have to issue a second, unfiltered request to get it. They agree with
  // `total` only when no filter is set, and that agreement is a coincidence.
  out["total_count"] = summary.sensor_frames.total_count;
  out["segmented_count"] = summary.sensor_frames.segmented_count;
  out["width"] = summary.sensor_frames.width;
  out["height"] = summary.sensor_frames.height;

  // Per-scan grouping (#462). Omit when the project has no scans table (schema
  // < v17) — those projects already have a flat id list and nothing breaks.
  if (!summary.sensor_frames.scans.empty()) {
    const auto id_scan_pairs = db.sensor_frame_ids_with_scan();
    std::map<int, int> id_to_scan;
    for (const auto &[node_id, scan_id] : id_scan_pairs)
      id_to_scan[node_id] = scan_id;

    std::map<int, std::vector<int>> ids_by_scan;
    for (int id : matching) {
      const auto it = id_to_scan.find(id);
      ids_by_scan[it != id_to_scan.end() ? it->second : 0].push_back(id);
    }

    json scans_arr = json::array();
    for (const auto &scan : summary.sensor_frames.scans) {
      const auto it = ids_by_scan.find(scan.scan_id);
      json entry;
      entry["scan_id"] = scan.scan_id;
      entry["source_path"] = scan.source_path;
      entry["imported_at"] = scan.imported_at;
      entry["ids"] = it != ids_by_scan.end() ? json(it->second) : json::array();
      scans_arr.push_back(std::move(entry));
    }
    out["scans"] = std::move(scans_arr);
  }

  return out;
}

json frames_visibility_json(const reusex::ProjectDB &db, const Params &params) {
  const double x = require_double(params, "x");
  const double y = require_double(params, "y");
  const double z = require_double(params, "z");
  const auto frames = reusex::core::visible_frames(db, Eigen::Vector3d(x, y, z),
                                                   visibility_query_of(params));
  return visibility_json({x, y, z}, frames, params);
}

json instance_frames_json(const reusex::ProjectDB &db, const std::string &cloud,
                          int instance_id, const Params &params) {
  if (!db.has_point_cloud(cloud))
    not_found("cloud", cloud);

  // The instance-label cloud stores labels only; its positions are the base
  // `cloud` it was segmented from, index-aligned point for point (the
  // `instances` stage reads exactly that — see run_instances). Without it there
  // is nowhere to take a centroid from.
  static constexpr const char *kPositions = "cloud";
  if (!db.has_point_cloud(kPositions))
    throw HttpError(409, std::string("instance visibility needs the base '") +
                             kPositions +
                             "' point cloud for positions, which this project "
                             "does not have");

  const auto positions = db.point_cloud_xyzrgb(kPositions);
  const auto labels = db.point_cloud_label(cloud);
  if (!positions || !labels)
    throw HttpError(500, "could not load positions or instance labels");
  if (positions->size() != labels->size())
    throw HttpError(409, "instance-label cloud '" + cloud + "' has " +
                             std::to_string(labels->size()) +
                             " points but base cloud '" +
                             std::string(kPositions) + "' has " +
                             std::to_string(positions->size()) +
                             " — they are not index-aligned");

  // Centroid of the points carrying this instance id. Non-finite points are
  // skipped so one NaN cannot poison the average.
  double sx = 0.0, sy = 0.0, sz = 0.0;
  std::size_t n = 0;
  for (std::size_t i = 0; i < labels->size(); ++i) {
    if (labels->points[i].label != static_cast<uint32_t>(instance_id))
      continue;
    const auto &p = positions->points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    sx += p.x;
    sy += p.y;
    sz += p.z;
    ++n;
  }
  if (n == 0)
    not_found("instance", cloud + "/" + std::to_string(instance_id));

  const double cx = sx / static_cast<double>(n);
  const double cy = sy / static_cast<double>(n);
  const double cz = sz / static_cast<double>(n);
  const auto frames = reusex::core::visible_frames(
      db, Eigen::Vector3d(cx, cy, cz), visibility_query_of(params));

  json out = visibility_json({cx, cy, cz}, frames, params);
  out["cloud"] = cloud;
  out["instance_id"] = instance_id;
  out["instance_point_count"] = n;
  return out;
}

json frame_json(const reusex::ProjectDB &db, int id) {
  if (!db.has_sensor_frame(id))
    not_found("sensor frame", std::to_string(id));

  // `pose` stays the verbatim stored value — this is a read-out of what the
  // project contains, and a client debugging a bad import needs to see the
  // bytes rather than a sanitised substitute. `has_pose` is what lets it tell
  // a real identity pose from `sensor_frame_pose()`'s identity fallback
  // (#336); the pipeline stages gate on the same predicate.
  json out{{"id", id},
           {"timestamp", db.sensor_frame_timestamp(id)},
           {"pose", pose_array(db.sensor_frame_pose(id))},
           {"has_pose", db.has_sensor_frame_pose(id)},
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
// frame segmentation (#409)
// ===========================================================================

json segment_frame_result_json(int frame_id, const cv::Mat &label_map,
                               const std::vector<std::string> &class_names,
                               bool saved) {
  const int labeled = label_map.empty() ? 0 : cv::countNonZero(label_map != -1);
  json out{
      {"frame_id", frame_id}, {"saved", saved}, {"labeled_pixels", labeled}};
  json labels_obj = json::object();
  for (std::size_t i = 0; i < class_names.size(); ++i)
    labels_obj[std::to_string(i)] = class_names[i];
  out["labels"] = std::move(labels_obj);
  return out;
}

// ===========================================================================
// panoramas
// ===========================================================================

namespace {
json panorama_entry_json(const reusex::ProjectDB &db,
                         const reusex::ProjectDB::PanoramicImage &pano) {
  json out{{"id", pano.id},
           {"filename", pano.filename},
           {"timestamp", pano.timestamp},
           {"node_id", pano.node_id},
           {"has_pose", pano.has_pose},
           {"pose", pose_array(pano.pose)},
           {"pose_source", pano.pose_source},
           {"align_inliers", pano.align_inliers},
           {"align_rms", pano.align_rms}};

  // The pose of the timestamp-matched sensor frame, as a SEPARATE field.
  //
  // `pose` is the content-aligned pose and is identity until `rux align 360`
  // has run — which is the state of every panorama in a project that has only
  // been imported. A client that wants to place those panoramas in space has
  // no other source than the frame they were matched to, and reaching it
  // itself costs one `GET /frames/{node_id}` per panorama, each of which
  // decodes that frame's depth and confidence blobs to answer `has_depth`.
  //
  // Merging the two into one `pose` was rejected: `has_pose`/`pose_source`
  // exist precisely so a caller can tell a resected pose from a borrowed one,
  // and a client that draws them identically is claiming an accuracy it does
  // not have. Two fields, and the caller decides.
  if (pano.node_id >= 0 && db.has_sensor_frame_pose(pano.node_id)) {
    out["frame_pose"] = pose_array(db.sensor_frame_pose(pano.node_id));
    out["has_frame_pose"] = true;
  } else {
    out["has_frame_pose"] = false;
  }
  return out;
}
} // namespace

json panoramas_json(const reusex::ProjectDB &db, const Params &params) {
  return paged_collection(
      db.list_panoramic_images(), "panoramas", params,
      [&db](const auto &pano) { return panorama_entry_json(db, pano); });
}

json panorama_json(const reusex::ProjectDB &db, int id) {
  for (const auto &pano : db.list_panoramic_images())
    if (pano.id == id)
      return panorama_entry_json(db, pano);
  not_found("panorama", std::to_string(id));
}

Blob panorama_image_blob(const reusex::ProjectDB &db, int id,
                         const Params &params) {
  cv::Mat image = db.panoramic_image(id);
  if (image.empty())
    not_found("panorama", std::to_string(id));

  image = resized_to(image, max_size_param(params), /*is_label=*/false);

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

  // `total` counts the components matching `type`, not every component in the
  // project — the filter is applied by the query above, so the page is over
  // the filtered set. component_record() is a query per name, so again only
  // the page is inflated.
  return paged_collection(
      names, "components", params, [&db](const std::string &name) {
        return component_summary_json(db.component_record(name));
      });
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

json materials_json(const reusex::ProjectDB &db, const Params &params) {
  const auto summary = db.project_summary();
  return paged_collection(
      summary.materials, "materials", params,
      [](const auto &material) { return material_info_json(material); });
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
           {"properties", std::move(props)},
           {"has_thumbnail", db.material_thumbnail(guid).has_value()}};

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

namespace {

/// Serialize one user-defined material column definition (schema v18).
json definition_json(const reusex::ProjectDB::PropertyDefinition &d) {
  return json{{"id", d.id},
              {"name", d.name},
              {"type", d.type},
              {"options", d.options},
              {"sort_order", d.sort_order},
              {"width", d.width}};
}

/// The column value types the editor understands. Anything else is a 400 —
/// a column with an unknown type would have no editor widget to render it.
bool is_valid_column_type(const std::string &type) {
  return type == "text" || type == "number" || type == "date" ||
         type == "boolean" || type == "select" || type == "multiselect" ||
         type == "url";
}

} // namespace

json create_material(reusex::ProjectDB &db) {
  const std::string guid = reusex::core::generate_guid();
  const std::string created_at = pipeline::iso8601_utc_now();

  reusex::core::MaterialPassport passport;
  passport.metadata.document_guid = guid;
  passport.metadata.creation_date = created_at;
  passport.metadata.version_number = "0.1.0";

  db.add_material_passport(passport, "");

  return json{{"guid", guid},
              {"id", guid},
              {"properties", json::object()},
              {"has_thumbnail", false},
              {"created_at", created_at},
              {"version_number", "0.1.0"}};
}

void delete_material(reusex::ProjectDB &db, const std::string &guid) {
  // A missing passport is a 404, not a 500: delete_material_passport throws
  // when the guid is unknown, which is precisely "no such passport".
  try {
    db.delete_material_passport(guid);
  } catch (const std::exception &e) {
    throw HttpError(404, e.what());
  }
}

Blob material_thumbnail_blob(const reusex::ProjectDB &db,
                             const std::string &guid) {
  const auto thumb = db.material_thumbnail(guid);
  if (!thumb.has_value())
    throw HttpError(404, "no thumbnail stored for material '" + guid + "'");

  Blob blob;
  blob.content_type = thumb->second;
  blob.data = thumb->first;
  return blob;
}

void set_material_thumbnail(reusex::ProjectDB &db, const std::string &guid,
                            const std::string &body, const std::string &mime) {
  // Default to JPEG when the client sends no Content-Type: the thumbnail is
  // shown, not parsed, and a wrong-but-plausible mime is better than a refused
  // upload the user cannot diagnose.
  const std::string content_type = mime.empty() ? "image/jpeg" : mime;
  const std::vector<std::uint8_t> bytes(body.begin(), body.end());
  db.set_material_thumbnail(guid, bytes, content_type);
}

json material_columns_json(const reusex::ProjectDB &db) {
  json list = json::array();
  for (const auto &def : db.list_property_definitions())
    list.push_back(definition_json(def));
  return list;
}

json create_material_column(reusex::ProjectDB &db, const std::string &body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  auto name_it = parsed.find("name");
  if (name_it == parsed.end() || !name_it->is_string())
    throw HttpError(400, "'name' is required and must be a string");
  auto type_it = parsed.find("type");
  if (type_it == parsed.end() || !type_it->is_string())
    throw HttpError(400, "'type' is required and must be a string");

  const auto name = name_it->get<std::string>();
  const auto type = type_it->get<std::string>();
  if (!is_valid_column_type(type))
    throw HttpError(400, "'type' must be one of "
                         "text/number/date/boolean/select/multiselect");

  std::vector<std::string> options;
  auto options_it = parsed.find("options");
  if (options_it != parsed.end() && options_it->is_array())
    for (const auto &option : *options_it)
      if (option.is_string())
        options.push_back(option.get<std::string>());

  int sort_order = 0;
  auto sort_it = parsed.find("sort_order");
  if (sort_it != parsed.end() && sort_it->is_number_integer())
    sort_order = sort_it->get<int>();

  int width = 200;
  auto width_it = parsed.find("width");
  if (width_it != parsed.end() && width_it->is_number_integer())
    width = width_it->get<int>();

  reusex::ProjectDB::PropertyDefinition created;
  created.id =
      db.add_property_definition(name, type, options, sort_order, width);
  created.name = name;
  created.type = type;
  created.options = options;
  created.sort_order = sort_order;
  created.width = width;
  return definition_json(created);
}

json patch_material_column(reusex::ProjectDB &db, const std::string &id,
                           const std::string &body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  const auto defs = db.list_property_definitions();
  const auto it = std::find_if(defs.begin(), defs.end(),
                               [&](const auto &d) { return d.id == id; });
  if (it == defs.end())
    not_found("material column", id);

  // Sparse update: start from the stored values and overlay only the fields the
  // request carries, so an absent field keeps what it had rather than being
  // cleared to a default.
  std::string name = it->name;
  std::string type = it->type;
  std::vector<std::string> options = it->options;
  int sort_order = it->sort_order;
  int width = it->width;

  if (parsed.contains("name")) {
    if (!parsed["name"].is_string())
      throw HttpError(400, "'name' must be a string");
    name = parsed["name"].get<std::string>();
  }
  if (parsed.contains("type")) {
    if (!parsed["type"].is_string())
      throw HttpError(400, "'type' must be a string");
    type = parsed["type"].get<std::string>();
    if (!is_valid_column_type(type))
      throw HttpError(400, "'type' must be one of "
                           "text/number/date/boolean/select/multiselect");
  }
  if (parsed.contains("options")) {
    if (!parsed["options"].is_array())
      throw HttpError(400, "'options' must be an array");
    options.clear();
    for (const auto &option : parsed["options"])
      if (option.is_string())
        options.push_back(option.get<std::string>());
  }
  if (parsed.contains("sort_order")) {
    if (!parsed["sort_order"].is_number_integer())
      throw HttpError(400, "'sort_order' must be an integer");
    sort_order = parsed["sort_order"].get<int>();
  }
  if (parsed.contains("width")) {
    if (!parsed["width"].is_number_integer())
      throw HttpError(400, "'width' must be an integer");
    width = parsed["width"].get<int>();
  }

  db.update_property_definition(id, name, type, options, sort_order, width);

  reusex::ProjectDB::PropertyDefinition updated;
  updated.id = id;
  updated.name = name;
  updated.type = type;
  updated.options = options;
  updated.sort_order = sort_order;
  updated.width = width;
  return definition_json(updated);
}

void delete_material_column(reusex::ProjectDB &db, const std::string &id) {
  try {
    db.delete_property_definition(id);
  } catch (const std::exception &e) {
    throw HttpError(404, e.what());
  }
}

json instances_json(const reusex::ProjectDB &db, const std::string &cloud,
                    const Params &params) {
  if (!db.has_point_cloud(cloud))
    not_found("cloud", cloud);

  const auto links = db.instance_materials(cloud);
  json out = paged_collection(
      db.instances(cloud), "instances", params, [&links](const auto &record) {
        json entry{{"instance_id", record.instance_id},
                   {"guid", record.guid},
                   {"semantic_class", record.semantic_class},
                   {"point_count", record.point_count}};
        auto link = links.find(static_cast<int>(record.instance_id));
        entry["material_guid"] =
            link == links.end() ? json(nullptr) : json(link->second);
        return entry;
      });
  out["cloud"] = cloud;
  return out;
}

json link_instance_material(reusex::ProjectDB &db, const std::string &cloud,
                            int instance_id, const std::string &body) {
  if (!db.has_point_cloud(cloud))
    not_found("cloud", cloud);

  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  auto guid_it = parsed.find("guid");
  if (guid_it == parsed.end() || !guid_it->is_string())
    throw HttpError(400, "'guid' is required and must be a string");

  const auto guid = guid_it->get<std::string>();

  try {
    db.set_instance_material(cloud, instance_id, guid);
  } catch (const std::exception &e) {
    throw HttpError(404, e.what());
  }

  const auto records = db.instances(cloud);
  const auto it = std::find_if(
      records.begin(), records.end(), [instance_id](const auto &r) {
        return static_cast<int>(r.instance_id) == instance_id;
      });
  if (it == records.end())
    not_found("instance", cloud + "/" + std::to_string(instance_id));

  const auto links = db.instance_materials(cloud);
  json entry{{"instance_id", it->instance_id},
             {"guid", it->guid},
             {"semantic_class", it->semantic_class},
             {"point_count", it->point_count}};
  auto link = links.find(static_cast<int>(it->instance_id));
  entry["material_guid"] =
      link == links.end() ? json(nullptr) : json(link->second);
  return entry;
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
  // Validate before querying: a malformed `offset`/`limit` should cost a 400,
  // not a database read first (STANDARDS §5, fail fast).
  (void)parse_page_request(params, kDefaultLogEntries, kMaxLogEntries);

  // ProjectDB's query takes a row limit but no offset, so the whole retainable
  // window is fetched once and paged here. Always fetching kMaxLogEntries
  // rather than offset+limit keeps `total` meaningful: it is the number of
  // rows this server is willing to serve, so `offset + count < total` works as
  // the "there is more" test at every offset.
  //
  // `total` therefore SATURATES at kMaxLogEntries. A project with a longer
  // history reports the maximum, not its true length — the contract says so
  // rather than pretending a count query exists.
  const auto rows = db.pipeline_log(kMaxLogEntries);

  // Default of 100 rather than the maximum: this is a history view, and the
  // first screen of one is almost never the whole of it.
  return paged_collection(
      rows, "entries", params,
      [](const auto &entry) {
        return json{{"id", entry.id},
                    {"stage", entry.stage},
                    {"status", entry.status},
                    {"started_at", entry.started_at},
                    {"finished_at", entry.finished_at},
                    {"parameters", entry.parameters},
                    {"error_msg", entry.error_msg}};
      },
      kMaxLogEntries, kDefaultLogEntries);
}

// ===========================================================================
// pose graph
// ===========================================================================

json posegraph_json(const reusex::ProjectDB &db) {
  // Nodes: every sensor frame that has a stored pose.  Positions are world-
  // space; the frontend maps them directly to 3D scene coordinates.
  json nodes = json::array();
  for (int id : db.sensor_frame_ids()) {
    if (!db.has_sensor_frame_pose(id))
      continue;
    const auto pose = db.sensor_frame_pose(id);
    nodes.push_back(json{{"id", id}, {"pose", pose}});
  }

  // Edges: stored by `rux optimize`; empty until the stage has run once.
  json edges = json::array();
  for (const auto &e : db.list_pose_graph_edges()) {
    json entry{{"from", e.from_node_id},
               {"to", e.to_node_id},
               {"type", e.edge_type},
               {"residual", e.residual}};
    if (!std::isnan(e.weight))
      entry["weight"] = e.weight;
    edges.push_back(std::move(entry));
  }

  return json{{"nodes", std::move(nodes)}, {"edges", std::move(edges)}};
}

namespace {
/// Valid edge types accepted by the editor endpoints.
bool is_valid_edge_type(std::string_view t) {
  return t == "odometry" || t == "loop_closure" || t == "panorama";
}
} // namespace

json delete_posegraph_edge(reusex::ProjectDB &db, int from, int to,
                           std::string_view edge_type) {
  if (!edge_type.empty() && !is_valid_edge_type(edge_type))
    throw HttpError(400,
                    "'type' must be one of odometry/loop_closure/panorama");

  // The underlying delete will throw if the table doesn't exist, but since
  // migration always creates it, the only failure mode worth reporting is no
  // matching rows — which we surface as 404 below.
  const int deleted = db.delete_pose_graph_edges(from, to, edge_type);

  if (deleted == 0)
    throw HttpError(
        404, "no pose-graph edge found with from=" + std::to_string(from) +
                 " to=" + std::to_string(to) +
                 (edge_type.empty() ? "" : " type=" + std::string(edge_type)));

  return json{
      {"deleted", deleted},
      {"from", from},
      {"to", to},
      {"type", edge_type.empty() ? nullptr : json(std::string(edge_type))}};
}

json add_posegraph_edge(reusex::ProjectDB &db, const std::string &body) {
  auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
  if (parsed.is_discarded() || !parsed.is_object())
    throw HttpError(400, "request body must be a JSON object");

  auto from_it = parsed.find("from");
  if (from_it == parsed.end() || !from_it->is_number_integer())
    throw HttpError(400, "'from' is required and must be an integer");
  auto to_it = parsed.find("to");
  if (to_it == parsed.end() || !to_it->is_number_integer())
    throw HttpError(400, "'to' is required and must be an integer");

  const int from = from_it->get<int>();
  const int to = to_it->get<int>();

  std::string edge_type = "loop_closure";
  auto type_it = parsed.find("type");
  if (type_it != parsed.end() && !type_it->is_null()) {
    if (!type_it->is_string())
      throw HttpError(400, "'type' must be a string");
    edge_type = type_it->get<std::string>();
    if (!is_valid_edge_type(edge_type))
      throw HttpError(400,
                      "'type' must be one of odometry/loop_closure/panorama");
  }

  double weight = 1.0;
  auto weight_it = parsed.find("weight");
  if (weight_it != parsed.end() && !weight_it->is_null()) {
    if (!weight_it->is_number())
      throw HttpError(400, "'weight' must be a number");
    weight = weight_it->get<double>();
    if (weight <= 0.0)
      throw HttpError(400, "'weight' must be positive");
  }

  reusex::ProjectDB::PoseGraphEdge edge;
  edge.from_node_id = from;
  edge.to_node_id = to;
  edge.edge_type = edge_type;
  edge.residual = 0.0;
  edge.weight = weight;

  if (!db.has_pose_graph())
    throw HttpError(409, "no pose graph yet — run `rux optimize` first to "
                         "create the graph, then edit it");

  db.add_pose_graph_edge(edge);

  json out{{"from", from},
           {"to", to},
           {"type", edge_type},
           {"residual", 0.0},
           {"weight", weight}};
  return out;
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

  json out{{"id", record.id},
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

  // `result` is present only on success, and only because the stage reported
  // something. It names what this run actually wrote, so a client refreshes
  // that and leaves the rest of its screen alone instead of re-fetching every
  // collection on the chance that one of them changed.
  //
  // A failed or cancelled job says what happened in `error`; duplicating that
  // into a "result" would invite a UI to render a failure as an outcome.
  if (record.status == pipeline::JobStatus::succeeded) {
    json outputs = json::array();
    for (const auto &artifact : record.result_outputs) {
      json entry{{"kind", artifact.kind}, {"name", artifact.name}};
      // A negative count means the stage had no honest number for this
      // artifact. Omit the field rather than send -1 or 0: an absent count
      // reads as "not measured", and 0 would read as "wrote nothing".
      if (artifact.count >= 0)
        entry["count"] = artifact.count;
      outputs.push_back(std::move(entry));
    }
    out["result"] = json{{"summary", record.result_summary},
                         {"outputs", std::move(outputs)}};
  }
  return out;
}

json jobs_json(const std::vector<pipeline::JobRecord> &jobs,
               std::string_view project) {
  json list = json::array();
  for (const auto &record : jobs)
    list.push_back(job_json(record, project));
  return json{{"jobs", std::move(list)}};
}

json jobs_page_json(const std::vector<pipeline::JobRecord> &jobs,
                    std::string_view project, const Params &params) {
  return paged_collection(jobs, "jobs", params,
                          [project](const pipeline::JobRecord &record) {
                            return job_json(record, project);
                          });
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
