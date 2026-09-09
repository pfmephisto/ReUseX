// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The GUI API surface, expressed as plain functions over a ProjectDB.
//
// Everything here is deliberately HTTP-framework-free: handlers take a
// ProjectDB (plus already-parsed parameters) and return JSON or bytes, and
// signal failure by throwing HttpError. gui/Server.hpp is the only place that
// knows about Crow. That split is what makes the contract unit-testable
// (tests/unit/rux_gui/) without standing up a socket.
//
// The contract these functions implement is docs/gui/openapi.yaml. Any change
// here that alters a response shape is a change to that document too.

#include <reusex/pipeline/JobRunner.hpp>
#include <reusex/pipeline/stages.hpp>

#include <nlohmann/json.hpp>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace reusex {
class ProjectDB;
}

namespace rux::gui {

/// Contract version served under kApiPrefix. Bump only for breaking changes,
/// and take a new prefix when you do.
inline constexpr std::string_view kApiVersion = "1.0.0";
inline constexpr std::string_view kApiPrefix = "/api/v1";
/// Identifies which implementation of the contract is answering.
inline constexpr std::string_view kImplementation = "rux-gui";

/// Upper bound on `limit` for the paged points endpoint, so a stray query
/// cannot ask the server to materialize an unbounded response.
inline constexpr size_t kMaxPointsPerPage = 1000000;
inline constexpr size_t kDefaultPointsPerPage = 100000;

/// Upper bound on `limit` for the pipeline-log endpoint. `limit=0` means "as
/// many as the server will give", which is this — not "unbounded", which would
/// let one query serialize an entire project's history into memory.
inline constexpr int kMaxLogEntries = 1000;
inline constexpr int kDefaultLogEntries = 100;

/// Thrown by a handler to produce a non-200 JSON error response.
class HttpError : public std::runtime_error {
    public:
  HttpError(int status, std::string message)
      : std::runtime_error(std::move(message)), status_(status) {}

  int status() const noexcept { return status_; }

    private:
  int status_;
};

/// A non-JSON response body (image, mesh blob).
struct Blob {
  std::string content_type;
  std::vector<uint8_t> data;
};

/// One row of the route table. The table is the single source of truth for
/// both route registration and the self-describing /endpoints response.
struct Endpoint {
  std::string method;
  std::string path;    ///< Crow route pattern, e.g. "/api/v1/clouds/<string>".
  std::string summary; ///< Matches the OpenAPI `summary` for the same path.
  bool binary = false; ///< True when the response is not JSON.
};

/// Every endpoint this server serves, in documentation order.
const std::vector<Endpoint> &endpoint_table();

/// Already-decoded query parameters, so handlers never touch crow::request.
class Params {
    public:
  void set(std::string key, std::string value);
  std::optional<std::string> find(std::string_view key) const;

  /// String value, or @p fallback when absent/empty.
  std::string str(std::string_view key, std::string fallback) const;

  /// Integer value, or @p fallback when absent.
  /// @throws HttpError(400) when present but not a valid integer.
  long long integer(std::string_view key, long long fallback) const;

    private:
  std::map<std::string, std::string, std::less<>> values_;
};

// --- error / meta ---------------------------------------------------------

nlohmann::json error_json(int status, std::string_view message);

/// @param db  nullptr when the project could not be opened; the response then
///            reports `project.open == false` instead of failing outright, so
///            a browser pointed at a broken project still gets a usable page.
nlohmann::json health_json(const reusex::ProjectDB *db,
                           const std::filesystem::path &project);

nlohmann::json endpoints_json();

// --- project --------------------------------------------------------------

nlohmann::json project_summary_json(const reusex::ProjectDB &db);
nlohmann::json projects_json(const reusex::ProjectDB &db);

// --- clouds ---------------------------------------------------------------

nlohmann::json clouds_json(const reusex::ProjectDB &db);
nlohmann::json cloud_json(const reusex::ProjectDB &db, const std::string &name);

/// One page of point data as JSON, ignoring `format`.
///
/// Backed by ProjectDB::point_cloud_page(), so it reads only the bytes the
/// page occupies. Use cloud_points() to serve the endpoint — this is the
/// `format=json` half of it.
/// @throws HttpError(404) when @p name is not a stored cloud, HttpError(400)
///         on a malformed `offset`/`limit`.
nlohmann::json cloud_points_json(const reusex::ProjectDB &db,
                                 const std::string &name, const Params &params);

/// One points-endpoint response. Exactly one of @c body / @c blob is set,
/// decided by the `format` parameter — the route needs two different return
/// paths because a RUXP page is not JSON.
struct PointsResponse {
  std::optional<nlohmann::json> body; ///< Set on `format=json`.
  std::optional<Blob> blob;           ///< Set on `format=binary` (RUXP v1).
  /// `X-Ruxp-*` mirrors of the body header, set on the binary path only.
  /// A debugging convenience: the body header is the contract, and a client
  /// must not depend on these being present (docs/gui/binary-points.md).
  std::vector<std::pair<std::string, std::string>> headers;
};

/// Serve one page of point data in the requested wire format.
/// @throws HttpError(404) for an unknown cloud, HttpError(400) for a `format`
///         other than json|binary or a malformed `offset`/`limit`.
PointsResponse cloud_points(const reusex::ProjectDB &db,
                            const std::string &name, const Params &params);

// --- meshes ---------------------------------------------------------------

nlohmann::json meshes_json(const reusex::ProjectDB &db);
nlohmann::json mesh_json(const reusex::ProjectDB &db, const std::string &name);
nlohmann::json mesh_textures_json(const reusex::ProjectDB &db,
                                  const std::string &name);
Blob mesh_data_blob(const reusex::ProjectDB &db, const std::string &name);
Blob mesh_texture_blob(const reusex::ProjectDB &db, const std::string &name,
                       const std::string &texture);

// --- sensor frames --------------------------------------------------------

nlohmann::json frames_json(const reusex::ProjectDB &db);
nlohmann::json frame_json(const reusex::ProjectDB &db, int id);
/// @param kind one of color|depth|confidence|segmentation.
Blob frame_image_blob(const reusex::ProjectDB &db, int id,
                      const std::string &kind);

// --- panoramas ------------------------------------------------------------

nlohmann::json panoramas_json(const reusex::ProjectDB &db);
nlohmann::json panorama_json(const reusex::ProjectDB &db, int id);
Blob panorama_image_blob(const reusex::ProjectDB &db, int id);

// --- components / materials / instances -----------------------------------

nlohmann::json components_json(const reusex::ProjectDB &db,
                               const Params &params);
nlohmann::json component_json(const reusex::ProjectDB &db,
                              const std::string &name);
nlohmann::json materials_json(const reusex::ProjectDB &db);
nlohmann::json material_json(const reusex::ProjectDB &db,
                             const std::string &guid);
nlohmann::json instances_json(const reusex::ProjectDB &db,
                              const std::string &cloud);

// --- pipeline -------------------------------------------------------------

nlohmann::json stages_json(const reusex::ProjectDB &db);

/// Input-contract validation for a single stage.
///
/// Same record shape as one element of stages_json(), so a client refreshing
/// one card after a run does not have to reconcile two schemas.
/// @throws HttpError(404) when @p stage is not in the catalogue.
nlohmann::json stage_validation_json(const reusex::ProjectDB &db,
                                     const std::string &stage);
nlohmann::json pipeline_log_json(const reusex::ProjectDB &db,
                                 const Params &params);

// --- jobs -----------------------------------------------------------------

/// @param project  Name of the project the job belongs to. Present on every
///                 job and event so a client that later talks to a multi-
///                 project ruxd (Phase 6) does not need a new message shape.
nlohmann::json job_json(const reusex::pipeline::JobRecord &record,
                        std::string_view project);
nlohmann::json jobs_json(const std::vector<reusex::pipeline::JobRecord> &jobs,
                         std::string_view project);
nlohmann::json job_event_json(const reusex::pipeline::JobEvent &event,
                              std::string_view project);
nlohmann::json hello_json(const std::vector<reusex::pipeline::JobRecord> &jobs,
                          const std::filesystem::path &project);

/// A validated `POST /jobs` body.
struct JobSubmission {
  reusex::pipeline::JobStage stage = reusex::pipeline::JobStage::clouds;
  std::string parameters; ///< Serialized JSON object, "" when omitted.
  /// Project the client believes it is addressing. Optional; when present the
  /// server checks it against the project it actually has open, so a client
  /// pointed at the wrong server is told so instead of quietly running a stage
  /// against the wrong data.
  std::optional<std::string> project;
};

/// Parse and validate a job submission body.
/// @throws HttpError(400) on malformed JSON, a missing/unknown stage, or a
///         `parameters` value that is not an object.
JobSubmission parse_job_request(std::string_view body);

/// Reject a submission aimed at a different project.
/// @throws HttpError(409) when @p submission names a project that is not
///         @p open_project.
void check_job_project(const JobSubmission &submission,
                       std::string_view open_project);

/// Handle one client message on the WebSocket channel.
/// @param body      the raw text frame
/// @param subscribe called with the requested filter when the message is a
///                  `subscribe` (nullopt clears the filter)
/// @return the reply to send back, or nullopt when no reply is due.
std::optional<nlohmann::json> handle_ws_message(
    std::string_view body,
    const std::function<void(std::optional<std::string>)> &subscribe);

/// True when @p event should be delivered to a connection filtered to
/// @p subscription (nullopt = unfiltered, receives everything).
bool event_matches_subscription(const reusex::pipeline::JobEvent &event,
                                const std::optional<std::string> &subscription);

} // namespace rux::gui
