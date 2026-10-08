// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The web GUI's HTTP + WebSocket server (#265; `ruxd --local` until it moved
// into ruxd, where `ruxd --local` runs it).
//
// Implements docs/gui/openapi.yaml over a set of *cases* — `.rux` projects
// served under /api/v1/cases/{cid}/... (spec 2026-10-08, phase S2) — serves
// the frontend bundle as static files, and streams each case's pipeline job
// progress over /api/v1/cases/{cid}/events. Cases open lazily and close when
// idle (ProjectRegistry); jobs from every case share one bounded worker pool
// (reusex::pipeline::JobScheduler).
//
// Crow is an implementation detail: this header is pimpl'd so crow.h never
// reaches the rest of the app or the tests. The handler logic itself lives in
// gui/api.hpp, which is framework-free.

#include <api/api.hpp>
#include <api/cases.hpp>
#include <reusex/pipeline/JobStore.hpp>
#include <reusex/pipeline/stages.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Forward-declared so Server.hpp does not need to include FrameSegmenter.hpp
// (and hence OpenCV / reusex headers) just to store a pointer.
namespace ruxd::api {
class IFrameSegmenter;
class IPanoramaSegmenter;
class IModelProvider;
class IViewRenderer;
} // namespace ruxd::api

namespace ruxd::api {

class AuthService;

/// When the session cookie carries `Secure`.
enum class CookieSecure {
  /// Unless the server binds loopback — but always when the request arrived
  /// through a TLS proxy that says so (`X-Forwarded-Proto: https`).
  automatic,
  always,
  never,
};

/// Everything a server — `ruxd --local`, or the multi-user server — needs to
/// stand up.
struct ServerOptions {
  /// What `ruxd --local` serves: one `.rux` file, or a directory whose `.rux`
  /// files (and `<id>/project.rux` case directories) are the cases. See
  /// LocalCaseStore. Ignored when `case_store` is set.
  std::filesystem::path target;

  /// Where created and uploaded cases are stored, one directory per case.
  /// Empty: the `target` directory itself, or — for a lone `target` file —
  /// none, which makes the catalogue read-only.
  std::filesystem::path data_dir;

  /// The case catalogue, injected (tests; the Postgres-backed catalogue of
  /// phase S3). nullptr = a LocalCaseStore over `target` and `data_dir`.
  std::shared_ptr<ICaseStore> case_store;

  /// Pipeline worker threads shared by every case (`--job-workers`). At most
  /// one job per case runs at a time whatever this is. Default 1: the GPU is
  /// shared, and the stages are parallel internally.
  std::size_t job_workers = 1;

  /// Most cases open at once (`--max-open-cases`).
  std::size_t max_open_cases = 16;

  /// How long an unused case stays open (`--case-idle-minutes`).
  std::chrono::seconds case_idle_timeout{std::chrono::minutes(10)};

  /// Limits on `.rux` uploads (`POST /api/v1/uploads`).
  UploadLimits upload_limits;

  /// Interface to bind. Defaults to loopback: without `auth_token` this
  /// server has **no authentication** and executes pipeline stages, so a bind
  /// beyond loopback is refused unless `auth_token` is set.
  std::string bind_address = "127.0.0.1";

  /// Server mode (phase S3): users, sessions, API tokens and case
  /// membership. nullptr = local mode, whose one implicit user (`local`)
  /// owns every case and never logs in. With it, every API route but
  /// health, readiness, the route table and login needs a session or a
  /// token; every case route checks membership; mutations are audited.
  std::shared_ptr<AuthService> auth;

  /// Where job records go (server mode: Postgres). nullptr = in memory.
  std::shared_ptr<reusex::pipeline::IJobStore> job_store;

  /// Whether the session cookie is `Secure` (server mode).
  CookieSecure cookie_secure = CookieSecure::automatic;

  /// Reverse proxies (CIDRs or addresses) whose `X-Forwarded-For` names the
  /// client (`--trusted-proxy`); from anyone else the header is ignored.
  /// Used for the login back-off.
  std::vector<std::string> trusted_proxies;

  /// How long audit entries are kept (`--audit-retention-days`); 0 = for
  /// ever. Pruned hourly on the case registry's sweep.
  std::chrono::seconds audit_retention{std::chrono::hours(24 * 365)};

  /// Backs `GET /api/v1/readyz`: true when the server's backends are
  /// reachable. Empty = always ready (local mode has no backends).
  std::function<bool()> readiness;

  /// Local mode: a shared access token. Empty = no authentication, which is
  /// allowed only on a loopback bind (the constructor throws otherwise). In
  /// server mode: the superuser token (AuthOptions::superuser_token), which
  /// the server takes from `auth` — this field is then ignored.
  ///
  /// When set, every HTTP request and the WebSocket upgrade must present it:
  /// as `Authorization: Bearer <token>` (scripts), as the `ruxd_token` cookie,
  /// or as a `?token=` query parameter. A request that presents it in the
  /// query also gets the cookie set (HttpOnly, SameSite=Strict), so a browser
  /// opened at `http://host:port/?token=...` stays signed in for its `<img>`,
  /// `fetch` and WebSocket traffic, none of which can carry a header.
  std::string auth_token;

  /// TCP port. 0 is rejected — Crow gives no way to read back an
  /// ephemeral port, so a caller could not tell where to connect.
  uint16_t port = 8420;

  /// Crow worker threads. 0 means hardware concurrency.
  unsigned threads = 0;

  /// Extra browser origins permitted to call the API, beyond loopback (which
  /// is always allowed). Needed only when the frontend is served from
  /// somewhere that is not localhost. Cross-origin requests from anything not
  /// listed are refused with 403 — there is no authentication here, so a
  /// wildcard would let any page the user visits read the project and start
  /// pipeline stages.
  std::vector<std::string> allowed_origins;

  /// Frontend bundle directory. Empty selects the search order in
  /// gui/assets.hpp, falling back to the built-in placeholder page.
  std::filesystem::path asset_dir;

  /// Launch the system browser at the server URL once it is listening.
  bool open_browser = true;

  /// Custom stage executor injected by the app layer (#464).
  ///
  /// When set, the job runner uses this instead of
  /// pipeline::default_stage_executor(). ruxd's main (ruxd_lib) uses this to
  /// add the `optimize` stage, which calls reusex_slam (GTSAM) — a dep that
  /// must not enter ruxd_api_lib's link graph (it would bloat the light test
  /// binary). Leave empty to use the default executor (clouds/planes/rooms/
  /// instances/mesh only).
  reusex::pipeline::StageExecutor stage_executor;

  /// Server-wide default for CUDA inference on the segment endpoints (#467).
  /// Overridable per-request via the `use_cuda` body field.
  /// Set to false with `--no-segment-cuda` when no GPU is present.
  bool segment_cuda = true;

  /// ICP refine callback injected by the app layer (#465).
  ///
  /// When set, POST /api/v1/posegraph/icp runs depth-based ICP between two
  /// stored sensor frames to estimate their relative pose. ruxd_api_lib must
  /// not link PCL directly, so the implementation lives in ruxd_lib and is
  /// injected here. Leave empty to have the endpoint return HTTP 503.
  ruxd::api::IcpRefineFn icp_refine_fn;
};

/// Crow-backed implementation of the GUI API contract.
class Server {
    public:
  /// Builds the case catalogue, the job scheduler and the case registry, and
  /// registers every route. No case is opened until a request names it.
  /// @throws std::runtime_error if the options are invalid or the catalogue
  ///         cannot be built.
  explicit Server(ServerOptions options);
  ~Server();

  Server(const Server &) = delete;
  Server &operator=(const Server &) = delete;

  /// The resolved options, with `asset_dir` filled in by the search order.
  const ServerOptions &options() const noexcept;

  /// Where the server can be reached, e.g. "http://127.0.0.1:8420".
  std::string url() const;

  /// True when a frontend bundle was found; false when the placeholder page is
  /// being served instead.
  bool has_assets() const noexcept;

  /// The cases being served right now.
  std::vector<CaseInfo> cases() const;

  /// Serve until SIGINT/SIGTERM or stop(). Returns a process exit code.
  int run();

  /// Ask a running server to shut down, unblocking run().
  ///
  /// `ruxd --local` itself relies on Crow's own signal handling, so this exists
  /// for callers that drive run() on a thread — chiefly the socket-level tests,
  /// which need a real listening server and then need it to go away again.
  /// Safe to call from another thread; join the thread running run() before
  /// destroying the Server.
  void stop();

  /// Register a SAM3 segmenter for POST /api/v1/frames/<id>/segment (#409).
  ///
  /// Must be called before run(). The server does NOT take ownership of the
  /// pointer — the caller must keep it alive for the server's lifetime.
  /// When nullptr (the default), the endpoint returns HTTP 503.
  void set_segmenter(IFrameSegmenter *segmenter);

  /// Register a SAM3 segmenter for POST /api/v1/panoramas/<id>/segment (#448).
  ///
  /// Same ownership and lifetime rules as set_segmenter(). When nullptr (the
  /// default), the endpoint returns HTTP 503.
  void set_panorama_segmenter(IPanoramaSegmenter *segmenter);

  /// Register the managed-model provider that resolves an omitted `model_path`
  /// on the segment endpoints and backs GET /api/v1/models/sam3/status.
  ///
  /// Same ownership/lifetime rules as set_segmenter(). When nullptr (the
  /// default), a segment request without `model_path` returns HTTP 400 and the
  /// status route returns HTTP 501.
  void set_model_provider(IModelProvider *provider);

  /// Register the renderer for GET /api/v1/renders (#265 Phase 2 Task 8,
  /// Kortlægning evidence images).
  ///
  /// Same ownership/lifetime rules as set_segmenter(). When nullptr (the
  /// default), the endpoint returns HTTP 503.
  void set_view_renderer(IViewRenderer *renderer);

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ruxd::api
