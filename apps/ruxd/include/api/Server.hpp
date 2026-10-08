// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The web GUI's HTTP + WebSocket server (#265; `ruxd --local` until it moved
// into ruxd, where `ruxd --local` runs it).
//
// Implements docs/gui/openapi.yaml over one `.rux` project, serves the frontend
// bundle as static files, and streams pipeline job progress over
// /api/v1/events.
//
// Crow is an implementation detail: this header is pimpl'd so crow.h never
// reaches the rest of the app or the tests. The handler logic itself lives in
// gui/api.hpp, which is framework-free.

#include <api/api.hpp>
#include <reusex/pipeline/stages.hpp>

#include <cstdint>
#include <filesystem>
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

/// Everything `ruxd --local` needs to stand a server up.
struct ServerOptions {
  /// The single project this server is bound to for its lifetime.
  std::filesystem::path project;

  /// Interface to bind. Defaults to loopback: without `auth_token` this
  /// server has **no authentication** and executes pipeline stages, so a bind
  /// beyond loopback is refused unless `auth_token` is set.
  std::string bind_address = "127.0.0.1";

  /// Shared access token. Empty = no authentication, which is allowed only on
  /// a loopback bind (the constructor throws otherwise).
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
  /// Opens the project read-write once (creating/migrating it if needed), then
  /// constructs the job runner and registers every route.
  /// @throws std::runtime_error if the project cannot be opened or the options
  ///         are invalid.
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
