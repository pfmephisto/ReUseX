// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/Server.hpp"

#include "api/FrameSegmenter.hpp"
#include "api/ModelProvider.hpp"
#include "api/ProjectContext.hpp"
#include "api/ProjectRegistry.hpp"
#include "api/ViewRenderer.hpp"
#include "api/api.hpp"
#include "api/assets.hpp"
#include "api/case_meta.hpp"
#include "api/cases.hpp"
#include "api/edits.hpp"
#include "api/gsplat.hpp"
#include "api/local_mode.hpp"
#include "api/photo_cache.hpp"
#include "api/resources.hpp"
#include "api/survey.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/JobScheduler.hpp>

#include <crow.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>

#include <sys/wait.h>
#include <unistd.h>

namespace ruxd::api {
namespace {

namespace pipeline = reusex::pipeline;
using json = nlohmann::json;

crow::response json_response(int status, const json &body) {
  // dump(-1): compact. The pretty-printer added ~20% to every response for a
  // reader that is a browser, not a person; `curl | jq` is the debugging path.
  crow::response res(status, body.dump(-1));
  res.set_header("Content-Type", "application/json");
  return res;
}

crow::response blob_response(const Blob &blob) {
  crow::response res(200);
  res.body.assign(reinterpret_cast<const char *>(blob.data.data()),
                  blob.data.size());
  res.set_header("Content-Type", blob.content_type);
  return res;
}

crow::response error_response(int status, std::string_view message) {
  return json_response(status, error_json(status, message));
}

std::string to_lower(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

/// The media type of a Content-Type header, without parameters or case.
/// "application/json; charset=utf-8" -> "application/json".
std::string media_type_of(std::string_view header) {
  auto end = header.find(';');
  std::string value(header.substr(0, end));
  // Trim.
  const auto first = value.find_first_not_of(" \t");
  const auto last = value.find_last_not_of(" \t");
  if (first == std::string::npos)
    return {};
  return to_lower(value.substr(first, last - first + 1));
}

/// True for an Origin that is some form of loopback, on any port.
///
/// The frontend is served by this same server in production and by a Vite dev
/// server on another localhost port during development, so loopback is the
/// origin set that has to work out of the box. Anything else must be named
/// explicitly with --allow-origin.
bool is_loopback_origin(std::string_view origin) {
  static constexpr std::array<std::string_view, 6> kPrefixes{{
      "http://localhost",
      "http://127.0.0.1",
      "http://[::1]",
      "https://localhost",
      "https://127.0.0.1",
      "https://[::1]",
  }};
  for (std::string_view prefix : kPrefixes) {
    if (origin.rfind(prefix, 0) != 0)
      continue;
    const auto rest = origin.substr(prefix.size());
    // Either exactly the host, or the host followed by ":<port>".
    if (rest.empty())
      return true;
    if (rest.front() != ':')
      continue;
    const auto port = rest.substr(1);
    if (!port.empty() &&
        std::all_of(port.begin(), port.end(),
                    [](unsigned char c) { return std::isdigit(c) != 0; }))
      return true;
  }
  return false;
}

/// Reject a bind address that is not a plain host/IP literal.
///
/// The value ends up in the URL handed to the browser launcher; keeping it to
/// a conservative character set means nothing exotic can travel further even
/// though the launcher no longer goes through a shell.
bool is_plausible_host(std::string_view host) {
  if (host.empty() || host.size() > 255)
    return false;
  return std::all_of(host.begin(), host.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '.' || c == ':' || c == '-' ||
           c == '_' || c == '[' || c == ']';
  });
}

/// Open the system browser.
///
/// fork + exec rather than std::system: the URL embeds --bind, and handing a
/// caller-influenced string to /bin/sh is a command-injection waiting to
/// happen. execlp takes the URL as one argv entry, so quoting never enters
/// into it.
void launch_browser(const std::string &url) {
  std::thread([url] {
    const pid_t pid = ::fork();
    if (pid < 0) {
      spdlog::warn("Could not fork to open a browser; visit {} manually", url);
      return;
    }
    if (pid == 0) {
      ::execlp("xdg-open", "xdg-open", url.c_str(), nullptr);
      ::_exit(127); // Only reached if exec failed.
    }
    int status = 0;
    ::waitpid(pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
      spdlog::warn("Could not open a browser; visit {} manually", url);
  }).detach();
}

} // namespace

/// Crow's own logger, routed through spdlog with every query string redacted.
///
/// Crow logs each request and response line at INFO, URL included, straight
/// to stderr — and `/?token=<secret>` is a URL. Its INFO becomes spdlog debug
/// (`-vv`), so request logging stays available without the noise, and the
/// redaction keeps a token (or any other query value) out of every log.
class RedactingCrowLog final : public crow::ILogHandler {
    public:
  void log(const std::string &message, crow::LogLevel level) override {
    const std::string text = redact_query_strings(message);
    switch (level) {
    case crow::LogLevel::Debug:
      spdlog::trace("crow: {}", text);
      break;
    case crow::LogLevel::Info:
      spdlog::debug("crow: {}", text);
      break;
    case crow::LogLevel::Warning:
      spdlog::warn("crow: {}", text);
      break;
    case crow::LogLevel::Error:
      spdlog::error("crow: {}", text);
      break;
    case crow::LogLevel::Critical:
      spdlog::critical("crow: {}", text);
      break;
    }
  }
};

// ===========================================================================
// SecurityMiddleware
// ===========================================================================

/// Cross-origin policy and content-type enforcement for the whole HTTP API.
///
/// WHY THIS EXISTS: this server has no authentication and `POST /jobs`
/// executes pipeline stages. With `Access-Control-Allow-Origin: *` any page the
/// user happened to be visiting could read the whole project — and because a
/// `text/plain` POST is a CORS-*simple* request, it could also start a stage,
/// since the browser dispatches such a request before it ever looks at the
/// response headers. Binding to loopback does not help: the attacker's
/// JavaScript runs inside the user's own machine.
///
/// The policy is therefore two-layered:
///
///  0. **Host allowlist** (DNS-rebinding guard): the Host header must name this
///     server — see host_allowed(). Then, with --auth-token, the token
///     (Bearer, the per-port cookie, or `?token=`, which redirects).
///  1. **Origin allowlist.** Loopback, plus anything named with
///     --allow-origin, plus — with a token — the request's own origin. A
///     request carrying any other Origin is refused with 403 *before routing*,
///     so no handler runs and nothing is disclosed. This is server-side
///     enforcement, not a hint to the browser — it holds for simple requests,
///     which are dispatched before CORS is consulted.
///  2. **application/json required on the JSON-body verbs (POST, PATCH).** A
///     form or simple POST cannot set that header without triggering a
///     preflight, so this closes the CSRF-shaped hole that a `text/plain` POST
///     would otherwise leave. PUT and DELETE are exempt: both are preflighted
///     (already covered by (1)), and the sole PUT uploads a raw image whose
///     Content-Type is its image type, not JSON.
///
/// Responses to allowlisted origins echo that specific origin (never `*`) with
/// `Vary: Origin`.
///
/// KNOWN LIMITATION — preflighted cross-origin requests are not supported.
/// Crow 1.3 answers OPTIONS inside Router::handle_initial(), which runs from
/// handle_url(): the request line has been parsed but the headers have not.
/// The Origin is therefore unknowable at preflight time, there is no hook and
/// no opt-out, and decorating the response afterwards proved unreliable (the
/// first OPTIONS on a fresh connection loses the added headers). Rather than
/// ship an intermittent header, the supported model is same-origin: in
/// production this server serves the bundle itself, and in development the
/// Vite dev server proxies /api to it (`server.proxy`), which makes the
/// requests same-origin and removes CORS from the picture entirely. Simple
/// cross-origin GETs from an allowlisted origin do work. See docs/gui/README.
///
/// Middleware rather than per-route code so it cannot be forgotten on a route
/// added later.
class SecurityMiddleware {
    public:
  struct context {
    /// The request proved the token with `?token=`; answer with the cookie so
    /// the browser's later <img>/fetch/WebSocket requests carry it.
    bool set_token_cookie = false;
  };

  void configure(std::vector<std::string> extra_origins, std::string auth_token,
                 std::string bind_address, uint16_t port) {
    extra_origins_ = std::move(extra_origins);
    auth_token_ = std::move(auth_token);
    bind_address_ = std::move(bind_address);
    cookie_name_ = token_cookie_name(port);
  }

  /// The token check for @p req. Always ok when no token is configured.
  /// Shared with the WebSocket onaccept, which middleware cannot guard.
  TokenCheck check(const crow::request &req) const {
    if (auth_token_.empty())
      return {true, false};
    const char *query = req.url_params.get("token");
    return check_token(req.get_header_value("Authorization"),
                       req.get_header_value("Cookie"), query ? query : "",
                       cookie_name_, auth_token_);
  }

  /// DNS-rebinding guard: the Host header must name this server (see
  /// host_allowed() for the rules).
  bool host_ok(const crow::request &req) const {
    return host_allowed(req.get_header_value("Host"), bind_address_,
                        extra_origins_);
  }

  /// Loopback, an --allow-origin entry, or — when a token is configured —
  /// the request's own origin (`Origin` equal to `<scheme>://<Host>`), which
  /// is what a browser on another machine sends for the page this server
  /// served it.
  bool origin_ok(const crow::request &req, const std::string &origin) const {
    if (is_allowed(origin))
      return true;
    return !auth_token_.empty() &&
           origin_matches_host(origin, req.get_header_value("Host"));
  }

  std::string token_cookie() const {
    return cookie_name_ + "=" + auth_token_ +
           "; Path=/; HttpOnly; SameSite=Strict";
  }

  void before_handle(crow::request &req, crow::response &res, context &ctx) {
    // WebSocket upgrades bypass this: Crow calls handle_upgrade regardless of
    // whether middleware completed the response, so a rejection here would be
    // ignored. The checks for /events live in its onaccept handler.
    if (req.upgrade)
      return;

    if (!host_ok(req)) {
      spdlog::warn("Refused a request for host '{}'",
                   req.get_header_value("Host"));
      res = error_response(403, "host '" + req.get_header_value("Host") +
                                    "' is not served here");
      res.end();
      return;
    }

    // OPTIONS never reaches here — see the note in after_handle.
    const std::string origin = req.get_header_value("Origin");
    if (!origin.empty() && !origin_ok(req, origin)) {
      spdlog::warn("Refused a cross-origin request from '{}' to {}", origin,
                   req.url);
      res = error_response(403, "origin '" + origin +
                                    "' is not allowed; pass --allow-origin to "
                                    "permit it");
      res.end();
      return;
    }

    // Access token (`ruxd --local --auth-token`, required beyond loopback).
    // After the origin check so a foreign page learns nothing either way.
    const TokenCheck token = check(req);
    if (!token.ok) {
      res = error_response(401, "missing or wrong access token");
      res.set_header("WWW-Authenticate", "Bearer");
      res.end();
      return;
    }
    ctx.set_token_cookie = token.via_query;

    // A browser that arrived with `?token=`: set the cookie and send it on to
    // the same URL without the token, so the secret leaves the address bar,
    // the history and any Referer.
    if (token.via_query && req.method == crow::HTTPMethod::Get) {
      res = crow::response(303);
      res.set_header("Location", strip_query_param(req.raw_url, "token"));
      res.set_header("Set-Cookie", token_cookie());
      res.set_header("Referrer-Policy", "no-referrer");
      res.end();
      return;
    }

    // Require JSON on the verbs that carry a JSON body. A `text/plain` POST is
    // a CORS *simple request*, dispatched by the browser before it reads a
    // single response header, and demanding JSON is what a forged one cannot
    // satisfy. PATCH is folded in here too: it always carries a JSON patch, so
    // the check is free and keeps the CSRF floor uniform for the JSON verbs.
    //
    // PUT and DELETE are exempt on purpose. Both are preflighted (never a CORS
    // simple request), so the origin check above already covers them, and the
    // one PUT this server serves uploads a raw image body whose Content-Type is
    // its image type, not application/json — demanding JSON there would refuse
    // every legitimate thumbnail upload.
    if (requires_json_body(req.method)) {
      const auto media = media_type_of(req.get_header_value("Content-Type"));
      if (media != "application/json") {
        res = error_response(
            415, "Content-Type must be application/json, got '" +
                     (media.empty() ? std::string("(none)") : media) + "'");
        res.end();
        return;
      }
    }
  }

  static bool requires_json_body(crow::HTTPMethod method) {
    return method == crow::HTTPMethod::Post ||
           method == crow::HTTPMethod::Patch;
  }

  void after_handle(crow::request &req, crow::response &res, context &ctx) {
    if (req.upgrade)
      return;

    if (ctx.set_token_cookie)
      res.set_header("Set-Cookie", token_cookie());
    // No page this server sends may leak its URL to another site.
    res.set_header("Referrer-Policy", "no-referrer");
    apply_cors(req.get_header_value("Origin"), res);
  }

  bool is_allowed(const std::string &origin) const {
    if (is_loopback_origin(origin))
      return true;
    return std::find(extra_origins_.begin(), extra_origins_.end(), origin) !=
           extra_origins_.end();
  }

    private:
  /// Deliberately keyed on is_allowed() (loopback and --allow-origin), NOT on
  /// origin_matches_host() as origin_ok() is: a same-origin request needs no
  /// Access-Control-Allow-Origin at all, so echoing the page's own origin
  /// back would grant nothing a browser does not already allow — and keeping
  /// ACAO to the explicit allowlist means a token-mode server never advertises
  /// itself to an origin nobody named.
  void apply_cors(const std::string &origin, crow::response &res) const {
    if (origin.empty() || !is_allowed(origin))
      return;
    // Echo the specific origin, never "*": the allowlist is the policy, and
    // Vary tells caches the response depends on who asked.
    res.set_header("Access-Control-Allow-Origin", origin);
    res.set_header("Vary", "Origin");
  }

  std::vector<std::string> extra_origins_;
  std::string auth_token_;
  std::string bind_address_;
  std::string cookie_name_;
};

/// The concrete Crow application type for `ruxd --local`, with the security
/// middleware installed. Used everywhere instead of crow::SimpleApp.
using App = crow::App<SecurityMiddleware>;

// ===========================================================================
// Server::Impl
// ===========================================================================

class Server::Impl {
    public:
  explicit Impl(ServerOptions options) : options_(std::move(options)) {
    // Crow logs every URL, query included; route it through the redactor.
    static RedactingCrowLog crow_log;
    crow::logger::setHandler(&crow_log);

    if (options_.port == 0)
      throw std::runtime_error(
          "--port 0 is not supported: Crow cannot report back which ephemeral "
          "port it bound, so nothing could tell you where to connect");

    // Before any project is touched: a refused bind must not create or
    // migrate anything.
    if (options_.auth_token.empty() && !is_loopback_bind(options_.bind_address))
      throw std::runtime_error(
          "--bind '" + options_.bind_address +
          "' reaches beyond this machine, and the API can read and change the "
          "project and run pipeline stages: set --auth-token as well");

    if (!is_plausible_host(options_.bind_address))
      throw std::runtime_error("--bind '" + options_.bind_address +
                               "' is not a valid host or IP literal");

    // The case catalogue. Cases open lazily (ProjectRegistry), so a broken
    // project fails on its first request — with a 500 naming it — rather
    // than taking every other case down at startup.
    cases_ = options_.case_store ? options_.case_store
                                 : std::make_shared<LocalCaseStore>(
                                       options_.target, options_.data_dir);
    const auto listed = cases_->list();
    spdlog::info("Serving {} case(s){}", listed.size(),
                 cases_->writable() ? "" : " (read-only catalogue)");
    for (const auto &info : listed)
      spdlog::info("  case '{}': {}", info.id, info.path.filename().string());

    options_.asset_dir = resolve_asset_dir(options_.asset_dir);

    app_.get_middleware<SecurityMiddleware>().configure(
        options_.allowed_origins, options_.auth_token, options_.bind_address,
        options_.port);
    if (!options_.auth_token.empty())
      spdlog::info("Access token required (Bearer header, '{}' cookie or "
                   "?token=)",
                   token_cookie_name(options_.port));
    for (const auto &origin : options_.allowed_origins)
      spdlog::info("Additional allowed origin: {}", origin);

    executor_ = options_.stage_executor ? options_.stage_executor
                                        : pipeline::default_stage_executor();
    scheduler_ = std::make_unique<pipeline::JobScheduler>(
        pipeline::JobSchedulerOptions{options_.job_workers});
    spdlog::info("Job workers: {}", scheduler_->workers());

    RegistryOptions registry_options;
    registry_options.max_open = options_.max_open_cases;
    registry_options.idle_timeout = options_.case_idle_timeout;
    registry_ = std::make_unique<ProjectRegistry>(
        cases_,
        [this](const CaseInfo &info) {
          // No photo warm-up here: it starts on the case's first survey
          // request, so opening a case costs no photo work.
          return std::make_shared<ProjectContext>(info.id, info.path,
                                                  *scheduler_, executor_);
        },
        registry_options);
    uploads_ = std::make_unique<UploadManager>(cases_->staging_dir(),
                                               options_.upload_limits);
    // Abandoned uploads expire on the sweeper's beat, not only when the next
    // upload begins.
    registry_->set_maintenance([this] {
      try {
        if (const auto n = uploads_->expire(); n > 0)
          spdlog::info("Discarded {} abandoned upload(s)", n);
      } catch (const std::exception &e) {
        spdlog::warn("Expiring uploads failed: {}", e.what());
      }
    });

    register_routes();
  }

  ~Impl() {
    // Stop the producers before the things they touch go away: app_.stop()
    // closes the WebSocket connections, the registry closes every case (each
    // waits for its running job), and the scheduler joins its workers last.
    app_.stop();
    registry_.reset();
    scheduler_.reset();
  }

  const ServerOptions &options() const noexcept { return options_; }

  std::string url() const {
    // 0.0.0.0 is not a connectable address; point the user at loopback.
    const std::string host = options_.bind_address == "0.0.0.0"
                                 ? "127.0.0.1"
                                 : options_.bind_address;
    return "http://" + host + ":" + std::to_string(options_.port);
  }

  bool has_assets() const noexcept { return !options_.asset_dir.empty(); }

  std::vector<CaseInfo> cases() const { return cases_->list(); }

  int run() {
    // The events socket only ever receives tiny control frames (ping,
    // subscribe); Crow's default limit is unbounded.
    app_.websocket_max_payload(kMaxWebSocketPayload);
    app_.validate();
    app_.bindaddr(options_.bind_address).port(options_.port);
    if (options_.threads > 0)
      app_.concurrency(options_.threads);
    else
      app_.multithreaded();

    spdlog::info("ruxd --local listening on {}", url());
    if (has_assets())
      spdlog::info("Serving frontend assets from {}",
                   options_.asset_dir.string());
    else
      spdlog::info("No frontend bundle found; serving the built-in "
                   "placeholder page (see docs/gui/README.md)");

    // Crow installs its own SIGINT/SIGTERM handling. run_async +
    // wait_for_server_start lets the browser be launched only once the socket
    // is actually listening — launching before bind raced the browser against
    // the server and produced a spurious connection-refused page.
    auto serving = app_.run_async();
    if (app_.wait_for_server_start() != std::cv_status::no_timeout)
      spdlog::warn("Server did not report started within the timeout; "
                   "continuing anyway");
    else if (options_.open_browser)
      launch_browser(url());

    serving.wait();
    spdlog::info("ruxd --local shutting down");
    return 0;
  }

  void stop() { app_.stop(); }

  void set_segmenter(IFrameSegmenter *seg) noexcept { segmenter_ = seg; }
  void set_panorama_segmenter(IPanoramaSegmenter *seg) noexcept {
    panorama_segmenter_ = seg;
  }
  void set_model_provider(IModelProvider *provider) noexcept {
    model_provider_ = provider;
  }
  void set_view_renderer(IViewRenderer *renderer) noexcept {
    view_renderer_ = renderer;
  }

    private:
  /// A per-case route path: `/api/v1/cases/<string>` + @p rest.
  static std::string C(std::string_view rest) {
    return std::string(kCasePrefix) + std::string(rest);
  }

  /// Resolve the model path a segment request should load. An explicit
  /// (non-empty) path is used verbatim (back-compat). An omitted path is
  /// resolved to the managed SAM3 model, kicking off lazy background
  /// preparation; while that is in flight the request gets a 503 and the
  /// client polls GET /models/sam3/status.
  ///
  /// @throws HttpError(400) when no path is given and no provider is
  /// configured.
  /// @throws HttpError(503) while the managed model is downloading/building.
  /// @throws HttpError(500) when preparation has failed.
  std::string resolve_model_path(const std::string &requested, bool use_cuda) {
    if (!requested.empty())
      return requested;
    if (!model_provider_)
      throw HttpError(400, "'model_path' is required (no managed SAM3 model is "
                           "configured on this server)");
    const ModelPrepStatus st = model_provider_->ensure(use_cuda);
    if (st.state == "ready")
      return st.model_path;
    if (st.state == "error")
      throw HttpError(500, "SAM3 model preparation failed: " + st.message);
    // absent / downloading / building
    throw HttpError(503, "SAM3 model is being prepared (" + st.state +
                             "): " + st.message +
                             " — poll GET /api/v1/models/sam3/status");
  }

  // --- cases ---------------------------------------------------------------

  /// Run @p handler against case @p cid, opening it if needed. The context
  /// is leased for the duration of the call, which keeps it from being closed
  /// as idle underneath the request.
  template <typename Handler>
  crow::response in_case(const std::string &cid, Handler &&handler) {
    std::shared_ptr<ProjectContext> ctx;
    try {
      ctx = registry_->acquire(cid);
    } catch (const HttpError &e) {
      return error_response(e.status(), e.what());
    } catch (const std::exception &e) {
      spdlog::error("Could not open case '{}': {}", cid, e.what());
      return error_response(500,
                            "could not open case '" + cid + "': " + e.what());
    }
    if (!ctx)
      return error_response(404, "no such case '" + cid + "'");
    return handler(*ctx);
  }

  // --- ProjectDB access ----------------------------------------------------

  /// Run @p handler against a fresh read-only ProjectDB of @p ctx's project.
  ///
  /// ProjectDB is not thread-safe and Crow is multi-threaded, so each request
  /// gets its own connection rather than sharing one behind a mutex — sqlite3
  /// handles concurrent readers natively and a global lock would serialize the
  /// whole GUI behind whichever request is decoding a mesh blob.
  ///
  /// Writers are the case's job worker and the editor endpoints (with_write
  /// below); both hold the case's writer lock, so at most one of them is
  /// writing at any moment. Each connection waits up to ProjectDB's busy
  /// timeout (5 s), and the case's WAL anchor keeps the WAL index alive, so a
  /// 503 here means a lock really was held that long — by a writer.
  template <typename Handler>
  crow::response with_db(ProjectContext &ctx, Handler &&handler) {
    try {
      reusex::ProjectDB db(ctx.project(), /*readOnly=*/true);
      return handler(db);
    } catch (const HttpError &e) {
      return error_response(e.status(), e.what());
    } catch (const std::exception &e) {
      // A writer job holding the database is the one expected transient
      // failure here, and 503 is the honest answer for it.
      const std::string what = e.what();
      if (what.find("locked") != std::string::npos ||
          what.find("busy") != std::string::npos) {
        spdlog::debug("ProjectDB busy: {}", what);
        return error_response(503, "project database is busy (a job is "
                                   "writing); retry shortly");
      }
      spdlog::error("Request failed: {}", what);
      return error_response(500, what);
    }
  }

  /// Run @p handler against a writable ProjectDB of @p ctx's project, under
  /// the case's writer lock.
  ///
  /// The editor endpoints are the second writer of a case; its pipeline job
  /// is the first. They exclude each other through the queue's writer lock,
  /// which the worker holds for the whole of every stage. Two rules follow,
  /// and both are deliberate:
  ///
  ///  * **A queued or running job means 409, not a wait.** A stage holds the
  ///    lock for minutes; blocking a request that long is indistinguishable
  ///    from a hung UI, and the user can retry when the run is done.
  ///  * **Failing to take the writer lock means 503 after a short wait.** That
  ///    path is another editor request mid-write — milliseconds — so
  ///    kWriteLockTimeoutMs absorbs the normal case and anything past it is
  ///    worth reporting. Once the lock is held, the connection itself waits up
  ///    to ProjectDB's busy timeout (5 s) for sqlite's own file lock, which
  ///    covers another connection's checkpoint or WAL-index rebuild; a 503
  ///    from there means sqlite stayed locked that long.
  ///
  /// Nothing is written when either check fails, so both are safe to retry.
  template <typename Handler>
  crow::response with_write(ProjectContext &ctx, Handler &&handler) {
    if (ctx.is_busy())
      return error_response(409,
                            "a pipeline job is running or queued; edits are "
                            "refused while a stage is writing the project");

    auto lease = ctx.jobs().try_acquire_writer(
        std::chrono::milliseconds(kWriteLockTimeoutMs));
    if (!lease.owns_lock())
      return error_response(503, "the project is being written to; retry "
                                 "shortly");

    try {
      reusex::ProjectDB db(ctx.project(), /*readOnly=*/false);
      return handler(db);
    } catch (const HttpError &e) {
      return error_response(e.status(), e.what());
    } catch (const std::exception &e) {
      const std::string what = e.what();
      if (what.find("locked") != std::string::npos ||
          what.find("busy") != std::string::npos) {
        spdlog::debug("ProjectDB busy during a write: {}", what);
        return error_response(503, "project database is busy; retry shortly");
      }
      spdlog::error("Edit failed: {}", what);
      return error_response(500, what);
    }
  }

  /// Wrap a handler that needs no database (jobs, meta).
  template <typename Handler> crow::response guarded(Handler &&handler) {
    try {
      return handler();
    } catch (const HttpError &e) {
      return error_response(e.status(), e.what());
    } catch (const std::exception &e) {
      spdlog::error("Request failed: {}", e.what());
      return error_response(500, e.what());
    }
  }

  static Params params_of(const crow::request &req) {
    Params params;
    for (const auto &key : req.url_params.keys())
      if (const char *value = req.url_params.get(key))
        params.set(key, value);
    return params;
  }

  /// The JSON for one case, with whether it is open right now and its card
  /// figures (`summary`), read without opening it (case_meta.hpp).
  nlohmann::json case_of(const CaseInfo &info) {
    auto out = case_json(info, registry_->find_open(info.id) != nullptr);
    out["summary"] = summaries_.get(info);
    return out;
  }

  // --- server-level routes: cases and uploads -------------------------------

  void register_case_routes() {
    app_.route_dynamic("/api/v1/cases")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            if (req.method == crow::HTTPMethod::GET) {
              json list = json::array();
              for (const auto &info : cases_->list())
                list.push_back(case_of(info));
              return json_response(200, cases_list_json(std::move(list),
                                                        cases_->writable(),
                                                        uploads_->limits()));
            }
            const std::string name = parse_case_create(req.body);
            const CaseInfo info = cases_->create(name);
            spdlog::info("Case '{}' created", info.id);
            return json_response(201, case_of(info));
          });
        });

    app_.route_dynamic("/api/v1/cases/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH,
                 crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string cid) {
          return guarded([&] {
            if (req.method == crow::HTTPMethod::GET) {
              auto info = cases_->find(cid);
              if (!info)
                throw HttpError(404, "no such case '" + cid + "'");
              return json_response(200, case_of(*info));
            }
            if (req.method == crow::HTTPMethod::PATCH) {
              const auto patch = parse_case_patch(req.body);
              return json_response(200, case_of(cases_->update(cid, patch)));
            }
            const auto info = cases_->find(cid);
            if (!info)
              throw HttpError(404, "no such case '" + cid + "'");
            if (!info->deletable)
              throw HttpError(409, "case '" + cid +
                                       "' is not in the server's data dir and "
                                       "cannot be deleted here");
            // Tombstone it and close it, here, so its WAL is checkpointed and
            // its files released before they move. The tombstone holds until
            // the move is done (or failed): no request can reopen it between.
            switch (registry_->begin_delete(cid, std::chrono::seconds(5))) {
            case DeleteStart::started:
              break;
            case DeleteStart::busy:
              throw HttpError(409,
                              "case '" + cid + "' has a job queued or running");
            case DeleteStart::in_use:
              throw HttpError(409, "case '" + cid +
                                       "' is still in use by a request; retry "
                                       "shortly");
            case DeleteStart::deleting:
              throw HttpError(409,
                              "case '" + cid + "' is already being deleted");
            }
            try {
              cases_->move_to_trash(cid);
            } catch (...) {
              registry_->end_delete(cid); // Rolled back: openable again.
              throw;
            }
            // A new case may reuse the id; it must not inherit this history.
            scheduler_->store()->forget(cid);
            summaries_.forget(info->path);
            registry_->end_delete(cid);
            return crow::response(204);
          });
        });

    app_.route_dynamic("/api/v1/uploads")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            const auto request = parse_upload_request(req.body);
            const auto session = uploads_->begin(request.name, request.size);
            return json_response(201, upload_json(session, uploads_->limits()));
          });
        });

    app_.route_dynamic("/api/v1/uploads/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PUT,
                 crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string id) {
          return guarded([&] {
            if (req.method == crow::HTTPMethod::GET)
              return json_response(
                  200, upload_json(uploads_->status(id), uploads_->limits()));
            if (req.method == crow::HTTPMethod::DELETE) {
              uploads_->abort(id);
              return crow::response(204);
            }
            const auto offset =
                parse_upload_offset(req.url_params.get("offset"));
            const auto session = uploads_->append(id, offset, req.body);
            return json_response(200, upload_json(session, uploads_->limits()));
          });
        });

    app_.route_dynamic("/api/v1/uploads/<string>/complete")
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &, std::string id) {
              return guarded([&] {
                auto [session, staged] = uploads_->finish(id);
                // From here the staging file is ours alone (the session is
                // gone): it must not outlive a failure.
                auto discard = [path = staged] {
                  std::error_code ec;
                  std::filesystem::remove(path, ec);
                };
                const std::string why = reusex_project_problem(staged);
                if (!why.empty()) {
                  discard();
                  throw HttpError(
                      422, "the uploaded file is not a ReUseX project: " + why);
                }
                CaseInfo info;
                try {
                  info = cases_->adopt(session.name, staged);
                } catch (...) {
                  discard();
                  throw;
                }
                spdlog::info("Case '{}' created from an upload of {} bytes",
                             info.id, session.size);
                return json_response(201, case_of(info));
              });
            });
  }

  // --- routes --------------------------------------------------------------

  void register_routes() {
    auto get = [this](const std::string &path) -> crow::DynamicRule & {
      return app_.route_dynamic(path).methods(crow::HTTPMethod::GET);
    };

    register_case_routes();

    // ---- meta ----
    // Server-level: answers without opening any case.
    get("/api/v1/health")([this](const crow::request &) {
      return json_response(200, server_health_json(cases_->list().size()));
    });

    // Per case: the version handshake plus that project's state. Never fails
    // for a known case: a browser pointed at a broken project should still be
    // told *that*, in the documented shape, rather than get a bare 500.
    get(C("/health"))([this](const crow::request &, std::string cid) {
      const auto info = cases_->find(cid);
      if (!info)
        return error_response(404, "no such case '" + cid + "'");
      // A read-only probe, never an open: health is what a tab polls, and it
      // must not pin a case in the registry.
      try {
        reusex::ProjectDB db(info->path, /*readOnly=*/true);
        auto out = health_json(&db, info->path);
        out["case"] = info->id;
        return json_response(200, out);
      } catch (const std::exception &e) {
        spdlog::warn("Health check could not open case '{}': {}", cid,
                     e.what());
        auto out = health_json(nullptr, info->path);
        out["case"] = info->id;
        return json_response(200, out);
      }
    });

    get("/api/v1/endpoints")([](const crow::request &) {
      crow::response res(200, endpoints_json().dump(-1));
      res.set_header("Content-Type", "application/json");
      res.set_header("Access-Control-Allow-Origin", "*");
      return res;
    });

    // ---- project ----
    get(C("/project"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [](const reusex::ProjectDB &db) {
          return json_response(200, project_summary_json(db));
        });
      });
    });

    get(C("/projects"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, projects_json(db, params));
        });
      });
    });

    app_.route_dynamic(C("/projects/<string>"))
        .methods(crow::HTTPMethod::PATCH)(
            [this](const crow::request &req, std::string cid, std::string id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(200, patch_project(db, id, req.body));
                });
              });
            });

    // ---- clouds ----
    get(C("/clouds"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, clouds_json(db, params));
        });
      });
    });

    get(C("/clouds/<string>"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, cloud_json(db, name));
            });
          });
        });

    // The only route with two wire formats, so the only one that cannot go
    // straight through json_response(): `format=binary` answers a RUXP page
    // (docs/gui/binary-points.md), which is not JSON.
    get(C("/clouds/<string>/points"))(
        [this](const crow::request &req, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              const auto points = cloud_points(db, name, params);
              if (points.body)
                return json_response(200, *points.body);
              crow::response res = blob_response(*points.blob);
              for (const auto &[key, value] : points.headers)
                res.set_header(key, value);
              return res;
            });
          });
        });

    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules (same reasoning as /jobs below).
    app_.route_dynamic(C("/clouds/<string>/labels"))
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::PATCH)([this](const crow::request &req,
                                                 std::string cid,
                                                 std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            if (req.method == crow::HTTPMethod::GET)
              return with_db(ctx, [&](const reusex::ProjectDB &db) {
                return json_response(200, cloud_labels_json(db, name));
              });
            return with_write(ctx, [&](reusex::ProjectDB &db) {
              return json_response(200, patch_cloud_labels(db, name, req.body));
            });
          });
        });

    get(C("/clouds/<string>/tiles"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, cloud_tiles_json(db, name));
            });
          });
        });

    // ---- meshes ----
    get(C("/meshes"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, meshes_json(db, params));
        });
      });
    });

    get(C("/meshes/<string>"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, mesh_json(db, name));
            });
          });
        });

    get(C("/meshes/<string>/data"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return blob_response(mesh_data_blob(db, name));
            });
          });
        });

    get(C("/meshes/<string>/textures"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, mesh_textures_json(db, name));
            });
          });
        });

    get(C("/meshes/<string>/textures/<string>"))(
        [this](const crow::request &, std::string cid, std::string name,
               std::string texture) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return blob_response(mesh_texture_blob(db, name, texture));
            });
          });
        });

    // ---- evidence renders (#265 Phase 2 Task 8) ----
    // A render only reads the project, so it neither opens the case (the
    // case list shows one plan thumbnail per card) nor renders twice: images
    // are cached by the project's file stamp and the query.
    get(C("/renders"))([this](const crow::request &req, std::string cid) {
      return guarded([&]() -> crow::response {
        const auto info = cases_->find(cid);
        if (!info)
          throw HttpError(404, "no such case '" + cid + "'");
        const std::string key = req.raw_url.substr(
            std::min(req.raw_url.size(), req.raw_url.find('?')));
        if (auto hit = renders_.get(info->path, key))
          return blob_response(*hit);
        const Params params = params_of(req);
        Blob blob;
        {
          reusex::ProjectDB db(info->path, /*readOnly=*/true);
          blob = render_blob(db, view_renderer_, params);
        }
        renders_.put(info->path, key, blob);
        return blob_response(blob);
      });
    });

    // ---- gaussian splats (#322) ----
    get(C("/gsplats"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, gsplats_json(db, params));
        });
      });
    });

    get(C("/gsplats/<string>"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, gsplat_json(db, name));
            });
          });
        });

    get(C("/gsplats/<string>/data"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return blob_response(gsplat_blob(db, name));
            });
          });
        });

    // ---- sensor frames ----
    get(C("/frames"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, frames_json(db, params));
        });
      });
    });

    // Registered before /frames/<int> so the static "visibility" segment is
    // matched ahead of the integer rule.
    get(C("/frames/visibility"))(
        [this](const crow::request &req, std::string cid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, frames_visibility_json(db, params));
            });
          });
        });

    get(C("/frames/<int>"))(
        [this](const crow::request &, std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, frame_json(db, id));
            });
          });
        });

    get(C("/frames/<int>/image"))(
        [this](const crow::request &req, std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              const auto image = frame_image(db, id, params);
              crow::response res = blob_response(image.blob);
              // Convenience only, exactly like the X-Ruxp-* headers: the
              // picture is the response, and a client must not need these to
              // use it.
              if (image.range.valid) {
                res.set_header("X-Image-Range-Min",
                               std::to_string(image.range.min));
                res.set_header("X-Image-Range-Max",
                               std::to_string(image.range.max));
              }
              return res;
            });
          });
        });

    // GET /api/v1/models/sam3/status — managed-model provisioning status
    // (self-contained SAM3 packaging). Lets the UI poll while the ONNX bundle
    // downloads and the device-specific engines build on first use. 501 when
    // no managed model is configured.
    get("/api/v1/models/sam3/status")(
        [this](const crow::request &req) -> crow::response {
          if (!model_provider_)
            return error_response(
                501, "no managed SAM3 model is configured on this server");
          bool use_cuda = options_.segment_cuda;
          if (const char *c = req.url_params.get("cuda")) {
            const std::string v(c);
            use_cuda = !(v == "0" || v == "false" || v == "no");
          }
          ModelPrepStatus st;
          try {
            st = model_provider_->status(use_cuda);
          } catch (const std::exception &e) {
            return error_response(500, e.what());
          }
          nlohmann::json out{{"state", st.state},
                             {"progress", st.progress},
                             {"message", st.message},
                             {"use_cuda", use_cuda},
                             {"update_engines", st.update_engines}};
          if (st.state == "ready")
            out["model_path"] = st.model_path;
          return json_response(200, out);
        });

    // POST /api/v1/frames/<id>/segment — interactive SAM3 segmentation (#409,
    // #467). The segmenter is injected by ruxd's main; returns 503 if
    // absent. use_cuda defaults to ServerOptions::segment_cuda; can be
    // overridden per-request via the body.
    app_.route_dynamic(C("/frames/<int>/segment"))
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req,
                                                std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return guarded([&]() -> crow::response {
              if (!segmenter_)
                return error_response(503,
                                      "no SAM3 model registered; start the "
                                      "server via 'ruxd --local' and ensure a "
                                      "model is available");

              // Parse and validate the request body.
              const auto seg_req =
                  parse_segment_frame_request(req.body, options_.segment_cuda);

              // Resolve the model path (managed model when omitted). May 503
              // while the managed model is downloading/building.
              const std::string model_path =
                  resolve_model_path(seg_req.model_path, seg_req.use_cuda);

              // Load frame image (brief read-only DB connection).
              cv::Mat image;
              {
                try {
                  reusex::ProjectDB db(ctx.project(), /*readOnly=*/true);
                  if (!db.has_sensor_frame(id))
                    throw HttpError(404, "sensor frame " + std::to_string(id) +
                                             " not found");
                  image = db.sensor_frame_image(id);
                } catch (const HttpError &) {
                  throw;
                } catch (const std::exception &e) {
                  throw HttpError(500, e.what());
                }
              }
              if (image.empty())
                throw HttpError(404, "frame " + std::to_string(id) +
                                         " has no color image");

              // Run inference (outside any DB connection — may take seconds).
              const auto result = segmenter_->segment(
                  image, seg_req.prompts, seg_req.confidence, model_path,
                  seg_req.use_cuda);

              // Optionally persist the mask (uses write lock to exclude jobs).
              bool saved = false;
              if (seg_req.save && !result.label_map.empty()) {
                auto write_res = with_write(
                    ctx, [&](reusex::ProjectDB &wdb) -> crow::response {
                      wdb.save_segmentation_image(id, result.label_map);
                      return json_response(200, nlohmann::json{});
                    });
                if (write_res.code != 200)
                  return write_res;
                saved = true;
              }

              return json_response(
                  200, segment_frame_result_json(id, result.label_map,
                                                 result.class_names, saved,
                                                 result.geometry_prompts_used));
            });
          });
        });

    // POST /api/v1/frames/<id>/segment/resource — project one label of the
    // frame's saved segmentation into the base cloud, file it as a new
    // instance + survey part (spec B2), then tell every client which clouds
    // changed so a viewport can reload them.
    app_.route_dynamic(C("/frames/<int>/segment/resource"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                // Parse first: a bad body is a 400 whether or not a job holds
                // the writer lock.
                SegmentResourceRequest body;
                try {
                  body = parse_segment_resource_request(req.body);
                } catch (const HttpError &e) {
                  return error_response(e.status(), e.what());
                }
                std::vector<std::string> changed;
                auto res = with_write(ctx, [&](reusex::ProjectDB &db) {
                  auto out = execute_segment_resource(db, id, body);
                  changed = out.at("clouds").get<std::vector<std::string>>();
                  return json_response(201, out);
                });
                if (res.code == 201 && !changed.empty())
                  ctx.broadcast_message(
                      clouds_changed_json(changed, ctx.file_name()));
                return res;
              });
            });

    // ---- panoramas ----
    get(C("/panoramas"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, panoramas_json(db, params));
        });
      });
    });

    get(C("/panoramas/<int>"))(
        [this](const crow::request &, std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, panorama_json(db, id));
            });
          });
        });

    get(C("/panoramas/<int>/image"))(
        [this](const crow::request &req, std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return blob_response(panorama_image_blob(db, id, params));
            });
          });
        });

    // POST /api/v1/panoramas/<id>/segment — interactive SAM3 segmentation on
    // 360 panoramas (#448, #467). Mirrors POST /frames/<id>/segment but tiles
    // the equirect through segment_panorama() rather than segment_image().
    app_.route_dynamic(C("/panoramas/<int>/segment"))
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req,
                                                std::string cid, int id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return guarded([&]() -> crow::response {
              if (!panorama_segmenter_)
                return error_response(
                    503, "no SAM3 panorama segmenter registered; start the "
                         "server via 'ruxd --local' and ensure a model is "
                         "available");

              const auto seg_req = parse_segment_panorama_request(
                  req.body, options_.segment_cuda);

              // Resolve the model path (managed model when omitted). May 503
              // while the managed model is downloading/building.
              const std::string model_path =
                  resolve_model_path(seg_req.model_path, seg_req.use_cuda);

              // Load panorama image (brief read-only connection).
              cv::Mat image;
              {
                try {
                  reusex::ProjectDB db(ctx.project(), /*readOnly=*/true);
                  image = db.panoramic_image(id);
                } catch (const HttpError &) {
                  throw;
                } catch (const std::exception &e) {
                  throw HttpError(500, e.what());
                }
              }
              if (image.empty())
                throw HttpError(404, "panorama " + std::to_string(id) +
                                         " not found or has no image");

              // Run inference outside any DB connection — may take seconds.
              const auto result = panorama_segmenter_->segment(
                  image, seg_req.prompts, seg_req.confidence, seg_req.n_yaw,
                  seg_req.fov_deg, model_path, seg_req.use_cuda);

              // Optionally persist the label map.
              bool saved = false;
              if (seg_req.save && !result.label_map.empty()) {
                auto write_res = with_write(
                    ctx, [&](reusex::ProjectDB &wdb) -> crow::response {
                      wdb.save_panorama_segmentation(id, result.label_map);
                      return json_response(200, nlohmann::json{});
                    });
                if (write_res.code != 200)
                  return write_res;
                saved = true;
              }

              return json_response(
                  200, segment_panorama_result_json(id, result.label_map,
                                                    result.class_names, saved));
            });
          });
        });

    // ---- components / materials / instances ----
    get(C("/components"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, components_json(db, params));
        });
      });
    });

    get(C("/components/<string>"))(
        [this](const crow::request &, std::string cid, std::string name) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, component_json(db, name));
            });
          });
        });

    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules (same reasoning as /jobs below).
    app_.route_dynamic(C("/materials"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET) {
                  const Params params = params_of(req);
                  return with_db(ctx, [&](const reusex::ProjectDB &db) {
                    return json_response(200, materials_json(db, params));
                  });
                }
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, create_material(db));
                });
              });
            });

    app_.route_dynamic(C("/materials/<string>"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH,
                 crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string cid,
                                                  std::string guid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            if (req.method == crow::HTTPMethod::GET)
              return with_db(ctx, [&](const reusex::ProjectDB &db) {
                return json_response(200, material_json(db, guid));
              });
            if (req.method == crow::HTTPMethod::PATCH)
              return with_write(ctx, [&](reusex::ProjectDB &db) {
                return json_response(200, patch_material(db, guid, req.body));
              });
            return with_write(ctx, [&](reusex::ProjectDB &db) {
              delete_material(db, guid);
              return crow::response(204);
            });
          });
        });

    app_.route_dynamic(C("/materials/<string>/thumbnail"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PUT)(
            [this](const crow::request &req, std::string cid,
                   std::string guid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET)
                  return with_db(ctx, [&](const reusex::ProjectDB &db) {
                    return blob_response(material_thumbnail_blob(db, guid));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  set_material_thumbnail(db, guid, req.body,
                                         req.get_header_value("Content-Type"));
                  return crow::response(204);
                });
              });
            });

    app_.route_dynamic(C("/resources/columns"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET)
                  return with_db(ctx, [&](const reusex::ProjectDB &db) {
                    return json_response(200, material_columns_json(db));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201,
                                       create_material_column(db, req.body));
                });
              });
            });
    app_.route_dynamic(C("/resources/columns/<string>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid, std::string id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::PATCH)
                  return with_write(ctx, [&](reusex::ProjectDB &db) {
                    return json_response(
                        200, patch_material_column(db, id, req.body));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  delete_material_column(db, id);
                  return crow::response(204);
                });
              });
            });

    // ---- resources (schema v25) ----
    // Static paths are registered before /resources/<string>. Crow keeps one
    // trie per method and that route takes only PATCH/DELETE, so a GET can
    // never reach it either way.
    get(C("/resources/keys"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [](const reusex::ProjectDB &db) {
          return json_response(200, resource_keys_json(db));
        });
      });
    });
    get(C("/resources/export.csv"))(
        [this](const crow::request &req, std::string cid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              const auto b = resources_csv_blob(db, params);
              crow::response res(200);
              res.set_header("Content-Type", b.content_type);
              res.set_header("Content-Disposition",
                             "attachment; filename=\"ressourcer.csv\"");
              res.body.assign(reinterpret_cast<const char *>(b.data.data()),
                              b.data.size());
              return res;
            });
          });
        });
    app_.route_dynamic(C("/resources"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET) {
                  const Params params = params_of(req);
                  return with_db(ctx, [&](const reusex::ProjectDB &db) {
                    return json_response(200, resources_json(db, params));
                  });
                }
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, create_resource_json(db, req.body));
                });
              });
            });
    app_.route_dynamic(C("/resources/<string>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid,
                   std::string code) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::PATCH) {
                  const Params params = params_of(req);
                  return with_write(ctx, [&](reusex::ProjectDB &db) {
                    return json_response(
                        200, patch_resource_json(db, code, params, req.body));
                  });
                }
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  delete_resource(db, code);
                  return crow::response(204);
                });
              });
            });

    // ---- templates (schema v25) ----
    app_.route_dynamic(C("/templates"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET)
                  return with_db(ctx, [](const reusex::ProjectDB &db) {
                    return json_response(200, templates_json(db));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, create_template_json(db, req.body));
                });
              });
            });
    app_.route_dynamic(C("/templates/restore-seeds"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [](reusex::ProjectDB &db) {
                  return json_response(200, restore_seed_templates_json(db));
                });
              });
            });
    app_.route_dynamic(C("/templates/<int>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::PATCH)
                  return with_write(ctx, [&](reusex::ProjectDB &db) {
                    return json_response(200,
                                         patch_template_json(db, id, req.body));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  delete_template(db, id);
                  return crow::response(204);
                });
              });
            });
    app_.route_dynamic(C("/templates/<int>/duplicate"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, duplicate_template_json(db, id));
                });
              });
            });

    get(C("/instances/<string>"))(
        [this](const crow::request &req, std::string cid, std::string cloud) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, instances_json(db, cloud, params));
            });
          });
        });

    get(C("/instances/<string>/<int>/frames"))(
        [this](const crow::request &req, std::string cid, std::string cloud,
               int instance_id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(
                  200, instance_frames_json(
                           db, cloud, instance_id, params,
                           [&ctx](const reusex::ProjectDB &conn,
                                  const std::string &c, std::uint32_t id) {
                             return ctx.photo_cache().peek(conn, c, id);
                           }));
            });
          });
        });

    get(C("/instances/<string>/<int>/panoramas"))(
        [this](const crow::request &req, std::string cid, std::string cloud,
               int instance_id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const Params params = params_of(req);
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(
                  200, instance_panoramas_json(db, cloud, instance_id, params));
            });
          });
        });

    app_.route_dynamic(C("/instances/<string>/<int>/material"))
        .methods(crow::HTTPMethod::PUT)(
            [this](const crow::request &req, std::string cid, std::string cloud,
                   int instance_id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(
                      200,
                      link_instance_material(db, cloud, instance_id, req.body));
                });
              });
            });

    // ---- pose graph ----
    get(C("/posegraph"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [](const reusex::ProjectDB &db) {
          return json_response(200, posegraph_json(db));
        });
      });
    });

    app_.route_dynamic(C("/posegraph/edges/<int>/<int>"))
        .methods(crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string cid, int from,
                                                  int to) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            const auto type = params_of(req).str("type", "");
            return with_write(ctx, [&](reusex::ProjectDB &db) {
              return json_response(200,
                                   delete_posegraph_edge(db, from, to, type));
            });
          });
        });

    app_.route_dynamic(C("/posegraph/edges"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, add_posegraph_edge(db, req.body));
                });
              });
            });

    app_.route_dynamic(C("/posegraph/icp"))
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req,
                                                std::string cid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(
                  200,
                  refine_posegraph_icp(db, options_.icp_refine_fn, req.body));
            });
          });
        });

    // ---- pipeline ----
    get(C("/stages"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [](const reusex::ProjectDB &db) {
          return json_response(200, stages_json(db));
        });
      });
    });

    get(C("/stages/<string>/validation"))(
        [this](const crow::request &, std::string cid, std::string stage) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_db(ctx, [&](const reusex::ProjectDB &db) {
              return json_response(200, stage_validation_json(db, stage));
            });
          });
        });

    get(C("/pipeline-log"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        const Params params = params_of(req);
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, pipeline_log_json(db, params));
        });
      });
    });

    // ---- jobs ----
    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules.
    app_.route_dynamic(C("/jobs"))
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req,
                                                std::string cid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return guarded([&] {
              const auto project = ctx.file_name();
              if (req.method == crow::HTTPMethod::GET)
                return json_response(
                    200,
                    jobs_page_json(ctx.jobs().jobs(), project, params_of(req)));

              const auto submission = parse_job_request(req.body);
              check_job_project(submission, project);
              const auto id =
                  ctx.jobs().submit(submission.stage, submission.parameters);
              auto record = ctx.jobs().job(id);
              if (!record)
                throw HttpError(500, "job vanished immediately after submit");
              return json_response(202, job_json(*record, project));
            });
          });
        });

    get(C("/jobs/<string>"))(
        [this](const crow::request &, std::string cid, std::string id) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return guarded([&] {
              auto record = ctx.jobs().job(id);
              if (!record)
                throw HttpError(404, "no such job '" + id + "'");
              return json_response(200, job_json(*record, ctx.file_name()));
            });
          });
        });

    app_.route_dynamic(C("/jobs/<string>/cancel"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &, std::string cid, std::string id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return guarded([&] {
                  if (!ctx.jobs().cancel(id))
                    throw HttpError(404, "no such job '" + id + "'");
                  auto record = ctx.jobs().job(id);
                  if (!record)
                    throw HttpError(404, "no such job '" + id + "'");
                  return json_response(200, job_json(*record, ctx.file_name()));
                });
              });
            });

    // ---- report PDFs (#456) ----
    app_.route_dynamic(C("/reports/ressourcekortlaegning"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET)
                  return with_db(ctx, [](const reusex::ProjectDB &db) {
                    return json_response(200, list_report_pdfs_json(db));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201,
                                       generate_report_pdf_json(db, req.body));
                });
              });
            });

    app_.route_dynamic(C("/reports/ressourcekortlaegning/<int>"))
        .methods(crow::HTTPMethod::GET)(
            [this](const crow::request &, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_db(ctx, [&](const reusex::ProjectDB &db) {
                  return blob_response(report_pdf_blob(db, id));
                });
              });
            });

    // ---- CSV export (#459) ----
    get(C("/exports/csv"))([this](const crow::request &req, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        std::vector<std::string> columns;
        if (const char *raw = req.url_params.get("columns"); raw && *raw) {
          std::istringstream ss(raw);
          std::string col;
          while (std::getline(ss, col, ','))
            if (!col.empty())
              columns.push_back(col);
        }
        return with_write(ctx, [&](reusex::ProjectDB &db) {
          auto b = export_csv_blob(db, columns);
          crow::response res(200);
          res.set_header("Content-Type", b.content_type);
          res.set_header("Content-Disposition",
                         "attachment; filename=\"elements.csv\"");
          res.body.assign(reinterpret_cast<const char *>(b.data.data()),
                          b.data.size());
          return res;
        });
      });
    });

    // ---- survey (Ressourcekortlægning, #265 Phase 2) ----
    get(C("/survey"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        ctx.start_photo_warmup(); // Kortlægning is open: photos come next.
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, survey_json(db));
        });
      });
    });
    get(C("/survey/summary"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, survey_summary_json(db));
        });
      });
    });
    get(C("/survey/photos"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        ctx.start_photo_warmup();
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(
              200, survey_photos_json(
                       db, [&ctx](const reusex::ProjectDB &conn,
                                  const std::string &cloud,
                                  const std::set<std::uint32_t> &wanted) {
                         return ctx.photo_cache().get(conn, cloud, wanted);
                       }));
        });
      });
    });
    get(C("/survey/fractions"))([this](const crow::request &, std::string cid) {
      return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
        return with_db(ctx, [&](const reusex::ProjectDB &db) {
          return json_response(200, survey_fractions_json(db));
        });
      });
    });

    app_.route_dynamic(C("/survey/sync"))
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(200, sync_survey_json(db, req.body));
                });
              });
            });
    app_.route_dynamic(C("/survey/types"))
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req,
                                                std::string cid) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_write(ctx, [&](reusex::ProjectDB &db) {
              return json_response(201, create_survey_type_json(db, req.body));
            });
          });
        });
    app_.route_dynamic(C("/survey/types/<int>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::DELETE)
                  return with_write(ctx, [&](reusex::ProjectDB &db) {
                    return json_response(200, delete_survey_type_json(db, id));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(
                      200, patch_survey_type_json(db, id, req.body));
                });
              });
            });
    app_.route_dynamic(C("/survey/parts/<string>"))
        .methods(crow::HTTPMethod::PATCH)([this](const crow::request &req,
                                                 std::string cid,
                                                 std::string code) {
          return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
            return with_write(ctx, [&](reusex::ProjectDB &db) {
              return json_response(200,
                                   patch_survey_part_json(db, code, req.body));
            });
          });
        });

    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules (same reasoning as /jobs above).
    app_.route_dynamic(C("/samples"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::GET)
                  return with_db(ctx, [&](const reusex::ProjectDB &db) {
                    return json_response(200, samples_json(db));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(201, create_sample_json(db, req.body));
                });
              });
            });

    app_.route_dynamic(C("/samples/<int>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                if (req.method == crow::HTTPMethod::PATCH)
                  return with_write(ctx, [&](reusex::ProjectDB &db) {
                    return json_response(200,
                                         patch_sample_json(db, id, req.body));
                  });
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  delete_sample(db, id);
                  return crow::response(204);
                });
              });
            });

    app_.route_dynamic(C("/samples/<int>/links"))
        .methods(crow::HTTPMethod::PUT)(
            [this](const crow::request &req, std::string cid, int id) {
              return in_case(cid, [&](ProjectContext &ctx) -> crow::response {
                return with_write(ctx, [&](reusex::ProjectDB &db) {
                  return json_response(200,
                                       set_sample_links_json(db, id, req.body));
                });
              });
            });

    register_websocket();
    register_static();
  }

  void register_websocket() {
    app_.route_dynamic(C("/events"))
        .websocket<App>(&app_)
        .onaccept([this](const crow::request &req,
                         std::optional<crow::response> &res, void **userdata) {
          // WebSockets are NOT subject to CORS — a browser will happily open
          // one cross-origin and hand the frames to the attacker's script. The
          // handshake is therefore the only place this can be enforced, and
          // Crow ignores a middleware response on the upgrade path, so the
          // checks live here rather than in SecurityMiddleware.
          const auto &security = app_.get_middleware<SecurityMiddleware>();
          if (!security.host_ok(req)) {
            spdlog::warn("Refused a WebSocket upgrade for host '{}'",
                         req.get_header_value("Host"));
            res = error_response(403, "host is not served here");
            return;
          }
          if (!security.check(req).ok) {
            spdlog::warn("Refused a WebSocket upgrade without the access "
                         "token");
            res = error_response(401, "missing or wrong access token");
            return;
          }
          const std::string origin = req.get_header_value("Origin");
          // An empty Origin is a non-browser client (curl, the CLI, a test).
          if (!origin.empty() && !security.origin_ok(req, origin)) {
            spdlog::warn("Refused a WebSocket upgrade from origin '{}'",
                         origin);
            res = error_response(403, "origin is not allowed");
            return;
          }
          const auto cid = case_id_of_events_url(req.url);
          if (!cid || !cases_->find(*cid)) {
            res = error_response(404, "no such case");
            return;
          }
          // Handed to onopen, which owns it from there.
          *userdata = new std::string(*cid);
        })
        .onopen([this](crow::websocket::connection &conn) {
          std::unique_ptr<std::string> cid(
              static_cast<std::string *>(conn.userdata()));
          conn.userdata(nullptr);
          std::shared_ptr<ProjectContext> ctx;
          try {
            ctx = cid ? registry_->acquire(*cid) : nullptr;
          } catch (const std::exception &e) {
            spdlog::warn("WebSocket for case '{}' refused: {}",
                         cid ? *cid : "?", e.what());
          }
          if (!ctx) {
            conn.close("case unavailable");
            return;
          }
          {
            // The socket keeps no lease (a weak reference only): an open tab
            // keeps its case open through the subscriber count, not by
            // pinning it, so deleting the case still works.
            std::lock_guard<std::mutex> lock(sockets_mutex_);
            sockets_[&conn] = ctx;
          }
          ctx->subscribe(&conn, [&conn](const std::string &payload) {
            conn.send_text(payload);
          });
          spdlog::debug("WebSocket client connected to case '{}'", ctx->id());
        })
        .onmessage([this](crow::websocket::connection &conn,
                          const std::string &data, bool is_binary) {
          if (is_binary) {
            conn.send_text(json{{"type", "error"},
                                {"timestamp", pipeline::iso8601_utc_now()},
                                {"error", "binary frames are not accepted"}}
                               .dump());
            return;
          }
          auto ctx = socket_case(conn);
          auto reply =
              handle_ws_message(data, [&](std::optional<std::string> job_id) {
                if (ctx)
                  ctx->set_filter(&conn, std::move(job_id));
              });
          if (reply)
            conn.send_text(reply->dump());
        })
        .onclose([this](crow::websocket::connection &conn, const std::string &,
                        uint16_t) {
          drop_socket(conn);
          spdlog::debug("WebSocket client disconnected");
        })
        .onerror([this](crow::websocket::connection &conn,
                        const std::string &reason) {
          spdlog::debug("WebSocket error: {}", reason);
          drop_socket(conn);
        });
  }

  /// The case a socket subscribed to, if it is still open.
  std::shared_ptr<ProjectContext>
  socket_case(crow::websocket::connection &conn) {
    std::lock_guard<std::mutex> lock(sockets_mutex_);
    auto it = sockets_.find(&conn);
    return it == sockets_.end() ? nullptr : it->second.lock();
  }

  /// Unsubscribe and forget a socket. Runs before Crow frees the connection,
  /// which is what makes ProjectContext::broadcast's send-under-lock safe.
  void drop_socket(crow::websocket::connection &conn) {
    std::weak_ptr<ProjectContext> weak;
    {
      std::lock_guard<std::mutex> lock(sockets_mutex_);
      auto it = sockets_.find(&conn);
      if (it == sockets_.end())
        return;
      weak = it->second;
      sockets_.erase(it);
    }
    if (auto ctx = weak.lock()) {
      ctx->unsubscribe(&conn);
      ctx->touch(ProjectContext::Clock::now());
    }
  }

  /// Serve one file from the bundle, or an empty optional if it is not there.
  static std::optional<crow::response>
  file_response(const std::filesystem::path &file,
                std::string_view content_type) {
    if (file.empty())
      return std::nullopt;
    std::ifstream stream(file, std::ios::binary);
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    crow::response res(200, buffer.str());
    res.set_header("Content-Type", std::string(content_type));
    return res;
  }

  /// Decide what a request that matched no registered route should get.
  ///
  /// Pure: it returns the response rather than writing into one, which is what
  /// keeps the catchall handler below free of `res.end()` — see the comment
  /// there for why that matters.
  crow::response static_response(const crow::request &req) {
    // Anything under the API prefix that reached the catchall is a genuine
    // 404 — do not shadow it with the SPA fallback, or a client typo would
    // silently return HTML where JSON was expected.
    if (req.url.rfind(std::string(kApiPrefix), 0) == 0)
      return error_response(404, "no route matches " + req.url);

    // A request that names a file (has an extension) must never be answered
    // with HTML, bundle or no bundle: handing back index.html where the
    // browser expects JavaScript turns a missing file into an inscrutable
    // syntax error.
    if (!looks_like_spa_route(req.url) && !has_assets())
      return error_response(404, "no such asset " + req.url);

    if (has_assets()) {
      const auto file = resolve_asset(options_.asset_dir, req.url);
      if (auto served = file_response(file, mime_type_for(file)))
        return std::move(*served);

      // SPA fallback, but ONLY for paths that look like client-side routes.
      // A missing /assets/app.js must 404: answering it with index.html
      // hands the browser HTML where it expects JavaScript, which surfaces
      // as an inscrutable syntax error instead of the missing file it is.
      if (!looks_like_spa_route(req.url))
        return error_response(404, "no such asset " + req.url);

      if (auto served =
              file_response(resolve_asset(options_.asset_dir, "/index.html"),
                            "text/html; charset=utf-8"))
        return std::move(*served);
    }

    crow::response res(200, placeholder_page("ruxd"));
    res.set_header("Content-Type", "text/html; charset=utf-8");
    return res;
  }

  void register_static() {
    CROW_CATCHALL_ROUTE(app_)
    ([this](const crow::request &req, crow::response &res) {
      // DO NOT CALL res.end() HERE — it breaks every keep-alive request after
      // the first (#265). Crow's Router::handle() calls res.end() itself once
      // the catchall handler returns (crow/routing.h:1714 in 1.3.2, :1707 in
      // the 1.3.0 build we pin), so a handler that also ends the response ends
      // it *twice*. The first end() runs the whole write synchronously —
      // complete_request() -> do_write_general() -> do_write_sync(), which
      // calls res.clear() and so resets `completed_` to false — and the
      // router's second end() then re-sets `completed_ = true` on the response
      // object that http_connection reuses for the next request on the same
      // connection. On that next request, http_connection::handle() sees
      // `res.completed_` (crow/http_connection.h:192) and short-circuits
      // straight to complete_request() without ever dispatching, emitting the
      // bare 404 that Router::handle_initial() had stamped on `res.code`.
      // Result: the bundle loads on request 1 and 404s on request 2, which is
      // every <script>/<link> a browser asks for on its keep-alive connection.
      //
      // Normal CROW_ROUTEs are unaffected: handle_rule() does not add an
      // end() of its own, so their handlers must (and do) end the response.
      // This asymmetry is undocumented upstream and is still present in Crow
      // 1.3.2, hence this comment and the socket-level regression test in
      // tests/unit/ruxd_api/test_api_server_socket.cpp.
      res = static_response(req);
    });
  }

  ServerOptions options_;

  // MEMBER ORDER IS LOAD-BEARING (destruction runs in reverse). The scheduler
  // outlives the registry (each case's queue lives on it), the registry
  // outlives Crow (whose threads lease cases), and the socket table outlives
  // Crow too, since its close handlers run until Crow is stopped. ~Impl also
  // tears these down explicitly, so this ordering is the belt to those braces.
  std::shared_ptr<ICaseStore> cases_;
  std::unique_ptr<UploadManager> uploads_;
  pipeline::StageExecutor executor_;
  std::unique_ptr<pipeline::JobScheduler> scheduler_;
  std::unique_ptr<ProjectRegistry> registry_;
  /// Card figures and renders, read without opening a case.
  CaseSummaryCache summaries_;
  RenderCache renders_;

  std::mutex sockets_mutex_;
  /// Open WebSocket -> the case it subscribed to.
  std::unordered_map<crow::websocket::connection *,
                     std::weak_ptr<ProjectContext>>
      sockets_;

  App app_;

  /// Optional SAM3 segmenter registered by ruxd's main (#409).
  /// Not owned; lifetime must exceed the server's. nullptr ⟹ 503.
  IFrameSegmenter *segmenter_ = nullptr;

  /// Optional panorama segmenter registered by ruxd's main (#448).
  /// Not owned; lifetime must exceed the server's. nullptr ⟹ 503.
  IPanoramaSegmenter *panorama_segmenter_ = nullptr;

  /// Optional managed-model provider (self-contained SAM3 packaging).
  /// Not owned; lifetime must exceed the server's. nullptr ⟹ omitted
  /// model_path is a 400 and the status route is a 501.
  IModelProvider *model_provider_ = nullptr;

  /// Optional evidence-render renderer (#265 Phase 2 Task 8). Not owned;
  /// lifetime must exceed the server's. nullptr ⟹ 503.
  IViewRenderer *view_renderer_ = nullptr;
};

// ===========================================================================
// Server
// ===========================================================================

Server::Server(ServerOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}

Server::~Server() = default;

const ServerOptions &Server::options() const noexcept {
  return impl_->options();
}

std::string Server::url() const { return impl_->url(); }

bool Server::has_assets() const noexcept { return impl_->has_assets(); }

std::vector<CaseInfo> Server::cases() const { return impl_->cases(); }

int Server::run() { return impl_->run(); }

void Server::stop() { impl_->stop(); }

void Server::set_segmenter(IFrameSegmenter *segmenter) {
  impl_->set_segmenter(segmenter);
}

void Server::set_panorama_segmenter(IPanoramaSegmenter *segmenter) {
  impl_->set_panorama_segmenter(segmenter);
}

void Server::set_model_provider(IModelProvider *provider) {
  impl_->set_model_provider(provider);
}

void Server::set_view_renderer(IViewRenderer *renderer) {
  impl_->set_view_renderer(renderer);
}

} // namespace ruxd::api
