// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/Server.hpp"

#include "api/AuthService.hpp"
#include "api/FrameSegmenter.hpp"
#include "api/ModelProvider.hpp"
#include "api/ProjectContext.hpp"
#include "api/ProjectRegistry.hpp"
#include "api/ViewRenderer.hpp"
#include "api/access.hpp"
#include "api/api.hpp"
#include "api/assets.hpp"
#include "api/case_meta.hpp"
#include "api/cases.hpp"
#include "api/credentials.hpp"
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

// overlays/crow.nix patches Crow with a request-body cap and a header-phase
// check; without them an oversized or unauthenticated body is buffered
// whole. A stale Crow_DIR (an unpatched store path) must fail the build, not
// silently drop both (S2 re-review N8): reconfigure with `cmake -B build
// -UCrow_DIR` inside `nix develop`.
#if !defined(CROW_REUSEX_MAX_BODY_PATCH) || !defined(CROW_REUSEX_HEADER_CHECK)
#error                                                                         \
    "ruxd needs the patched Crow from overlays/crow.nix (reconfigure: cmake -B build -UCrow_DIR)"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <semaphore>
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

/// Evidence/thumbnail renders (`GET /cases/{cid}/renders` cache misses) that
/// may run at once, and how long one waits for a slot before a 503.
constexpr std::ptrdiff_t kMaxConcurrentRenders = 2;
constexpr std::chrono::seconds kRenderSlotWait{30};

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
/// The access decision for one request (Server::Impl::evaluate): who it is,
/// what it touches, and whether it may.
struct Gate {
  AccessDecision decision;
  Principal principal;
  RouteAccess route;
  std::optional<Role> role;
  /// Local mode: the token came as `?token=` (set the cookie, redirect).
  bool token_via_query = false;
};

class SecurityMiddleware {
    public:
  struct context {
    /// The request proved the token with `?token=`; answer with the cookie so
    /// the browser's later <img>/fetch/WebSocket requests carry it.
    bool set_token_cookie = false;
    /// Who the request is (server mode: a user, a token, the superuser;
    /// local mode: the implicit user) and what it touches.
    Principal principal;
    RouteAccess route;
    std::optional<Role> role;
  };

  using Evaluate = std::function<Gate(const crow::request &)>;

  void configure(std::vector<std::string> extra_origins, std::string auth_token,
                 std::string bind_address, uint16_t port, bool server_mode,
                 Evaluate evaluate, std::shared_ptr<AuthService> auth) {
    extra_origins_ = std::move(extra_origins);
    auth_token_ = std::move(auth_token);
    bind_address_ = std::move(bind_address);
    cookie_name_ = token_cookie_name(port);
    server_mode_ = server_mode;
    evaluate_ = std::move(evaluate);
    auth_ = std::move(auth);
  }

  /// The JSON error response for a refused gate, with its headers.
  static crow::response refusal(const AccessDecision &decision) {
    auto res = error_response(decision.status, decision.message);
    if (decision.status == 401)
      res.set_header("WWW-Authenticate", "Bearer");
    res.set_header("Referrer-Policy", "no-referrer");
    return res;
  }

  /// Run the access decision (also the Crow header-phase check).
  Gate evaluate(const crow::request &req) const { return evaluate_(req); }

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

  /// Loopback, an --allow-origin entry, or — when a token is configured, or
  /// in server mode, where authentication is mandatory — the request's own
  /// origin (`Origin` equal to `<scheme>://<Host>`), which is what a browser
  /// on another machine sends for the page this server served it.
  bool origin_ok(const crow::request &req, const std::string &origin) const {
    if (is_allowed(origin))
      return true;
    return (server_mode_ || !auth_token_.empty()) &&
           origin_matches_host(origin, req.get_header_value("Host"));
  }

  std::string token_cookie() const {
    return cookie_name_ + "=" + auth_token_ +
           "; Path=/; HttpOnly; SameSite=Strict";
  }

  /// Whether Crow will hand @p req to its upgrade path (Router::
  /// handle_upgrade) rather than to an ordinary route handler — mirroring
  /// http_connection.h's handle() exactly: HTTP/1.1, a Host header, the
  /// parser's upgrade flag, not OPTIONS, and an Upgrade value that does not
  /// start with "h2" (which Crow ignores and serves normally).
  ///
  /// Only then may the middleware stand aside: on that path Crow ignores a
  /// middleware response anyway, a non-WebSocket route answers 404 without
  /// running its handler, and the WebSocket route's onaccept runs the same
  /// access decision. On EVERY other path — an HTTP/1.0 request with an
  /// Upgrade header, an h2c upgrade — the request reaches an ordinary
  /// handler, so it is evaluated like any other (S3 review C1: skipping on
  /// `req.upgrade` alone let such requests through unauthenticated).
  static bool crow_takes_upgrade_path(const crow::request &req) {
    return req.upgrade && req.http_ver_major == 1 && req.http_ver_minor == 1 &&
           req.headers.count("host") > 0 &&
           req.method != crow::HTTPMethod::Options &&
           req.get_header_value("upgrade").find("h2") != 0;
  }

  void before_handle(crow::request &req, crow::response &res, context &ctx) {
    if (crow_takes_upgrade_path(req))
      return;

    // Host, origin, authentication and authorization, all in one decision
    // (Server::Impl::evaluate). Crow's header phase ran the same decision
    // before the body was read, so a refusal here is the rare request whose
    // standing changed in between (a session revoked mid-upload, say).
    // OPTIONS never reaches here — see the note in after_handle.
    const Gate gate = evaluate(req);
    if (!gate.decision.allowed()) {
      res = refusal(gate.decision);
      res.end();
      return;
    }
    ctx.principal = gate.principal;
    ctx.route = gate.route;
    ctx.role = gate.role;
    ctx.set_token_cookie = gate.token_via_query;

    // A browser that arrived with `?token=`: set the cookie and send it on to
    // the same URL without the token, so the secret leaves the address bar,
    // the history and any Referer.
    if (gate.token_via_query && req.method == crow::HTTPMethod::Get) {
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
    // Never reached on Crow's upgrade path (handle() returns before the
    // after-handlers there), so every response that went through
    // before_handle is decorated and audited.

    if (ctx.set_token_cookie)
      res.set_header("Set-Cookie", token_cookie());
    // No page this server sends may leak its URL to another site.
    res.set_header("Referrer-Policy", "no-referrer");
    apply_cors(req.get_header_value("Origin"), res);

    // Every mutation that went through is audited (server mode). Central, so
    // a route added later cannot forget it. Login and logout are audited by
    // AuthService itself (the principal is not known here for a login).
    const std::string method = crow::method_name(req.method);
    if (auth_ && !is_safe_method(method) && res.code < 400 &&
        ctx.principal.authenticated() && req.url != "/api/v1/auth/login" &&
        req.url != "/api/v1/auth/logout")
      auth_->audit(ctx.principal,
                   ctx.route.kind == RouteAccess::Kind::case_route
                       ? std::optional<std::string>(ctx.route.cid)
                       : std::nullopt,
                   method + " " + req.url, std::to_string(res.code));
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
  bool server_mode_ = false;
  Evaluate evaluate_;
  std::shared_ptr<AuthService> auth_;
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

    auth_ = options_.auth;
    // In server mode the superuser token lives in AuthService; the local
    // access-token machinery (cookie, ?token=) is off.
    if (auth_)
      options_.auth_token.clear();

    // Before any project is touched: a refused bind must not create or
    // migrate anything. Server mode always authenticates, so any bind goes.
    if (!auth_ && options_.auth_token.empty() &&
        !is_loopback_bind(options_.bind_address))
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

    session_cookie_name_ = session_cookie_name(options_.port);
    secure_cookie_name_ = "__Host-" + session_cookie_name_;
    app_.get_middleware<SecurityMiddleware>().configure(
        options_.allowed_origins, options_.auth_token, options_.bind_address,
        options_.port, auth_ != nullptr,
        [this](const crow::request &req) { return evaluate(req); }, auth_);
    // The same decision once the headers are in and BEFORE the body is read
    // (the Crow patch in overlays/crow.nix): an unauthenticated or forbidden
    // upload is refused without its body ever being buffered.
    app_.header_check([this](const crow::request &req, crow::response &res) {
      const Gate gate = evaluate(req);
      if (gate.decision.allowed())
        return true;
      res = SecurityMiddleware::refusal(gate.decision);
      return false;
    });
    if (auth_)
      spdlog::info("Server mode: users and sessions ({} cookie{}); "
                   "superuser token {}",
                   session_cookie_name_,
                   options_.cookie_secure == CookieSecure::never
                       ? ", not Secure"
                   : options_.cookie_secure == CookieSecure::always
                       ? ", Secure"
                       : ", Secure unless on loopback",
                   auth_->options().superuser_token.empty() ? "off" : "on");
    else if (!options_.auth_token.empty())
      spdlog::info("Access token required (Bearer header, '{}' cookie or "
                   "?token=)",
                   token_cookie_name(options_.port));
    for (const auto &origin : options_.allowed_origins)
      spdlog::info("Additional allowed origin: {}", origin);

    executor_ = options_.stage_executor ? options_.stage_executor
                                        : pipeline::default_stage_executor();
    scheduler_ = std::make_unique<pipeline::JobScheduler>(
        pipeline::JobSchedulerOptions{options_.job_workers},
        options_.job_store);
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
      if (auth_) {
        try {
          if (const auto n = auth_->purge_expired(); n > 0)
            spdlog::debug("Purged {} expired session(s)", n);
        } catch (const std::exception &e) {
          spdlog::warn("Purging expired sessions failed: {}", e.what());
        }
        revalidate_sockets();
        // Audit retention, hourly.
        const auto now = std::chrono::steady_clock::now();
        if (now - audit_pruned_ >= std::chrono::hours(1)) {
          audit_pruned_ = now;
          try {
            if (const auto n = auth_->prune_audit(options_.audit_retention);
                n > 0)
              spdlog::info("Pruned {} audit entr(y/ies) past retention", n);
          } catch (const std::exception &e) {
            spdlog::warn("Pruning the audit log failed: {}", e.what());
          }
        }
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

    spdlog::info("ruxd listening on {}{}", url(), auth_ ? "" : " (local mode)");
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
    spdlog::info("ruxd shutting down");
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
  /// A client-chosen path would make the server deserialise any file it can
  /// read as a model (review I3): in server mode the model comes only from
  /// the server's own configuration. Local mode (one trusted person) keeps
  /// the field.
  void refuse_client_model_path(const std::string &requested) const {
    if (!requested.empty() && auth_)
      throw HttpError(400, "'model_path' is not accepted by this server: it "
                           "uses its configured SAM3 model; omit the field");
  }

  std::string resolve_model_path(const std::string &requested, bool use_cuda) {
    refuse_client_model_path(requested);
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

  // --- access ----------------------------------------------------------------

  /// The one access decision for @p req: Host, Origin, who it is, what it
  /// touches and whether that is allowed (access.hpp holds the matrix). Runs
  /// twice per request — in Crow's header phase, before the body is read,
  /// and again in SecurityMiddleware — and must therefore be side-effect
  /// free apart from a session's sliding renewal.
  Gate evaluate(const crow::request &req) {
    const auto &security = app_.get_middleware<SecurityMiddleware>();
    Gate gate;
    const std::string method = crow::method_name(req.method);
    gate.route = classify_route(method, req.url);

    // RFC 9112 §3.2: a request with more than one Host field is a 400. Crow
    // keeps every copy and get_header_value() returns an arbitrary one, so
    // the check below could otherwise vet a different Host than a proxy or
    // the page saw (final review #4).
    if (req.headers.count("Host") > 1) {
      spdlog::warn("Refused a request with {} Host headers",
                   req.headers.count("Host"));
      gate.decision = {400, "more than one Host header"};
      return gate;
    }
    if (!security.host_ok(req)) {
      spdlog::warn("Refused a request for host '{}'",
                   req.get_header_value("Host"));
      gate.decision = {403, "host '" + req.get_header_value("Host") +
                                "' is not served here"};
      return gate;
    }
    const std::string origin = req.get_header_value("Origin");
    if (!origin.empty() && !security.origin_ok(req, origin)) {
      spdlog::warn("Refused a cross-origin request from '{}' to {}", origin,
                   req.url);
      gate.decision = {403, "origin '" + origin +
                                "' is not allowed; pass --allow-origin to "
                                "permit it"};
      return gate;
    }

    if (!auth_) {
      // Local mode: the optional access token guards everything, static
      // files included; whoever has it is the implicit owner of every case.
      // After the origin check so a foreign page learns nothing either way.
      const TokenCheck token = security.check(req);
      if (!token.ok) {
        gate.decision = {401, "missing or wrong access token"};
        return gate;
      }
      gate.token_via_query = token.via_query;
      gate.principal = local_principal();
      gate.role = Role::owner;
      gate.decision = decide_access(gate.principal, gate.route, gate.role);
      return gate;
    }

    // Server mode.
    PresentedCredentials credentials;
    const std::string authorization = req.get_header_value("Authorization");
    const std::string cookies = req.get_header_value("Cookie");
    credentials.bearer = bearer_token(authorization);
    // Both spellings: `__Host-` when the cookie was set Secure (review M7),
    // the plain per-port name on a loopback / plain-HTTP server.
    credentials.session_cookies = cookie_values(cookies, session_cookie_name_);
    for (const auto value : cookie_values(cookies, secure_cookie_name_))
      credentials.session_cookies.push_back(value);
    const std::string client = client_of(req);
    credentials.client_ip = client;
    try {
      gate.principal = auth_->authenticate(credentials);
      if (gate.route.kind == RouteAccess::Kind::case_route)
        gate.role = auth_->role_in(gate.principal, gate.route.cid);
    } catch (const std::exception &e) {
      spdlog::error("Authentication failed: {}", e.what());
      gate.decision = {503, "the user database is unavailable; retry shortly"};
      return gate;
    }
    gate.decision = decide_access(gate.principal, gate.route, gate.role);
    // A signed-in browser's mutation must say where it came from: browsers
    // send Origin on every non-GET request, so its absence means a forged
    // or non-browser request riding on the cookie. Bearer clients (scripts)
    // are exempt — they carry no ambient credential to abuse.
    if (gate.decision.allowed() &&
        gate.principal.kind == PrincipalKind::session &&
        !is_safe_method(method) && origin.empty())
      gate.decision = {403, "a signed-in request that changes something must "
                            "carry an Origin header"};
    return gate;
  }

  /// Who @p req is (set by SecurityMiddleware).
  const Principal &principal_of(const crow::request &req) {
    return app_.get_context<SecurityMiddleware>(req).principal;
  }

  /// Whether the session cookie set in answer to @p req is `Secure`.
  bool cookie_secure_for(const crow::request &req) const {
    switch (options_.cookie_secure) {
    case CookieSecure::always:
      return true;
    case CookieSecure::never:
      return false;
    case CookieSecure::automatic:
      break;
    }
    // Over TLS terminated by a proxy the bind is often loopback, so the
    // proxy's word counts too. Trusting it is harmless: it only adds Secure.
    std::string proto = req.get_header_value("X-Forwarded-Proto");
    std::transform(proto.begin(), proto.end(), proto.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return proto == "https" || !is_loopback_bind(options_.bind_address);
  }

  /// The client address of @p req: the TCP peer, or — when the peer is a
  /// `--trusted-proxy` — the address it forwarded (X-Forwarded-For).
  std::string client_of(const crow::request &req) const {
    return client_address(req.remote_ip_address,
                          req.get_header_value("X-Forwarded-For"),
                          options_.trusted_proxies);
  }

  /// The Set-Cookie value for a session token (empty + Max-Age 0 clears).
  /// A Secure cookie is named `__Host-ruxd_session_<port>`: browsers then
  /// refuse it unless it is Secure, host-only and Path=/, so another
  /// service on the same host cannot plant or shadow it over plain HTTP.
  std::string session_set_cookie(const crow::request &req,
                                 std::string_view token,
                                 std::chrono::seconds max_age) const {
    const bool secure = cookie_secure_for(req);
    return session_cookie(secure ? secure_cookie_name_ : session_cookie_name_,
                          token, max_age, secure);
  }

  /// The JSON for an API token: never the token or its hash.
  static json token_json(const ApiTokenRecord &t) {
    auto iso = [](SystemClock::time_point tp) {
      const auto secs = std::chrono::floor<std::chrono::seconds>(tp);
      const std::time_t c = SystemClock::to_time_t(secs);
      std::tm utc{};
      ::gmtime_r(&c, &utc);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &utc);
      return std::string(buf);
    };
    return json{
        {"id", t.id},
        {"name", t.name},
        {"case", t.case_id ? json(*t.case_id) : json(nullptr)},
        {"created_at", iso(t.created_at)},
        {"expires_at", t.expires_at ? json(iso(*t.expires_at)) : json(nullptr)},
        {"last_used_at",
         t.last_used_at ? json(iso(*t.last_used_at)) : json(nullptr)}};
  }

  /// The JSON for a user (never the password hash).
  static json user_json(const User &user) {
    return json{{"id", user.id},
                {"email", user.email},
                {"display_name", user.display_name},
                {"is_admin", user.is_admin},
                {"disabled", user.disabled},
                {"created_at", user.created_at}};
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
  nlohmann::json case_of(const CaseInfo &info, const Principal &who) {
    auto out = case_json(info, registry_->find_open(info.id) != nullptr);
    out["summary"] = summaries_.get(info);
    // The caller's role, so the UI can hide what it may not do.
    std::optional<Role> role =
        auth_ ? auth_->role_in(who, info.id) : std::optional(Role::owner);
    out["role"] = role ? json(std::string(to_string(*role))) : json(nullptr);
    return out;
  }

  /// Whether @p who may see case @p id in a list.
  bool can_see(const Principal &who, const std::string &id) {
    return !auth_ || auth_->role_in(who, id).has_value();
  }

  /// Make @p who the owner of the case they just created (server mode).
  void make_owner(const Principal &who, const std::string &cid) {
    if (auth_)
      if (const auto uid = who.user_id())
        auth_->stores().members->set_role(cid, *uid, Role::owner);
  }

  /// Who started an upload: only they may continue, inspect or finish it.
  std::string uploader_key(const Principal &who) const {
    return std::string(to_string(who.kind)) + ":" + std::to_string(who.user.id);
  }
  void check_uploader(const crow::request &req, const std::string &id) {
    std::lock_guard<std::mutex> lock(uploaders_mutex_);
    auto it = uploaders_.find(id);
    if (it == uploaders_.end() || it->second != uploader_key(principal_of(req)))
      throw HttpError(404, "no such upload");
  }

  // --- server-level routes: cases and uploads -------------------------------

  void register_case_routes() {
    app_.route_dynamic("/api/v1/cases")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            const Principal &who = principal_of(req);
            if (req.method == crow::HTTPMethod::GET) {
              // Only the cases the caller may see: a member's, or all for
              // an admin. The rest do not exist, as far as they can tell.
              json list = json::array();
              for (const auto &info : cases_->list())
                if (can_see(who, info.id))
                  list.push_back(case_of(info, who));
              return json_response(200, cases_list_json(std::move(list),
                                                        cases_->writable(),
                                                        uploads_->limits()));
            }
            const std::string name = parse_case_create(req.body);
            const CaseInfo info = cases_->create(name, who.user_id());
            make_owner(who, info.id);
            spdlog::info("Case '{}' created by {}", info.id, who.user.email);
            return json_response(201, case_of(info, who));
          });
        });

    app_.route_dynamic("/api/v1/cases/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH,
                 crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string cid) {
          return guarded([&] {
            const Principal &who = principal_of(req);
            if (req.method == crow::HTTPMethod::GET) {
              auto info = cases_->find(cid);
              if (!info)
                throw HttpError(404, "no such case '" + cid + "'");
              return json_response(200, case_of(*info, who));
            }
            if (req.method == crow::HTTPMethod::PATCH) {
              const auto patch = parse_case_patch(req.body);
              return json_response(200,
                                   case_of(cases_->update(cid, patch), who));
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
            // A new case may reuse the id; it must not inherit this history
            // — nor its members.
            scheduler_->store()->forget(cid);
            summaries_.forget(info->path);
            if (auth_)
              auth_->stores().members->forget_case(cid);
            registry_->end_delete(cid);
            return crow::response(204);
          });
        });

    app_.route_dynamic("/api/v1/uploads")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            const auto request = parse_upload_request(req.body);
            const auto session = uploads_->begin(request.name, request.size);
            {
              std::lock_guard<std::mutex> lock(uploaders_mutex_);
              uploaders_[session.id] = uploader_key(principal_of(req));
            }
            return json_response(201, upload_json(session, uploads_->limits()));
          });
        });

    app_.route_dynamic("/api/v1/uploads/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PUT,
                 crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string id) {
          return guarded([&] {
            check_uploader(req, id);
            if (req.method == crow::HTTPMethod::GET)
              return json_response(
                  200, upload_json(uploads_->status(id), uploads_->limits()));
            if (req.method == crow::HTTPMethod::DELETE) {
              uploads_->abort(id);
              forget_uploader(id);
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
            [this](const crow::request &req, std::string id) {
              return guarded([&] {
                check_uploader(req, id);
                auto [session, staged] = uploads_->finish(id);
                forget_uploader(id);
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
                // Runs on this request's worker thread: a large file holds
                // it for the length of one sequential read of the file.
                if (const auto bad = case_file_integrity_problem(staged);
                    !bad.empty()) {
                  discard();
                  spdlog::warn("Upload '{}' refused by the integrity check: "
                               "{}",
                               session.name, bad);
                  throw HttpError(422, bad);
                }
                const Principal &who = principal_of(req);
                CaseInfo info;
                try {
                  info = cases_->adopt(session.name, staged, who.user_id());
                } catch (...) {
                  discard();
                  throw;
                }
                make_owner(who, info.id);
                spdlog::info("Case '{}' created from an upload of {} bytes",
                             info.id, session.size);
                return json_response(201, case_of(info, who));
              });
            });
  }

  // --- server-level routes: auth, users, members ---------------------------

  /// 409 for a route that only makes sense with users (server mode).
  void require_server_mode() const {
    if (!auth_)
      throw HttpError(409, "local mode has no users: it serves one person, "
                           "who owns every case (run ruxd without --local "
                           "for users and members)");
  }

  static json parse_body_object(const std::string &body) {
    auto parsed = json::parse(body, nullptr, /*allow_exceptions=*/false);
    if (parsed.is_discarded() || !parsed.is_object())
      throw HttpError(400, "the body must be a JSON object");
    return parsed;
  }

  static std::string string_field(const json &body, const char *key,
                                  bool required) {
    if (!body.contains(key) || body[key].is_null()) {
      if (required)
        throw HttpError(400, std::string("'") + key + "' is required");
      return {};
    }
    if (!body[key].is_string())
      throw HttpError(400, std::string("'") + key + "' must be a string");
    return body[key].get<std::string>();
  }

  static std::optional<bool> bool_field(const json &body, const char *key) {
    if (!body.contains(key) || body[key].is_null())
      return std::nullopt;
    if (!body[key].is_boolean())
      throw HttpError(400, std::string("'") + key + "' must be true or false");
    return body[key].get<bool>();
  }

  static Role role_field(const json &body) {
    const auto name = string_field(body, "role", true);
    const auto role = parse_role(name);
    if (!role)
      throw HttpError(400, "'role' must be viewer, editor or owner");
    return *role;
  }

  static std::int64_t parse_id(const std::string &text) {
    try {
      std::size_t used = 0;
      const auto id = std::stoll(text, &used);
      if (used == text.size() && id > 0)
        return id;
    } catch (const std::exception &) {
    }
    throw HttpError(404, "no such user");
  }

  static json member_json(const Member &member) {
    return json{{"user", json{{"id", member.user.id},
                              {"email", member.user.email},
                              {"display_name", member.user.display_name}}},
                {"role", std::string(to_string(member.role))}};
  }

  void register_auth_routes() {
    app_.route_dynamic("/api/v1/auth/login")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req) {
          if (!auth_)
            return error_response(409, "local mode has no login");
          try {
            const auto body = parse_body_object(req.body);
            const auto result = auth_->login(
                string_field(body, "email", true),
                string_field(body, "password", true), client_of(req));
            auto res =
                json_response(200, json{{"mode", "server"},
                                        {"via", "session"},
                                        {"user", user_json(result.user)}});
            res.set_header(
                "Set-Cookie",
                session_set_cookie(req, result.session_token, result.max_age));
            res.set_header("Cache-Control", "no-store");
            return res;
          } catch (const LoginRateLimited &e) {
            auto res = error_response(e.status(), e.what());
            res.set_header("Retry-After",
                           std::to_string(e.retry_after().count()));
            return res;
          } catch (const HttpError &e) {
            return error_response(e.status(), e.what());
          } catch (const std::exception &e) {
            spdlog::error("Login failed: {}", e.what());
            return error_response(503, "the user database is unavailable");
          }
        });

    app_.route_dynamic("/api/v1/auth/logout")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            crow::response res(204);
            if (auth_) {
              const Principal &who = principal_of(req);
              auth_->logout(who);
              // Its open tabs' event sockets end with it (review I2).
              if (who.kind == PrincipalKind::session)
                close_sockets_if([&](const SocketCase &s) {
                  return s.principal.session_hash == who.session_hash;
                });
              res.set_header(
                  "Set-Cookie",
                  session_set_cookie(req, "", std::chrono::seconds(0)));
            }
            return res;
          });
        });

    app_.route_dynamic("/api/v1/auth/me")
        .methods(crow::HTTPMethod::GET)([this](const crow::request &req) {
          const Principal &who = principal_of(req);
          json out{{"mode", auth_ ? "server" : "local"},
                   {"via", std::string(to_string(who.kind))},
                   {"user", user_json(who.user)}};
          if (who.case_scope)
            out["case_scope"] = *who.case_scope;
          auto res = json_response(200, out);
          res.set_header("Cache-Control", "no-store");
          return res;
        });

    // ---- API tokens (one's own; an administrator's DELETE reaches any) ----
    app_.route_dynamic("/api/v1/auth/tokens")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            require_server_mode();
            const Principal &who = principal_of(req);
            const auto uid = who.user_id();
            // A token cannot mint more tokens; the superuser has no user.
            if (!uid || who.kind != PrincipalKind::session)
              throw HttpError(403, "API tokens are managed from a signed-in "
                                   "session (or `ruxd admin`)");
            if (req.method == crow::HTTPMethod::GET) {
              json list = json::array();
              for (const auto &t : auth_->stores().tokens->list(*uid))
                list.push_back(token_json(t));
              return json_response(200, json{{"tokens", std::move(list)}});
            }
            const auto body = parse_body_object(req.body);
            std::optional<std::string> scope;
            if (const auto c = string_field(body, "case", false); !c.empty()) {
              if (!auth_->role_in(who, c))
                throw HttpError(404, "no such case '" + c + "'");
              scope = c;
            }
            std::optional<std::chrono::seconds> life;
            if (body.contains("expires_days")) {
              if (!body["expires_days"].is_number_integer() ||
                  body["expires_days"].get<int>() < 0 ||
                  body["expires_days"].get<int>() > 3650)
                throw HttpError(400, "'expires_days' must be 0-3650 (0 = "
                                     "never)");
              life = std::chrono::hours(24) * body["expires_days"].get<int>();
            }
            const auto token = auth_->create_api_token(
                *uid, string_field(body, "name", true), scope, life);
            const auto stored = auth_->stores().tokens->find(sha256_hex(token));
            json out = stored ? token_json(*stored) : json::object();
            out["token"] = token; // Shown once; only its hash is kept.
            auto res = json_response(201, out);
            res.set_header("Cache-Control", "no-store");
            return res;
          });
        });

    app_.route_dynamic("/api/v1/auth/tokens/<string>")
        .methods(crow::HTTPMethod::DELETE)([this](const crow::request &req,
                                                  std::string raw_id) {
          return guarded([&] {
            require_server_mode();
            const Principal &who = principal_of(req);
            if (who.kind == PrincipalKind::api_token)
              throw HttpError(403, "a token cannot revoke tokens");
            std::int64_t id = 0;
            try {
              id = parse_id(raw_id);
            } catch (const HttpError &) {
              throw HttpError(404, "no such token");
            }
            const auto owner =
                who.is_admin() ? std::optional<std::int64_t>() : who.user_id();
            if (!auth_->stores().tokens->revoke(id, owner))
              throw HttpError(404, "no such token");
            return crow::response(204);
          });
        });

    // ---- users (admin) ----
    app_.route_dynamic("/api/v1/users")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            require_server_mode();
            if (req.method == crow::HTTPMethod::GET) {
              json list = json::array();
              for (const auto &user : auth_->stores().users->list())
                list.push_back(user_json(user));
              return json_response(200, json{{"users", std::move(list)}});
            }
            const auto body = parse_body_object(req.body);
            const auto user = auth_->create_user(
                string_field(body, "email", true),
                string_field(body, "display_name", false),
                string_field(body, "password", true),
                bool_field(body, "is_admin").value_or(false));
            return json_response(201, user_json(user));
          });
        });

    app_.route_dynamic("/api/v1/users/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH)(
            [this](const crow::request &req, std::string raw_id) {
              return guarded([&] {
                require_server_mode();
                const auto id = parse_id(raw_id);
                auto &users = *auth_->stores().users;
                if (!users.find_by_id(id))
                  throw HttpError(404, "no such user");
                if (req.method == crow::HTTPMethod::PATCH) {
                  const auto body = parse_body_object(req.body);
                  if (body.contains("display_name"))
                    users.set_display_name(
                        id, string_field(body, "display_name", true));
                  if (const auto admin = bool_field(body, "is_admin"))
                    users.set_admin(id, *admin);
                  const auto disabled = bool_field(body, "disabled");
                  if (disabled)
                    auth_->set_disabled(id, *disabled);
                  if (body.contains("password"))
                    auth_->set_password(id,
                                        string_field(body, "password", true));
                  // Ended sessions end their sockets too (review I2).
                  if (disabled.value_or(false) || body.contains("password"))
                    close_sockets_if([&](const SocketCase &s) {
                      return s.principal.user_id() == id;
                    });
                }
                return json_response(200, user_json(*users.find_by_id(id)));
              });
            });

    // ---- case members ----
    app_.route_dynamic(C("/members"))
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::POST)(
            [this](const crow::request &req, std::string cid) {
              return guarded([&] {
                require_server_mode();
                if (!cases_->find(cid))
                  throw HttpError(404, "no such case '" + cid + "'");
                auto &members = *auth_->stores().members;
                if (req.method == crow::HTTPMethod::POST) {
                  const auto body = parse_body_object(req.body);
                  const auto email =
                      normalize_email(string_field(body, "email", true));
                  const auto role = role_field(body);
                  // Who can be found: an administrator finds anyone; an
                  // owner only someone they already share a case with, so
                  // adding members is no oracle for which accounts exist
                  // (review M2). Both misses read the same.
                  auto user = auth_->stores().users->find_by_email(email);
                  const Principal &who = principal_of(req);
                  if (user && !who.is_admin()) {
                    const auto mine = who.user_id()
                                          ? members.cases_of(*who.user_id())
                                          : std::set<std::string>{};
                    const auto theirs = members.cases_of(user->id);
                    const bool shared = std::any_of(
                        theirs.begin(), theirs.end(),
                        [&](const std::string &c) { return mine.count(c); });
                    if (!shared)
                      user.reset();
                  }
                  if (!user)
                    throw HttpError(404, "no user you can add has the email '" +
                                             email +
                                             "'; an administrator adds people "
                                             "you have not worked with yet");
                  if (members.role_of(cid, user->id))
                    throw HttpError(409, "'" + email +
                                             "' is already a member; change "
                                             "their role instead");
                  members.set_role(cid, user->id, role);
                  return json_response(201, member_json(Member{*user, role}));
                }
                json list = json::array();
                for (const auto &member : members.members(cid))
                  list.push_back(member_json(member));
                return json_response(200, json{{"members", std::move(list)}});
              });
            });

    app_.route_dynamic(C("/members/<string>"))
        .methods(crow::HTTPMethod::PATCH, crow::HTTPMethod::DELETE)(
            [this](const crow::request &req, std::string cid,
                   std::string raw_id) {
              return guarded([&] {
                require_server_mode();
                if (!cases_->find(cid))
                  throw HttpError(404, "no such case '" + cid + "'");
                const auto id = parse_id(raw_id);
                auto &members = *auth_->stores().members;
                // Check and write in one atomic step in the store (review
                // M1): two owners demoting each other cannot both succeed.
                auto check = [&](MemberChange change) {
                  if (change == MemberChange::not_member)
                    throw HttpError(404, "that user is not a member of '" +
                                             cid + "'");
                  if (change == MemberChange::last_owner)
                    throw HttpError(409, "a case must keep at least one "
                                         "owner; make someone else owner "
                                         "first");
                };
                if (req.method == crow::HTTPMethod::DELETE) {
                  check(members.remove_member(cid, id));
                  // Their open tabs on this case stop hearing it (I2).
                  close_sockets_if([&](const SocketCase &s) {
                    return s.cid == cid && s.principal.user_id() == id;
                  });
                  return crow::response(204);
                }
                const auto role = role_field(parse_body_object(req.body));
                check(members.change_role(cid, id, role));
                const auto user = auth_->stores().users->find_by_id(id);
                return json_response(200, member_json(Member{*user, role}));
              });
            });

    // ---- readiness ----
    app_.route_dynamic("/api/v1/readyz")
        .methods(crow::HTTPMethod::GET)([this](const crow::request &) {
          // Cached for a second: an anonymous caller must not be able to
          // drive a database round trip per request (review M3).
          bool ready = true;
          {
            std::lock_guard<std::mutex> lock(ready_mutex_);
            const auto now = std::chrono::steady_clock::now();
            if (now - ready_checked_ >= std::chrono::seconds(1)) {
              try {
                ready_ = !options_.readiness || options_.readiness();
              } catch (const std::exception &e) {
                spdlog::warn("Readiness probe failed: {}", e.what());
                ready_ = false;
              }
              ready_checked_ = now;
            }
            ready = ready_;
          }
          return json_response(ready ? 200 : 503,
                               json{{"status", ready ? "ready" : "not_ready"}});
        });
  }

  // --- routes --------------------------------------------------------------

  void register_routes() {
    auto get = [this](const std::string &path) -> crow::DynamicRule & {
      return app_.route_dynamic(path).methods(crow::HTTPMethod::GET);
    };

    register_case_routes();
    register_auth_routes();

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
        // A render is seconds of VTK work on a Crow thread: a case list full
        // of new cards must not start one per card at once (review N7).
        RenderSlot slot(render_slots_);
        if (!slot.acquired())
          throw HttpError(503, "the server is busy rendering; retry shortly");
        if (auto hit = renders_.get(info->path, key)) // Rendered meanwhile.
          return blob_response(*hit);
        const Params params = params_of(req);
        // Stamp first: a write committing during the render must leave the
        // entry stale, not cache the old image under the new stamp (N2).
        const FileStamp stamp = file_stamp(info->path);
        Blob blob;
        {
          reusex::ProjectDB db(info->path, /*readOnly=*/true);
          blob = render_blob(db, view_renderer_, params);
        }
        renders_.put(info->path, key, stamp, blob);
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
              // Parse and validate the request body first: a refused
              // model_path is a 400 whatever the server has loaded.
              const auto seg_req =
                  parse_segment_frame_request(req.body, options_.segment_cuda);
              refuse_client_model_path(seg_req.model_path);
              if (!segmenter_)
                return error_response(503,
                                      "no SAM3 model registered; start the "
                                      "server via 'ruxd --local' and ensure a "
                                      "model is available");

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
              const auto seg_req = parse_segment_panorama_request(
                  req.body, options_.segment_cuda);
              refuse_client_model_path(seg_req.model_path);
              if (!panorama_segmenter_)
                return error_response(
                    503, "no SAM3 panorama segmenter registered; start the "
                         "server via 'ruxd --local' and ensure a model is "
                         "available");

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
              const auto who = principal_of(req).user_id();
              const auto id =
                  ctx.jobs().submit(submission.stage, submission.parameters,
                                    who ? std::to_string(*who) : std::string());
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
          // Host, origin, authentication and membership: the same decision
          // as every HTTP request (the header phase already ran it; this is
          // the guard Crow's upgrade path does honour). An empty Origin is a
          // non-browser client (curl, the CLI, a test).
          const Gate gate = evaluate(req);
          if (!gate.decision.allowed()) {
            spdlog::warn("Refused a WebSocket upgrade: {}",
                         gate.decision.message);
            res = SecurityMiddleware::refusal(gate.decision);
            return;
          }
          const auto cid = case_id_of_events_url(req.url);
          if (!cid || !cases_->find(*cid)) {
            res = error_response(404, "no such case");
            return;
          }
          // Handed to onopen, which owns it from there.
          *userdata = new SocketCase{*cid, nullptr, gate.principal};
        })
        .onopen([this](crow::websocket::connection &conn) {
          std::unique_ptr<SocketCase> accepted(
              static_cast<SocketCase *>(conn.userdata()));
          conn.userdata(nullptr);
          const std::string *cid = accepted ? &accepted->cid : nullptr;
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
            // The socket keeps no reference to the context at all — only the
            // case id and its subscriber hub. An open tab keeps its case open
            // through the subscriber count, not by pinning it, and no socket
            // handler can ever be the one that destroys a context (review
            // N1: a deleted case's WAL anchor closing on an IO thread after
            // its files had moved).
            std::lock_guard lock(sockets_mutex_);
            sockets_[&conn] =
                SocketCase{ctx->id(), ctx->subscribers(), accepted->principal};
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
          auto hub = socket_hub(conn);
          auto reply =
              handle_ws_message(data, [&](std::optional<std::string> job_id) {
                if (hub)
                  hub->set_filter(&conn, std::move(job_id));
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

  void forget_uploader(const std::string &id) {
    std::lock_guard<std::mutex> lock(uploaders_mutex_);
    uploaders_.erase(id);
  }

  /// Close every events socket @p pred picks. Held under sockets_mutex_,
  /// which keeps each connection alive (its close handler takes the same
  /// lock before Crow frees it); close() only queues the close frame.
  template <typename Pred> std::size_t close_sockets_if(Pred &&pred) {
    std::size_t closed = 0;
    std::lock_guard lock(sockets_mutex_);
    std::vector<crow::websocket::connection *> picked;
    for (auto &[conn, entry] : sockets_)
      if (pred(entry)) {
        entry.hub->unsubscribe(conn);
        picked.push_back(conn);
      }
    // Still under the lock (each connection stays alive), but no longer
    // iterating the table: an inline close handler erases from it.
    for (auto *conn : picked)
      if (sockets_.count(conn) > 0) {
        conn->close("access ended");
        ++closed;
      }
    return closed;
  }

  /// Re-check every socket's principal and membership (the sweeper's beat):
  /// catches what no request announces — an expired session, a user
  /// disabled with `ruxd admin`, a revoked token.
  void revalidate_sockets() {
    if (!auth_)
      return;
    std::vector<std::pair<std::string, Principal>> open;
    {
      std::lock_guard lock(sockets_mutex_);
      for (const auto &[conn, entry] : sockets_)
        open.emplace_back(entry.cid, entry.principal);
    }
    std::set<std::pair<std::string, std::string>> gone;
    for (const auto &[cid, who] : open) {
      bool ok = false;
      try {
        ok = auth_->still_valid(who) && auth_->role_in(who, cid).has_value();
      } catch (const std::exception &e) {
        spdlog::warn("Re-checking a socket failed: {}", e.what());
        continue; // A database hiccup closes nothing.
      }
      if (!ok)
        gone.emplace(cid, who.session_hash + "|" + who.token_hash + "|" +
                              std::to_string(who.user.id));
    }
    if (gone.empty())
      return;
    const auto n = close_sockets_if([&](const SocketCase &s) {
      return gone.count({s.cid, s.principal.session_hash + "|" +
                                    s.principal.token_hash + "|" +
                                    std::to_string(s.principal.user.id)}) > 0;
    });
    spdlog::info("Closed {} events socket(s) whose access ended", n);
  }

  /// The subscriber hub of the case a socket subscribed to.
  std::shared_ptr<SubscriberHub> socket_hub(crow::websocket::connection &conn) {
    std::lock_guard lock(sockets_mutex_);
    auto it = sockets_.find(&conn);
    return it == sockets_.end() ? nullptr : it->second.hub;
  }

  /// Unsubscribe and forget a socket. Runs before Crow frees the connection,
  /// which is what makes SubscriberHub::send_if's send-under-lock safe. Goes
  /// through the hub, never the context (see SocketCase).
  void drop_socket(crow::websocket::connection &conn) {
    SocketCase entry;
    {
      std::lock_guard lock(sockets_mutex_);
      auto it = sockets_.find(&conn);
      if (it == sockets_.end())
        return;
      entry = std::move(it->second);
      sockets_.erase(it);
    }
    entry.hub->unsubscribe(&conn);
    // Counts as use; find_open never hands out a case that is closing or
    // being deleted, and its lease is the ordinary kind begin_delete waits
    // out.
    if (auto ctx = registry_->find_open(entry.cid))
      ctx->touch(ProjectContext::Clock::now());
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
  /// Server mode; nullptr in local mode.
  std::shared_ptr<AuthService> auth_;
  std::string session_cookie_name_;
  std::string secure_cookie_name_;
  std::mutex ready_mutex_;
  std::chrono::steady_clock::time_point ready_checked_{};
  bool ready_ = false;
  std::chrono::steady_clock::time_point audit_pruned_{};
  /// Upload id -> who started it (uploader_key).
  std::mutex uploaders_mutex_;
  std::unordered_map<std::string, std::string> uploaders_;
  /// Renders that may run at once (cache misses only).
  std::counting_semaphore<kMaxConcurrentRenders> render_slots_{
      kMaxConcurrentRenders};
  /// One held render slot, released on scope exit.
  class RenderSlot {
      public:
    explicit RenderSlot(std::counting_semaphore<kMaxConcurrentRenders> &slots)
        : slots_(slots), acquired_(slots.try_acquire_for(kRenderSlotWait)) {}
    ~RenderSlot() {
      if (acquired_)
        slots_.release();
    }
    RenderSlot(const RenderSlot &) = delete;
    RenderSlot &operator=(const RenderSlot &) = delete;
    bool acquired() const noexcept { return acquired_; }

      private:
    std::counting_semaphore<kMaxConcurrentRenders> &slots_;
    bool acquired_;
  };

  /// Recursive: closing a socket from a request handler can run Crow's close
  /// handler inline (asio::dispatch on that io thread), which re-enters
  /// drop_socket.
  std::recursive_mutex sockets_mutex_;
  /// What an open WebSocket subscribed to: the case id and its subscriber
  /// hub — deliberately not the context (review N1).
  struct SocketCase {
    std::string cid;
    std::shared_ptr<SubscriberHub> hub;
    /// Who opened it, re-checked on the sweep and when their access ends.
    Principal principal;
  };
  /// Open WebSocket -> the case it subscribed to.
  std::unordered_map<crow::websocket::connection *, SocketCase> sockets_;

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
