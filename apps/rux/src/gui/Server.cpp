// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/Server.hpp"

#include "gui/api.hpp"
#include "gui/assets.hpp"
#include "gui/edits.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/JobRunner.hpp>

#include <crow.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
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

namespace rux::gui {
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
///  1. **Origin allowlist.** Loopback, plus anything named with
///     --allow-origin. A request carrying any other Origin is refused with
///     403 *before routing*, so no handler runs and nothing is disclosed.
///     This is server-side enforcement, not a hint to the browser — it holds
///     for simple requests, which are dispatched before CORS is consulted.
///  2. **application/json required on mutating routes.** A form or simple POST
///     cannot set that header without triggering a preflight, so this closes
///     the CSRF-shaped hole that a `text/plain` POST would otherwise leave.
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
  struct context {};

  void configure(std::vector<std::string> extra_origins) {
    extra_origins_ = std::move(extra_origins);
  }

  void before_handle(crow::request &req, crow::response &res, context &) {
    // WebSocket upgrades bypass this: Crow calls handle_upgrade regardless of
    // whether middleware completed the response, so a rejection here would be
    // ignored. The origin check for /events lives in its onaccept handler.
    if (req.upgrade)
      return;

    // OPTIONS never reaches here — see the note in after_handle.
    const std::string origin = req.get_header_value("Origin");
    const bool cross_origin = !origin.empty() && !is_allowed(origin);

    if (cross_origin) {
      spdlog::warn("Refused a cross-origin request from '{}' to {}", origin,
                   req.url);
      res = error_response(403, "origin '" + origin +
                                    "' is not allowed; pass --allow-origin to "
                                    "permit it");
      res.end();
      return;
    }

    // Every method that can change the project, not just POST. A `text/plain`
    // POST is a CORS *simple request*, dispatched by the browser before it
    // reads a single response header, and demanding JSON is what a forged one
    // cannot satisfy. PUT/PATCH/DELETE are preflighted and so are already
    // covered by the origin check — but gating on the method rather than on
    // that reasoning means a future route cannot quietly opt out of CSRF
    // protection by choosing a different verb.
    if (is_mutating(req.method)) {
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

  static bool is_mutating(crow::HTTPMethod method) {
    return method == crow::HTTPMethod::Post ||
           method == crow::HTTPMethod::Put ||
           method == crow::HTTPMethod::Patch ||
           method == crow::HTTPMethod::Delete;
  }

  void after_handle(crow::request &req, crow::response &res, context &) {
    if (req.upgrade)
      return;

    apply_cors(req.get_header_value("Origin"), res);
  }

  bool is_allowed(const std::string &origin) const {
    if (is_loopback_origin(origin))
      return true;
    return std::find(extra_origins_.begin(), extra_origins_.end(), origin) !=
           extra_origins_.end();
  }

    private:
  void apply_cors(const std::string &origin, crow::response &res) const {
    if (origin.empty() || !is_allowed(origin))
      return;
    // Echo the specific origin, never "*": the allowlist is the policy, and
    // Vary tells caches the response depends on who asked.
    res.set_header("Access-Control-Allow-Origin", origin);
    res.set_header("Vary", "Origin");
  }

  std::vector<std::string> extra_origins_;
};

/// The concrete Crow application type for `rux gui`, with the security
/// middleware installed. Used everywhere instead of crow::SimpleApp.
using App = crow::App<SecurityMiddleware>;

// ===========================================================================
// Server::Impl
// ===========================================================================

class Server::Impl {
    public:
  explicit Impl(ServerOptions options) : options_(std::move(options)) {
    if (options_.port == 0)
      throw std::runtime_error(
          "--port 0 is not supported: Crow cannot report back which ephemeral "
          "port it bound, so nothing could tell you where to connect");

    // Open read-write exactly once, on the way up. This creates and migrates
    // the database if needed, so every later per-request connection can be
    // read-only, and a broken project fails at startup instead of on the first
    // fetch (STANDARDS §5).
    {
      reusex::ProjectDB db(options_.project, /*readOnly=*/false);
      spdlog::info("Project '{}' opened (schema v{})",
                   options_.project.filename().string(), db.schema_version());
    }

    options_.asset_dir = resolve_asset_dir(options_.asset_dir);

    if (!is_plausible_host(options_.bind_address))
      throw std::runtime_error("--bind '" + options_.bind_address +
                               "' is not a valid host or IP literal");

    app_.get_middleware<SecurityMiddleware>().configure(
        options_.allowed_origins);
    for (const auto &origin : options_.allowed_origins)
      spdlog::info("Additional allowed origin: {}", origin);

    runner_ = std::make_unique<pipeline::JobRunner>(options_.project);
    listener_ = runner_->add_listener(
        [this](const pipeline::JobEvent &event) { broadcast(event); });

    register_routes();
  }

  ~Impl() {
    // Stop the producers before the things they touch go away. app_.stop()
    // closes the WebSocket connections (running their close handlers, which
    // take clients_mutex_), and resetting the runner joins its worker thread,
    // which may be mid-broadcast. Doing this here — rather than relying on
    // member destruction order alone — keeps the shutdown sequence explicit.
    app_.stop();
    if (runner_) {
      runner_->remove_listener(listener_);
      runner_.reset();
    }
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.clear();
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

  int run() {
    app_.validate();
    app_.bindaddr(options_.bind_address).port(options_.port);
    if (options_.threads > 0)
      app_.concurrency(options_.threads);
    else
      app_.multithreaded();

    spdlog::info("rux gui listening on {}", url());
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
    spdlog::info("rux gui shutting down");
    return 0;
  }

  void stop() { app_.stop(); }

    private:
  // --- ProjectDB access ----------------------------------------------------

  /// Run @p handler against a fresh read-only ProjectDB.
  ///
  /// ProjectDB is not thread-safe and Crow is multi-threaded, so each request
  /// gets its own connection rather than sharing one behind a mutex — sqlite3
  /// handles concurrent readers natively and a global lock would serialize the
  /// whole GUI behind whichever request is decoding a mesh blob.
  ///
  /// Writers are the job worker and the editor endpoints (with_write below);
  /// both hold the runner's writer lock, so at most one of them is writing at
  /// any moment and this connection never has to reason about them separately.
  template <typename Handler> crow::response with_db(Handler &&handler) {
    try {
      reusex::ProjectDB db(options_.project, /*readOnly=*/true);
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

  /// Run @p handler against a writable ProjectDB, under the project's writer
  /// lock.
  ///
  /// The editor endpoints are the second writer in this process; the pipeline
  /// job worker is the first. They exclude each other through
  /// JobRunner::try_acquire_writer, which the worker holds for the whole of
  /// every stage. Two rules follow, and both are deliberate:
  ///
  ///  * **A queued or running job means 409, not a wait.** A stage holds the
  ///    lock for minutes; blocking a request that long is indistinguishable
  ///    from a hung UI, and the user can retry when the run is done.
  ///  * **Failing to take the lock means 503, not a longer wait.** That path is
  ///    another editor request mid-write — milliseconds — so a short timeout
  ///    absorbs the normal case and anything past it is worth reporting.
  ///
  /// Nothing is written when either check fails, so both are safe to retry.
  template <typename Handler> crow::response with_write(Handler &&handler) {
    if (runner_->is_busy() || runner_->queued_count() > 0)
      return error_response(409,
                            "a pipeline job is running or queued; edits are "
                            "refused while a stage is writing the project");

    auto lease = runner_->try_acquire_writer(
        std::chrono::milliseconds(kWriteLockTimeoutMs));
    if (!lease.owns_lock())
      return error_response(503, "the project is being written to; retry "
                                 "shortly");

    try {
      reusex::ProjectDB db(options_.project, /*readOnly=*/false);
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

  // --- WebSocket -----------------------------------------------------------

  void broadcast(const pipeline::JobEvent &event) {
    const std::string payload =
        job_event_json(event, options_.project.filename().string()).dump();

    // LOCKING INVARIANT — the sends happen INSIDE clients_mutex_ on purpose.
    //
    // crow::websocket::connection is a raw pointer we do not own, and Crow
    // does not hand out a shared_ptr for it: check_destroy() invokes the close
    // handler and then frees the object. The close handler registered below
    // erases the entry while holding this same mutex, so a connection present
    // in clients_ cannot be destroyed while we hold the lock. Snapshotting the
    // pointers and sending after unlocking — the obvious-looking version —
    // races a closing tab against the job worker and sends into freed memory.
    //
    // Holding the lock across the send is cheap and cannot deadlock:
    // send_data() serialises the frame and posts it to the asio io_context, it
    // never runs a handler inline, so nothing re-enters this mutex.
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (auto &[connection, subscription] : clients_) {
      if (!event_matches_subscription(event, subscription))
        continue;
      try {
        connection->send_text(payload);
      } catch (const std::exception &e) {
        spdlog::debug("WebSocket send failed: {}", e.what());
      }
    }
  }

  // --- routes --------------------------------------------------------------

  void register_routes() {
    auto get = [this](const std::string &path) -> crow::DynamicRule & {
      return app_.route_dynamic(path).methods(crow::HTTPMethod::GET);
    };

    // ---- meta ----
    get("/api/v1/health")([this](const crow::request &) {
      // Never fails: a browser pointed at a broken project should still be told
      // *that*, in the documented shape, rather than get a bare 500.
      try {
        reusex::ProjectDB db(options_.project, /*readOnly=*/true);
        return json_response(200, health_json(&db, options_.project));
      } catch (const std::exception &e) {
        spdlog::warn("Health check could not open the project: {}", e.what());
        return json_response(200, health_json(nullptr, options_.project));
      }
    });

    get("/api/v1/endpoints")([](const crow::request &) {
      crow::response res(200, endpoints_json().dump(-1));
      res.set_header("Content-Type", "application/json");
      res.set_header("Access-Control-Allow-Origin", "*");
      return res;
    });

    // ---- project ----
    get("/api/v1/project")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, project_summary_json(db));
      });
    });

    get("/api/v1/projects")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, projects_json(db, params));
      });
    });

    // ---- clouds ----
    get("/api/v1/clouds")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, clouds_json(db, params));
      });
    });

    get("/api/v1/clouds/<string>")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, cloud_json(db, name));
          });
        });

    // The only route with two wire formats, so the only one that cannot go
    // straight through json_response(): `format=binary` answers a RUXP page
    // (docs/gui/binary-points.md), which is not JSON.
    get("/api/v1/clouds/<string>/points")(
        [this](const crow::request &req, std::string name) {
          const Params params = params_of(req);
          return with_db([&](const reusex::ProjectDB &db) {
            const auto points = cloud_points(db, name, params);
            if (points.body)
              return json_response(200, *points.body);
            crow::response res = blob_response(*points.blob);
            for (const auto &[key, value] : points.headers)
              res.set_header(key, value);
            return res;
          });
        });

    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules (same reasoning as /jobs below).
    app_.route_dynamic("/api/v1/clouds/<string>/labels")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH)(
            [this](const crow::request &req, std::string name) {
              if (req.method == crow::HTTPMethod::GET)
                return with_db([&](const reusex::ProjectDB &db) {
                  return json_response(200, cloud_labels_json(db, name));
                });
              return with_write([&](reusex::ProjectDB &db) {
                return json_response(200,
                                     patch_cloud_labels(db, name, req.body));
              });
            });

    // ---- meshes ----
    get("/api/v1/meshes")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, meshes_json(db, params));
      });
    });

    get("/api/v1/meshes/<string>")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, mesh_json(db, name));
          });
        });

    get("/api/v1/meshes/<string>/data")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return blob_response(mesh_data_blob(db, name));
          });
        });

    get("/api/v1/meshes/<string>/textures")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, mesh_textures_json(db, name));
          });
        });

    get("/api/v1/meshes/<string>/textures/<string>")(
        [this](const crow::request &, std::string name, std::string texture) {
          return with_db([&](const reusex::ProjectDB &db) {
            return blob_response(mesh_texture_blob(db, name, texture));
          });
        });

    // ---- sensor frames ----
    get("/api/v1/frames")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, frames_json(db, params));
      });
    });

    get("/api/v1/frames/<int>")([this](const crow::request &, int id) {
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, frame_json(db, id));
      });
    });

    get("/api/v1/frames/<int>/image")([this](const crow::request &req, int id) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        const auto image = frame_image(db, id, params);
        crow::response res = blob_response(image.blob);
        // Convenience only, exactly like the X-Ruxp-* headers: the picture is
        // the response, and a client must not need these to use it.
        if (image.range.valid) {
          res.set_header("X-Image-Range-Min", std::to_string(image.range.min));
          res.set_header("X-Image-Range-Max", std::to_string(image.range.max));
        }
        return res;
      });
    });

    // ---- panoramas ----
    get("/api/v1/panoramas")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, panoramas_json(db, params));
      });
    });

    get("/api/v1/panoramas/<int>")([this](const crow::request &, int id) {
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, panorama_json(db, id));
      });
    });

    get("/api/v1/panoramas/<int>/image")([this](const crow::request &, int id) {
      return with_db([&](const reusex::ProjectDB &db) {
        return blob_response(panorama_image_blob(db, id));
      });
    });

    // ---- components / materials / instances ----
    get("/api/v1/components")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, components_json(db, params));
      });
    });

    get("/api/v1/components/<string>")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, component_json(db, name));
          });
        });

    get("/api/v1/materials")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, materials_json(db, params));
      });
    });

    app_.route_dynamic("/api/v1/materials/<string>")
        .methods(crow::HTTPMethod::GET, crow::HTTPMethod::PATCH)(
            [this](const crow::request &req, std::string guid) {
              if (req.method == crow::HTTPMethod::GET)
                return with_db([&](const reusex::ProjectDB &db) {
                  return json_response(200, material_json(db, guid));
                });
              return with_write([&](reusex::ProjectDB &db) {
                return json_response(200, patch_material(db, guid, req.body));
              });
            });

    get("/api/v1/instances/<string>")(
        [this](const crow::request &req, std::string cloud) {
          const Params params = params_of(req);
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, instances_json(db, cloud, params));
          });
        });

    // ---- pipeline ----
    get("/api/v1/stages")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, stages_json(db));
      });
    });

    get("/api/v1/stages/<string>/validation")(
        [this](const crow::request &, std::string stage) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, stage_validation_json(db, stage));
          });
        });

    get("/api/v1/pipeline-log")([this](const crow::request &req) {
      const Params params = params_of(req);
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, pipeline_log_json(db, params));
      });
    });

    // ---- jobs ----
    // One rule for both methods: registering the same path twice would create
    // two competing Crow rules.
    app_.route_dynamic("/api/v1/jobs")
        .methods(crow::HTTPMethod::GET,
                 crow::HTTPMethod::POST)([this](const crow::request &req) {
          return guarded([&] {
            const auto project = options_.project.filename().string();
            if (req.method == crow::HTTPMethod::GET)
              return json_response(200, jobs_page_json(runner_->jobs(), project,
                                                       params_of(req)));

            const auto submission = parse_job_request(req.body);
            check_job_project(submission, project);
            const auto id =
                runner_->submit(submission.stage, submission.parameters);
            auto record = runner_->job(id);
            if (!record)
              throw HttpError(500, "job vanished immediately after submit");
            return json_response(202, job_json(*record, project));
          });
        });

    get("/api/v1/jobs/<string>")([this](const crow::request &, std::string id) {
      return guarded([&] {
        auto record = runner_->job(id);
        if (!record)
          throw HttpError(404, "no such job '" + id + "'");
        return json_response(
            200, job_json(*record, options_.project.filename().string()));
      });
    });

    app_.route_dynamic("/api/v1/jobs/<string>/cancel")
        .methods(crow::HTTPMethod::POST)([this](const crow::request &,
                                                std::string id) {
          return guarded([&] {
            if (!runner_->cancel(id))
              throw HttpError(404, "no such job '" + id + "'");
            auto record = runner_->job(id);
            if (!record)
              throw HttpError(404, "no such job '" + id + "'");
            return json_response(
                200, job_json(*record, options_.project.filename().string()));
          });
        });

    register_websocket();
    register_static();
  }

  void register_websocket() {
    CROW_WEBSOCKET_ROUTE(app_, "/api/v1/events")
        .onaccept([this](const crow::request &req, void **) {
          // WebSockets are NOT subject to CORS — a browser will happily open
          // one cross-origin and hand the frames to the attacker's script. The
          // handshake is therefore the only place this can be enforced, and
          // Crow ignores a middleware response on the upgrade path, so the
          // check lives here rather than in SecurityMiddleware.
          const std::string origin = req.get_header_value("Origin");
          if (origin.empty())
            return true; // Non-browser client (curl, the CLI, a test).
          if (app_.get_middleware<SecurityMiddleware>().is_allowed(origin))
            return true;
          spdlog::warn("Refused a WebSocket upgrade from origin '{}'", origin);
          return false;
        })
        .onopen([this](crow::websocket::connection &conn) {
          {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            clients_.emplace(&conn, std::nullopt);
          }
          spdlog::debug("WebSocket client connected");
          // Snapshot on connect, so a client that joins mid-run is immediately
          // consistent without a separate GET /jobs.
          conn.send_text(hello_json(runner_->jobs(), options_.project).dump());
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
          auto reply = handle_ws_message(
              data, [this, &conn](std::optional<std::string> job_id) {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                auto it = clients_.find(&conn);
                if (it != clients_.end())
                  it->second = std::move(job_id);
              });
          if (reply)
            conn.send_text(reply->dump());
        })
        .onclose([this](crow::websocket::connection &conn, const std::string &,
                        uint16_t) {
          std::lock_guard<std::mutex> lock(clients_mutex_);
          clients_.erase(&conn);
          spdlog::debug("WebSocket client disconnected");
        })
        .onerror([this](crow::websocket::connection &conn,
                        const std::string &reason) {
          spdlog::debug("WebSocket error: {}", reason);
          std::lock_guard<std::mutex> lock(clients_mutex_);
          clients_.erase(&conn);
        });
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

    crow::response res(200,
                       placeholder_page(options_.project.filename().string()));
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
      // tests/unit/rux_gui/test_gui_server_socket.cpp.
      res = static_response(req);
    });
  }

  ServerOptions options_;

  // MEMBER ORDER IS LOAD-BEARING (destruction runs in reverse). The client
  // table and its mutex are declared FIRST so they are destroyed LAST: the job
  // worker inside runner_, and Crow's own threads inside app_, both touch them
  // right up until they are stopped. ~Impl also tears those two down
  // explicitly, so this ordering is the belt to that braces.
  std::mutex clients_mutex_;
  /// Connection -> its job filter (nullopt = receives every event).
  std::unordered_map<crow::websocket::connection *, std::optional<std::string>>
      clients_;

  App app_;
  std::unique_ptr<pipeline::JobRunner> runner_;
  size_t listener_ = 0;
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

int Server::run() { return impl_->run(); }

void Server::stop() { impl_->stop(); }

} // namespace rux::gui
