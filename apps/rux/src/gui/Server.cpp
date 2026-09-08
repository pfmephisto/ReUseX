// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/Server.hpp"

#include "gui/api.hpp"
#include "gui/assets.hpp"

#include <reusex/core/ProjectDB.hpp>
#include <reusex/pipeline/JobRunner.hpp>

#include <crow.h>
#include <spdlog/spdlog.h>

#include <cstdlib>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace rux::gui {
namespace {

namespace pipeline = reusex::pipeline;
using json = nlohmann::json;

crow::response json_response(int status, const json &body) {
  crow::response res(status, body.dump(2));
  res.set_header("Content-Type", "application/json");
  // The frontend may be served from a Vite dev server on another port during
  // Phase 2 development, so permit cross-origin reads. Safe here because the
  // server is loopback-only by default and exposes no credentials.
  res.set_header("Access-Control-Allow-Origin", "*");
  return res;
}

crow::response blob_response(const Blob &blob) {
  crow::response res(200);
  res.body.assign(reinterpret_cast<const char *>(blob.data.data()),
                  blob.data.size());
  res.set_header("Content-Type", blob.content_type);
  res.set_header("Access-Control-Allow-Origin", "*");
  return res;
}

crow::response error_response(int status, std::string_view message) {
  return json_response(status, error_json(status, message));
}

/// Open the system browser without blocking or polluting stdout.
void launch_browser(const std::string &url) {
  std::thread([url] {
    const std::string command = "xdg-open '" + url + "' >/dev/null 2>&1";
    if (std::system(command.c_str()) != 0)
      spdlog::warn("Could not open a browser; visit {} manually", url);
  }).detach();
}

} // namespace

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

    runner_ = std::make_unique<pipeline::JobRunner>(options_.project);
    listener_ = runner_->add_listener(
        [this](const pipeline::JobEvent &event) { broadcast(event); });

    register_routes();
  }

  ~Impl() {
    if (runner_)
      runner_->remove_listener(listener_);
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

    if (options_.open_browser)
      launch_browser(url());

    app_.run(); // Crow installs its own SIGINT/SIGTERM handling.
    spdlog::info("rux gui shutting down");
    return 0;
  }

    private:
  // --- ProjectDB access ----------------------------------------------------

  /// Run @p handler against a fresh read-only ProjectDB.
  ///
  /// ProjectDB is not thread-safe and Crow is multi-threaded, so each request
  /// gets its own connection rather than sharing one behind a mutex — sqlite3
  /// handles concurrent readers natively and a global lock would serialize the
  /// whole GUI behind whichever request is decoding a mesh blob. The job worker
  /// is the only writer and owns a separate connection of its own.
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
    const std::string payload = job_event_json(event).dump();

    std::vector<crow::websocket::connection *> targets;
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      targets.reserve(clients_.size());
      for (const auto &[connection, subscription] : clients_)
        if (event_matches_subscription(event, subscription))
          targets.push_back(connection);
    }
    // Send outside the lock: a slow client must not stall the job worker's
    // event emission, and send_text can re-enter Crow.
    for (auto *connection : targets) {
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
      crow::response res(200, endpoints_json().dump(2));
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

    get("/api/v1/projects")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, projects_json(db));
      });
    });

    // ---- clouds ----
    get("/api/v1/clouds")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, clouds_json(db));
      });
    });

    get("/api/v1/clouds/<string>")(
        [this](const crow::request &, std::string name) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, cloud_json(db, name));
          });
        });

    get("/api/v1/clouds/<string>/points")(
        [this](const crow::request &req, std::string name) {
          const Params params = params_of(req);
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, cloud_points_json(db, name, params));
          });
        });

    // ---- meshes ----
    get("/api/v1/meshes")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, meshes_json(db));
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
    get("/api/v1/frames")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, frames_json(db));
      });
    });

    get("/api/v1/frames/<int>")([this](const crow::request &, int id) {
      return with_db([&](const reusex::ProjectDB &db) {
        return json_response(200, frame_json(db, id));
      });
    });

    get("/api/v1/frames/<int>/image")([this](const crow::request &req, int id) {
      const std::string kind = params_of(req).str("kind", "color");
      return with_db([&](const reusex::ProjectDB &db) {
        return blob_response(frame_image_blob(db, id, kind));
      });
    });

    // ---- panoramas ----
    get("/api/v1/panoramas")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, panoramas_json(db));
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

    get("/api/v1/materials")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, materials_json(db));
      });
    });

    get("/api/v1/materials/<string>")(
        [this](const crow::request &, std::string guid) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, material_json(db, guid));
          });
        });

    get("/api/v1/instances/<string>")(
        [this](const crow::request &, std::string cloud) {
          return with_db([&](const reusex::ProjectDB &db) {
            return json_response(200, instances_json(db, cloud));
          });
        });

    // ---- pipeline ----
    get("/api/v1/stages")([this](const crow::request &) {
      return with_db([](const reusex::ProjectDB &db) {
        return json_response(200, stages_json(db));
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
            if (req.method == crow::HTTPMethod::GET)
              return json_response(200, jobs_json(runner_->jobs()));

            const auto submission = parse_job_request(req.body);
            const auto id =
                runner_->submit(submission.stage, submission.parameters);
            auto record = runner_->job(id);
            if (!record)
              throw HttpError(500, "job vanished immediately after submit");
            return json_response(202, job_json(*record));
          });
        });

    get("/api/v1/jobs/<string>")([this](const crow::request &, std::string id) {
      return guarded([&] {
        auto record = runner_->job(id);
        if (!record)
          throw HttpError(404, "no such job '" + id + "'");
        return json_response(200, job_json(*record));
      });
    });

    app_.route_dynamic("/api/v1/jobs/<string>/cancel")
        .methods(crow::HTTPMethod::POST)(
            [this](const crow::request &, std::string id) {
              return guarded([&] {
                if (!runner_->cancel(id))
                  throw HttpError(404, "no such job '" + id + "'");
                auto record = runner_->job(id);
                if (!record)
                  throw HttpError(404, "no such job '" + id + "'");
                return json_response(200, job_json(*record));
              });
            });

    register_websocket();
    register_static();
  }

  void register_websocket() {
    CROW_WEBSOCKET_ROUTE(app_, "/api/v1/events")
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

  void register_static() {
    CROW_CATCHALL_ROUTE(app_)
    ([this](const crow::request &req, crow::response &res) {
      // Anything under the API prefix that reached the catchall is a genuine
      // 404 — do not shadow it with the SPA fallback, or a client typo would
      // silently return HTML where JSON was expected.
      if (req.url.rfind(std::string(kApiPrefix), 0) == 0) {
        res = error_response(404, "no route matches " + req.url);
        res.end();
        return;
      }

      if (has_assets()) {
        if (auto file = resolve_asset(options_.asset_dir, req.url);
            !file.empty()) {
          std::ifstream stream(file, std::ios::binary);
          std::ostringstream buffer;
          buffer << stream.rdbuf();
          res = crow::response(200, buffer.str());
          res.set_header("Content-Type", mime_type_for(file));
          res.end();
          return;
        }
        // SPA fallback: an unknown non-asset path is a client-side route, so
        // hand back index.html and let the router deal with it.
        if (auto index = resolve_asset(options_.asset_dir, "/index.html");
            !index.empty()) {
          std::ifstream stream(index, std::ios::binary);
          std::ostringstream buffer;
          buffer << stream.rdbuf();
          res = crow::response(200, buffer.str());
          res.set_header("Content-Type", "text/html; charset=utf-8");
          res.end();
          return;
        }
      }

      res = crow::response(
          200, placeholder_page(options_.project.filename().string()));
      res.set_header("Content-Type", "text/html; charset=utf-8");
      res.end();
    });
  }

  ServerOptions options_;
  crow::SimpleApp app_;
  std::unique_ptr<pipeline::JobRunner> runner_;
  size_t listener_ = 0;

  std::mutex clients_mutex_;
  /// Connection -> its job filter (nullopt = receives every event).
  std::unordered_map<crow::websocket::connection *, std::optional<std::string>>
      clients_;
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

} // namespace rux::gui
