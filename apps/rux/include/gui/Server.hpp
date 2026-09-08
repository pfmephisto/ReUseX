// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The `rux gui` HTTP + WebSocket server (#265, Phase 1).
//
// Implements docs/gui/openapi.yaml over one `.rux` project, serves the frontend
// bundle as static files, and streams pipeline job progress over
// /api/v1/events.
//
// Crow is an implementation detail: this header is pimpl'd so crow.h never
// reaches the rest of the app or the tests. The handler logic itself lives in
// gui/api.hpp, which is framework-free.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace rux::gui {

/// Everything `rux gui` needs to stand a server up.
struct ServerOptions {
  /// The single project this server is bound to for its lifetime.
  std::filesystem::path project;

  /// Interface to bind. Defaults to loopback: this server has **no
  /// authentication** and executes pipeline stages, so exposing it on a
  /// routable interface is a deliberate act, not an accident.
  std::string bind_address = "127.0.0.1";

  /// TCP port. 0 is rejected — Crow gives no way to read back an
  /// ephemeral port, so a caller could not tell where to connect.
  uint16_t port = 8420;

  /// Crow worker threads. 0 means hardware concurrency.
  unsigned threads = 0;

  /// Frontend bundle directory. Empty selects the search order in
  /// gui/assets.hpp, falling back to the built-in placeholder page.
  std::filesystem::path asset_dir;

  /// Launch the system browser at the server URL once it is listening.
  bool open_browser = true;
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

  /// Serve until SIGINT/SIGTERM. Returns a process exit code.
  int run();

    private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rux::gui
