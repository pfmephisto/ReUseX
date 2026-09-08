// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui.hpp"
#include "gui/Server.hpp"

#include <spdlog/spdlog.h>

void setup_subcommand_gui(CLI::App &app,
                          std::shared_ptr<RuxOptions> global_opt) {
  auto opt = std::make_shared<SubcommandGuiOptions>();

  auto *sub = app.add_subcommand("gui", "Serve the web interface locally");

  sub->footer(R"(
DESCRIPTION:
  Starts a local web server for the project selected with -p/--project and
  opens it in a browser. The server implements the REST + WebSocket contract in
  docs/gui/openapi.yaml: project summary, clouds, meshes, sensor frames,
  panoramas, building components, material passports, instances, the pipeline
  log, and job submit/status/cancel with live progress.

  Pipeline stages submitted through the API run IN-PROCESS, one at a time, on a
  single worker thread — progress is reported through the same observer the CLI
  progress bar uses.

EXAMPLES:
  rux -p scan.rux gui                  # Serve and open a browser
  rux -p scan.rux gui --no-browser     # Headless (CI, remote shell)
  rux -p scan.rux gui --port 9000      # Custom port
  rux -p scan.rux gui --assets ./dist  # Serve a locally built frontend

NOTES:
  - Binds to 127.0.0.1 by default. There is NO authentication and the API can
    execute pipeline stages, so only change --bind if you know what that means.
  - Without a frontend bundle a built-in placeholder page is served that lists
    the live API. The real frontend is Phase 2 of issue #265.
  - Asset lookup order: --assets, then $RUX_GUI_ASSETS, then
    <install prefix>/share/reusex/gui.
  - The project is created if it does not exist, exactly like other
    project-writing subcommands.
)");

  sub->add_option("--port", opt->port, "TCP port to listen on")
      ->default_val(opt->port)
      ->check(CLI::Range(1, 65535));

  sub->add_option("--bind", opt->bind_address,
                  "Interface to bind (default: loopback only)")
      ->default_val(opt->bind_address);

  sub->add_option("--threads", opt->threads,
                  "HTTP worker threads (0 = hardware concurrency)")
      ->default_val(opt->threads);

  sub->add_option("--assets", opt->asset_dir,
                  "Directory holding the frontend bundle")
      ->check(CLI::ExistingDirectory);

  sub->add_flag("--no-browser", opt->no_browser,
                "Do not open a browser on startup");

  sub->callback([opt, global_opt]() {
    int exit_code = run_subcommand_gui(*opt, *global_opt);
    if (exit_code != RuxError::SUCCESS)
      throw CLI::RuntimeError(exit_code);
  });
}

int run_subcommand_gui(SubcommandGuiOptions const &opt,
                       const RuxOptions &global_opt) {
  try {
    rux::gui::ServerOptions server_options;
    server_options.project = global_opt.project_db;
    server_options.bind_address = opt.bind_address;
    server_options.port = opt.port;
    server_options.threads = opt.threads;
    server_options.asset_dir = opt.asset_dir;
    server_options.open_browser = !opt.no_browser;

    rux::gui::Server server(std::move(server_options));
    spdlog::info("Serving {} at {}", global_opt.project_db.string(),
                 server.url());
    return server.run();

  } catch (const std::exception &e) {
    spdlog::error("Could not start the GUI server: {}", e.what());
    return RuxError::GENERIC;
  }
}
