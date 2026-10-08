// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "local.hpp"

#include "icp.hpp"
#include "injected.hpp"

#include <api/local_mode.hpp>

#include <spdlog/spdlog.h>

#include <exception>
#include <utility>

namespace ruxd {

int run_local(LocalOptions options) {
  try {
    return serve_web(std::move(options));
  } catch (const std::exception &e) {
    spdlog::error("Could not start ruxd --local: {}", e.what());
    return 1;
  }
}

int serve_web(LocalOptions options) {
  {
    api::ServerOptions server_options = std::move(options.server);
    server_options.target = options.target;
    server_options.stage_executor = make_stage_executor();
    server_options.icp_refine_fn = make_icp_refine_fn();

    // The server keeps raw pointers to these, so they are declared first and
    // therefore destroyed after it.
    const auto segmenter = make_frame_segmenter();
    const auto panorama_segmenter = make_panorama_segmenter();
    const auto renderer = make_view_renderer();
    const auto model_provider = make_sam3_model_provider(
        options.models_dir, options.sam3_model_dir, options.sam3_manifest_url);

    api::Server server(std::move(server_options));
    server.set_segmenter(segmenter.get());
    server.set_panorama_segmenter(panorama_segmenter.get());
    server.set_view_renderer(renderer.get());
    server.set_model_provider(model_provider.get());

    spdlog::info("Serving {} case(s) at {}", server.cases().size(),
                 server.url());
    if (!server.has_assets())
      spdlog::warn("No frontend bundle found (--assets, $RUX_GUI_ASSETS, "
                   "<prefix>/share/reusex/gui); serving the placeholder page");
    return server.run();
  }
}

} // namespace ruxd
