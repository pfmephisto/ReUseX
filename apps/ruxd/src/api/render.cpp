// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "api/ViewRenderer.hpp"
#include "api/api.hpp"

#include <reusex/core/survey.hpp>

#include <sstream>

namespace ruxd::api {

RenderRequest render_request_from(const Params &params) {
  RenderRequest r;
  r.view = params.str("view", r.view);
  if (r.view != "plan" && r.view != "top" && r.view != "front" &&
      r.view != "orbit")
    throw HttpError(400, "'view' must be plan, top, front or orbit, not '" +
                             r.view + "'");
  r.orbit_index = static_cast<int>(params.integer("orbit_index", 0));
  if (r.orbit_index < 0 || r.orbit_index > 7)
    throw HttpError(400, "'orbit_index' must be 0..7");
  if (const auto layers = params.find("layers"); layers && !layers->empty()) {
    r.layers.clear();
    std::stringstream ss(*layers);
    for (std::string item; std::getline(ss, item, ',');)
      if (!item.empty())
        r.layers.push_back(item);
  }
  if (params.find("highlight_instance")) {
    const auto id = params.integer("highlight_instance", 0);
    if (id <= 0)
      throw HttpError(400,
                      "'highlight_instance' must be a positive instance id");
    r.highlight_instance = static_cast<std::uint32_t>(id);
    r.highlight_cloud = params.str(
        "highlight_cloud", std::string(reusex::core::kDefaultInstanceCloud));
  }
  r.width = static_cast<int>(params.integer("width", r.width));
  r.height = static_cast<int>(params.integer("height", r.height));
  if (r.width < 64 || r.width > 2048 || r.height < 64 || r.height > 2048)
    throw HttpError(400, "'width' and 'height' must be 64..2048");
  return r;
}

Blob render_blob(const reusex::ProjectDB &db, IViewRenderer *renderer,
                 const Params &params) {
  const auto req = render_request_from(params);
  if (!renderer)
    throw HttpError(503, "this server has no view renderer (built without the "
                         "visualize module)");
  try {
    return Blob{"image/png", renderer->render_png(db, req)};
  } catch (const RenderUnavailable &e) {
    throw HttpError(503, e.what());
  } catch (const std::invalid_argument &e) {
    throw HttpError(400, e.what());
  } catch (const HttpError &) {
    throw;
  } catch (const std::runtime_error &e) {
    throw HttpError(422, e.what());
  }
}

} // namespace ruxd::api
