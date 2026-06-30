// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <handlers.hpp>

namespace ruxd {

void register_not_found_handler(crow::SimpleApp &app) {
  // Catchall for any route/method that no handler matched.
  CROW_CATCHALL_ROUTE(app)
  ([](const crow::request &req, crow::response &res) {
    res = json_response(crow::status::NOT_FOUND,
                        {{"status", "not_found"},
                         {"message", "no route matches this request"},
                         {"path", req.url}});
    res.end();
  });
}

} // namespace ruxd
