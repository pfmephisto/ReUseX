// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <clients.hpp>

namespace ruxd {

Clients::Clients(const Config &cfg)
    : aws_guard(), postgres(cfg.pg_url), redis(cfg.redis_url), s3(cfg) {}

} // namespace ruxd
