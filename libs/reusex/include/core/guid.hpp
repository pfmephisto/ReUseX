// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

namespace reusex::core {

/// Generate a fresh UUID-v4-like identifier (36-char, hyphenated hex).
///
/// Used for stable entity identity (material passports, building components,
/// instances). Non-deterministic by design: each call yields a new value from
/// a random_device-seeded generator. Do not use where reproducibility across
/// runs is required.
std::string generate_guid();

} // namespace reusex::core
