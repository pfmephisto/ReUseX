// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// RUXP v1 — the binary point transport served by
// `GET /api/v1/clouds/{name}/points?format=binary` (#283).
//
// The byte layout is normative in docs/gui/binary-points.md; this header is the
// C++ side of that contract and must not drift from it. The client side is
// apps/rux/frontend/src/viewport/binaryPoints.ts.
//
// Deliberately framework-free, like the rest of gui/api.hpp: a page of storage
// records in, a buffer out.

// ProjectDB is included rather than forward-declared: the page type this
// header takes is the nested ProjectDB::CloudPage, which needs the complete
// class. ProjectDB.hpp is itself forward-declaration-disciplined (STANDARDS
// §2) — it pulls in the PCL point types and nothing heavier.
#include <reusex/core/ProjectDB.hpp>

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace rux::gui {

/// Format version written into every page. Bump only when the layout changes;
/// a client that sees a version it does not know must refuse the page.
inline constexpr uint16_t kRuxpVersion = 1;

/// Fixed part of the header, before the field descriptors.
inline constexpr size_t kRuxpHeaderBase = 40;
/// Bytes per field descriptor.
inline constexpr size_t kRuxpFieldSize = 16;

/// Field value types, as written into a descriptor's `type` byte.
enum class RuxpType : uint8_t { f32 = 1, u8 = 2, u32 = 3 };

/**
 * @brief Encode one stored page as a RUXP v1 buffer.
 *
 * Produces the 40-byte header, `16 * field_count` bytes of descriptors, and
 * then one tightly packed little-endian section per field, in descriptor
 * order. Header integers are written byte-by-byte so the output is
 * byte-identical regardless of host endianness (the payload is not — see the
 * Endianness section of docs/gui/binary-points.md, which makes little-endian
 * a deliberate constraint of the format).
 *
 * A page with `count == 0` still yields a complete, valid header.
 *
 * @throws std::runtime_error when the page's `point_type` has no RUXP field
 *         mapping, or when its `data` is shorter than `count * point_step`.
 */
std::vector<uint8_t> encode_ruxp(const reusex::ProjectDB::CloudPage &page);

/// True when @p type is one encode_ruxp() knows how to lay out.
bool ruxp_supports(std::string_view point_type);

} // namespace rux::gui
