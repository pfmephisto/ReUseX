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

/// `flags` bit 0 — this page is a level-of-detail view of the whole cloud
/// rather than a contiguous window of it (#320).
///
/// When set, `count` points were drawn from all `total` of them, `offset` is 0
/// and carries no meaning, and the page must **not** be zipped positionally
/// against a page that does not come from the same selection. The layout is
/// unchanged, which is why this is a flag and not a version: a reader that
/// predates it refuses the page (v1's rule for an unknown flag), and it only
/// ever reaches a reader that asked for it with `max_points`.
inline constexpr uint32_t kRuxpFlagLod = 0x1U;

/// Every flag bit this encoder will emit. Anything else is a programming
/// error, not a wire condition.
inline constexpr uint32_t kRuxpKnownFlags = kRuxpFlagLod;

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
 * @param flags Bits to write into the header's `flags` word — today only
 *              #kRuxpFlagLod. A client rejects a page carrying a bit it does
 *              not know, so a bit must never be set speculatively.
 * @throws std::runtime_error when the page's `point_type` has no RUXP field
 *         mapping, when its `data` is shorter than `count * point_step`, or
 *         when @p flags contains a bit outside #kRuxpKnownFlags.
 */
std::vector<uint8_t> encode_ruxp(const reusex::ProjectDB::CloudPage &page,
                                 uint32_t flags = 0);

/// True when @p type is one encode_ruxp() knows how to lay out.
bool ruxp_supports(std::string_view point_type);

} // namespace rux::gui
