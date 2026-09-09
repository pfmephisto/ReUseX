// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/binary_points.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace rux::gui {
namespace {

/// One RUXP field, plus how to lift it out of a stored record.
struct FieldSpec {
  const char *name;
  RuxpType type;
  uint8_t components;
  /// Byte offset of this field's first byte within a stored record.
  uint32_t record_offset;
  /// True when the field's bytes must be reordered per point rather than
  /// copied straight out of the record (see the RGB swizzle below).
  bool swizzle_bgr = false;
};

size_t size_of(RuxpType type) {
  switch (type) {
  case RuxpType::f32:
  case RuxpType::u32:
    return 4;
  case RuxpType::u8:
    return 1;
  }
  return 0;
}

/// The field table from docs/gui/binary-points.md, keyed by cloud type.
///
/// Storage record layouts are the ones ProjectDB writes (see the serialize*
/// helpers in libs/reusex/src/core/ProjectDB.cpp):
///   PointXYZRGB — x,y,z as f32 then the packed `rgba` word   (16 B)
///   PointXYZ    — x,y,z as f32                               (12 B)
///   Normal      — nx,ny,nz as f32 then curvature             (16 B)
///   Label       — one u32                                    ( 4 B)
std::vector<FieldSpec> fields_for(std::string_view point_type) {
  if (point_type == "PointXYZRGB")
    return {{"xyz", RuxpType::f32, 3, 0},
            // The stored word at byte 12 is pcl::PointXYZRGB::rgba, and PCL
            // declares that union member over `struct { uint8_t b, g, r, a; }`
            // (PCL_ADD_UNION_RGB in pcl/impl/point_types.hpp). ProjectDB
            // memcpy's the uint32 verbatim, so on a little-endian host the
            // stored bytes are b,g,r,a in that order: b at record offset 12,
            // g at 13, r at 14. RUXP emits r,g,b, so the section starts at 14
            // and walks backwards — hence the swizzle rather than a copy.
            {"rgb", RuxpType::u8, 3, 12, /*swizzle_bgr=*/true}};
  if (point_type == "PointXYZ")
    return {{"xyz", RuxpType::f32, 3, 0}};
  if (point_type == "Normal")
    // Only the first three floats: curvature is stored but deliberately not
    // exposed by either wire format.
    return {{"normal", RuxpType::f32, 3, 0}};
  if (point_type == "Label")
    return {{"label", RuxpType::u32, 1, 0}};
  return {};
}

// Explicit little-endian writes. The header must be byte-identical on any
// host, so nothing here may depend on the host's byte order.

void put_u8(std::vector<uint8_t> &out, size_t at, uint8_t value) {
  out[at] = value;
}

void put_u16(std::vector<uint8_t> &out, size_t at, uint16_t value) {
  out[at] = static_cast<uint8_t>(value & 0xFF);
  out[at + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

void put_u32(std::vector<uint8_t> &out, size_t at, uint32_t value) {
  for (size_t i = 0; i < 4; ++i)
    out[at + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
}

void put_u64(std::vector<uint8_t> &out, size_t at, uint64_t value) {
  for (size_t i = 0; i < 8; ++i)
    out[at + i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);
}

} // namespace

bool ruxp_supports(std::string_view point_type) {
  return !fields_for(point_type).empty();
}

std::vector<uint8_t> encode_ruxp(const reusex::ProjectDB::CloudPage &page) {
  const auto fields = fields_for(page.point_type);
  if (fields.empty())
    throw std::runtime_error("RUXP has no field layout for point type '" +
                             page.point_type + "'");

  const size_t count = static_cast<size_t>(page.count);
  const size_t step = page.point_step;
  if (page.data.size() < count * step)
    throw std::runtime_error(
        "RUXP page is short: " + std::to_string(page.data.size()) +
        " bytes for " + std::to_string(count) + " records of " +
        std::to_string(step));

  const size_t header_size = kRuxpHeaderBase + kRuxpFieldSize * fields.size();
  // header_size is a u16 on the wire, so field_count can never exceed 4093.
  // Nothing today comes near it (the largest table is two fields), but a
  // silent truncation here would produce a page that parses and is wrong.
  if (header_size > std::numeric_limits<uint16_t>::max())
    throw std::runtime_error("RUXP header does not fit a u16: " +
                             std::to_string(fields.size()) + " fields");

  size_t body = header_size;
  std::vector<size_t> section_offset;
  section_offset.reserve(fields.size());
  for (const auto &field : fields) {
    section_offset.push_back(body);
    body += count * field.components * size_of(field.type);
  }

  std::vector<uint8_t> out(body, 0);

  // ---- header ----
  static constexpr std::array<uint8_t, 4> kMagic = {'R', 'U', 'X', 'P'};
  std::memcpy(out.data(), kMagic.data(), kMagic.size());
  put_u16(out, 4, kRuxpVersion);
  put_u16(out, 6, static_cast<uint16_t>(header_size));
  put_u32(out, 8, 0); // flags
  put_u32(out, 12, static_cast<uint32_t>(fields.size()));
  put_u32(out, 16, static_cast<uint32_t>(count));
  put_u32(out, 20, 0); // reserved
  put_u64(out, 24, page.offset);
  put_u64(out, 32, page.total);

  // ---- field descriptors ----
  for (size_t f = 0; f < fields.size(); ++f) {
    const auto &field = fields[f];
    const size_t at = kRuxpHeaderBase + kRuxpFieldSize * f;
    // char[8], NUL-padded; never NUL-terminated when the name is 8 chars.
    const std::string_view name(field.name);
    std::memcpy(out.data() + at, name.data(), std::min<size_t>(name.size(), 8));
    put_u8(out, at + 8, static_cast<uint8_t>(field.type));
    put_u8(out, at + 9, field.components);
    put_u16(out, at + 10, 0); // reserved
    // Absolute from the start of the buffer, so a client needs no arithmetic.
    put_u32(out, at + 12, static_cast<uint32_t>(section_offset[f]));
  }

  // ---- planar payload ----
  for (size_t f = 0; f < fields.size(); ++f) {
    const auto &field = fields[f];
    const size_t width = field.components * size_of(field.type);
    uint8_t *dst = out.data() + section_offset[f];
    const uint8_t *src = page.data.data() + field.record_offset;

    if (field.swizzle_bgr) {
      for (size_t i = 0; i < count; ++i, dst += width, src += step) {
        dst[0] = src[2]; // r
        dst[1] = src[1]; // g
        dst[2] = src[0]; // b
      }
    } else {
      for (size_t i = 0; i < count; ++i, dst += width, src += step)
        std::memcpy(dst, src, width);
    }
  }

  return out;
}

} // namespace rux::gui
