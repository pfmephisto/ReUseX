// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "core/gaussian_splat.hpp"

#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace reusex::core {
namespace {

/// Split @p text into lines, dropping a trailing CR so CRLF headers parse.
std::vector<std::string_view> lines_of(std::string_view text) {
  std::vector<std::string_view> lines;
  size_t start = 0;
  while (start <= text.size()) {
    const auto end = text.find('\n', start);
    auto line = text.substr(start, end == std::string_view::npos
                                       ? std::string_view::npos
                                       : end - start);
    if (!line.empty() && line.back() == '\r')
      line.remove_suffix(1);
    lines.push_back(line);
    if (end == std::string_view::npos)
      break;
    start = end + 1;
  }
  return lines;
}

std::vector<std::string> tokens_of(std::string_view line) {
  std::vector<std::string> tokens;
  std::istringstream stream{std::string(line)};
  std::string token;
  while (stream >> token)
    tokens.push_back(std::move(token));
  return tokens;
}

/// The spherical-harmonic degree implied by @p rest_count `f_rest_*`
/// properties, or 0 when it matches no standard degree.
///
/// The reference layout stores 3 * ((degree + 1)^2 - 1) rest coefficients:
/// 0, 9, 24, 45 for degrees 0..3.
int sh_degree_from_rest_count(size_t rest_count) {
  for (int degree = 0; degree <= 8; ++degree) {
    const auto expected =
        static_cast<size_t>(3 * ((degree + 1) * (degree + 1) - 1));
    if (expected == rest_count)
      return degree;
  }
  return 0;
}

} // namespace

GaussianSplatHeader parse_gaussian_splat_ply_header(std::string_view head) {
  const auto lines = lines_of(head);
  if (lines.empty() || lines.front() != "ply")
    throw std::runtime_error("the Gaussian splat data is not a PLY file (it "
                             "does not start with 'ply')");

  GaussianSplatHeader header;
  bool saw_end = false;
  bool saw_format = false;
  bool in_vertex_element = false;
  size_t rest_count = 0;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_dc = false;
  bool has_opacity = false;
  bool has_scale = false;
  bool has_rotation = false;

  for (const auto &line : lines) {
    const auto tokens = tokens_of(line);
    if (tokens.empty())
      continue;

    if (tokens[0] == "end_header") {
      saw_end = true;
      break;
    }

    if (tokens[0] == "format") {
      saw_format = true;
      // ASCII is legal PLY and the renderer parses it, but the trainer never
      // writes it and a big-endian body would be silently misread, so only the
      // two forms that can actually be drawn are accepted.
      if (tokens.size() < 2 ||
          (tokens[1] != "binary_little_endian" && tokens[1] != "ascii"))
        throw std::runtime_error(
            "the Gaussian splat PLY uses an unsupported format; only "
            "binary_little_endian and ascii can be rendered");
      continue;
    }

    if (tokens[0] == "element") {
      in_vertex_element = tokens.size() >= 2 && tokens[1] == "vertex";
      if (in_vertex_element && tokens.size() >= 3)
        header.gaussian_count = std::strtoull(tokens[2].c_str(), nullptr, 10);
      continue;
    }

    if (tokens[0] == "property" && in_vertex_element && tokens.size() >= 3) {
      const std::string &name = tokens.back();
      if (name == "x")
        has_x = true;
      else if (name == "y")
        has_y = true;
      else if (name == "z")
        has_z = true;
      else if (name == "opacity")
        has_opacity = true;
      else if (name.rfind("f_dc_", 0) == 0)
        has_dc = true;
      else if (name.rfind("f_rest_", 0) == 0)
        ++rest_count;
      else if (name.rfind("scale_", 0) == 0)
        has_scale = true;
      else if (name.rfind("rot_", 0) == 0)
        has_rotation = true;
    }
  }

  if (!saw_end)
    throw std::runtime_error("the Gaussian splat PLY header is truncated or "
                             "larger than the header probe window");
  if (!saw_format)
    throw std::runtime_error("the Gaussian splat PLY has no 'format' line");
  if (!(has_x && has_y && has_z))
    throw std::runtime_error("the Gaussian splat PLY has no vertex positions");

  // THE interesting failure. `rux export ply` writes a valid point-cloud PLY
  // with x/y/z/red/green/blue and nothing else; storing one as a splat
  // produces an empty viewport and no clue why. Say what is missing instead.
  if (!(has_dc && has_opacity && has_scale && has_rotation))
    throw std::runtime_error(
        "the file is a PLY but not an INRIA 3D Gaussian Splatting file: "
        "it has no f_dc_*/opacity/scale_*/rot_* properties. A point "
        "cloud exported with `rux export ply` cannot be rendered as a "
        "splat — use the output of `rux create gsplat`");

  header.sh_degree = sh_degree_from_rest_count(rest_count);
  return header;
}

} // namespace reusex::core
