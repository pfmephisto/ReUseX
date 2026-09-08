// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <reusex/gsplat/GaussianCloud.hpp>

#include <reusex/core/logging.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace reusex::gsplat {

float inverse_sigmoid(float x) {
  // Keep the argument strictly inside (0,1) so an initial opacity of exactly
  // 0 or 1 yields a large finite logit instead of +/-inf, which would poison
  // the very first optimizer step.
  const float e = 1e-6f;
  const float c = std::clamp(x, e, 1.0f - e);
  return std::log(c / (1.0f - c));
}

void GaussianCloud::validate() const {
  const std::size_t n = means.size();
  auto check = [n](const char *name, std::size_t m) {
    if (m != n)
      throw std::runtime_error(fmt::format(
          "GaussianCloud: '{}' has {} entries but 'means' has {}", name, m, n));
  };
  check("scales", scales.size());
  check("quats", quats.size());
  check("opacities", opacities.size());
  check("sh_dc", sh_dc.size());
  if (!sh_rest.empty())
    check("sh_rest", sh_rest.size());
}

namespace {

/// Number of higher-order SH coefficients per colour channel for a degree.
int sh_rest_coeffs(int degree) {
  const int bands = (degree + 1) * (degree + 1);
  return bands - 1; // minus the DC band
}

} // namespace

GaussianCloud init_from_point_cloud(const CloudPtr &cloud,
                                    const GaussianInitOptions &opt) {
  if (!cloud)
    throw std::runtime_error(
        "gsplat: cannot seed Gaussians from a null point cloud");
  if (cloud->empty())
    throw std::runtime_error(
        "gsplat: cannot seed Gaussians from an empty point cloud");
  if (opt.knn < 1)
    throw std::runtime_error("gsplat: knn must be >= 1");
  if (opt.sh_degree < 0 || opt.sh_degree > 3)
    throw std::runtime_error(
        fmt::format("gsplat: sh_degree must be 0..3, got {}", opt.sh_degree));

  // ---- deterministic uniform stride ---------------------------------------
  // A stride (rather than a random sample) keeps the seed reproducible without
  // touching an RNG, and preserves the cloud's spatial ordering.
  const std::size_t total = cloud->size();
  std::size_t stride = 1;
  if (opt.max_points > 0 && total > opt.max_points)
    stride = (total + opt.max_points - 1) / opt.max_points;

  std::vector<int> picked;
  picked.reserve(total / stride + 1);
  for (std::size_t i = 0; i < total; i += stride)
    picked.push_back(static_cast<int>(i));

  const std::size_t n = picked.size();
  reusex::info("gsplat: seeding {} Gaussians from {} cloud points (stride {})",
               n, total, stride);

  // ---- neighbour spacing sets the initial scale ---------------------------
  // Build the KD-tree on the *selected* subset: after striding, the spacing
  // that matters is the spacing between seeds, not between original points.
  CloudPtr sub(new Cloud);
  sub->reserve(n);
  for (int idx : picked)
    sub->push_back((*cloud)[idx]);

  pcl::KdTreeFLANN<PointT> tree;
  tree.setInputCloud(sub);

  GaussianCloud g;
  g.sh_degree = opt.sh_degree;
  g.means.resize(n);
  g.scales.resize(n);
  g.quats.resize(n);
  g.opacities.assign(n, inverse_sigmoid(opt.initial_opacity));
  g.sh_dc.resize(n);

  const int rest = sh_rest_coeffs(opt.sh_degree);
  if (rest > 0)
    g.sh_rest.assign(
        n, std::vector<float>(static_cast<std::size_t>(rest) * 3, 0.0f));

  // k+1 because the query point is its own nearest neighbour.
  const int k = std::min<int>(opt.knn + 1, static_cast<int>(n));
  std::vector<int> nn_idx(k);
  std::vector<float> nn_sq(k);

  std::size_t degenerate = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const PointT &p = (*sub)[i];
    g.means[i] = {p.x, p.y, p.z};
    g.quats[i] = {1.0f, 0.0f, 0.0f, 0.0f}; // identity rotation (w, x, y, z)
    g.sh_dc[i] = {rgb_to_sh_dc(static_cast<float>(p.r) / 255.0f),
                  rgb_to_sh_dc(static_cast<float>(p.g) / 255.0f),
                  rgb_to_sh_dc(static_cast<float>(p.b) / 255.0f)};

    float spacing = opt.min_scale;
    if (k > 1 && tree.nearestKSearch(p, k, nn_idx, nn_sq) > 1) {
      // Mean distance over the true neighbours (skip self at index 0).
      double acc = 0.0;
      int used = 0;
      for (int j = 1; j < static_cast<int>(nn_sq.size()); ++j) {
        acc += std::sqrt(static_cast<double>(nn_sq[j]));
        ++used;
      }
      if (used > 0)
        spacing = static_cast<float>(acc / used);
    }
    if (!(spacing > opt.min_scale)) {
      spacing = opt.min_scale;
      ++degenerate;
    }
    spacing = std::clamp(spacing, opt.min_scale, opt.max_scale);

    const float log_s = std::log(spacing);
    g.scales[i] = {log_s, log_s, log_s};
  }

  // Fail loudly rather than silently training on degenerate seeds
  // (STANDARDS §5): coincident points produce zero-extent Gaussians.
  if (degenerate > 0)
    reusex::warn("gsplat: {} of {} seeds ({:.1f}%) had a degenerate neighbour "
                 "spacing and were clamped to min_scale={} m — the seed cloud "
                 "likely contains duplicate points",
                 degenerate, n, 100.0 * double(degenerate) / double(n),
                 opt.min_scale);

  g.validate();
  return g;
}

// ---------------------------------------------------------------------------
// .ply I/O — the reference 3DGS layout (binary_little_endian, float32).
// ---------------------------------------------------------------------------

void save_gaussian_ply(const GaussianCloud &gaussians,
                       const std::filesystem::path &path) {
  gaussians.validate();
  if (gaussians.empty())
    throw std::runtime_error("gsplat: refusing to write an empty .ply");

  const std::size_t n = gaussians.size();
  const int rest = gaussians.sh_rest.empty()
                       ? 0
                       : static_cast<int>(gaussians.sh_rest.front().size());

  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error(
        fmt::format("gsplat: cannot open '{}' for writing", path.string()));

  std::ostringstream hdr;
  hdr << "ply\nformat binary_little_endian 1.0\n";
  hdr << "element vertex " << n << "\n";
  hdr << "property float x\nproperty float y\nproperty float z\n";
  // Viewers expect the normal properties to be present even though 3DGS does
  // not use them; they are written as zeros.
  hdr << "property float nx\nproperty float ny\nproperty float nz\n";
  for (int i = 0; i < 3; ++i)
    hdr << "property float f_dc_" << i << "\n";
  for (int i = 0; i < rest; ++i)
    hdr << "property float f_rest_" << i << "\n";
  hdr << "property float opacity\n";
  for (int i = 0; i < 3; ++i)
    hdr << "property float scale_" << i << "\n";
  for (int i = 0; i < 4; ++i)
    hdr << "property float rot_" << i << "\n";
  hdr << "end_header\n";
  const std::string header = hdr.str();
  out.write(header.data(), static_cast<std::streamsize>(header.size()));

  std::vector<float> row;
  row.reserve(3 + 3 + 3 + rest + 1 + 3 + 4);
  for (std::size_t i = 0; i < n; ++i) {
    row.clear();
    row.insert(row.end(), gaussians.means[i].begin(), gaussians.means[i].end());
    row.insert(row.end(), {0.0f, 0.0f, 0.0f}); // normals
    row.insert(row.end(), gaussians.sh_dc[i].begin(), gaussians.sh_dc[i].end());
    if (rest > 0)
      row.insert(row.end(), gaussians.sh_rest[i].begin(),
                 gaussians.sh_rest[i].end());
    row.push_back(gaussians.opacities[i]);
    row.insert(row.end(), gaussians.scales[i].begin(),
               gaussians.scales[i].end());
    row.insert(row.end(), gaussians.quats[i].begin(), gaussians.quats[i].end());
    out.write(reinterpret_cast<const char *>(row.data()),
              static_cast<std::streamsize>(row.size() * sizeof(float)));
  }
  if (!out)
    throw std::runtime_error(
        fmt::format("gsplat: write failed for '{}'", path.string()));

  reusex::info("gsplat: wrote {} Gaussians (SH degree {}) to {}", n,
               gaussians.sh_degree, path.string());
}

GaussianCloud load_gaussian_ply(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error(
        fmt::format("gsplat: cannot open '{}' for reading", path.string()));

  std::string line;
  std::size_t n = 0;
  std::vector<std::string> props;
  bool have_format = false;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::istringstream ls(line);
    std::string tok;
    ls >> tok;
    if (tok == "format") {
      std::string fmt_name;
      ls >> fmt_name;
      if (fmt_name != "binary_little_endian")
        throw std::runtime_error(
            fmt::format("gsplat: '{}' is '{}', only binary_little_endian is "
                        "supported",
                        path.string(), fmt_name));
      have_format = true;
    } else if (tok == "element") {
      std::string what;
      ls >> what;
      if (what == "vertex")
        ls >> n;
    } else if (tok == "property") {
      std::string type, name;
      ls >> type >> name;
      if (type != "float" && type != "float32")
        throw std::runtime_error(fmt::format(
            "gsplat: '{}' has non-float property '{}'", path.string(), name));
      props.push_back(name);
    } else if (tok == "end_header") {
      break;
    }
  }
  if (!have_format || n == 0 || props.empty())
    throw std::runtime_error(
        fmt::format("gsplat: '{}' is not a readable 3DGS ply", path.string()));

  auto index_of = [&props, &path](const std::string &name) -> std::size_t {
    auto it = std::find(props.begin(), props.end(), name);
    if (it == props.end())
      throw std::runtime_error(
          fmt::format("gsplat: '{}' is missing required property '{}'",
                      path.string(), name));
    return static_cast<std::size_t>(std::distance(props.begin(), it));
  };

  const std::size_t stride = props.size();
  std::vector<float> buf(stride * n);
  in.read(reinterpret_cast<char *>(buf.data()),
          static_cast<std::streamsize>(buf.size() * sizeof(float)));
  if (in.gcount() != static_cast<std::streamsize>(buf.size() * sizeof(float)))
    throw std::runtime_error(
        fmt::format("gsplat: '{}' body is truncated ({} of {} bytes)",
                    path.string(), in.gcount(), buf.size() * sizeof(float)));

  const std::size_t ix = index_of("x"), iy = index_of("y"), iz = index_of("z");
  const std::size_t idc = index_of("f_dc_0");
  const std::size_t iop = index_of("opacity");
  const std::size_t isc = index_of("scale_0");
  const std::size_t irot = index_of("rot_0");

  int rest = 0;
  while (std::find(props.begin(), props.end(),
                   "f_rest_" + std::to_string(rest)) != props.end())
    ++rest;
  std::size_t irest = 0;
  if (rest > 0)
    irest = index_of("f_rest_0");

  GaussianCloud g;
  g.means.resize(n);
  g.scales.resize(n);
  g.quats.resize(n);
  g.opacities.resize(n);
  g.sh_dc.resize(n);
  if (rest > 0)
    g.sh_rest.assign(n, std::vector<float>(static_cast<std::size_t>(rest)));
  // (bands-1)*3 == rest  =>  bands == rest/3 + 1  =>  degree = sqrt(bands)-1
  const int bands = rest / 3 + 1;
  g.sh_degree = static_cast<int>(std::lround(std::sqrt(double(bands)))) - 1;

  for (std::size_t i = 0; i < n; ++i) {
    const float *r = buf.data() + i * stride;
    g.means[i] = {r[ix], r[iy], r[iz]};
    g.sh_dc[i] = {r[idc], r[idc + 1], r[idc + 2]};
    g.opacities[i] = r[iop];
    g.scales[i] = {r[isc], r[isc + 1], r[isc + 2]};
    g.quats[i] = {r[irot], r[irot + 1], r[irot + 2], r[irot + 3]};
    for (int j = 0; j < rest; ++j)
      g.sh_rest[i][static_cast<std::size_t>(j)] = r[irest + j];
  }

  g.validate();
  return g;
}

} // namespace reusex::gsplat
