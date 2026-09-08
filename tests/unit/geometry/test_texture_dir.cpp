// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Texture staging directory contract (#245).
//
// The library must never write to — or delete from — the process working
// directory. prepare_texture_dir() is the single place all three texture_mesh
// entry points go through, so pinning its behaviour here pins theirs.

#include <reusex/reconstruction/texture_mesh.hpp>

#include "../../support/temp_path.hpp"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <system_error>
#include <vector>

namespace fs = std::filesystem;
using reusex::geometry::prepare_texture_dir;
using reusex::test_support::TempDir;

namespace {

/// Removes a directory the *test* asked prepare_texture_dir() to create.
struct ScopedDir {
  fs::path path;
  ~ScopedDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

/// True when `child` is `parent` itself or lives underneath it.
bool is_within(const fs::path &child, const fs::path &parent) {
  const auto c = fs::weakly_canonical(child);
  const auto p = fs::weakly_canonical(parent);
  auto ci = c.begin();
  auto pi = p.begin();
  for (; pi != p.end(); ++pi, ++ci) {
    if (ci == c.end() || *ci != *pi)
      return false;
  }
  return true;
}

} // namespace

TEST_CASE("prepare_texture_dir stages outside the working directory",
          "[geometry][texture][cwd]") {
  ScopedDir dir{prepare_texture_dir()};

  REQUIRE(fs::exists(dir.path));
  REQUIRE(fs::is_directory(dir.path));
  CHECK(dir.path.is_absolute());
  CHECK(fs::is_empty(dir.path));

  // The whole point of #245: nothing lands in, or next to, the CWD.
  CHECK_FALSE(is_within(dir.path, fs::current_path()));
  CHECK(is_within(dir.path, fs::temp_directory_path()));
}

TEST_CASE("prepare_texture_dir yields a distinct directory every call",
          "[geometry][texture][cwd]") {
  // Two concurrent `rux create texture` runs must not share staging space.
  constexpr int kRuns = 8;
  std::set<std::string> seen;
  std::vector<ScopedDir> dirs;
  dirs.reserve(kRuns);

  for (int i = 0; i < kRuns; ++i) {
    dirs.push_back(ScopedDir{prepare_texture_dir()});
    CHECK(seen.insert(dirs.back().path.string()).second);
  }

  CHECK(seen.size() == static_cast<size_t>(kRuns));
}

TEST_CASE("prepare_texture_dir never deletes a caller-supplied directory",
          "[geometry][texture][cwd]") {
  TempDir owned("test_texture_dir_owned");

  const fs::path keeper = owned.path / "do-not-delete.txt";
  {
    std::ofstream ofs(keeper);
    ofs << "precious";
  }
  REQUIRE(fs::exists(keeper));

  const fs::path returned = prepare_texture_dir(owned.path);

  CHECK(returned == owned.path);
  // The old code did remove_all() on the staging path — a user with a
  // mesh_textures/ directory in their CWD lost it. That must never happen to
  // a path the caller named.
  CHECK(fs::exists(keeper));
}

TEST_CASE("prepare_texture_dir creates a missing caller-supplied directory",
          "[geometry][texture][cwd]") {
  TempDir parent("test_texture_dir_missing");
  const fs::path nested = parent.path / "a" / "b" / "textures";
  REQUIRE_FALSE(fs::exists(nested));

  const fs::path returned = prepare_texture_dir(nested);

  CHECK(returned == nested);
  CHECK(fs::is_directory(returned));
}

TEST_CASE("prepare_texture_dir returns an absolute path for a relative request",
          "[geometry][texture][cwd]") {
  // Texture paths end up verbatim in pcl::TexMaterial::tex_file and from
  // there in the MTL, so they must not be resolved against the CWD later.
  TempDir parent("test_texture_dir_relative");
  const fs::path relative =
      fs::relative(parent.path / "textures", fs::current_path());
  REQUIRE_FALSE(relative.is_absolute());

  const fs::path returned = prepare_texture_dir(relative);

  CHECK(returned.is_absolute());
  CHECK(fs::is_directory(returned));
  CHECK(is_within(returned, parent.path));
}
