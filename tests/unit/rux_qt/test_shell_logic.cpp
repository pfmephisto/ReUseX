// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Unit tests for the Qt client's app-shell logic (Stream Q, phase Q1): the
// command palette's fuzzy ranking, the recent-projects list, what plain `rux`
// launches, and how a failed project open is explained. All of it lives in
// `rux_qt_core` (no Qt), so it runs in the light test binary.

#include <catch2/catch_test_macros.hpp>

#include <rux_qt/fuzzy.hpp>
#include <rux_qt/launch.hpp>
#include <rux_qt/recent.hpp>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

using namespace rux::qt;

namespace {

std::vector<std::string> titles_of(const std::vector<PaletteMatch> &m,
                                   const std::vector<PaletteCandidate> &c) {
  std::vector<std::string> out;
  for (const auto &x : m)
    out.push_back(c[x.index].title);
  return out;
}

const std::vector<PaletteCandidate> kCommands = {
    {"Start", "forside velkomst"},
    {"Database", "tabeller billeder"},
    {"3D", "punktsky viewer"},
    {"Posegraf", "pose graph"},
    {"Pipeline", "kør trin"},
    {"Log", "kørselslog"},
    {"Åbn projekt…", "open fil"},
    {"Luk projekt", "close"},
    {"Vis eller skjul inspektør", "panel"},
    {"Skift til lyst tema", "theme"},
    {"Ændr billede", ""},
};

} // namespace

// ------------------------------------------------------------------ fuzzy --

TEST_CASE("Palette_EmptyQuery_ReturnsEveryCandidateInOrder",
          "[rux_qt][palette]") {
  for (const char *q : {"", "   "}) {
    const auto m = rank_palette(q, kCommands);
    REQUIRE(m.size() == kCommands.size());
    for (std::size_t i = 0; i < m.size(); ++i) {
      CHECK(m[i].index == i);
      CHECK(m[i].score == 0);
      CHECK(m[i].positions.empty());
    }
  }
}

TEST_CASE("Palette_NonSubsequence_IsLeftOut", "[rux_qt][palette]") {
  CHECK(fuzzy_score("xyz", "Database") == -1);
  CHECK(fuzzy_score("esab", "Database") == -1); // order matters
  const auto m = rank_palette("qqq", kCommands);
  CHECK(m.empty());
}

TEST_CASE("Palette_IsCaseInsensitive_AndIgnoresQuerySpaces",
          "[rux_qt][palette]") {
  CHECK(fuzzy_score("DATA", "Database") > 0);
  CHECK(fuzzy_score("luk pro", "Luk projekt") > 0);
  CHECK(fuzzy_score("lukpro", "Luk projekt") ==
        fuzzy_score("luk pro", "Luk projekt"));
}

TEST_CASE("Palette_PrefixAndWordStarts_BeatScatteredMatches",
          "[rux_qt][palette]") {
  // "db" hits D-ata-B-ase? No: word-start D plus a later b. "Ændr billede"
  // has a word-start b too, but no word-start d. Database must still win
  // because its D is the very first character.
  const auto m = rank_palette("db", kCommands);
  REQUIRE_FALSE(m.empty());
  CHECK(kCommands[m.front().index].title == "Database");

  // A prefix beats the same letters later in a title.
  CHECK(fuzzy_score("log", "Log") > fuzzy_score("log", "Vis log for trin"));
  // Consecutive beats scattered.
  CHECK(fuzzy_score("pose", "Posegraf") >
        fuzzy_score("pose", "Pipeline og scener"));
  // Word starts: "vsi" = Vis Skjul Inspektør beats an in-word scatter.
  CHECK(fuzzy_score("vsi", "Vis eller skjul inspektør") >
        fuzzy_score("vsi", "avsi"));
}

TEST_CASE("Palette_Positions_PointAtTheMatchedCodePoints",
          "[rux_qt][palette]") {
  std::vector<int> pos;
  REQUIRE(fuzzy_score("pg", "Posegraf", &pos) > 0);
  CHECK(pos == std::vector<int>{0, 4});

  // Multi-byte letters count as one position each (QString indices).
  pos.clear();
  REQUIRE(fuzzy_score("ab", "Åbn projekt…", &pos) > 0);
  CHECK(pos == std::vector<int>{0, 1});
}

TEST_CASE("Palette_DanishLetters_FoldToTheirBaseLetter", "[rux_qt][palette]") {
  CHECK(fuzzy_score("korsel", "Kørselslog") > 0);
  CHECK(fuzzy_score("abn", "Åbn projekt…") > 0);
  CHECK(fuzzy_score("aendr", "Ændr billede") ==
        -1); // æ is one letter, not "ae"
  CHECK(fuzzy_score("andr", "Ændr billede") > 0);
  // The folded letters themselves, in either case, match too.
  CHECK(fuzzy_score("ø", "Kørselslog") > 0);
  CHECK(fuzzy_score("Å", "åbn") > 0);
  // An exact accented hit outranks the folded one.
  CHECK(fuzzy_score("ø", "kø") > fuzzy_score("o", "kø"));
}

TEST_CASE("Palette_KeywordOnlyMatch_RanksBelowTitleMatch_WithoutPositions",
          "[rux_qt][palette]") {
  // "theme" only appears in the keywords of "Skift til lyst tema".
  auto m = rank_palette("theme", kCommands);
  REQUIRE(m.size() == 1);
  CHECK(kCommands[m[0].index].title == "Skift til lyst tema");
  CHECK(m[0].positions.empty());

  // "log": a title match (Log) outranks a keyword match (kørselslog).
  m = rank_palette("log", kCommands);
  REQUIRE(m.size() >= 1);
  CHECK(kCommands[m[0].index].title == "Log");
  CHECK_FALSE(m[0].positions.empty());
}

TEST_CASE("Palette_Ties_AreBrokenByLengthThenOrder", "[rux_qt][palette]") {
  const std::vector<PaletteCandidate> c = {
      {"Projekt B lang", ""}, {"Projekt A", ""}, {"Projekt C", ""}};
  const auto t = titles_of(rank_palette("projekt", c), c);
  CHECK(t ==
        std::vector<std::string>{"Projekt A", "Projekt C", "Projekt B lang"});
}

TEST_CASE("Palette_LongTitles_DoNotBlowUp", "[rux_qt][palette]") {
  std::string long_path(4000, 'a');
  long_path += "/project.rux";
  // Matches past any internal cap still need not crash; a match in the
  // first part is found.
  CHECK(fuzzy_score("aaa", long_path) > 0);
  const std::vector<PaletteCandidate> c = {{"x", long_path}};
  CHECK_NOTHROW(rank_palette("project", c));
}

// ----------------------------------------------------------------- recent --

TEST_CASE("Recent_Normalise_IsLexicalAndAbsolute", "[rux_qt][recent]") {
  CHECK(normalise_project_path("a/./b/../c.rux", "/home/u") ==
        "/home/u/a/c.rux");
  CHECK(normalise_project_path("/x//y/z.rux", "/home/u") == "/x/y/z.rux");
  CHECK(normalise_project_path("", "/home/u").empty());
}

TEST_CASE("Recent_Push_MovesToFront_DeDuplicates_AndCaps", "[rux_qt][recent]") {
  std::vector<std::string> l;
  for (int i = 0; i < 12; ++i)
    l = recent_push(l, "/p/" + std::to_string(i) + ".rux");
  REQUIRE(l.size() == kMaxRecent);
  CHECK(l.front() == "/p/11.rux");
  CHECK(l.back() == "/p/2.rux");

  l = recent_push(l, "/p/5.rux");
  CHECK(l.size() == kMaxRecent);
  CHECK(l.front() == "/p/5.rux");
  CHECK(std::count(l.begin(), l.end(), "/p/5.rux") == 1);

  CHECK(recent_push({}, "").empty());
  CHECK(recent_push({"/a", "", "/b"}, "/c") ==
        std::vector<std::string>{"/c", "/a", "/b"});
  CHECK(recent_push({"/a", "/b", "/c"}, "/d", 2) ==
        std::vector<std::string>{"/d", "/a"});
}

TEST_CASE("Recent_Remove_DropsEveryCopy", "[rux_qt][recent]") {
  CHECK(recent_remove({"/a", "/b", "/a"}, "/a") ==
        std::vector<std::string>{"/b"});
  CHECK(recent_remove({"/a"}, "/zzz") == std::vector<std::string>{"/a"});
}

TEST_CASE("Recent_Sanitise_CleansAStoredList", "[rux_qt][recent]") {
  std::vector<std::string> stored = {"/a/b.rux", "", "/a/./b.rux", "rel.rux"};
  for (int i = 0; i < 20; ++i)
    stored.push_back("/many/" + std::to_string(i));
  const auto l = recent_sanitise(stored, "/cwd");
  REQUIRE(l.size() == kMaxRecent);
  CHECK(l[0] == "/a/b.rux");
  CHECK(l[1] == "/cwd/rel.rux");
  CHECK(l[2] == "/many/0");
}

TEST_CASE("Recent_Annotate_MarksMissingFiles", "[rux_qt][recent]") {
  const std::set<std::string> on_disk = {"/here.rux"};
  const auto e =
      recent_annotate({"/here.rux", "/gone.rux"}, [&](const std::string &p) {
        return on_disk.count(p) > 0;
      });
  REQUIRE(e.size() == 2);
  CHECK_FALSE(e[0].missing);
  CHECK(e[1].missing);
  CHECK(e[1].path == "/gone.rux");
}

// ----------------------------------------------------------------- launch --

TEST_CASE("Launch_Subcommand_AlwaysRunsTheCli", "[rux_qt][launch]") {
  LaunchInputs in;
  in.has_subcommand = true;
  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::run_subcommand);
  in.display = "";
  CHECK(decide_launch(in) == LaunchAction::run_subcommand);
}

TEST_CASE("Launch_NoSubcommand_OpensTheAppOnlyWithADisplay",
          "[rux_qt][launch]") {
  LaunchInputs in;
  CHECK(decide_launch(in) == LaunchAction::print_help);

  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  in.display = "";
  in.wayland_display = "wayland-1";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  // A desktop profile's platform list leaks into SSH sessions: not a display.
  in.wayland_display = "";
  in.qpa_platform = "wayland;xcb";
  CHECK(decide_launch(in) == LaunchAction::print_help);

  // Explicit offscreen (tests, screenshots) counts.
  in.qpa_platform = "offscreen";
  CHECK(decide_launch(in) == LaunchAction::open_app);

  // Built without the Qt client: help, never a crash.
  in.qt_client_built = false;
  in.display = ":0";
  CHECK(decide_launch(in) == LaunchAction::print_help);
}

// ------------------------------------------------------------ open errors --

TEST_CASE("OpenError_Classify_MapsSqliteAndProjectDbMessages",
          "[rux_qt][open]") {
  CHECK(classify_open_error("Cannot open database: database is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("Failed to create table: database is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("SQL exec failed: database table is locked") ==
        OpenErrorKind::locked);
  CHECK(classify_open_error("SQL exec failed: file is not a database") ==
        OpenErrorKind::not_a_project);
  CHECK(
      classify_open_error("Required table 'projects' not found in database") ==
      OpenErrorKind::not_a_project);
  CHECK(classify_open_error(
            "Cannot open database: unable to open database file") ==
        OpenErrorKind::permission);
  CHECK(classify_open_error("attempt to write a readonly database") ==
        OpenErrorKind::permission);
  CHECK(classify_open_error("database disk image is malformed") ==
        OpenErrorKind::corrupt);
  CHECK(classify_open_error("Migration to v12 failed: no such column") ==
        OpenErrorKind::other);
}

TEST_CASE("OpenError_EveryKind_HasDanishTitleAndHint", "[rux_qt][open]") {
  for (auto k : {OpenErrorKind::not_found, OpenErrorKind::not_a_file,
                 OpenErrorKind::permission, OpenErrorKind::locked,
                 OpenErrorKind::not_a_project, OpenErrorKind::corrupt,
                 OpenErrorKind::other}) {
    CHECK_FALSE(open_error_title_da(k).empty());
    CHECK_FALSE(open_error_hint_da(k).empty());
  }
  CHECK(open_error_title_da(OpenErrorKind::locked).find("låst") !=
        std::string::npos);
  CHECK(open_error_title_da(OpenErrorKind::none).empty());
}
