// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// The Qt-free logic of the Database workspace (Stream Q, Q2): the A/B frame
// pair and its keys, the pending pose-graph edits, the table paging, and the
// formatters the widgets show. All in rux_qt_core, so it runs in the light
// test binary.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <rux_qt/database_logic.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

using namespace rux::qt;
using Catch::Approx;

// ------------------------------------------------------------ frame pair --

TEST_CASE("FramePair_Reset_SortsIdsAndPicksTheFirstTwo",
          "[rux_qt][database][pair]") {
  FramePair p;
  CHECK(p.empty());
  CHECK(p.a_id() == -1);
  CHECK(p.b_id() == -1);
  p.reset({30, 10, 20, 10});
  REQUIRE(p.size() == 3);
  CHECK(p.ids() == std::vector<int>{10, 20, 30});
  CHECK(p.a_id() == 10);
  CHECK(p.b_id() == 20);

  p.reset({7});
  CHECK(p.a_id() == 7);
  CHECK(p.b_id() == 7);
  p.reset({});
  CHECK(p.a_index() == -1);
}

TEST_CASE("FramePair_Steps_AreClampedAndReportMovement",
          "[rux_qt][database][pair]") {
  FramePair p;
  p.reset({1, 2, 3, 4, 5});
  CHECK_FALSE(p.step_a(-1)); // already first
  CHECK(p.step_a(1));
  CHECK(p.a_id() == 2);
  CHECK(p.step_a(100));
  CHECK(p.a_id() == 5);
  CHECK_FALSE(p.step_a(1));
  CHECK(p.step_b(-100));
  CHECK(p.b_id() == 1);
  CHECK(p.a_id() == 5); // B never drags A along
}

TEST_CASE("FramePair_SetById_RejectsUnknownIds", "[rux_qt][database][pair]") {
  FramePair p;
  p.reset({100, 200, 300});
  CHECK(p.set_b_id(300));
  CHECK(p.b_index() == 2);
  CHECK_FALSE(p.set_a_id(150));
  CHECK(p.a_id() == 100);
  CHECK_FALSE(p.set_a_index(3));
  CHECK_FALSE(p.set_a_index(-1));
  CHECK(p.index_of(200) == 1);
  CHECK(p.index_of(201) == -1);
}

TEST_CASE("FramePair_NearestIndex_FindsTheClosestId",
          "[rux_qt][database][pair]") {
  FramePair p;
  CHECK(p.nearest_index(5) == -1);
  p.reset({10, 20, 40});
  CHECK(p.nearest_index(0) == 0);
  CHECK(p.nearest_index(14) == 0);
  CHECK(p.nearest_index(15) == 0); // tie -> lower id
  CHECK(p.nearest_index(16) == 1);
  CHECK(p.nearest_index(31) == 2);
  CHECK(p.nearest_index(999) == 2);
}

TEST_CASE("BrowserKey_ArrowsMoveAAndShiftArrowsMoveB",
          "[rux_qt][database][keys]") {
  CHECK(browser_key(ArrowKey::left, false, false) == BrowserKey::a_prev);
  CHECK(browser_key(ArrowKey::right, false, false) == BrowserKey::a_next);
  CHECK(browser_key(ArrowKey::left, true, false) == BrowserKey::b_prev);
  CHECK(browser_key(ArrowKey::right, true, false) == BrowserKey::b_next);
  CHECK(browser_key(ArrowKey::other, false, false) == BrowserKey::none);
  // A focused text field or spin box keeps its own cursor keys.
  CHECK(browser_key(ArrowKey::left, false, true) == BrowserKey::none);
  CHECK(browser_key(ArrowKey::right, true, true) == BrowserKey::none);
  CHECK(browser_step(false) == 1);
  CHECK(browser_step(true) == 10);
}

// --------------------------------------------------------- pending edits --

namespace {

EdgeRecord edge(int from, int to, const char *type = "loop_closure",
                double residual = 0.0, double weight = 1.0) {
  return {{from, to, type}, residual, weight};
}

} // namespace

TEST_CASE("PendingEdits_AddAndRemove_StageAndCancelOut",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry", 0.5, 4.0), edge(2, 3, "odometry")});
  CHECK(e.empty());

  CHECK(e.add(edge(1, 5)) == PendingEdgeEdits::Result::added);
  CHECK(e.count() == 1);
  // Same key twice: duplicate, still one edit.
  CHECK(e.add(edge(1, 5)) == PendingEdgeEdits::Result::duplicate);
  // An edge that is already stored is a duplicate too.
  CHECK(e.add(edge(1, 2, "odometry")) == PendingEdgeEdits::Result::duplicate);
  // A different type between the same frames is a different edge.
  CHECK(e.add(edge(1, 2, "loop_closure")) == PendingEdgeEdits::Result::added);
  CHECK(e.count() == 2);

  // Removing a staged addition just unstages it.
  CHECK(e.remove({1, 5, "loop_closure"}) == PendingEdgeEdits::Result::unstaged);
  CHECK(e.count() == 1);
  // Removing a stored edge stages its deletion.
  CHECK(e.remove({2, 3, "odometry"}) == PendingEdgeEdits::Result::removed);
  CHECK(e.count() == 2);
  CHECK(e.remove({2, 3, "odometry"}) == PendingEdgeEdits::Result::not_found);
  // Re-adding a deleted edge restores it instead of adding a copy.
  CHECK(e.add(edge(2, 3, "odometry")) == PendingEdgeEdits::Result::restored);
  CHECK(e.count() == 1);
  CHECK(e.remove({9, 8, "odometry"}) == PendingEdgeEdits::Result::not_found);

  e.discard();
  CHECK(e.empty());
  CHECK(e.base().size() == 2);
}

TEST_CASE("PendingEdits_RejectsInvalidEdges", "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  CHECK(e.add(edge(3, 3)) == PendingEdgeEdits::Result::invalid);
  CHECK(e.add(edge(1, 2, "bogus")) == PendingEdgeEdits::Result::invalid);
  CHECK(e.add(edge(1, 2, "loop_closure", 0.0, 0.0)) ==
        PendingEdgeEdits::Result::invalid);
  CHECK(e.add(edge(1, 2, "loop_closure", 0.0,
                   std::numeric_limits<double>::quiet_NaN())) ==
        PendingEdgeEdits::Result::invalid);
  CHECK(e.empty());
}

TEST_CASE("PendingEdits_Ops_DeleteFirstThenAddInStagingOrder",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base(
      {edge(1, 2, "odometry"), edge(1, 2, "odometry"), edge(4, 5, "panorama")});
  REQUIRE(e.add(edge(7, 8)) == PendingEdgeEdits::Result::added);
  REQUIRE(e.remove({1, 2, "odometry"}) == PendingEdgeEdits::Result::removed);
  REQUIRE(e.add(edge(2, 9)) == PendingEdgeEdits::Result::added);
  REQUIRE(e.remove({4, 5, "panorama"}) == PendingEdgeEdits::Result::removed);

  const auto ops = e.ops();
  REQUIRE(ops.size() == 4);
  using K = PendingEdgeEdits::Op::Kind;
  CHECK(ops[0].kind == K::remove);
  CHECK(ops[0].edge.key == EdgeKey{1, 2, "odometry"});
  CHECK(ops[1].kind == K::remove);
  CHECK(ops[1].edge.key == EdgeKey{4, 5, "panorama"});
  CHECK(ops[2].kind == K::add);
  CHECK(ops[2].edge.key == EdgeKey{7, 8, "loop_closure"});
  CHECK(ops[3].edge.key == EdgeKey{2, 9, "loop_closure"});
  // Both stored rows of a duplicated key count as one removal.
  CHECK(e.count() == 4);

  e.commit_succeeded();
  CHECK(e.empty());
  REQUIRE(e.base().size() == 2);
  CHECK(e.base()[0].key == EdgeKey{7, 8, "loop_closure"});
  CHECK(e.base()[1].key == EdgeKey{2, 9, "loop_closure"});
}

TEST_CASE("PendingEdits_Between_ShowsBothDirectionsWithState",
          "[rux_qt][database][pending]") {
  PendingEdgeEdits e;
  e.set_base({edge(1, 2, "odometry", 0.25, 9.0), edge(2, 1, "panorama"),
              edge(2, 3, "odometry")});
  REQUIRE(e.remove({2, 1, "panorama"}) == PendingEdgeEdits::Result::removed);
  REQUIRE(e.add(edge(1, 2, "loop_closure")) == PendingEdgeEdits::Result::added);

  const auto v = e.between(1, 2);
  REQUIRE(v.size() == 3);
  CHECK(v[0].edge.key.type == "odometry");
  CHECK(v[0].edge.residual == 0.25);
  CHECK_FALSE(v[0].reversed);
  CHECK_FALSE(v[0].pending_add);
  CHECK(v[1].edge.key.type == "panorama");
  CHECK(v[1].reversed);
  CHECK(v[1].pending_delete);
  CHECK(v[2].pending_add);
  // Seen from B, the stored 1->2 edge is reversed.
  CHECK(e.between(2, 1)[0].reversed);
  CHECK(e.between(1, 3).empty());

  CHECK(e.degree(1) == 2); // odometry + staged loop closure
  CHECK(e.degree(2) == 3);
  CHECK(e.degree(9) == 0);
}

TEST_CASE("EdgeTypes_HaveDanishNames", "[rux_qt][database]") {
  CHECK(is_edge_type("odometry"));
  CHECK(is_edge_type("loop_closure"));
  CHECK(is_edge_type("panorama"));
  CHECK_FALSE(is_edge_type("gps"));
  CHECK(edge_type_da("odometry") == "Odometri");
  CHECK(edge_type_da("loop_closure") == "Løkkelukning");
  CHECK(edge_type_da("panorama") == "Panorama");
  CHECK(edge_type_da("gps") == "gps");
}

TEST_CASE("WeightFromIcp_IsInverseVarianceWithAFloor", "[rux_qt][database]") {
  CHECK(weight_from_icp_fitness(0.1) == Approx(100.0));
  CHECK(weight_from_icp_fitness(0.02) == Approx(2500.0));
  CHECK(weight_from_icp_fitness(0.0) == Approx(10000.0));
  CHECK(weight_from_icp_fitness(-1.0) == Approx(10000.0));
  CHECK(weight_from_icp_fitness(std::nan("")) == Approx(10000.0));
}

// ---------------------------------------------------------------- paging --

TEST_CASE("Paging_FetchesPagesUntilTheEnd", "[rux_qt][database][paging]") {
  Paging p{450, 0, 200};
  CHECK(p.can_fetch_more());
  CHECK(p.next_count() == 200);
  p.loaded += p.next_count();
  CHECK(p.next_count() == 200);
  p.loaded += p.next_count();
  CHECK(p.next_count() == 50);
  p.loaded += p.next_count();
  CHECK_FALSE(p.can_fetch_more());
  CHECK(p.next_count() == 0);

  Paging empty{0, 0, 200};
  CHECK_FALSE(empty.can_fetch_more());
  CHECK(empty.next_count() == 0);
}

TEST_CASE("Paging_CountToReach_RoundsUpToWholePages",
          "[rux_qt][database][paging]") {
  Paging p{1000, 200, 200};
  CHECK(p.count_to_reach(150) == 0);   // already loaded
  CHECK(p.count_to_reach(200) == 200); // first row of the next page
  CHECK(p.count_to_reach(401) == 400);
  CHECK(p.count_to_reach(999) == 800);
  CHECK(p.count_to_reach(5000) == 800); // clamped to the table
  CHECK(p.count_to_reach(-3) == 0);
}

// ------------------------------------------------------------ formatting --

TEST_CASE("FormatBytes_IsDanishAndScaled", "[rux_qt][database][format]") {
  CHECK(format_bytes_da(0) == "0 B");
  CHECK(format_bytes_da(834) == "834 B");
  CHECK(format_bytes_da(33391) == "33,4 kB");
  CHECK(format_bytes_da(1234567) == "1,2 MB");
  CHECK(format_bytes_da(3400000000ULL) == "3,40 GB");
  CHECK(format_bytes_da(999999) == "1,0 MB");
}

TEST_CASE("SniffBlob_RecognisesCommonFormats", "[rux_qt][database][format]") {
  CHECK(sniff_blob(std::string("\x89PNG\r\n\x1a\n....", 12)) == "PNG");
  CHECK(sniff_blob(std::string("\xff\xd8\xff\xe0", 4)) == "JPEG");
  CHECK(sniff_blob("ply\nformat binary") == "PLY");
  CHECK(sniff_blob("%PDF-1.7") == "PDF");
  CHECK(sniff_blob(std::string("RIFF\x10\0\0\0WEBPVP8 ", 16)) == "WebP");
  CHECK(sniff_blob(std::string("\x1f\x8b\x08", 3)) == "GZIP");
  CHECK(sniff_blob("PK\x03\x04") == "ZIP");
  CHECK(sniff_blob("  {\"a\":1}") == "JSON");
  CHECK(sniff_blob(std::string("\0\1\2", 3)) == "");
  CHECK(sniff_blob("") == "");
}

TEST_CASE("DescribeBlob_CombinesMeaningFormatAndSize",
          "[rux_qt][database][format]") {
  const std::string jpeg("\xff\xd8\xff\xe0", 4);
  CHECK(describe_blob("sensor_frames", "color", jpeg, 176846) ==
        "Farvebillede · JPEG · 176,8 kB");
  CHECK(describe_blob("sensor_frames", "transform", std::string(16, '\0'),
                      128) == "Pose 4×4 · 128 B");
  CHECK(describe_blob("whatever", "payload", std::string(4, '\1'), 2048) ==
        "2,0 kB");
  CHECK(describe_blob("whatever", "image", "%PDF-1.4", 10) == "PDF · 10 B");
  CHECK(blob_column_meaning("segmentation_images", "label_image") ==
        "Mærkatbillede");
  CHECK(blob_column_meaning("nope", "color").empty());
}

TEST_CASE("SummarizePose_ReadsTranslationAndAngles",
          "[rux_qt][database][pose]") {
  const std::array<double, 16> identity{1, 0, 0, 0, 0, 1, 0, 0,
                                        0, 0, 1, 0, 0, 0, 0, 1};
  auto s = summarize_pose(identity);
  CHECK(s.t[0] == 0.0);
  CHECK(s.angle_deg == Approx(0.0).margin(1e-9));

  // 90 degrees about z, translated (1, 2, 3): row-major, translation in
  // the last column.
  const std::array<double, 16> yaw90{0, -1, 0, 1, 1, 0, 0, 2,
                                     0, 0,  1, 3, 0, 0, 0, 1};
  s = summarize_pose(yaw90);
  CHECK(s.t[0] == 1.0);
  CHECK(s.t[1] == 2.0);
  CHECK(s.t[2] == 3.0);
  CHECK(s.yaw_deg == Approx(90.0));
  CHECK(s.pitch_deg == Approx(0.0).margin(1e-9));
  CHECK(s.roll_deg == Approx(0.0).margin(1e-9));
  CHECK(s.angle_deg == Approx(90.0));
}

TEST_CASE("PoseDelta_DistanceAndRelativeAngle", "[rux_qt][database][pose]") {
  const std::array<double, 16> a{1, 0, 0, 0, 0, 1, 0, 0,
                                 0, 0, 1, 0, 0, 0, 0, 1};
  const std::array<double, 16> b{0, -1, 0, 3, 1, 0, 0, 4,
                                 0, 0,  1, 0, 0, 0, 0, 1};
  const auto d = pose_delta(a, b);
  CHECK(d.distance_m == Approx(5.0));
  CHECK(d.angle_deg == Approx(90.0));
  CHECK(pose_delta(b, b).angle_deg == Approx(0.0).margin(1e-6));
}

TEST_CASE("FormatDecimal_UsesADanishComma", "[rux_qt][database][format]") {
  CHECK(format_decimal_da(0.4213, 2) == "0,42");
  CHECK(format_decimal_da(-1.5, 1) == "-1,5");
  CHECK(format_decimal_da(12.0, 0) == "12");
  CHECK(format_decimal_da(-0.0001, 2) == "0,00");
}

// ----------------------------------------------------------- write errors --

TEST_CASE("WriteErrors_AreClassifiedWithDanishMessages",
          "[rux_qt][database][errors]") {
  CHECK(classify_write_error("SQL exec failed: database is locked") ==
        WriteErrorKind::locked);
  CHECK(classify_write_error("pose_graph_edges insert failed: database is "
                             "busy") == WriteErrorKind::locked);
  CHECK(classify_write_error("attempt to write a readonly database") ==
        WriteErrorKind::read_only);
  CHECK(classify_write_error("Database is opened read-only") ==
        WriteErrorKind::read_only);
  CHECK(classify_write_error("disk full") == WriteErrorKind::other);
  CHECK(write_error_da(WriteErrorKind::locked).find("låst") !=
        std::string::npos);
  CHECK(write_error_da(WriteErrorKind::read_only).find("skrivebeskyttet") !=
        std::string::npos);
  CHECK_FALSE(write_error_da(WriteErrorKind::other).empty());
}
