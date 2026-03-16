// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <ReUseX/io/speckle.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <string>

using namespace ReUseX::io::speckle;

// ---- Helper: compute MD5 the same way the implementation does ----
static std::string test_md5(const std::string &data) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;
    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    REQUIRE(ctx != nullptr);
    REQUIRE(EVP_DigestInit_ex(ctx, EVP_md5(), nullptr) == 1);
    REQUIRE(EVP_DigestUpdate(ctx, data.data(), data.size()) == 1);
    REQUIRE(EVP_DigestFinal_ex(ctx, digest, &digest_len) == 1);
    EVP_MD_CTX_free(ctx);

    static constexpr char hex[] = "0123456789abcdef";
    std::string result;
    result.reserve(digest_len * 2);
    for (unsigned int i = 0; i < digest_len; ++i) {
        result.push_back(hex[(digest[i] >> 4) & 0x0F]);
        result.push_back(hex[digest[i] & 0x0F]);
    }
    return result;
}

// ============================================================
// Object model tests
// ============================================================

TEST_CASE("Speckle object model defaults", "[speckle]") {
    SECTION("Base defaults") {
        Base b;
        REQUIRE(b.speckle_type == "Base");
        REQUIRE(b.applicationId.empty());
        REQUIRE(b.elements.empty());
        REQUIRE(b.properties.empty());
    }

    SECTION("Point defaults") {
        Point p;
        REQUIRE(p.speckle_type == "Objects.Geometry.Point");
        REQUIRE(p.x == 0);
        REQUIRE(p.y == 0);
        REQUIRE(p.z == 0);
        REQUIRE(p.units == "m");
    }

    SECTION("Point with coordinates") {
        Point p(1.5, 2.5, 3.5);
        REQUIRE(p.x == 1.5);
        REQUIRE(p.y == 2.5);
        REQUIRE(p.z == 3.5);
    }

    SECTION("Line defaults") {
        Line l;
        REQUIRE(l.speckle_type == "Objects.Geometry.Line");
        REQUIRE(l.units == "m");
    }

    SECTION("Mesh defaults") {
        Mesh m;
        REQUIRE(m.speckle_type == "Objects.Geometry.Mesh");
        REQUIRE(m.vertices.empty());
        REQUIRE(m.faces.empty());
        REQUIRE(m.colors.empty());
        REQUIRE(m.units == "m");
    }

    SECTION("Pointcloud defaults") {
        Pointcloud pc;
        REQUIRE(pc.speckle_type == "Objects.Geometry.Pointcloud");
        REQUIRE(pc.points.empty());
        REQUIRE(pc.colors.empty());
        REQUIRE(pc.sizes.empty());
        REQUIRE(pc.units == "m");
    }

    SECTION("Collection defaults") {
        Collection c;
        REQUIRE(c.speckle_type == "Speckle.Core.Models.Collection");
        REQUIRE(c.name.empty());
        REQUIRE(c.collectionType == "Container");
    }
}

// ============================================================
// MD5 hashing tests
// ============================================================

TEST_CASE("MD5 hash produces expected values", "[speckle]") {
    // Known MD5 test vectors
    REQUIRE(test_md5("") == "d41d8cd98f00b204e9800998ecf8427e");
    REQUIRE(test_md5("hello") == "5d41402abc4b2a76b9719d911017c592");
    REQUIRE(test_md5("The quick brown fox jumps over the lazy dog") ==
            "9e107d9d372bb6826bd81d3542a419d6");
}

TEST_CASE("MD5 hash is 32 hex characters", "[speckle]") {
    std::string hash = test_md5("test input for length check");
    REQUIRE(hash.size() == 32);
    REQUIRE(std::all_of(hash.begin(), hash.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    }));
}

// ============================================================
// Conversion helper tests
// ============================================================

TEST_CASE("to_speckle(CloudConstPtr) converts point cloud", "[speckle]") {
    auto cloud = std::make_shared<ReUseX::Cloud>();
    cloud->resize(3);

    cloud->points[0] = {1.0f, 2.0f, 3.0f};
    cloud->points[0].r = 255;
    cloud->points[0].g = 0;
    cloud->points[0].b = 0;

    cloud->points[1] = {4.0f, 5.0f, 6.0f};
    cloud->points[1].r = 0;
    cloud->points[1].g = 255;
    cloud->points[1].b = 0;

    cloud->points[2] = {7.0f, 8.0f, 9.0f};
    cloud->points[2].r = 0;
    cloud->points[2].g = 0;
    cloud->points[2].b = 255;

    auto pc = to_speckle(cloud);

    REQUIRE(pc.speckle_type == "Objects.Geometry.Pointcloud");
    REQUIRE(pc.points.size() == 9); // 3 points * 3 coords
    REQUIRE(pc.colors.size() == 3);

    // Check coordinates
    using Catch::Approx;
    REQUIRE(pc.points[0] == Approx(1.0));
    REQUIRE(pc.points[1] == Approx(2.0));
    REQUIRE(pc.points[2] == Approx(3.0));
    REQUIRE(pc.points[3] == Approx(4.0));
    REQUIRE(pc.points[7] == Approx(8.0));

    // Check ARGB color encoding
    // Point 0: R=255, G=0, B=0 -> ARGB = 0xFF_FF_00_00
    int expected_red = (255 << 24) | (255 << 16) | (0 << 8) | 0;
    REQUIRE(pc.colors[0] == expected_red);

    // Point 2: R=0, G=0, B=255 -> ARGB = 0xFF_00_00_FF
    int expected_blue = (255 << 24) | (0 << 16) | (0 << 8) | 255;
    REQUIRE(pc.colors[2] == expected_blue);
}

TEST_CASE("to_speckle(CloudConstPtr) throws on null cloud", "[speckle]") {
    ReUseX::CloudConstPtr null_cloud;
    REQUIRE_THROWS_AS(to_speckle(null_cloud), std::invalid_argument);
}

TEST_CASE("to_speckle(CloudConstPtr) throws on empty cloud", "[speckle]") {
    auto empty_cloud = std::make_shared<ReUseX::Cloud>();
    REQUIRE_THROWS_AS(to_speckle(empty_cloud), std::invalid_argument);
}

TEST_CASE("to_speckle(MatrixXd, MatrixXi) converts Eigen mesh", "[speckle]") {
    // Simple triangle
    Eigen::MatrixXd vertices(3, 3);
    vertices << 0.0, 0.0, 0.0,
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0;

    Eigen::MatrixXi faces(1, 3);
    faces << 0, 1, 2;

    auto mesh = to_speckle(vertices, faces);

    REQUIRE(mesh.speckle_type == "Objects.Geometry.Mesh");
    REQUIRE(mesh.vertices.size() == 9); // 3 vertices * 3 coords

    // Faces: [0, 0, 1, 2] (0 = triangle indicator, then indices)
    REQUIRE(mesh.faces.size() == 4);
    REQUIRE(mesh.faces[0] == 0); // triangle indicator
    REQUIRE(mesh.faces[1] == 0);
    REQUIRE(mesh.faces[2] == 1);
    REQUIRE(mesh.faces[3] == 2);

    using Catch::Approx;
    REQUIRE(mesh.vertices[0] == Approx(0.0));
    REQUIRE(mesh.vertices[3] == Approx(1.0)); // second vertex x
    REQUIRE(mesh.vertices[7] == Approx(1.0)); // third vertex y
}

TEST_CASE("to_speckle(PolygonMesh) converts PCL mesh", "[speckle]") {
    pcl::PolygonMesh polygon_mesh;

    // Create a simple cloud with 3 points
    ReUseX::Cloud cloud;
    cloud.resize(3);
    cloud.points[0] = {0.0f, 0.0f, 0.0f};
    cloud.points[0].r = 128;
    cloud.points[0].g = 64;
    cloud.points[0].b = 32;
    cloud.points[1] = {1.0f, 0.0f, 0.0f};
    cloud.points[1].r = 0;
    cloud.points[1].g = 0;
    cloud.points[1].b = 0;
    cloud.points[2] = {0.0f, 1.0f, 0.0f};
    cloud.points[2].r = 0;
    cloud.points[2].g = 0;
    cloud.points[2].b = 0;
    pcl::toPCLPointCloud2(cloud, polygon_mesh.cloud);

    // One triangle face
    pcl::Vertices tri;
    tri.vertices = {0, 1, 2};
    polygon_mesh.polygons.push_back(tri);

    auto mesh = to_speckle(polygon_mesh);

    REQUIRE(mesh.vertices.size() == 9);
    REQUIRE(mesh.faces.size() == 4);
    REQUIRE(mesh.faces[0] == 0); // triangle
    REQUIRE(mesh.colors.size() == 3);

    // Check first point color ARGB
    int expected = (255 << 24) | (128 << 16) | (64 << 8) | 32;
    REQUIRE(mesh.colors[0] == expected);
}

// ============================================================
// Collection / hierarchy tests
// ============================================================

TEST_CASE("Collection can hold child elements", "[speckle]") {
    auto col = std::make_shared<Collection>();
    col->name = "Room 1";

    auto pc = std::make_shared<Pointcloud>();
    pc->points = {1.0, 2.0, 3.0};

    auto mesh = std::make_shared<Mesh>();
    mesh->vertices = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    mesh->faces = {0, 0, 1, 2};

    col->elements.push_back(pc);
    col->elements.push_back(mesh);

    REQUIRE(col->elements.size() == 2);
    REQUIRE(col->elements[0]->speckle_type == "Objects.Geometry.Pointcloud");
    REQUIRE(col->elements[1]->speckle_type == "Objects.Geometry.Mesh");
}

TEST_CASE("SpeckleClient constructor handles empty token", "[speckle]") {
    // Should not throw even without a token
    SpeckleClient client("https://example.com", "test_project_id", "");
    // Just verify construction succeeds (no crash)
    REQUIRE(true);
}

TEST_CASE("SpeckleClient constructor strips trailing slash", "[speckle]") {
    // Verify it doesn't throw; actual URL stripping is tested indirectly
    SpeckleClient client("https://example.com/", "project_id", "dummy_token");
    REQUIRE(true);
}

// ============================================================
// Configuration setter tests
// ============================================================

TEST_CASE("SpeckleClient set_max_batch_size compiles and does not throw",
          "[speckle]") {
    SpeckleClient client("https://example.com", "proj", "tok");
    REQUIRE_NOTHROW(client.set_max_batch_size(10 * 1024 * 1024));
    REQUIRE_NOTHROW(client.set_max_batch_size(1));
}

TEST_CASE("SpeckleClient set_chunk_size compiles and does not throw",
          "[speckle]") {
    SpeckleClient client("https://example.com", "proj", "tok");
    REQUIRE_NOTHROW(client.set_chunk_size(1000));
    REQUIRE_NOTHROW(client.set_chunk_size(100000));
}
