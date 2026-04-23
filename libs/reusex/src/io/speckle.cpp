// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "io/speckle.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include "core/logging.hpp"

#include <pcl/common/io.h>

#include <algorithm>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace reusex::io::speckle {

// ============================================================
// Internal helpers
// ============================================================
namespace {

// ---- MD5 hashing via OpenSSL EVP ----

std::string md5_hash(const std::string &data) {
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;

    EVP_MD_CTX *ctx = EVP_MD_CTX_new();
    if (!ctx)
        throw std::runtime_error("EVP_MD_CTX_new failed");

    if (EVP_DigestInit_ex(ctx, EVP_md5(), nullptr) != 1 ||
        EVP_DigestUpdate(ctx, data.data(), data.size()) != 1 ||
        EVP_DigestFinal_ex(ctx, digest, &digest_len) != 1) {
        EVP_MD_CTX_free(ctx);
        throw std::runtime_error("EVP MD5 digest failed");
    }
    EVP_MD_CTX_free(ctx);

    static constexpr char hex_chars[] = "0123456789abcdef";
    std::string result;
    result.reserve(digest_len * 2);
    for (unsigned int i = 0; i < digest_len; ++i) {
        result.push_back(hex_chars[(digest[i] >> 4) & 0x0F]);
        result.push_back(hex_chars[digest[i] & 0x0F]);
    }
    return result;
}

// ---- Chunking ----

template <typename T>
std::vector<nlohmann::json>
make_chunks(const std::vector<T> &data, std::size_t chunk_size,
            std::vector<nlohmann::json> &all_objects) {
    std::vector<nlohmann::json> chunk_refs;
    for (std::size_t offset = 0; offset < data.size(); offset += chunk_size) {
        std::size_t end = std::min(offset + chunk_size, data.size());
        nlohmann::json chunk_data =
            nlohmann::json::array();
        for (std::size_t i = offset; i < end; ++i)
            chunk_data.push_back(data[i]);

        nlohmann::json chunk_obj;
        chunk_obj["speckle_type"] = "Speckle.Core.Models.DataChunk";
        chunk_obj["data"] = std::move(chunk_data);

        // Compute hash BEFORE adding metadata
        std::string canonical = chunk_obj.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
        chunk_obj["id"] = md5_hash(canonical);
        chunk_obj["totalChildrenCount"] = 0;

        chunk_refs.push_back(
            nlohmann::json({{"referencedId", chunk_obj["id"]},
                            {"speckle_type", "reference"},
                            {"__closure", nullptr}}));
        all_objects.push_back(std::move(chunk_obj));
    }
    return chunk_refs;
}

// ---- Serialization ----

// Closure map: detached child id -> depth relative to parent
using ClosureMap = std::map<std::string, int>;

// Forward declare
nlohmann::json serialize_base(const Base &obj,
                              std::vector<nlohmann::json> &all_objects,
                              ClosureMap &closure);

nlohmann::json serialize_point(const Point &pt) {
    nlohmann::json j;
    j["speckle_type"] = pt.speckle_type;
    j["x"] = pt.x;
    j["y"] = pt.y;
    j["z"] = pt.z;
    j["units"] = pt.units;
    return j;
}

/// Extract referenced IDs from chunk refs and add to closure at depth 1.
void track_chunk_refs(const std::vector<nlohmann::json> &refs,
                      ClosureMap &closure) {
    for (const auto &ref : refs)
        closure[ref["referencedId"].get<std::string>()] = 1;
}

nlohmann::json serialize_base(const Base &obj,
                              std::vector<nlohmann::json> &all_objects,
                              ClosureMap &closure) {
    nlohmann::json j;
    j["speckle_type"] = obj.speckle_type;

    if (!obj.applicationId.empty())
        j["applicationId"] = obj.applicationId;

    // Add custom properties
    for (const auto &[key, value] : obj.properties)
        j[key] = value;

    // Helper: embed small arrays directly, chunk large ones.
    // chunk_size is per-property, matching official Speckle SDK conventions:
    //   points/vertices/textureCoordinates: 31250 elements (~250 KB for doubles)
    //   colors/faces/sizes:                 62500 elements (~250 KB for ints)
    auto serialize_array = [&](const auto &vec, const std::string &name,
                               std::size_t chunk_size) {
        if (vec.empty())
            return;
        if (vec.size() <= chunk_size) {
            // Embed directly (no chunking needed)
            j[name] = vec;
        } else {
            // Chunk and detach
            auto refs = make_chunks(vec, chunk_size, all_objects);
            track_chunk_refs(refs, closure);
            j[name] = std::move(refs);
        }
    };

    // Official Speckle SDK chunk sizes (keep serialized chunks ~250 KB each)
    constexpr std::size_t kChunkCoords = 31250; // doubles: points, vertices
    constexpr std::size_t kChunkInts = 62500;   // ints: colors, faces, sizes

    // Type-specific fields
    if (auto *pc = dynamic_cast<const Pointcloud *>(&obj)) {
        j["units"] = pc->units;
        serialize_array(pc->points, "points", kChunkCoords);
        serialize_array(pc->colors, "colors", kChunkInts);
        serialize_array(pc->sizes, "sizes", kChunkInts);
    } else if (auto *mesh = dynamic_cast<const Mesh *>(&obj)) {
        j["units"] = mesh->units;
        serialize_array(mesh->vertices, "vertices", kChunkCoords);
        serialize_array(mesh->faces, "faces", kChunkInts);
        serialize_array(mesh->colors, "colors", kChunkInts);
    } else if (auto *line = dynamic_cast<const Line *>(&obj)) {
        j["units"] = line->units;
        j["start"] = serialize_point(line->start);
        j["end"] = serialize_point(line->end);
    } else if (auto *pt = dynamic_cast<const Point *>(&obj)) {
        j["x"] = pt->x;
        j["y"] = pt->y;
        j["z"] = pt->z;
        j["units"] = pt->units;
    } else if (auto *col = dynamic_cast<const Collection *>(&obj)) {
        j["name"] = col->name;
        j["collectionType"] = col->collectionType;
    }

    // Detached children (@elements)
    if (!obj.elements.empty()) {
        nlohmann::json element_refs = nlohmann::json::array();
        for (const auto &child : obj.elements) {
            ClosureMap child_closure;
            nlohmann::json child_json =
                serialize_base(*child, all_objects, child_closure);

            // Compute hash BEFORE adding metadata fields
            std::string canonical = child_json.dump(
                -1, ' ', false, nlohmann::json::error_handler_t::replace);
            std::string child_id = md5_hash(canonical);

            // Add metadata after hashing (Speckle convention)
            child_json["id"] = child_id;
            child_json["totalChildrenCount"] = nullptr;
            if (!child_closure.empty()) {
                nlohmann::json cc;
                for (const auto &[id, d] : child_closure)
                    cc[id] = d;
                child_json["__closure"] = std::move(cc);
            }

            // Track in parent closure: child at depth 1, its children at depth+1
            closure[child_id] = 1;
            for (const auto &[id, d] : child_closure)
                closure[id] = d + 1;

            element_refs.push_back(
                nlohmann::json({{"referencedId", child_id},
                                {"speckle_type", "reference"},
                                {"__closure", nullptr}}));
            all_objects.push_back(std::move(child_json));
        }
        j["@elements"] = std::move(element_refs);
    }

    return j;
}

// Collect all objects into a flat list with IDs
std::pair<std::string, std::vector<nlohmann::json>>
flatten(const Base &root) {
    std::vector<nlohmann::json> all_objects;
    ClosureMap root_closure;
    nlohmann::json root_json =
        serialize_base(root, all_objects, root_closure);

    // Compute root hash BEFORE adding metadata fields
    std::string canonical = root_json.dump(
        -1, ' ', false, nlohmann::json::error_handler_t::replace);
    std::string root_id = md5_hash(canonical);

    // Add metadata after hashing (Speckle convention)
    root_json["id"] = root_id;
    root_json["totalChildrenCount"] = nullptr;
    if (!root_closure.empty()) {
        nlohmann::json cc;
        for (const auto &[id, d] : root_closure)
            cc[id] = d;
        root_json["__closure"] = std::move(cc);
    }

    // Root goes first
    all_objects.insert(all_objects.begin(), std::move(root_json));
    return {root_id, std::move(all_objects)};
}

// ---- HTTP via libcurl ----

size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata) {
    auto *response = static_cast<std::string *>(userdata);
    response->append(ptr, size * nmemb);
    return size * nmemb;
}

std::string http_post_multipart(const std::string &url,
                                const std::string &token,
                                const std::string &json_payload) {
    CURL *curl = curl_easy_init();
    if (!curl)
        throw std::runtime_error("curl_easy_init failed");

    std::string response;
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers,
                                ("Authorization: Bearer " + token).c_str());

    curl_mime *mime = curl_mime_init(curl);
    curl_mimepart *part = curl_mime_addpart(mime);
    curl_mime_name(part, "batch-0");
    curl_mime_filename(part, "batch-0");
    curl_mime_data(part, json_payload.c_str(),
                   static_cast<curl_off_t>(json_payload.size()));
    curl_mime_type(part, "application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_mime_free(mime);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
        throw std::runtime_error(std::string("curl error: ") +
                                 curl_easy_strerror(res));

    if (http_code < 200 || http_code >= 300)
        throw std::runtime_error("Speckle upload failed (HTTP " +
                                 std::to_string(http_code) + "): " + response);

    return response;
}

std::string http_post_json(const std::string &url, const std::string &token,
                           const std::string &json_body) {
    CURL *curl = curl_easy_init();
    if (!curl)
        throw std::runtime_error("curl_easy_init failed");

    std::string response;
    struct curl_slist *headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    headers = curl_slist_append(headers,
                                ("Authorization: Bearer " + token).c_str());

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK)
        throw std::runtime_error(std::string("curl error: ") +
                                 curl_easy_strerror(res));

    if (http_code < 200 || http_code >= 300)
        throw std::runtime_error("GraphQL request failed (HTTP " +
                                 std::to_string(http_code) + "): " + response);

    return response;
}

} // anonymous namespace

// ============================================================
// SpeckleClient
// ============================================================

SpeckleClient::SpeckleClient(std::string server_url, std::string project_id,
                             std::string token)
    : server_url_(std::move(server_url)), project_id_(std::move(project_id)),
      token_(std::move(token)) {
    // Remove trailing slash from server URL
    if (!server_url_.empty() && server_url_.back() == '/')
        server_url_.pop_back();

    // Fall back to environment variable
    if (token_.empty()) {
        const char *env_token = std::getenv("SPECKLE_TOKEN");
        if (env_token)
            token_ = env_token;
    }

    if (token_.empty())
        core::warn("SpeckleClient: no auth token provided (set SPECKLE_TOKEN "
                     "env or pass token to constructor)");
}

void SpeckleClient::set_max_batch_size(std::size_t bytes) {
    max_batch_bytes_ = bytes;
}

std::string SpeckleClient::send(const Base &root) {
    auto [root_id, objects] = flatten(root);

    core::info("Speckle: uploading {} objects (root: {})", objects.size(),
                 root_id);

    std::string url = server_url_ + "/objects/" + project_id_;

    // Split objects into size-limited batches (JSON array format).
    nlohmann::json current_batch = nlohmann::json::array();
    std::size_t current_size = 0;
    std::size_t batch_num = 0;

    auto send_batch = [&]() {
        std::string payload = current_batch.dump();
        core::info("Speckle: sending batch {} ({} bytes, {} objects)",
                     batch_num, payload.size(), current_batch.size());
        http_post_multipart(url, token_, payload);
        current_batch = nlohmann::json::array();
        current_size = 0;
        ++batch_num;
    };

    for (auto &obj : objects) {
        std::string obj_str = obj.dump();
        std::size_t obj_size = obj_str.size();

        if (!current_batch.empty() &&
            current_size + obj_size > max_batch_bytes_) {
            send_batch();
        }

        current_batch.push_back(std::move(obj));
        current_size += obj_size;
    }

    if (!current_batch.empty())
        send_batch();

    core::info("Speckle: upload complete ({} batches)", batch_num);
    return root_id;
}

std::string SpeckleClient::commit(const std::string &object_id,
                                  const std::string &branch,
                                  const std::string &message) {
    nlohmann::json mutation = {
        {"query",
         "mutation commitCreate($input: CommitCreateInput!) { "
         "commitCreate(commit: $input) }"},
        {"variables",
         {{"input",
           {{"streamId", project_id_},
            {"objectId", object_id},
            {"branchName", branch},
            {"message", message},
            {"sourceApplication", "ReUseX"}}}}}};

    std::string url = server_url_ + "/graphql";
    std::string response = http_post_json(url, token_, mutation.dump());

    auto resp_json = nlohmann::json::parse(response);
    core::debug("GraphQL commit response: {}", resp_json.dump(2));

    if (resp_json.contains("errors") && !resp_json["errors"].empty()) {
        throw std::runtime_error(
            "Speckle commit failed: " +
            resp_json["errors"][0]["message"].get<std::string>());
    }

    std::string commit_id =
        resp_json["data"]["commitCreate"].get<std::string>();
    core::info("Speckle: commit created: {}", commit_id);
    return commit_id;
}

std::string SpeckleClient::upload(const Base &root, const std::string &branch,
                                  const std::string &message) {
    std::string root_id = send(root);
    return commit(root_id, branch, message);
}

// ============================================================
// Conversion Helpers
// ============================================================

Pointcloud to_speckle(CloudConstPtr cloud) {
    if (!cloud || cloud->empty())
        throw std::invalid_argument("to_speckle: input cloud is null or empty");

    Pointcloud pc;
    pc.points.reserve(cloud->size() * 3);
    pc.colors.reserve(cloud->size());

    for (const auto &pt : cloud->points) {
        pc.points.push_back(static_cast<double>(pt.x));
        pc.points.push_back(static_cast<double>(pt.y));
        pc.points.push_back(static_cast<double>(pt.z));

        // ARGB packed integer: A=255, R, G, B
        int argb = (255 << 24) | (pt.r << 16) | (pt.g << 8) | pt.b;
        pc.colors.push_back(argb);
    }

    return pc;
}

Mesh to_speckle(const pcl::PolygonMesh &mesh) {
    Mesh speckle_mesh;

    // Check if the mesh cloud has color data
    bool has_color = false;
    for (const auto &field : mesh.cloud.fields) {
        if (field.name == "rgb" || field.name == "rgba") {
            has_color = true;
            break;
        }
    }

    if (has_color) {
        // Extract vertices + colors via PointXYZRGB
        Cloud cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);

        speckle_mesh.vertices.reserve(cloud.size() * 3);
        speckle_mesh.colors.reserve(cloud.size());
        for (const auto &pt : cloud.points) {
            speckle_mesh.vertices.push_back(static_cast<double>(pt.x));
            speckle_mesh.vertices.push_back(static_cast<double>(pt.y));
            speckle_mesh.vertices.push_back(static_cast<double>(pt.z));

            int argb = (255 << 24) | (pt.r << 16) | (pt.g << 8) | pt.b;
            speckle_mesh.colors.push_back(argb);
        }
    } else {
        // No color data — extract XYZ only
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromPCLPointCloud2(mesh.cloud, cloud);

        speckle_mesh.vertices.reserve(cloud.size() * 3);
        for (const auto &pt : cloud.points) {
            speckle_mesh.vertices.push_back(static_cast<double>(pt.x));
            speckle_mesh.vertices.push_back(static_cast<double>(pt.y));
            speckle_mesh.vertices.push_back(static_cast<double>(pt.z));
        }
    }

    // Convert polygon indices to Speckle face format:
    // [n, i0, i1, ..., n, i0, i1, ...]
    // Speckle uses n=0 for triangles, n=1 for quads
    for (const auto &polygon : mesh.polygons) {
        int n = static_cast<int>(polygon.vertices.size());
        // Speckle convention: 0 = triangle (3 verts), 1 = quad (4 verts),
        // n for n+3 vertices
        speckle_mesh.faces.push_back(n == 3 ? 0 : (n == 4 ? 1 : n - 3));
        for (auto idx : polygon.vertices)
            speckle_mesh.faces.push_back(static_cast<int>(idx));
    }

    return speckle_mesh;
}

Mesh to_speckle(const Eigen::MatrixXd &vertices,
                const Eigen::MatrixXi &faces) {
    Mesh speckle_mesh;

    // vertices: Nx3 matrix
    speckle_mesh.vertices.reserve(vertices.rows() * 3);
    for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
        speckle_mesh.vertices.push_back(vertices(i, 0));
        speckle_mesh.vertices.push_back(vertices(i, 1));
        speckle_mesh.vertices.push_back(vertices(i, 2));
    }

    // faces: Nx3 matrix (triangles)
    speckle_mesh.faces.reserve(faces.rows() * 4);
    for (Eigen::Index i = 0; i < faces.rows(); ++i) {
        speckle_mesh.faces.push_back(0); // 0 = triangle in Speckle
        speckle_mesh.faces.push_back(static_cast<int>(faces(i, 0)));
        speckle_mesh.faces.push_back(static_cast<int>(faces(i, 1)));
        speckle_mesh.faces.push_back(static_cast<int>(faces(i, 2)));
    }

    return speckle_mesh;
}

} // namespace reusex::io::speckle
