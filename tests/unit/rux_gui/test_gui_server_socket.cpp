// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Socket-level tests for the `rux gui` static-asset routes (#265).
//
// WHY THIS FILE EXISTS, AND WHY IT TALKS TO A REAL SOCKET
//
// test_gui_api.cpp covers the pure helpers (resolve_asset,
// looks_like_spa_route, mime_type_for, ...) and every one of them was green
// while the server was nevertheless unable to serve the frontend at all: the
// catchall route answered only the FIRST request on a TCP connection and
// returned Crow's built-in 404 for every request after it. Browsers use
// keep-alive unconditionally, so the browser got index.html and then 404'd the
// <script> it referenced.
//
// A test that issues one request per connection cannot catch that class of bug,
// and neither can a test of the handler in isolation — the fault was in how
// Crow dispatches, not in what the handler computes. So the load-bearing case
// below issues SEVERAL requests on ONE connection and asserts they all succeed.
// Keep it that way; splitting it into a request per connection would make it
// pass against the broken server again. The mechanism, and the rule it implies
// for catchall handlers, is documented at register_static() in
// apps/rux/src/gui/Server.cpp.

#include <catch2/catch_test_macros.hpp>

#include <gui/Server.hpp>
#include <gui/assets.hpp>

#include <sqlite3.h>

#include <opencv2/core.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/resources.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace rux::gui;
using reusex::test_support::TempDir;
using reusex::test_support::TempPath;

namespace {

constexpr const char *kIndexBody =
    "<!doctype html><html><head><script src=\"/assets/app.js\"></script>"
    "</head><body><div id=\"root\"></div></body></html>";
constexpr const char *kScriptBody = "console.log('bundle');";

void write_file(const fs::path &path, std::string_view content) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path, std::ios::binary);
  out << content;
}

/// A port the kernel just handed out and nothing is listening on.
///
/// ServerOptions rejects port 0 (Crow cannot report an ephemeral port back),
/// so the test has to name one. Binding :0 and reading the assignment back is
/// the least collision-prone way to pick it under a parallel ctest.
std::uint16_t free_port() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  REQUIRE(::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0);
  socklen_t len = sizeof(addr);
  REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len) == 0);
  const std::uint16_t port = ::ntohs(addr.sin_port);
  ::close(fd);
  return port;
}

struct Response {
  int status = 0;
  std::string content_type;
  std::string content_disposition;
  std::string body;
};

/// A single HTTP/1.1 connection that is deliberately never closed between
/// requests — reuse is the whole point of these tests.
class KeepAliveConnection {
    public:
  explicit KeepAliveConnection(std::uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
      throw std::runtime_error("socket() failed");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    addr.sin_port = ::htons(port);
    if (::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) !=
        0) {
      ::close(fd_);
      fd_ = -1;
      throw std::runtime_error("connect() failed");
    }
    // A bug that stalls the response must fail the test, not hang the suite.
    timeval timeout{};
    timeout.tv_sec = 10;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

  KeepAliveConnection(const KeepAliveConnection &) = delete;
  KeepAliveConnection &operator=(const KeepAliveConnection &) = delete;

  ~KeepAliveConnection() {
    if (fd_ >= 0)
      ::close(fd_);
  }

  Response get(std::string_view path) { return send_request("GET", path, {}); }

  /// One request with a JSON body (PATCH/POST/PUT) on this connection.
  Response send_json(std::string_view method, std::string_view path,
                     std::string_view body) {
    return send_request(method, path, body);
  }

    private:
  Response send_request(std::string_view method, std::string_view path,
                        std::string_view body) {
    std::string request = std::string(method) + " " + std::string(path) +
                          " HTTP/1.1\r\nHost: 127.0.0.1\r\n"
                          "Connection: keep-alive\r\n";
    if (!body.empty() || method != "GET")
      request += "Content-Type: application/json\r\nContent-Length: " +
                 std::to_string(body.size()) + "\r\n";
    request += "\r\n";
    request += body;
    std::size_t sent = 0;
    while (sent < request.size()) {
      const ssize_t n = ::send(fd_, request.data() + sent,
                               request.size() - sent, MSG_NOSIGNAL);
      if (n <= 0)
        throw std::runtime_error("send() failed for " + std::string(path));
      sent += static_cast<std::size_t>(n);
    }
    return read_response(path);
  }

  /// Read exactly one response out of the stream, leaving any bytes that
  /// belong to the next one in the buffer.
  Response read_response(std::string_view path) {
    std::size_t header_end = std::string::npos;
    while ((header_end = buffer_.find("\r\n\r\n")) == std::string::npos)
      fill(path);

    const std::string head = buffer_.substr(0, header_end);
    Response response;

    const auto status_end = head.find("\r\n");
    const std::string status_line = head.substr(0, status_end);
    const auto first_space = status_line.find(' ');
    if (first_space == std::string::npos)
      throw std::runtime_error("malformed status line: " + status_line);
    response.status = std::stoi(status_line.substr(first_space + 1, 4));

    std::size_t content_length = 0;
    std::size_t cursor = status_end + 2;
    while (cursor < head.size()) {
      const auto line_end = std::min(head.find("\r\n", cursor), head.size());
      const std::string line = head.substr(cursor, line_end - cursor);
      cursor = line_end + 2;
      const auto colon = line.find(':');
      if (colon == std::string::npos)
        continue;
      std::string name = line.substr(0, colon);
      std::transform(
          name.begin(), name.end(), name.begin(),
          [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      std::string value = line.substr(colon + 1);
      const auto first = value.find_first_not_of(" \t");
      value = first == std::string::npos ? std::string() : value.substr(first);
      if (name == "content-length")
        content_length = static_cast<std::size_t>(std::stoul(value));
      else if (name == "content-type")
        response.content_type = value;
      else if (name == "content-disposition")
        response.content_disposition = value;
    }

    const std::size_t body_start = header_end + 4;
    while (buffer_.size() < body_start + content_length)
      fill(path);
    response.body = buffer_.substr(body_start, content_length);
    buffer_.erase(0, body_start + content_length);
    return response;
  }

  void fill(std::string_view path) {
    char chunk[8192];
    const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
    if (n <= 0)
      throw std::runtime_error(
          "the server closed or stalled the connection while answering " +
          std::string(path));
    buffer_.append(chunk, static_cast<std::size_t>(n));
  }

  int fd_ = -1;
  std::string buffer_;
};

/// Runs a Server on its own thread and shuts it down on destruction.
class RunningServer {
    public:
  explicit RunningServer(ServerOptions options)
      : port_(options.port), server_(std::move(options)) {
    thread_ = std::thread([this] { server_.run(); });
    wait_until_listening();
  }

  RunningServer(const RunningServer &) = delete;
  RunningServer &operator=(const RunningServer &) = delete;

  ~RunningServer() {
    server_.stop();
    if (thread_.joinable())
      thread_.join();
  }

  std::uint16_t port() const { return port_; }
  bool has_assets() const { return server_.has_assets(); }

    private:
  void wait_until_listening() {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (std::chrono::steady_clock::now() < deadline) {
      try {
        KeepAliveConnection probe(port_);
        return;
      } catch (const std::exception &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
      }
    }
    FAIL("the GUI server never started listening on port " << port_);
  }

  std::uint16_t port_;
  Server server_;
  std::thread thread_;
};

ServerOptions options_for(const fs::path &project, const fs::path &assets,
                          std::uint16_t port) {
  ServerOptions options;
  options.project = project;
  options.asset_dir = assets;
  options.port = port;
  options.open_browser = false;
  options.threads = 2;
  return options;
}

} // namespace

TEST_CASE("RunningServer_KeepAliveGetRequests_ServesFrontendBundleRepeatedly",
          "[gui][server][socket]") {
  // THE REGRESSION. Before the fix this failed on the second GET: Crow's
  // catchall answered request #1 and returned its built-in "404 Not Found" for
  // every request after it on the same connection, which is exactly what a
  // browser does when it loads index.html and then fetches the bundle it
  // names. Do not weaken this into one request per connection.
  TempPath project("test_gui_server_socket", ".rux");
  TempDir assets("test_gui_server_socket_assets");
  write_file(assets.path / "index.html", kIndexBody);
  write_file(assets.path / "assets" / "app.js", kScriptBody);

  RunningServer server(options_for(project.path, assets.path, free_port()));
  KeepAliveConnection connection(server.port());

  // What a browser actually does: fetch the document, then the assets it
  // references, all down one connection.
  const Response index = connection.get("/");
  CHECK(index.status == 200);
  CHECK(index.body == kIndexBody);

  const Response script = connection.get("/assets/app.js");
  CHECK(script.status == 200);
  CHECK(script.body == kScriptBody);
  CHECK(script.content_type == "text/javascript; charset=utf-8");

  // Several more, because the pre-fix failure alternated (the poisoned
  // `completed_` flag was cleared again by the very request it broke), so a
  // single extra request would have looked fine half the time.
  for (int i = 0; i < 6; ++i) {
    INFO("repeat " << i);
    const Response repeat = connection.get("/assets/app.js");
    CHECK(repeat.status == 200);
    CHECK(repeat.body == kScriptBody);
  }
}

TEST_CASE("RunningServer_KeepAliveVariousRoutes_HonorsRoutingContract",
          "[gui][server][socket]") {
  // Same reuse property, applied to the whole documented behaviour of the
  // static routes: each of these is answered on the SAME connection, in order,
  // so a regression that only survives the first dispatch fails here too.
  TempPath project("test_gui_server_socket", ".rux");
  TempDir assets("test_gui_server_socket_assets");
  write_file(assets.path / "index.html", kIndexBody);
  write_file(assets.path / "assets" / "app.js", kScriptBody);

  RunningServer server(options_for(project.path, assets.path, free_port()));
  KeepAliveConnection connection(server.port());

  SECTION("extensionless paths fall back to the SPA index") {
    for (const char *route : {"/", "/viewport", "/a/b/c", "/projects/42"}) {
      INFO("route: " << route);
      const Response response = connection.get(route);
      CHECK(response.status == 200);
      CHECK(response.body == kIndexBody);
      CHECK(response.content_type == "text/html; charset=utf-8");
    }
  }

  SECTION("a missing file 404s instead of being handed the index") {
    // Answering these with index.html turns a missing file into an
    // inscrutable JavaScript syntax error in the browser.
    for (const char *missing : {"/assets/nope.js", "/favicon.ico",
                                "/assets/app.4f2c.css", "/style.css"}) {
      INFO("missing: " << missing);
      const Response response = connection.get(missing);
      CHECK(response.status == 404);
      CHECK(response.body.find(kIndexBody) == std::string::npos);
    }
  }

  SECTION("an unrouted API path is a JSON 404, never the SPA fallback") {
    for (const char *route :
         {"/api/v1/nope", "/api/v1/definitely/not/a/route"}) {
      INFO("route: " << route);
      const Response response = connection.get(route);
      CHECK(response.status == 404);
      CHECK(response.content_type == "application/json");
      CHECK(response.body.find("\"error\"") != std::string::npos);
      CHECK(response.body.find(kIndexBody) == std::string::npos);
    }
  }

  SECTION("real API routes still answer on a reused connection") {
    for (int i = 0; i < 3; ++i) {
      INFO("repeat " << i);
      const Response response = connection.get("/api/v1/health");
      CHECK(response.status == 200);
      CHECK(response.content_type == "application/json");
    }
  }

  SECTION("path traversal never escapes the bundle") {
    // resolve_asset() refuses these, so the worst they can do is land on the
    // SPA fallback. What must never happen is the file coming back.
    for (const char *attack :
         {"/../../../etc/passwd", "/assets/../../etc/passwd",
          "/%2e%2e/%2e%2e/etc/passwd", "/..%2f..%2fetc/passwd"}) {
      INFO("attack: " << attack);
      const Response response = connection.get(attack);
      CHECK((response.status == 200 || response.status == 404));
      CHECK(response.body.find("root:") == std::string::npos);
    }
  }
}

TEST_CASE("RunningServer_ResourceRoutes_StaticPathsBeatTheCodeParam",
          "[gui][server][socket]") {
  // /resources/keys etc. sit next to /resources/<string>; Crow keeps one
  // trie per method, so GET never reaches the PATCH/DELETE code route.
  TempPath project("test_gui_server_socket", ".rux");
  TempDir assets("test_gui_server_socket_assets");
  write_file(assets.path / "index.html", kIndexBody);
  RunningServer server(options_for(project.path, assets.path, free_port()));
  KeepAliveConnection connection(server.port());
  for (const char *route :
       {"/api/v1/resources/keys", "/api/v1/resources/columns",
        "/api/v1/resources", "/api/v1/resources/export.csv?template=2"}) {
    INFO("route: " << route);
    CHECK(connection.get(route).status == 200);
  }
  CHECK(connection.get("/api/v1/resources/export.csv").status == 400);
  CHECK(connection.get("/api/v1/resources/export.csv?template=").status == 400);
  CHECK(connection.get("/api/v1/resources?template=").status == 400);
  const Response csv =
      connection.get("/api/v1/resources/export.csv?template=2");
  CHECK(csv.content_type == "text/csv; charset=utf-8");
  CHECK(csv.content_disposition == "attachment; filename=\"ressourcer.csv\"");
  CHECK(connection.get("/api/v1/templates").status == 200);
  CHECK(connection.send_json("POST", "/api/v1/templates/restore-seeds", "{}")
            .status == 200);
  CHECK(connection.send_json("POST", "/api/v1/templates/1/duplicate", "{}")
            .status == 201);
}

TEST_CASE("RunningServer_ResourceColumns_NameConflictsAre409",
          "[gui][server][socket]") {
  // A column name that a leksikon field owns, or that passports still store
  // values under (left by a deleted column), is a 409, for create and rename.
  TempPath project("test_gui_server_socket", ".rux");
  TempDir assets("test_gui_server_socket_assets");
  write_file(assets.path / "index.html", kIndexBody);
  std::string live_id;
  {
    reusex::ProjectDB db(project.path);
    const auto t = reusex::test_support::make_type(db, "Døre");
    reusex::test_support::make_part(db, "RX-001", t);
    reusex::ProjectDB::PropertyDefinition gone;
    gone.name = "Gammel";
    gone.type = "text";
    const auto col = reusex::core::create_column(db, gone);
    reusex::core::patch_resource(db, "RX-001", {{"col:" + col.id, "rest"}});
    db.delete_property_definition(col.id); // values stay, column gone
    reusex::ProjectDB::PropertyDefinition live;
    live.name = "Ny";
    live.type = "text";
    live_id = reusex::core::create_column(db, live).id;
  }
  RunningServer server(options_for(project.path, assets.path, free_port()));
  KeepAliveConnection connection(server.port());
  for (const std::string base : {"/api/v1/resources/columns"}) {
    INFO("base: " << base);
    for (const char *name : {"Gammel", "width_mm"}) {
      INFO("name: " << name);
      const std::string body =
          std::string(R"({"name":")") + name + R"(","type":"text"})";
      CHECK(connection.send_json("POST", base, body).status == 409);
      CHECK(connection
                .send_json("PATCH", base + "/" + live_id,
                           std::string(R"({"name":")") + name + "\"}")
                .status == 409);
    }
    CHECK(connection.send_json("DELETE", base + "/nope", "").status == 404);
  }
}

TEST_CASE("RunningServer_NoAssetsKeepAliveRequests_ServesPlaceholderRepeatedly",
          "[gui][server][socket]") {
  // With no bundle installed the server must still answer SPA routes, request
  // after request, or `rux gui` is unusable before the frontend is built.
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");

  ServerOptions options;
  options.project = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 2;
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  for (int i = 0; i < 4; ++i) {
    INFO("repeat " << i);
    const Response response = connection.get("/");
    CHECK(response.status == 200);
    CHECK(response.content_type == "text/html; charset=utf-8");
    if (!server.has_assets())
      CHECK(response.body.find("/api/v1") != std::string::npos);
  }

  // A file request without a bundle is still a 404, not an HTML page.
  const Response missing = connection.get("/assets/app.js");
  CHECK(missing.status == 404);
  CHECK(missing.content_type == "application/json");
}

TEST_CASE("RunningServer_ConcurrentReadsOnAFreshServer_NeverBusy",
          "[gui][server][socket][concurrency]") {
  // A full page load fires the shell's and the page's GETs at once, and each
  // opens its own read-only connection. None may answer 503 while nothing
  // writes (GUI Phase 5, R1).
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");

  ServerOptions options;
  options.project = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 8;
  RunningServer server(std::move(options));

  const std::vector<std::string> paths{
      "/api/v1/project",          "/api/v1/survey/summary",
      "/api/v1/survey/fractions", "/api/v1/survey",
      "/api/v1/samples",          "/api/v1/reports/ressourcekortlaegning"};
  constexpr int kClients = 8;
  constexpr int kRounds = 15;
  std::mutex mutex;
  std::vector<std::string> failures;
  std::vector<std::thread> clients;
  for (int c = 0; c < kClients; ++c)
    clients.emplace_back([&, c] {
      try {
        KeepAliveConnection connection(server.port());
        for (int i = 0; i < kRounds; ++i) {
          const std::string &path =
              paths[static_cast<std::size_t>(c + i) % paths.size()];
          const Response response = connection.get(path);
          if (response.status != 200) {
            std::lock_guard lock(mutex);
            failures.push_back(path + " -> " + std::to_string(response.status) +
                               " " + response.body);
          }
        }
      } catch (const std::exception &e) {
        std::lock_guard lock(mutex);
        failures.push_back(std::string("client error: ") + e.what());
      }
    });
  for (auto &client : clients)
    client.join();

  INFO((failures.empty() ? std::string() : failures.front()));
  CHECK(failures.empty());
}

TEST_CASE("RunningServer_EditThenShutdown_LeavesTheEditInTheMainFile",
          "[gui][server][socket][wal]") {
  // The server's long-lived connection is the last to close, so it is the one
  // that must checkpoint. Were it read-only it could not, and an edit made
  // through the GUI would survive only in project.rux-wal: copying or
  // uploading the .rux alone would silently drop it.
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");
  const fs::path wal = project.path.string() + "-wal";
  const std::string name = "Checkpoint probe";

  {
    ServerOptions options;
    options.project = project.path;
    options.port = free_port();
    options.open_browser = false;
    options.threads = 2;
    RunningServer server(std::move(options));
    KeepAliveConnection connection(server.port());
    const Response response = connection.send_json(
        "PATCH", "/api/v1/projects/p1", R"({"name":"Checkpoint probe"})");
    INFO(response.body);
    REQUIRE(response.status == 200);
  }

  std::error_code ec;
  const bool wal_exists = fs::exists(wal, ec);
  const auto wal_size = wal_exists ? fs::file_size(wal, ec) : 0;
  INFO("-wal exists: " << wal_exists << ", size " << wal_size);
  CHECK(wal_size == 0);

  // Read the main file alone, as a copy that left the -wal behind would.
  TempPath copy("test_gui_server_socket_copy", ".rux");
  fs::copy_file(project.path, copy.path, fs::copy_options::overwrite_existing);
  sqlite3 *raw = nullptr;
  REQUIRE(sqlite3_open_v2(copy.path.string().c_str(), &raw,
                          SQLITE_OPEN_READONLY, nullptr) == SQLITE_OK);
  sqlite3_stmt *stmt = nullptr;
  std::string stored;
  if (sqlite3_prepare_v2(raw, "SELECT name FROM projects WHERE id='p1';", -1,
                         &stmt, nullptr) == SQLITE_OK &&
      sqlite3_step(stmt) == SQLITE_ROW)
    if (const auto *text = sqlite3_column_text(stmt, 0))
      stored = reinterpret_cast<const char *>(text);
  sqlite3_finalize(stmt);
  sqlite3_close(raw);
  CHECK(stored == name);
}

TEST_CASE("RunningServer_SegmentResource_StatusesAndRouting",
          "[gui][server][socket][segment]") {
  // POST /frames/<id>/segment/resource sits under /frames/<id>/segment; both
  // must route. Frame 1 is posed with depth and a saved mask whose label 0
  // covers the one cloud point; frame 2 has a colour image only.
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");
  {
    reusex::ProjectDB db(project.path);
    reusex::core::SensorIntrinsics in;
    in.fx = in.fy = 50.0;
    in.cx = in.cy = 32.0;
    in.width = in.height = 64;
    db.save_sensor_frame(1, cv::Mat(64, 64, CV_8UC3, cv::Scalar(0)),
                         cv::Mat(64, 64, CV_16UC1, cv::Scalar(2000)), cv::Mat(),
                         {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}, in,
                         1.0, -1);
    db.save_segmentation_image(1, cv::Mat(64, 64, CV_32S, cv::Scalar(0)));
    db.save_sensor_frame(2, cv::Mat(8, 8, CV_8UC3, cv::Scalar(0)));
    reusex::Cloud cloud;
    reusex::PointT p;
    p.x = 0.0f;
    p.y = 0.0f;
    p.z = 2.0f;
    cloud.push_back(p);
    db.save_point_cloud("cloud", cloud);
  }
  ServerOptions options;
  options.project = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 2;
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  const std::string route = "/api/v1/frames/1/segment/resource";
  CHECK(connection.send_json("POST", route, "not json").status == 400);
  CHECK(connection.send_json("POST", route, R"({"mask_label":0})").status ==
        400);
  CHECK(connection
            .send_json("POST", "/api/v1/frames/2/segment/resource",
                       R"({"mask_label":0,"class_name":"Dør"})")
            .status == 422);
  CHECK(connection
            .send_json("POST", "/api/v1/frames/99/segment/resource",
                       R"({"mask_label":0,"class_name":"Dør"})")
            .status == 404);
  const Response created = connection.send_json(
      "POST", route, R"({"mask_label":0,"class_name":"Dør"})");
  INFO(created.body);
  CHECK(created.status == 201);
  CHECK(created.body.find("\"resource_code\":\"RX-001\"") != std::string::npos);
  // The sibling segment route still answers (no segmenter registered here).
  CHECK(connection.send_json("POST", "/api/v1/frames/1/segment", "{}").status ==
        503);
}
