// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Socket-level tests for the `ruxd --local` static-asset routes (#265).
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
// apps/ruxd/src/api/Server.cpp.

#include <catch2/catch_test_macros.hpp>

#include <api/AuthService.hpp>
#include <api/Server.hpp>
#include <api/assets.hpp>
#include <api/cases.hpp>

#include <nlohmann/json.hpp>

#include <sqlite3.h>

#include <opencv2/core.hpp>

#include "../../support/survey_fixture.hpp"
#include "../../support/temp_path.hpp"

#include <core/ProjectDB.hpp>
#include <core/SensorIntrinsics.hpp>
#include <core/resources.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
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
using namespace ruxd::api;
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
  std::string set_cookie;
  std::string location;
  std::string referrer_policy;
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

  Response get(std::string_view path, std::string_view extra_headers = {}) {
    return send_request("GET", path, {}, extra_headers);
  }

  /// One request with a JSON body (PATCH/POST/PUT) on this connection.
  Response send_json(std::string_view method, std::string_view path,
                     std::string_view body,
                     std::string_view extra_headers = {}) {
    return send_request(method, path, body, extra_headers);
  }

    private:
  Response send_request(std::string_view method, std::string_view path,
                        std::string_view body,
                        std::string_view extra_headers = {}) {
    std::string request =
        std::string(method) + " " + std::string(path) + " HTTP/1.1\r\n";
    // A test may name its own Host (the DNS-rebinding and same-origin cases).
    if (extra_headers.find("Host:") == std::string_view::npos)
      request += "Host: 127.0.0.1\r\n";
    request += "Connection: keep-alive\r\n";
    request += extra_headers; // each line already "\r\n"-terminated
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
      else if (name == "set-cookie")
        response.set_cookie = value;
      else if (name == "location")
        response.location = value;
      else if (name == "referrer-policy")
        response.referrer_policy = value;
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

/// A minimal WebSocket client for a case's events socket: performs the upgrade
/// and reads unfragmented server text frames (server frames are never masked).
class WebSocketClient {
    public:
  WebSocketClient(std::uint16_t port, const std::string &path,
                  const std::string &extra_headers = {}) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd_ >= 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    addr.sin_port = ::htons(port);
    REQUIRE(::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
            0);
    timeval timeout{};
    timeout.tv_sec = 10;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    const std::string upgrade =
        "GET " + path +
        " HTTP/1.1\r\nHost: 127.0.0.1\r\n"
        "Upgrade: websocket\r\nConnection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n" +
        extra_headers + "\r\n";
    REQUIRE(::send(fd_, upgrade.data(), upgrade.size(), MSG_NOSIGNAL) ==
            static_cast<ssize_t>(upgrade.size()));
    std::size_t end;
    while ((end = buffer_.find("\r\n\r\n")) == std::string::npos)
      fill();
    REQUIRE(buffer_.rfind("HTTP/1.1 101", 0) == 0);
    buffer_.erase(0, end + 4);
  }
  WebSocketClient(const WebSocketClient &) = delete;
  WebSocketClient &operator=(const WebSocketClient &) = delete;
  ~WebSocketClient() { ::close(fd_); }

  /// The payload of the next text frame.
  std::string next_text() {
    while (true) {
      need(2);
      const auto b0 = static_cast<unsigned char>(buffer_[0]);
      std::size_t len = static_cast<unsigned char>(buffer_[1]) & 0x7f;
      std::size_t head = 2;
      if (len == 126) {
        need(4);
        len = (static_cast<std::size_t>(static_cast<unsigned char>(buffer_[2]))
               << 8) |
              static_cast<unsigned char>(buffer_[3]);
        head = 4;
      } else if (len == 127) {
        need(10);
        len = 0;
        for (int i = 2; i < 10; ++i)
          len = (len << 8) | static_cast<unsigned char>(buffer_[i]);
        head = 10;
      }
      need(head + len);
      std::string payload = buffer_.substr(head, len);
      buffer_.erase(0, head + len);
      if ((b0 & 0x0f) == 0x1)
        return payload;
    }
  }

  /// True once the server closes the socket (a close frame, or EOF) within
  /// the receive timeout; false when it is still open after it.
  bool closed_by_server() {
    try {
      while (true) {
        need(2);
        const auto b0 = static_cast<unsigned char>(buffer_[0]);
        std::size_t len = static_cast<unsigned char>(buffer_[1]) & 0x7f;
        std::size_t head = 2;
        if (len == 126) {
          need(4);
          len =
              (static_cast<std::size_t>(static_cast<unsigned char>(buffer_[2]))
               << 8) |
              static_cast<unsigned char>(buffer_[3]);
          head = 4;
        }
        need(head + len);
        buffer_.erase(0, head + len);
        if ((b0 & 0x0f) == 0x8)
          return true;
      }
    } catch (const StillOpen &) {
      return false;
    } catch (const std::runtime_error &) {
      return true; // EOF
    }
  }

  void set_timeout(int seconds) {
    timeval timeout{};
    timeout.tv_sec = seconds;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  }

    private:
  struct StillOpen {};
  void need(std::size_t n) {
    while (buffer_.size() < n)
      fill();
  }
  void fill() {
    char chunk[4096];
    const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      throw StillOpen{};
    if (n <= 0)
      throw std::runtime_error("websocket closed or stalled");
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

/// `/api/v1/cases/<id of @p project>` + @p rest: the case a lone `.rux`
/// target is served as.
std::string case_path(const fs::path &project, std::string_view rest) {
  return "/api/v1/cases/" + case_slug(project.stem().string()) +
         std::string(rest);
}

ServerOptions options_for(const fs::path &project, const fs::path &assets,
                          std::uint16_t port) {
  ServerOptions options;
  options.target = project;
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
  for (const std::string &route :
       {case_path(project.path, "/resources/keys"),
        case_path(project.path, "/resources/columns"),
        case_path(project.path, "/resources"),
        case_path(project.path, "/resources/export.csv?template=2")}) {
    INFO("route: " << route);
    CHECK(connection.get(route).status == 200);
  }
  CHECK(
      connection.get(case_path(project.path, "/resources/export.csv")).status ==
      400);
  CHECK(
      connection.get(case_path(project.path, "/resources/export.csv?template="))
          .status == 400);
  CHECK(
      connection.get(case_path(project.path, "/resources?template=")).status ==
      400);
  const Response csv = connection.get(
      case_path(project.path, "/resources/export.csv?template=2"));
  CHECK(csv.content_type == "text/csv; charset=utf-8");
  CHECK(csv.content_disposition == "attachment; filename=\"ressourcer.csv\"");
  CHECK(connection.get(case_path(project.path, "/templates")).status == 200);
  CHECK(connection
            .send_json("POST",
                       case_path(project.path, "/templates/restore-seeds"),
                       "{}")
            .status == 200);
  CHECK(connection
            .send_json("POST",
                       case_path(project.path, "/templates/1/duplicate"), "{}")
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
  for (const std::string &base :
       {case_path(project.path, "/resources/columns")}) {
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
  // after request, or `ruxd --local` is unusable before the frontend is built.
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_gui_server_socket", ".rux");

  ServerOptions options;
  options.target = project.path;
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
  options.target = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 8;
  RunningServer server(std::move(options));

  const std::vector<std::string> paths{
      case_path(project.path, "/project"),
      case_path(project.path, "/survey/summary"),
      case_path(project.path, "/survey/fractions"),
      case_path(project.path, "/survey"),
      case_path(project.path, "/samples"),
      case_path(project.path, "/reports/ressourcekortlaegning")};
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
    options.target = project.path;
    options.port = free_port();
    options.open_browser = false;
    options.threads = 2;
    RunningServer server(std::move(options));
    KeepAliveConnection connection(server.port());
    const Response response =
        connection.send_json("PATCH", case_path(project.path, "/projects/p1"),
                             R"({"name":"Checkpoint probe"})");
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
  options.target = project.path;
  options.port = free_port();
  options.open_browser = false;
  options.threads = 2;
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  const std::string route =
      case_path(project.path, "/frames/1/segment/resource");
  CHECK(connection.send_json("POST", route, "not json").status == 400);
  CHECK(connection.send_json("POST", route, R"({"mask_label":0})").status ==
        400);
  CHECK(connection
            .send_json("POST",
                       case_path(project.path, "/frames/2/segment/resource"),
                       R"({"mask_label":0,"class_name":"Dør"})")
            .status == 422);
  CHECK(connection
            .send_json("POST",
                       case_path(project.path, "/frames/99/segment/resource"),
                       R"({"mask_label":0,"class_name":"Dør"})")
            .status == 404);
  const Response created = connection.send_json(
      "POST", route, R"({"mask_label":0,"class_name":"Dør"})");
  INFO(created.body);
  CHECK(created.status == 201);
  CHECK(created.body.find("\"resource_code\":\"RX-001\"") != std::string::npos);
  // Every WebSocket client hears which clouds changed.
  WebSocketClient ws(server.port(), case_path(project.path, "/events"));
  CHECK(ws.next_text().find("\"type\":\"hello\"") != std::string::npos);
  CHECK(connection
            .send_json("POST", route, R"({"mask_label":0,"class_name":"Væg"})")
            .status == 201);
  const std::string event = ws.next_text();
  INFO(event);
  CHECK(event.find("\"type\":\"clouds.changed\"") != std::string::npos);
  CHECK(event.find("\"names\":[\"labels\",\"instances\"]") !=
        std::string::npos);
  // The sibling segment route still answers (no segmenter registered here).
  CHECK(
      connection
          .send_json("POST", case_path(project.path, "/frames/1/segment"), "{}")
          .status == 503);
}

namespace {

/// The status line a WebSocket upgrade of @p path gets, sent with
/// @p extra_headers. Crow closes the connection on a refused upgrade, so an
/// empty string means "refused without a response".
std::string websocket_upgrade_status(std::uint16_t port, std::string_view path,
                                     std::string_view extra_headers) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  addr.sin_port = ::htons(port);
  REQUIRE(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
          0);
  timeval timeout{};
  timeout.tv_sec = 10;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  std::string upgrade = "GET " + std::string(path) + " HTTP/1.1\r\n";
  if (extra_headers.find("Host:") == std::string_view::npos)
    upgrade += "Host: 127.0.0.1\r\n";
  upgrade += "Upgrade: websocket\r\nConnection: Upgrade\r\n"
             "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
             "Sec-WebSocket-Version: 13\r\n";
  upgrade += extra_headers;
  upgrade += "\r\n";
  ::send(fd, upgrade.data(), upgrade.size(), MSG_NOSIGNAL);
  std::string buffer;
  char chunk[1024];
  while (buffer.find("\r\n") == std::string::npos) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0)
      break;
    buffer.append(chunk, static_cast<std::size_t>(n));
  }
  ::close(fd);
  return buffer.substr(0, buffer.find("\r\n"));
}

} // namespace

TEST_CASE("Server_NonLoopbackBindWithoutToken_RefusedBeforeOpeningProject",
          "[ruxd_api][server][auth]") {
  TempPath project("test_api_server_auth", ".rux");
  ServerOptions options = options_for(project.path, {}, free_port());
  options.bind_address = "0.0.0.0";
  CHECK_THROWS_AS(Server(options), std::runtime_error);
  // The refusal happens before the project is opened (which would create it).
  CHECK_FALSE(fs::exists(project.path));

  // With a token the same bind is accepted (not started: no run()).
  options.auth_token = "s3cret";
  CHECK_NOTHROW(Server(options));
}

TEST_CASE("RunningServer_AuthToken_RequiredOnEveryRouteAndUpgrade",
          "[ruxd_api][server][socket][auth]") {
  TempPath project("test_api_server_auth", ".rux");
  TempDir assets("test_api_server_auth_assets");
  write_file(assets.path / "index.html", kIndexBody);

  ServerOptions options = options_for(project.path, assets.path, free_port());
  options.auth_token = "s3cret";
  const std::string cookie = "ruxd_token_" + std::to_string(options.port);
  RunningServer server(std::move(options));
  const std::string events = case_path(project.path, "/events");
  KeepAliveConnection connection(server.port());

  SECTION("no token: API and static files are refused") {
    CHECK(connection.get("/api/v1/health").status == 401);
    CHECK(connection.get("/").status == 401);
    CHECK(connection.get("/api/v1/health", "Authorization: Bearer nope\r\n")
              .status == 401);
    // A cookie named for another port is another server's.
    CHECK(connection.get("/api/v1/health", "Cookie: ruxd_token_1=s3cret\r\n")
              .status == 401);
    CHECK(websocket_upgrade_status(server.port(), events, "").find("101") ==
          std::string::npos);
  }

  SECTION("Bearer header and the per-port cookie are accepted") {
    const Response bearer =
        connection.get("/api/v1/health", "Authorization: Bearer s3cret\r\n");
    CHECK(bearer.status == 200);
    CHECK(bearer.set_cookie.empty());
    CHECK(bearer.referrer_policy == "no-referrer");
    const Response with_cookie = connection.get(
        "/api/v1/health", "Cookie: a=1; " + cookie + "=s3cret\r\n");
    CHECK(with_cookie.status == 200);
    CHECK(websocket_upgrade_status(server.port(), events,
                                   "Cookie: " + cookie + "=s3cret\r\n")
              .find("101") != std::string::npos);
  }

  SECTION("?token= redirects to the URL without it and sets the cookie") {
    const Response index = connection.get("/?token=s3cret");
    CHECK(index.status == 303);
    CHECK(index.location == "/");
    CHECK(index.referrer_policy == "no-referrer");
    CHECK(index.set_cookie.rfind(cookie + "=s3cret;", 0) == 0);
    CHECK(index.set_cookie.find("HttpOnly") != std::string::npos);
    CHECK(index.set_cookie.find("SameSite=Strict") != std::string::npos);

    const Response deep =
        connection.get("/kortlaegning?part=RX-001&token=s3cret");
    CHECK(deep.status == 303);
    CHECK(deep.location == "/kortlaegning?part=RX-001");

    // A stale cookie does not shadow a correct ?token=; the answer replaces it.
    const Response stale =
        connection.get("/?token=s3cret", "Cookie: " + cookie + "=old\r\n");
    CHECK(stale.status == 303);
    CHECK(stale.set_cookie.rfind(cookie + "=s3cret;", 0) == 0);
  }
}

TEST_CASE("RunningServer_TokenAndWildcardBind_SameOriginBrowserMayMutate",
          "[ruxd_api][server][socket][auth]") {
  // The documented LAN recipe: --bind 0.0.0.0 --auth-token. A browser on
  // another machine sends Origin http://<host>:<port> with every POST and
  // with the WebSocket upgrade; that is its own origin, not a foreign one.
  TempPath project("test_api_server_auth", ".rux");
  ServerOptions options = options_for(project.path, {}, free_port());
  options.bind_address = "0.0.0.0";
  options.auth_token = "s3cret";
  const std::string host = "192.168.1.20:" + std::to_string(options.port);
  RunningServer server(std::move(options));
  const std::string events = case_path(project.path, "/events");
  KeepAliveConnection connection(server.port());

  const std::string auth = "Authorization: Bearer s3cret\r\n";
  const Response same = connection.send_json(
      "POST", case_path(project.path, "/jobs"), R"({"stage":"nope"})",
      "Host: " + host + "\r\nOrigin: http://" + host + "\r\n" + auth);
  INFO(same.body);
  CHECK(same.status != 403);
  CHECK(same.status != 401);

  const Response foreign = connection.send_json(
      "POST", case_path(project.path, "/jobs"), R"({"stage":"nope"})",
      "Host: " + host + "\r\nOrigin: http://evil.example\r\n" + auth);
  CHECK(foreign.status == 403);

  CHECK(websocket_upgrade_status(
            server.port(), events,
            "Origin: http://127.0.0.1:" + std::to_string(server.port()) +
                "\r\nCookie: ruxd_token_" + std::to_string(server.port()) +
                "=s3cret\r\n")
            .find("101") != std::string::npos);
}

TEST_CASE("RunningServer_LoopbackBind_RefusesForeignHostHeader",
          "[ruxd_api][server][socket][auth]") {
  // DNS rebinding: evil.example resolved to 127.0.0.1 sends same-origin GETs
  // with no Origin header and Host: evil.example. Only the Host check sees it.
  TempPath project("test_api_server_host", ".rux");
  RunningServer server(options_for(project.path, {}, free_port()));
  const std::string events = case_path(project.path, "/events");
  KeepAliveConnection connection(server.port());

  const auto port = std::to_string(server.port());
  CHECK(connection.get("/api/v1/health", "Host: evil.example:" + port + "\r\n")
            .status == 403);
  CHECK(connection.get("/api/v1/health", "Host: localhost:" + port + "\r\n")
            .status == 200);
  CHECK(
      connection.get("/api/v1/health", "Host: [::1]:" + port + "\r\n").status ==
      200);
  CHECK(connection.get("/api/v1/health").status == 200); // Host: 127.0.0.1
  CHECK(websocket_upgrade_status(server.port(), events,
                                 "Host: evil.example:" + port + "\r\n")
            .find("101") == std::string::npos);
}

// ===========================================================================
// Several cases from one server (spec 2026-10-08, phase S2)
// ===========================================================================

namespace {

std::string slurp(const fs::path &path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), {}};
}

/// Read text frames until one whose "type" is @p type.
nlohmann::json next_of_type(WebSocketClient &ws, std::string_view type) {
  for (int i = 0; i < 200; ++i) {
    auto message = nlohmann::json::parse(ws.next_text());
    if (message.value("type", "") == type)
      return message;
  }
  FAIL("no '" << type << "' message arrived");
  return {};
}

} // namespace

TEST_CASE("RunningServer_TwoCases_ConcurrentRequestsStayInTheirCase",
          "[ruxd_api][server][socket][cases]") {
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_cases");
  for (const char *name : {"alpha", "beta"})
    reusex::ProjectDB db(dir.path / (std::string(name) + ".rux"));

  ServerOptions options = options_for(dir.path, {}, free_port());
  options.threads = 8;
  options.job_workers = 2;
  // A stage that takes a moment, so case alpha has a job in flight while
  // case beta is browsed.
  options.stage_executor = [](const reusex::pipeline::StageContext &ctx) {
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    return reusex::pipeline::StageResult::success(
        "ran on " + ctx.project.filename().string());
  };
  RunningServer server(std::move(options));

  WebSocketClient ws_alpha(server.port(), "/api/v1/cases/alpha/events");
  WebSocketClient ws_beta(server.port(), "/api/v1/cases/beta/events");
  CHECK(next_of_type(ws_alpha, "hello")["case"] == "alpha");
  CHECK(next_of_type(ws_beta, "hello")["case"] == "beta");

  {
    KeepAliveConnection connection(server.port());
    for (const std::string cid : {"alpha", "beta"})
      REQUIRE(connection
                  .send_json("PATCH", "/api/v1/cases/" + cid + "/projects/p1",
                             R"({"name":"Sag )" + cid + "\"}")
                  .status == 200);
    const auto job = connection.send_json("POST", "/api/v1/cases/alpha/jobs",
                                          R"({"stage":"planes"})");
    INFO(job.body);
    REQUIRE(job.status == 202);
  }

  // Many clients on both cases at once, while alpha's job runs.
  std::mutex mutex;
  std::vector<std::string> failures;
  std::vector<std::thread> clients;
  for (int c = 0; c < 8; ++c)
    clients.emplace_back([&, c] {
      const std::string cid = c % 2 == 0 ? "alpha" : "beta";
      try {
        KeepAliveConnection connection(server.port());
        for (int i = 0; i < 10; ++i) {
          const Response r =
              connection.get("/api/v1/cases/" + cid + "/projects");
          const bool ok =
              r.status == 200 &&
              r.body.find("\"Sag " + cid + "\"") != std::string::npos &&
              r.body.find(cid == "alpha" ? "Sag beta" : "Sag alpha") ==
                  std::string::npos;
          if (!ok) {
            std::lock_guard lock(mutex);
            failures.push_back(cid + " -> " + std::to_string(r.status) + " " +
                               r.body);
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

  // Alpha's socket hears its job finish...
  const auto finished = next_of_type(ws_alpha, "job.finished");
  CHECK(finished["case"] == "alpha");
  CHECK(finished["job"]["result"]["summary"] == "ran on alpha.rux");

  // ...and beta's heard nothing of it: the first job message it gets is its
  // own job's.
  KeepAliveConnection connection(server.port());
  REQUIRE(
      connection
          .send_json("POST", "/api/v1/cases/beta/jobs", R"({"stage":"rooms"})")
          .status == 202);
  const auto first = nlohmann::json::parse(ws_beta.next_text());
  CHECK(first["type"] == "job.submitted");
  CHECK(first["case"] == "beta");
  CHECK(first["job"]["stage"] == "rooms");

  // Each case lists only its own jobs.
  const auto alpha_jobs =
      nlohmann::json::parse(connection.get("/api/v1/cases/alpha/jobs").body);
  REQUIRE(alpha_jobs["jobs"].size() == 1);
  CHECK(alpha_jobs["jobs"][0]["stage"] == "planes");
}

TEST_CASE("RunningServer_CasesApi_CreateUploadRenameDelete",
          "[ruxd_api][server][socket][cases]") {
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_cases");
  {
    reusex::ProjectDB db(dir.path / "eksisterende.rux");
  }
  TempDir src("test_api_server_cases_src");
  {
    reusex::ProjectDB db(src.path / "upload.rux");
  }
  const std::string bytes = slurp(src.path / "upload.rux");

  ServerOptions options = options_for(dir.path, {}, free_port());
  options.upload_limits.max_bytes = bytes.size() + 10;
  options.upload_limits.max_chunk_bytes = bytes.size() / 2 + 1;
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  auto list = nlohmann::json::parse(connection.get("/api/v1/cases").body);
  REQUIRE(list["cases"].size() == 1);
  CHECK(list["cases"][0]["id"] == "eksisterende");
  CHECK(list["cases"][0]["file_name"] == "eksisterende.rux");
  CHECK_FALSE(list["cases"][0].contains("path")); // no server paths
  CHECK(list["writable"] == true);

  // Create an empty case.
  const auto created =
      connection.send_json("POST", "/api/v1/cases", R"({"name":"Ny sag"})");
  INFO(created.body);
  REQUIRE(created.status == 201);
  CHECK(nlohmann::json::parse(created.body)["id"] == "ny-sag");
  CHECK(connection.get("/api/v1/cases/ny-sag/project").status == 200);

  // Rename it.
  const auto renamed = connection.send_json("PATCH", "/api/v1/cases/ny-sag",
                                            R"({"name":"Omdøbt"})");
  CHECK(renamed.status == 200);
  CHECK(nlohmann::json::parse(renamed.body)["name"] == "Omdøbt");

  // Upload a project in two chunks.
  CHECK(connection
            .send_json("POST", "/api/v1/uploads",
                       R"({"name":"For stor","size":)" +
                           std::to_string(bytes.size() + 11) + "}")
            .status == 413);
  const auto begun = connection.send_json(
      "POST", "/api/v1/uploads",
      R"({"name":"Uploadet","size":)" + std::to_string(bytes.size()) + "}");
  REQUIRE(begun.status == 201);
  const std::string upload = nlohmann::json::parse(begun.body)["id"];
  const std::size_t half = bytes.size() / 2;
  CHECK(connection
            .send_json("PUT", "/api/v1/uploads/" + upload + "?offset=0",
                       std::string_view(bytes).substr(0, half + 2))
            .status == 413); // over the chunk limit
  CHECK(connection
            .send_json("PUT", "/api/v1/uploads/" + upload + "?offset=0",
                       std::string_view(bytes).substr(0, half))
            .status == 200);
  CHECK(connection
            .send_json("POST", "/api/v1/uploads/" + upload + "/complete", "{}")
            .status == 409); // incomplete
  CHECK(connection
            .send_json("PUT", "/api/v1/uploads/" + upload + "?offset=0",
                       std::string_view(bytes).substr(half))
            .status == 409); // wrong offset
  CHECK(connection
            .send_json("PUT",
                       "/api/v1/uploads/" + upload +
                           "?offset=" + std::to_string(half),
                       std::string_view(bytes).substr(half))
            .status == 200);
  const auto done = connection.send_json(
      "POST", "/api/v1/uploads/" + upload + "/complete", "{}");
  INFO(done.body);
  REQUIRE(done.status == 201);
  CHECK(nlohmann::json::parse(done.body)["id"] == "uploadet");
  CHECK(connection.get("/api/v1/cases/uploadet/health").status == 200);

  list = nlohmann::json::parse(connection.get("/api/v1/cases").body);
  CHECK(list["cases"].size() == 3);

  // Unknown and hostile ids are 404s, on HTTP and on the socket.
  for (const char *path :
       {"/api/v1/cases/nope/project", "/api/v1/cases/..%2F..%2Fetc/project",
        "/api/v1/cases/%2e%2e/project", "/api/v1/cases/nope"}) {
    INFO(path);
    CHECK(connection.get(path).status == 404);
  }
  CHECK(websocket_upgrade_status(server.port(), "/api/v1/cases/nope/events", "")
            .find("101") == std::string::npos);

  // Delete moves the case to the trash.
  CHECK(connection.send_json("DELETE", "/api/v1/cases/uploadet", "").status ==
        204);
  CHECK(connection.get("/api/v1/cases/uploadet").status == 404);
  CHECK(connection.get("/api/v1/cases/uploadet/project").status == 404);
  CHECK_FALSE(fs::exists(dir.path / "uploadet"));
  bool in_trash = false;
  for (const auto &entry : fs::directory_iterator(dir.path / ".ruxd" / "trash"))
    in_trash |= fs::exists(entry.path() / "project.rux");
  CHECK(in_trash);
}

TEST_CASE("RunningServer_Upload_CorruptOrExecutableSchema_Refused",
          "[ruxd_api][server][socket][upload]") {
  // Final review #2: an upload is quick_checked, and a schema with a trigger
  // or a view is refused, before the server adopts the file.
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_upload_check");
  TempDir src("test_api_server_upload_check_src");
  const fs::path with_trigger = src.path / "trigger.rux";
  const fs::path corrupt = src.path / "corrupt.rux";
  for (const auto &file : {with_trigger, corrupt}) {
    reusex::ProjectDB db(file);
  }
  // Every page past this one is a filler row written below.
  const auto schema_bytes = fs::file_size(corrupt);
  for (const auto &file : {with_trigger, corrupt}) {
    reusex::ProjectDB db(file);
    for (int i = 0; i < 200; ++i)
      db.log_pipeline_start("filler",
                            "{\"pad\": \"" + std::string(512, 'x') + "\"}");
  }
  for (const auto &file : {with_trigger, corrupt}) {
    sqlite3 *db = nullptr;
    REQUIRE(sqlite3_open(file.string().c_str(), &db) == SQLITE_OK);
    const char *sql =
        file == with_trigger
            ? "CREATE TRIGGER evil AFTER INSERT ON pipeline_log BEGIN "
              "DELETE FROM projects; END; PRAGMA journal_mode=DELETE;"
            : "PRAGMA journal_mode=DELETE;";
    REQUIRE(sqlite3_exec(db, sql, nullptr, nullptr, nullptr) == SQLITE_OK);
    sqlite3_close(db);
  }
  {
    // Scribble over the filler rows' pages, keeping the header and the
    // schema pages readable.
    const auto size = fs::file_size(corrupt);
    REQUIRE(size > schema_bytes + 8 * 4096);
    std::fstream f(corrupt, std::ios::in | std::ios::out | std::ios::binary);
    const std::string junk(4000, '\xA5');
    for (std::uintmax_t at = schema_bytes; at + 4096 <= size; at += 4096) {
      f.seekp(static_cast<std::streamoff>(at + 50));
      f.write(junk.data(), static_cast<std::streamsize>(junk.size()));
    }
  }

  ServerOptions options = options_for(dir.path, {}, free_port());
  options.upload_limits.max_bytes = 64ull << 20;
  options.upload_limits.max_chunk_bytes = 64ull << 20;
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  const auto upload = [&](const fs::path &file) {
    const std::string bytes = slurp(file);
    const auto begun =
        connection.send_json("POST", "/api/v1/uploads",
                             R"({"name":"Mistænkelig","size":)" +
                                 std::to_string(bytes.size()) + "}");
    REQUIRE(begun.status == 201);
    const std::string id = nlohmann::json::parse(begun.body)["id"];
    REQUIRE(connection
                .send_json("PUT", "/api/v1/uploads/" + id + "?offset=0", bytes)
                .status == 200);
    return connection.send_json("POST", "/api/v1/uploads/" + id + "/complete",
                                "{}");
  };

  const auto refused_trigger = upload(with_trigger);
  INFO(refused_trigger.body);
  CHECK(refused_trigger.status == 422);
  CHECK(refused_trigger.body.find("sikkerhedshensyn") != std::string::npos);
  CHECK(refused_trigger.body.find("evil") != std::string::npos);

  const auto refused_corrupt = upload(corrupt);
  INFO(refused_corrupt.body);
  CHECK(refused_corrupt.status == 422);
  CAPTURE(refused_corrupt.status, refused_corrupt.body);
  CHECK(refused_corrupt.body.find("beskadiget") != std::string::npos);

  // Nothing was adopted, and no staged file is left behind.
  const auto list = nlohmann::json::parse(connection.get("/api/v1/cases").body);
  CHECK(list["cases"].empty());
  for (const auto &entry : fs::recursive_directory_iterator(dir.path))
    CHECK(entry.path().extension() != ".rux");
}

TEST_CASE("RunningServer_OversizedBody_RefusedBeforeItIsRead",
          "[ruxd_api][server][socket][upload]") {
  // M3: Crow buffers whole bodies. The patched parser (overlays/crow.nix,
  // CROW_MAX_REQUEST_BODY) refuses a declared body over the cap at the
  // headers, so nothing is buffered and no handler runs.
  ::unsetenv("RUX_GUI_ASSETS");
  TempPath project("test_api_server_body", ".rux");
  RunningServer server(options_for(project.path, {}, free_port()));

  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  addr.sin_port = ::htons(server.port());
  REQUIRE(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
          0);
  timeval timeout{};
  timeout.tv_sec = 10;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  const std::string head =
      "PUT /api/v1/uploads/0123456789abcdef0123456789abcdef?offset=0 "
      "HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: application/octet-stream"
      "\r\nContent-Length: 1000000000\r\n\r\n";
  ::send(fd, head.data(), head.size(), MSG_NOSIGNAL);
  std::string reply;
  char chunk[1024];
  for (;;) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0)
      break;
    reply.append(chunk, static_cast<std::size_t>(n));
  }
  ::close(fd);
  // Closed without a handler's answer (a handler would have said 404).
  CHECK(reply.find(" 404 ") == std::string::npos);
  CHECK(reply.find(" 200 ") == std::string::npos);

  // A body under the cap still reaches its handler.
  KeepAliveConnection connection(server.port());
  CHECK(connection
            .send_json("PUT",
                       "/api/v1/uploads/0123456789abcdef0123456789abcdef"
                       "?offset=0",
                       std::string(1024, 'x'))
            .status == 404);
}

TEST_CASE("RunningServer_CaseList_CarriesCardsWithoutOpeningCases",
          "[ruxd_api][server][socket][cases]") {
  // I3: the case list answers with every card's figures and opens no case;
  // deleting a case also forgets its job history (M4).
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_cases");
  {
    reusex::ProjectDB db(dir.path / "alpha.rux");
  }
  ServerOptions options = options_for(dir.path, {}, free_port());
  options.stage_executor = [](const reusex::pipeline::StageContext &) {
    return reusex::pipeline::StageResult::success("ok");
  };
  RunningServer server(std::move(options));
  KeepAliveConnection connection(server.port());

  auto list = nlohmann::json::parse(connection.get("/api/v1/cases").body);
  REQUIRE(list["cases"].size() == 1);
  CHECK(list["cases"][0]["open"] == false);
  REQUIRE(list["cases"][0]["summary"].is_object());
  CHECK(list["cases"][0]["summary"]["survey"].contains("counts"));
  // Health and the list never open a case.
  CHECK(connection.get("/api/v1/cases/alpha/health").status == 200);
  list = nlohmann::json::parse(connection.get("/api/v1/cases").body);
  CHECK(list["cases"][0]["open"] == false);

  // A case created, used and deleted leaves no history for its successor.
  REQUIRE(connection.send_json("POST", "/api/v1/cases", R"({"name":"Ny"})")
              .status == 201);
  REQUIRE(
      connection
          .send_json("POST", "/api/v1/cases/ny/jobs", R"({"stage":"planes"})")
          .status == 202);
  for (int i = 0; i < 200; ++i) {
    const auto jobs =
        nlohmann::json::parse(connection.get("/api/v1/cases/ny/jobs").body);
    if (jobs["jobs"][0]["status"] == "succeeded")
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  REQUIRE(connection.send_json("DELETE", "/api/v1/cases/ny", "").status == 204);
  REQUIRE(connection.send_json("POST", "/api/v1/cases", R"({"name":"Ny"})")
              .status == 201);
  const auto jobs =
      nlohmann::json::parse(connection.get("/api/v1/cases/ny/jobs").body);
  CHECK(jobs["jobs"].empty());
}

// ===========================================================================
// Server mode (phase S3): logins, roles, membership, the header-phase check
// ===========================================================================

namespace {

/// The cookie pair ("name=value") out of a Set-Cookie header.
std::string cookie_pair(const std::string &set_cookie) {
  return set_cookie.substr(0, set_cookie.find(';'));
}

/// What an unauthenticated request with a big declared body gets back while
/// its body has NOT been sent: the header-phase check answers it at once.
std::string reply_before_body(std::uint16_t port, const std::string &head) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  addr.sin_port = ::htons(port);
  REQUIRE(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
          0);
  timeval timeout{};
  timeout.tv_sec = 10;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::send(fd, head.data(), head.size(), MSG_NOSIGNAL);
  std::string reply;
  char chunk[1024];
  for (;;) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0)
      break;
    reply.append(chunk, static_cast<std::size_t>(n));
  }
  ::close(fd);
  return reply;
}

} // namespace

TEST_CASE("RunningServer_ServerMode_LoginRolesAndMembership",
          "[ruxd_api][server][socket][auth]") {
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_auth");
  for (const char *name : {"alpha.rux", "beta.rux"})
    reusex::ProjectDB(dir.path / name, /*readOnly=*/false);

  auto stores = in_memory_auth_stores();
  AuthOptions auth_options;
  auth_options.argon2.iterations = 1;
  auth_options.argon2.memory_kib = 256;
  auth_options.argon2.lanes = 1;
  auto auth = std::make_shared<AuthService>(stores, auth_options);
  const auto root =
      auth->create_user("root@example.dk", "Root", "hemmeligt1", true);
  const auto vera =
      auth->create_user("vera@example.dk", "Vera", "hemmeligt2", false);
  stores.members->set_role("alpha", vera.id, Role::viewer);

  ServerOptions options = options_for(dir.path, {}, free_port());
  options.auth = auth;
  RunningServer server(std::move(options));
  const std::string port = std::to_string(server.port());
  const std::string origin = "Origin: http://127.0.0.1:" + port + "\r\n";
  // A fresh connection per request: a request refused in the header phase
  // (before its body is read) closes its connection, as a browser expects.
  const auto fresh = [&] { return KeepAliveConnection(server.port()); };

  // Signed out: the page loads (the login page is part of it); the API
  // does not.
  CHECK(fresh().get("/").status == 200);
  CHECK(fresh().get("/api/v1/health").status == 200);
  CHECK(fresh().get("/api/v1/auth/me").status == 401);
  CHECK(fresh().get("/api/v1/cases").status == 401);
  CHECK(fresh().get("/api/v1/cases/alpha/project").status == 401);

  // A refused upload is answered before its body is read (header phase).
  const auto early = reply_before_body(
      server.port(),
      "PUT /api/v1/uploads/0123456789abcdef0123456789abcdef?offset=0 "
      "HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: "
      "application/octet-stream\r\nContent-Length: 50000000\r\n\r\n");
  CHECK(early.find(" 401 ") != std::string::npos);

  // Wrong password; then the right one sets an HttpOnly, SameSite=Strict
  // session cookie (not Secure: loopback).
  CHECK(fresh()
            .send_json("POST", "/api/v1/auth/login",
                       R"({"email":"vera@example.dk","password":"nej"})",
                       origin)
            .status == 401);
  const auto login = fresh().send_json(
      "POST", "/api/v1/auth/login",
      R"({"email":"Vera@example.dk","password":"hemmeligt2"})", origin);
  REQUIRE(login.status == 200);
  CHECK(login.set_cookie.rfind("ruxd_session_" + port + "=", 0) == 0);
  CHECK(login.set_cookie.find("HttpOnly") != std::string::npos);
  CHECK(login.set_cookie.find("SameSite=Strict") != std::string::npos);
  CHECK(login.set_cookie.find("Secure") == std::string::npos);
  const std::string vera_cookie =
      "Cookie: " + cookie_pair(login.set_cookie) + "\r\n";

  const auto me = fresh().get("/api/v1/auth/me", vera_cookie);
  REQUIRE(me.status == 200);
  CHECK(nlohmann::json::parse(me.body)["user"]["email"] == "vera@example.dk");
  CHECK(nlohmann::json::parse(me.body)["mode"] == "server");

  // She sees only her case, with her role.
  const auto list =
      nlohmann::json::parse(fresh().get("/api/v1/cases", vera_cookie).body);
  REQUIRE(list["cases"].size() == 1);
  CHECK(list["cases"][0]["id"] == "alpha");
  CHECK(list["cases"][0]["role"] == "viewer");

  // Viewer: reads, never writes (403); another case does not exist (404).
  CHECK(fresh().get("/api/v1/cases/alpha/project", vera_cookie).status == 200);
  CHECK(fresh().get("/api/v1/cases/beta/project", vera_cookie).status == 404);
  CHECK(fresh().get("/api/v1/cases/beta", vera_cookie).status == 404);
  CHECK(fresh().get("/api/v1/cases/nonexistent/project", vera_cookie).status ==
        404);
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases/alpha/jobs",
                       R"({"stage":"planes"})", vera_cookie + origin)
            .status == 403);
  CHECK(fresh()
            .send_json("PATCH", "/api/v1/cases/alpha", R"({"name":"x"})",
                       vera_cookie + origin)
            .status == 403);
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases/alpha/members",
                       R"({"email":"root@example.dk","role":"viewer"})",
                       vera_cookie + origin)
            .status == 403);
  CHECK(fresh().get("/api/v1/users", vera_cookie).status == 403);
  // The events socket follows membership too.
  CHECK(websocket_upgrade_status(server.port(), "/api/v1/cases/beta/events",
                                 vera_cookie)
            .find("101") == std::string::npos);
  CHECK(websocket_upgrade_status(server.port(), "/api/v1/cases/alpha/events",
                                 vera_cookie)
            .find("101") != std::string::npos);

  // The admin makes her an editor of beta; she can then write there.
  const auto root_login = fresh().send_json(
      "POST", "/api/v1/auth/login",
      R"({"email":"root@example.dk","password":"hemmeligt1"})", origin);
  REQUIRE(root_login.status == 200);
  const std::string root_cookie =
      "Cookie: " + cookie_pair(root_login.set_cookie) + "\r\n";
  // A signed-in mutation without an Origin is refused.
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases/beta/members",
                       R"({"email":"vera@example.dk","role":"editor"})",
                       root_cookie)
            .status == 403);
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases/beta/members",
                       R"({"email":"vera@example.dk","role":"editor"})",
                       root_cookie + origin)
            .status == 201);
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases/beta/members",
                       R"({"email":"nobody@example.dk","role":"editor"})",
                       root_cookie + origin)
            .status == 404);
  const auto members = nlohmann::json::parse(
      fresh().get("/api/v1/cases/beta/members", root_cookie).body);
  REQUIRE(members["members"].size() == 1);
  CHECK(members["members"][0]["role"] == "editor");
  CHECK(fresh()
            .send_json("PATCH", "/api/v1/cases/beta", R"({"name":"Beta"})",
                       vera_cookie + origin)
            .status == 200);
  // Editors cannot delete the case or manage members.
  CHECK(fresh()
            .send_json("DELETE", "/api/v1/cases/beta", "", vera_cookie + origin)
            .status == 403);
  CHECK(fresh()
            .send_json("PATCH",
                       "/api/v1/cases/beta/members/" + std::to_string(vera.id),
                       R"({"role":"owner"})", vera_cookie + origin)
            .status == 403);

  // A case the admin creates is owned by them; members can be changed and
  // the last owner cannot be removed.
  const auto created = fresh().send_json(
      "POST", "/api/v1/cases", R"({"name":"Gamma"})", root_cookie + origin);
  REQUIRE(created.status == 201);
  CHECK(stores.members->role_of("gamma", root.id) == Role::owner);
  CHECK(fresh()
            .send_json("DELETE",
                       "/api/v1/cases/gamma/members/" + std::to_string(root.id),
                       "", root_cookie + origin)
            .status == 409);

  // Every mutation that went through is in the audit log.
  const auto audit =
      std::dynamic_pointer_cast<InMemoryAuditLog>(stores.audit)->entries();
  const auto has = [&](const std::string &action) {
    return std::any_of(audit.begin(), audit.end(),
                       [&](const AuditEntry &e) { return e.action == action; });
  };
  CHECK(has("auth.login"));
  CHECK(has("auth.login_failed"));
  CHECK(has("POST /api/v1/cases/beta/members"));
  CHECK(has("PATCH /api/v1/cases/beta"));
  CHECK(has("POST /api/v1/cases"));
  CHECK_FALSE(has("PATCH /api/v1/cases/alpha")); // refused: not audited

  // Logout ends the session server-side and clears the cookie.
  const auto out = fresh().send_json("POST", "/api/v1/auth/logout", "",
                                     vera_cookie + origin);
  CHECK(out.status == 204);
  CHECK(out.set_cookie.find("Max-Age=0") != std::string::npos);
  CHECK(fresh().get("/api/v1/auth/me", vera_cookie).status == 401);
}

TEST_CASE("RunningServer_ServerMode_ApiTokenAndSuperuser",
          "[ruxd_api][server][socket][auth]") {
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_token");
  for (const char *name : {"alpha.rux", "beta.rux"})
    reusex::ProjectDB(dir.path / name, /*readOnly=*/false);
  auto stores = in_memory_auth_stores();
  AuthOptions auth_options;
  auth_options.argon2.iterations = 1;
  auth_options.argon2.memory_kib = 256;
  auth_options.argon2.lanes = 1;
  auth_options.superuser_token = "root-token-0123456789abcdefghijklmnop";
  auto auth = std::make_shared<AuthService>(stores, auth_options);
  const auto ci = auth->create_user("ci@example.dk", "CI", "hemmeligt1", false);
  stores.members->set_role("alpha", ci.id, Role::editor);
  stores.members->set_role("beta", ci.id, Role::editor);
  const auto token = auth->create_api_token(ci.id, "ci", std::string("alpha"));

  ServerOptions options = options_for(dir.path, {}, free_port());
  options.auth = auth;
  // Server mode authenticates every request: any bind goes.
  options.bind_address = "0.0.0.0";
  RunningServer server(std::move(options));
  const auto fresh = [&] { return KeepAliveConnection(server.port()); };
  const std::string bearer = "Authorization: Bearer " + token + "\r\n";

  // A scoped token: its case only, no Origin needed (no cookie to abuse).
  CHECK(fresh().get("/api/v1/cases/alpha/project", bearer).status == 200);
  CHECK(fresh().get("/api/v1/cases/beta/project", bearer).status == 404);
  CHECK(
      fresh()
          .send_json("PATCH", "/api/v1/cases/alpha", R"({"name":"A"})", bearer)
          .status == 200);
  CHECK(fresh()
            .send_json("POST", "/api/v1/cases", R"({"name":"x"})", bearer)
            .status == 403);
  CHECK(fresh()
            .get("/api/v1/cases/alpha/project",
                 "Authorization: Bearer rxt_wrong\r\n")
            .status == 401);

  // The superuser token may do everything.
  const std::string su =
      "Authorization: Bearer root-token-0123456789abcdefghijklmnop\r\n";
  CHECK(fresh().get("/api/v1/users", su).status == 200);
  CHECK(fresh().get("/api/v1/cases/beta/project", su).status == 200);
  const auto me =
      nlohmann::json::parse(fresh().get("/api/v1/auth/me", su).body);
  CHECK(me["via"] == "superuser");
}

// ===========================================================================
// Fix round 1 (S3 review): upgrade bypass, sockets, model_path, cookies
// ===========================================================================

namespace {

/// Send @p request raw and return the status code of the answer (0 when the
/// server closed without one). Reads only the head.
int raw_status(std::uint16_t port, const std::string &request) {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
  addr.sin_port = ::htons(port);
  REQUIRE(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
          0);
  timeval timeout{};
  timeout.tv_sec = 10;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  ::send(fd, request.data(), request.size(), MSG_NOSIGNAL);
  std::string reply;
  char chunk[2048];
  while (reply.find("\r\n") == std::string::npos) {
    const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
    if (n <= 0)
      break;
    reply.append(chunk, static_cast<std::size_t>(n));
  }
  ::close(fd);
  const auto space = reply.find(' ');
  if (space == std::string::npos)
    return 0;
  return std::atoi(reply.c_str() + space + 1);
}

struct ServerModeFixture {
  ServerModeFixture() {
    ::unsetenv("RUX_GUI_ASSETS");
    for (const char *name : {"alpha.rux", "beta.rux"})
      reusex::ProjectDB(dir.path / name, /*readOnly=*/false);
    AuthOptions auth_options;
    auth_options.argon2.iterations = 1;
    auth_options.argon2.memory_kib = 256;
    auth_options.argon2.lanes = 1;
    auth_options.superuser_token = superuser;
    auth = std::make_shared<AuthService>(stores, auth_options);
    root = auth->create_user("root@example.dk", "Root", "hemmeligt1", true);
    vera = auth->create_user("vera@example.dk", "Vera", "hemmeligt2", false);
    stores.members->set_role("alpha", root.id, Role::owner);
    stores.members->set_role("alpha", vera.id, Role::viewer);
    ServerOptions options = options_for(dir.path, {}, free_port());
    options.auth = auth;
    server = std::make_unique<RunningServer>(std::move(options));
  }
  std::string login(const std::string &email, const std::string &password) {
    KeepAliveConnection c(server->port());
    const auto r = c.send_json("POST", "/api/v1/auth/login",
                               R"({"email":")" + email + R"(","password":")" +
                                   password + R"("})",
                               origin());
    REQUIRE(r.status == 200);
    return "Cookie: " + r.set_cookie.substr(0, r.set_cookie.find(';')) + "\r\n";
  }
  std::string origin() const {
    return "Origin: http://127.0.0.1:" + std::to_string(server->port()) +
           "\r\n";
  }
  const std::string superuser = "superuser-token-0123456789abcdefghij";
  TempDir dir{"test_api_server_fix1"};
  AuthStores stores = in_memory_auth_stores();
  std::shared_ptr<AuthService> auth;
  User root, vera;
  std::unique_ptr<RunningServer> server;
};

} // namespace

TEST_CASE("RunningServer_UpgradeHeaderOutsideTheUpgradePath_IsAuthenticated",
          "[ruxd_api][server][socket][auth]") {
  // Review C1: Crow takes its WebSocket path only for HTTP/1.1. An HTTP/1.0
  // request (or an h2c one) with an Upgrade header reaches the ordinary
  // handler — and the middleware used to wave every `req.upgrade` through.
  ServerModeFixture f;
  const auto port = f.server->port();
  const std::string vid = std::to_string(f.vera.id);

  // HTTP/1.0 + Upgrade: anonymous reads, deletes and member removal refused.
  CHECK(raw_status(port, "GET /api/v1/cases HTTP/1.0\r\nHost: 127.0.0.1\r\n"
                         "Upgrade: x\r\n\r\n") == 401);
  CHECK(raw_status(port, "GET /api/v1/cases/alpha/project HTTP/1.0\r\n"
                         "Host: 127.0.0.1\r\nUpgrade: x\r\n\r\n") == 401);
  CHECK(raw_status(port, "DELETE /api/v1/cases/alpha HTTP/1.0\r\n"
                         "Host: 127.0.0.1\r\nUpgrade: x\r\n\r\n") == 401);
  CHECK(raw_status(port, "DELETE /api/v1/cases/alpha/members/" + vid +
                             " HTTP/1.0\r\nHost: 127.0.0.1\r\nUpgrade: "
                             "websocket\r\nConnection: Upgrade\r\n\r\n") ==
        401);
  // HTTP/1.0 needs no Host — the Host check refuses it.
  CHECK(raw_status(port, "GET /api/v1/cases HTTP/1.0\r\nUpgrade: x\r\n\r\n") ==
        403);
  // An h2c upgrade, which Crow ignores and serves normally: evaluated.
  CHECK(raw_status(port, "GET /api/v1/cases/alpha/project HTTP/1.1\r\n"
                         "Host: 127.0.0.1\r\nUpgrade: h2c\r\nConnection: "
                         "Upgrade, HTTP2-Settings\r\n\r\n") == 401);
  // HTTP/1.1 with Upgrade but no `Connection: upgrade`: whichever way the
  // parser classifies it — Crow's upgrade path (an ordinary route answers
  // 404 without running its handler) or an ordinary request (evaluated:
  // 401; Crow's upgrade path has handed the socket away and just closes
  // it: 0) — no handler runs for an anonymous caller.
  for (const std::string &request : std::vector<std::string>{
           "GET /api/v1/cases/alpha/project HTTP/1.1\r\nHost: 127.0.0.1\r\n"
           "Upgrade: x\r\n\r\n",
           "DELETE /api/v1/cases/alpha HTTP/1.1\r\nHost: 127.0.0.1\r\n"
           "Upgrade: x\r\n\r\n",
           "DELETE /api/v1/cases/alpha/members/" + vid +
               " HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: x\r\n\r\n"}) {
    INFO(request);
    const int status = raw_status(port, request);
    CHECK((status == 0 || status == 401 || status == 404));
  }

  // Nothing happened: the case and the membership are still there.
  CHECK(f.stores.members->role_of("alpha", f.vera.id) == Role::viewer);
  KeepAliveConnection c(port);
  const auto list = nlohmann::json::parse(
      c.get("/api/v1/cases", "Authorization: Bearer " + f.superuser + "\r\n")
          .body);
  CHECK(list["cases"].size() == 2);
}

TEST_CASE("RunningServer_DuplicateHostHeader_Is400",
          "[ruxd_api][server][socket][auth]") {
  // Final review #4: with two Host fields Crow returned one at random, so an
  // evil Host first and a good one second could pass the Host check.
  ServerModeFixture f;
  const auto port = f.server->port();
  const std::string bearer = "Authorization: Bearer " + f.superuser + "\r\n";
  CHECK(raw_status(port, "GET /api/v1/cases HTTP/1.1\r\nHost: 127.0.0.1\r\n" +
                             bearer + "Connection: close\r\n\r\n") == 200);
  for (const std::string &hosts :
       std::vector<std::string>{"Host: evil.example\r\nHost: 127.0.0.1\r\n",
                                "Host: 127.0.0.1\r\nHost: evil.example\r\n",
                                "Host: 127.0.0.1\r\nhost: 127.0.0.1\r\n"}) {
    INFO(hosts);
    CHECK(raw_status(port, "GET /api/v1/cases HTTP/1.1\r\n" + hosts + bearer +
                               "Connection: close\r\n\r\n") == 400);
    CHECK(raw_status(port, "DELETE /api/v1/cases/alpha HTTP/1.1\r\n" + hosts +
                               bearer + "Connection: close\r\n\r\n") == 400);
  }
  KeepAliveConnection c(port);
  CHECK(nlohmann::json::parse(c.get("/api/v1/cases", bearer).body)["cases"]
            .size() == 2);
}

TEST_CASE("RunningServer_EventsSocket_ClosedWhenAccessEnds",
          "[ruxd_api][server][socket][auth]") {
  // Review I2: logout and losing membership close the events sockets they
  // opened, instead of leaving them streaming the case.
  ServerModeFixture f;
  const auto port = f.server->port();
  const auto vera = f.login("vera@example.dk", "hemmeligt2");
  const auto root = f.login("root@example.dk", "hemmeligt1");

  SECTION("removed from the case") {
    WebSocketClient ws(port, "/api/v1/cases/alpha/events", vera);
    CHECK(nlohmann::json::parse(ws.next_text())["type"] == "hello");
    KeepAliveConnection c(port);
    CHECK(
        c.send_json("DELETE",
                    "/api/v1/cases/alpha/members/" + std::to_string(f.vera.id),
                    "", root + f.origin())
            .status == 204);
    CHECK(ws.closed_by_server());
  }
  SECTION("logged out") {
    WebSocketClient ws(port, "/api/v1/cases/alpha/events", vera);
    CHECK(nlohmann::json::parse(ws.next_text())["type"] == "hello");
    KeepAliveConnection c(port);
    CHECK(c.send_json("POST", "/api/v1/auth/logout", "", vera + f.origin())
              .status == 204);
    CHECK(ws.closed_by_server());
  }
  SECTION("someone else's change leaves it open") {
    WebSocketClient ws(port, "/api/v1/cases/alpha/events", vera);
    CHECK(nlohmann::json::parse(ws.next_text())["type"] == "hello");
    KeepAliveConnection c(port);
    CHECK(c.send_json("POST", "/api/v1/auth/logout", "", root + f.origin())
              .status == 204);
    ws.set_timeout(1);
    CHECK_FALSE(ws.closed_by_server());
  }
}

TEST_CASE("RunningServer_ServerMode_ModelPathAndTokensAndCookies",
          "[ruxd_api][server][socket][auth]") {
  ServerModeFixture f;
  const auto port = f.server->port();
  const auto root = f.login("root@example.dk", "hemmeligt1");
  KeepAliveConnection c(port);

  // Review I3: no client-chosen model file in server mode.
  CHECK(c.send_json("POST", "/api/v1/cases/alpha/frames/1/segment",
                    R"({"model_path":"/etc/passwd","prompts":[{"text":"x"}]})",
                    root + f.origin())
            .status == 400);

  // Review I5: API tokens — create (shown once), list (never the token),
  // revoke.
  const auto made =
      c.send_json("POST", "/api/v1/auth/tokens",
                  R"({"name":"ci","expires_days":7})", root + f.origin());
  REQUIRE(made.status == 201);
  const auto token = nlohmann::json::parse(made.body);
  CHECK(token["token"].get<std::string>().rfind("rxt_", 0) == 0);
  CHECK_FALSE(token["expires_at"].is_null());
  const auto bearer =
      "Authorization: Bearer " + token["token"].get<std::string>() + "\r\n";
  CHECK(c.get("/api/v1/cases/alpha/project", bearer).status == 200);
  const auto listed =
      nlohmann::json::parse(c.get("/api/v1/auth/tokens", root).body);
  REQUIRE(listed["tokens"].size() == 1);
  CHECK_FALSE(listed["tokens"][0].contains("token"));
  // A token cannot mint tokens.
  CHECK(c.get("/api/v1/auth/tokens", bearer).status == 403);
  CHECK(c.send_json("DELETE",
                    "/api/v1/auth/tokens/" +
                        std::to_string(token["id"].get<int>()),
                    "", root + f.origin())
            .status == 204);
  CHECK(c.get("/api/v1/cases/alpha/project", bearer).status == 401);

  // Review M2: an owner only finds people they already work with.
  const auto anna =
      f.auth->create_user("anna@example.dk", "Anna", "hemmeligt3", false);
  f.stores.members->set_role("beta", anna.id, Role::owner);
  const auto anna_cookie = f.login("anna@example.dk", "hemmeligt3");
  KeepAliveConnection a(port);
  CHECK(a.send_json("POST", "/api/v1/cases/beta/members",
                    R"({"email":"vera@example.dk","role":"viewer"})",
                    anna_cookie + f.origin())
            .status == 404);
  CHECK(a.send_json("POST", "/api/v1/cases/beta/members",
                    R"({"email":"nobody@example.dk","role":"viewer"})",
                    anna_cookie + f.origin())
            .status == 404);
  CHECK(c.send_json("POST", "/api/v1/cases/beta/members",
                    R"({"email":"vera@example.dk","role":"viewer"})",
                    root + f.origin())
            .status == 201); // an administrator finds anyone
}

TEST_CASE("RunningServer_SecureCookie_UsesTheHostPrefix",
          "[ruxd_api][server][socket][auth]") {
  ::unsetenv("RUX_GUI_ASSETS");
  TempDir dir("test_api_server_host_cookie");
  reusex::ProjectDB(dir.path / "alpha.rux", /*readOnly=*/false);
  auto stores = in_memory_auth_stores();
  AuthOptions auth_options;
  auth_options.argon2.iterations = 1;
  auth_options.argon2.memory_kib = 256;
  auth_options.argon2.lanes = 1;
  auto auth = std::make_shared<AuthService>(stores, auth_options);
  auth->create_user("u@example.dk", "U", "hemmeligt1", true);
  ServerOptions options = options_for(dir.path, {}, free_port());
  options.auth = auth;
  options.cookie_secure = CookieSecure::always;
  RunningServer server(std::move(options));
  KeepAliveConnection c(server.port());
  const auto origin =
      "Origin: http://127.0.0.1:" + std::to_string(server.port()) + "\r\n";
  const auto r = c.send_json(
      "POST", "/api/v1/auth/login",
      R"({"email":"u@example.dk","password":"hemmeligt1"})", origin);
  REQUIRE(r.status == 200);
  CHECK(r.set_cookie.rfind(
            "__Host-ruxd_session_" + std::to_string(server.port()) + "=", 0) ==
        0);
  CHECK(r.set_cookie.find("; Secure") != std::string::npos);
  CHECK(r.set_cookie.find("Path=/") != std::string::npos);
  CHECK(r.set_cookie.find("Domain") == std::string::npos);
  const auto cookie =
      "Cookie: " + r.set_cookie.substr(0, r.set_cookie.find(';')) + "\r\n";
  CHECK(c.get("/api/v1/auth/me", cookie).status == 200);
}
