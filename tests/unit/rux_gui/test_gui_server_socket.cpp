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

#include "../../support/temp_path.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
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

  Response get(std::string_view path) {
    const std::string request = "GET " + std::string(path) +
                                " HTTP/1.1\r\nHost: 127.0.0.1\r\n"
                                "Connection: keep-alive\r\n\r\n";
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

    private:
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
