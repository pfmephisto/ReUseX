// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui/assets.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <system_error>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace rux::gui {
namespace {

const std::map<std::string, std::string> &mime_table() {
  static const std::map<std::string, std::string> table{
      {".html", "text/html; charset=utf-8"},
      {".htm", "text/html; charset=utf-8"},
      {".js", "text/javascript; charset=utf-8"},
      {".mjs", "text/javascript; charset=utf-8"},
      {".css", "text/css; charset=utf-8"},
      {".json", "application/json"},
      {".map", "application/json"},
      {".svg", "image/svg+xml"},
      {".png", "image/png"},
      {".jpg", "image/jpeg"},
      {".jpeg", "image/jpeg"},
      {".webp", "image/webp"},
      {".ico", "image/x-icon"},
      {".woff", "font/woff"},
      {".woff2", "font/woff2"},
      {".ttf", "font/ttf"},
      {".wasm", "application/wasm"},
      {".txt", "text/plain; charset=utf-8"},
      {".glb", "model/gltf-binary"},
      {".gltf", "model/gltf+json"},
  };
  return table;
}

} // namespace

std::filesystem::path executable_dir() {
#if defined(__linux__)
  std::error_code ec;
  auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (!ec)
    return exe.parent_path();
#endif
  return {};
}

std::filesystem::path
resolve_asset_dir(const std::filesystem::path &override_dir) {
  std::error_code ec;

  if (!override_dir.empty()) {
    if (!std::filesystem::is_directory(override_dir, ec))
      throw std::runtime_error(fmt::format("--assets '{}' is not a directory",
                                           override_dir.string()));
    return std::filesystem::weakly_canonical(override_dir, ec);
  }

  if (const char *env = std::getenv("RUX_GUI_ASSETS");
      env != nullptr && *env != '\0') {
    std::filesystem::path candidate(env);
    if (std::filesystem::is_directory(candidate, ec))
      return std::filesystem::weakly_canonical(candidate, ec);
  }

  // Installed layout: bin/rux next to share/reusex/gui/.
  const auto bin = executable_dir();
  if (!bin.empty()) {
    const auto candidate = bin.parent_path() / "share" / "reusex" / "gui";
    if (std::filesystem::is_directory(candidate, ec))
      return std::filesystem::weakly_canonical(candidate, ec);
  }

  return {};
}

std::string percent_decode(std::string_view text) {
  auto hex_value = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };

  std::string out;
  out.reserve(text.size());
  for (size_t i = 0; i < text.size(); ++i) {
    if (text[i] != '%' || i + 2 >= text.size()) {
      out.push_back(text[i]);
      continue;
    }
    const int hi = hex_value(text[i + 1]);
    const int lo = hex_value(text[i + 2]);
    if (hi < 0 || lo < 0) {
      out.push_back(text[i]); // Malformed escape: leave it alone.
      continue;
    }
    out.push_back(static_cast<char>(hi * 16 + lo));
    i += 2;
  }
  return out;
}

bool looks_like_spa_route(std::string_view url_path) {
  std::string path(url_path);
  if (const auto query = path.find('?'); query != std::string::npos)
    path.erase(query);
  const auto slash = path.find_last_of('/');
  const std::string last =
      slash == std::string::npos ? path : path.substr(slash + 1);
  // No final segment at all ("/", "/projects/") is a route; a segment with a
  // dot in it ("app.js", "favicon.ico") is a file request.
  if (last.empty())
    return true;
  return last.find('.') == std::string::npos;
}

std::filesystem::path resolve_asset(const std::filesystem::path &root,
                                    std::string_view url_path) {
  if (root.empty())
    return {};

  std::string relative(url_path);
  // Strip the query string first, then decode: decoding first would let an
  // encoded '?' change where the path ends.
  if (const auto query = relative.find('?'); query != std::string::npos)
    relative.erase(query);
  relative = percent_decode(relative);
  // A decoded backslash is a separator on some platforms and a plain character
  // here; normalise it so the traversal check below sees the same components
  // the filesystem would.
  std::replace(relative.begin(), relative.end(), '\\', '/');
  while (!relative.empty() && relative.front() == '/')
    relative.erase(relative.begin());
  if (relative.empty())
    relative = "index.html";

  // Reject absolute or parent components before touching the filesystem: an
  // SPA server that can be talked into serving /etc/passwd is a security bug,
  // not a convenience.
  const std::filesystem::path requested(relative);
  if (requested.is_absolute())
    return {};
  for (const auto &part : requested)
    if (part == "..")
      return {};

  std::error_code ec;
  const auto canonical_root = std::filesystem::weakly_canonical(root, ec);
  if (ec)
    return {};
  auto candidate =
      std::filesystem::weakly_canonical(canonical_root / requested, ec);
  if (ec)
    return {};

  // Belt and braces: re-check containment after canonicalization, which is
  // what catches a symlink pointing outside the bundle.
  const auto root_str = canonical_root.string();
  const auto candidate_str = candidate.string();
  if (candidate_str.compare(0, root_str.size(), root_str) != 0)
    return {};
  if (candidate_str.size() > root_str.size() &&
      candidate_str[root_str.size()] !=
          std::filesystem::path::preferred_separator)
    return {};

  if (!std::filesystem::is_regular_file(candidate, ec))
    return {};
  return candidate;
}

std::string mime_type_for(const std::filesystem::path &path) {
  auto extension = path.extension().string();
  for (char &c : extension)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  const auto &table = mime_table();
  auto it = table.find(extension);
  return it == table.end() ? "application/octet-stream" : it->second;
}

std::string placeholder_page(std::string_view project_name) {
  // Deliberately dependency-free and unstyled beyond the basics: this page is a
  // placeholder for the Phase 2 React bundle, not a design artefact. Claude
  // Design owns the visual system (issue #265); anything pretty written here
  // would only have to be thrown away.
  //
  // Substitution is a plain sentinel replace rather than fmt::format: the
  // template is mostly CSS and JavaScript, i.e. mostly braces, and escaping
  // every one of them for fmt is a bug waiting to happen.
  static constexpr std::string_view kTemplate = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ReUseX &mdash; %%PROJECT%%</title>
<style>
  :root { color-scheme: light dark; }
  body {
    font: 15px/1.6 ui-sans-serif, system-ui, sans-serif;
    margin: 0; padding: 2.5rem 1.5rem; max-width: 60rem;
    margin-inline: auto;
  }
  h1 { font-size: 1.5rem; margin: 0 0 .25rem; }
  .sub { opacity: .65; margin: 0 0 2rem; }
  section { margin-bottom: 2rem; }
  h2 { font-size: .8rem; text-transform: uppercase; letter-spacing: .08em;
       opacity: .6; margin: 0 0 .6rem; }
  pre { background: rgba(127,127,127,.12); padding: .9rem 1rem;
        border-radius: 6px; overflow-x: auto; font-size: 13px; margin: 0; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  td { padding: .3rem .6rem .3rem 0; vertical-align: top;
       border-bottom: 1px solid rgba(127,127,127,.2); }
  td.m { font-family: ui-monospace, monospace; white-space: nowrap;
         opacity: .7; width: 4rem; }
  td.p { font-family: ui-monospace, monospace; white-space: nowrap; }
  a { color: inherit; }
  .note { font-size: 13px; opacity: .7; }
</style>
</head>
<body>
<h1>ReUseX</h1>
<p class="sub">%%PROJECT%% &middot; API placeholder page</p>

<p class="note">
  No frontend bundle is installed, so the server is serving this built-in page.
  The REST + WebSocket contract below is live and complete
  (<code>docs/gui/openapi.yaml</code>). The React frontend that replaces this
  page is Phase 2 of
  <a href="https://github.com/pfmephisto/ReUseX/issues/265">issue #265</a>.
</p>

<section>
  <h2>Health</h2>
  <pre id="health">loading&hellip;</pre>
</section>

<section>
  <h2>Project summary</h2>
  <pre id="summary">loading&hellip;</pre>
</section>

<section>
  <h2>Endpoints</h2>
  <table id="endpoints"><tbody></tbody></table>
</section>

<section>
  <h2>Event stream</h2>
  <pre id="events">connecting&hellip;</pre>
</section>

<script>
const base = '/api/v1';

async function show(id, path) {
  const el = document.getElementById(id);
  try {
    const res = await fetch(base + path);
    el.textContent = JSON.stringify(await res.json(), null, 2);
  } catch (err) {
    el.textContent = 'request failed: ' + err;
  }
}

show('health', '/health');
show('summary', '/project');

fetch(base + '/endpoints')
  .then(r => r.json())
  .then(data => {
    const body = document.querySelector('#endpoints tbody');
    for (const e of data.endpoints) {
      const row = document.createElement('tr');
      row.innerHTML =
        '<td class="m">' + e.method + '</td>' +
        '<td class="p">' + e.path + '</td>' +
        '<td>' + e.summary + '</td>';
      body.appendChild(row);
    }
  })
  .catch(err => {
    document.querySelector('#endpoints').textContent = 'failed: ' + err;
  });

const log = document.getElementById('events');
const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
const socket = new WebSocket(proto + '//' + location.host + base + '/events');
let lines = [];
socket.onopen = () => { log.textContent = 'connected, waiting for events…'; };
socket.onmessage = (ev) => {
  lines.push(ev.data);
  if (lines.length > 20) lines.shift();
  log.textContent = lines.join('\n');
};
socket.onclose = () => { log.textContent += '\n[disconnected]'; };
socket.onerror = () => { log.textContent = 'websocket error'; };
</script>
</body>
</html>
)HTML";
  static constexpr std::string_view kSentinel = "%%PROJECT%%";

  std::string page(kTemplate);
  for (size_t at = page.find(kSentinel); at != std::string::npos;
       at = page.find(kSentinel, at + project_name.size()))
    page.replace(at, kSentinel.size(), project_name);
  return page;
}

} // namespace rux::gui
