// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Static-asset serving for `rux gui`.
//
// The frontend bundle is a *data asset installed next to the binary*, never
// embedded in it (issue #265): Phase 2 builds it with Nix into
// `share/reusex/gui/` and the server picks it up from there. Until that bundle
// exists, a built-in placeholder page stands in so the server is usable and
// self-describing on day one.

#include <filesystem>
#include <string>
#include <string_view>

namespace rux::gui {

/// Locate the frontend asset directory, in precedence order:
///   1. @p override_dir, when non-empty (from `--assets`)
///   2. `$RUX_GUI_ASSETS`
///   3. `<dir of the running executable>/../share/reusex/gui`
/// Returns an empty path when none of them exists, in which case the server
/// serves placeholder_page() instead.
///
/// @throws std::runtime_error when @p override_dir is given but is not a
///         directory — an explicit flag that silently does nothing would be a
///         silent failure (STANDARDS §5).
std::filesystem::path
resolve_asset_dir(const std::filesystem::path &override_dir);

/// Directory of the running executable, or an empty path if it cannot be
/// determined.
std::filesystem::path executable_dir();

/// Map a request path onto a file inside @p root.
///
/// Returns an empty path when the resolved location escapes @p root (via `..`,
/// a symlink, or an absolute component) or does not name an existing regular
/// file. A request for "/" resolves to `index.html`.
std::filesystem::path resolve_asset(const std::filesystem::path &root,
                                    std::string_view url_path);

/// Content type for a file, by extension. Falls back to
/// "application/octet-stream".
std::string mime_type_for(const std::filesystem::path &path);

/// The built-in landing page served when no asset bundle is installed.
///
/// It is a real, functioning page — it calls the API it documents — so that
/// `rux gui` is verifiable end to end before any frontend exists.
std::string placeholder_page(std::string_view project_name);

} // namespace rux::gui
