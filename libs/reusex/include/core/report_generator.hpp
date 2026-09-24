// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Server-side PDF generation for Ressourcekortlægning (#456).
//
// The generator assembles material passports, column definitions and thumbnail
// blobs from a ProjectDB, writes them to a temporary directory alongside the
// embedded Typst template, and invokes `typst compile` as a subprocess. The
// resulting PDF bytes are returned so the caller can store them in the DB and
// serve them over HTTP.
//
// Requires `typst` to be present in PATH at runtime. The devshell gains it via
// `pkgs.typst` in shell.nix; in production supply it in the process's PATH.

#include <cstdint>
#include <vector>

namespace reusex {

class ProjectDB;

/// Generate a Ressourcekortlægning PDF for the given project.
///
/// Assembles material passports, user-defined column definitions and
/// thumbnail blobs from @p db into a temporary directory, writes the
/// Typst template there, and executes:
///   typst compile report.typ out.pdf --root <tmpdir>
///
/// @returns Raw PDF bytes on success.
/// @throws std::runtime_error if typst is not in PATH, data assembly fails,
///         or the compilation exits non-zero (the error text from typst is
///         included in the message).
std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(ProjectDB &db);

} // namespace reusex
