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
#include <optional>
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
/// With @p resource_template_id the PDF also gets a Ressourcetabel: every
/// non-rejected resource through that template's keys, in tables of at
/// most 8 columns each led by Betegnelse (core::resource_report_section).
///
/// @returns Raw PDF bytes on success.
/// @throws std::out_of_range when @p resource_template_id names no
///         template (checked before any typst work).
/// @throws std::runtime_error if typst is not in PATH, data assembly fails,
///         or the compilation exits non-zero (the error text from typst is
///         included in the message).
std::vector<std::uint8_t> generate_ressourcekortlaegning_pdf(
    ProjectDB &db,
    std::optional<std::int64_t> resource_template_id = std::nullopt);

/// How many survey types keep the report from being complete right now: the
/// length of core::fractions_by_eak's blocking list (review, sample or mass).
/// Stored with each PDF version (ProjectDB::add_report_pdf) so the GUI can
/// tell a complete version from a draft.
int report_blocking_types(const ProjectDB &db);

} // namespace reusex
