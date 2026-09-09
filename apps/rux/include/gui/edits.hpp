// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The mutating half of the GUI API (#265, Phase 4).
//
// Kept apart from gui/api.hpp deliberately. Everything there takes a
// `const ProjectDB &` and is a pure function of the stored project; everything
// here takes a writable one and changes it. That split is the reason the
// read side can keep its "open a fresh read-only connection per request"
// guarantee without qualification.
//
// WRITER DISCIPLINE. `rux gui` has exactly two writers: the pipeline job
// worker and these handlers. They exclude each other through the project's
// writer lock, which the JobRunner owns
// (pipeline::JobRunner::try_acquire_writer) and holds for the whole of every
// stage. A handler that cannot take the lock answers 503 rather than waiting,
// and a handler asked to write while a job is queued or running answers 409
// rather than interleaving with it.
//
// This matters beyond tidiness: both edits below are read-modify-writes.
// Renaming one label class rewrites the cloud's entire label map, because
// ProjectDB::save_label_definitions replaces it wholesale. Two unsynchronised
// writers would lose one of the renames with no error anywhere.

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace reusex {
class ProjectDB;
}

namespace rux::gui {

/// Rename label classes of one Label cloud.
///
/// @param body `{"labels": {"<id>": "<name>", ...}}` — a sparse patch. Ids
///        absent from it keep the names they had.
/// @return The full legend after the rename, as cloud_labels_json() returns it.
///
/// @throws HttpError(404) when @p name is not a cloud in this project.
/// @throws HttpError(400) on a malformed body, an empty name, or an id that is
///         not already in the legend — this renames classes, it does not
///         create them.
/// @throws HttpError(409) for the `instances` cloud, whose names encode the
///         semantic class and instance id in a form the v10 migration and
///         io/export_scene.cpp parse back out.
nlohmann::json patch_cloud_labels(reusex::ProjectDB &db,
                                  const std::string &name,
                                  const std::string &body);

/// Add, change or clear properties of one material passport.
///
/// @param body `{"properties": {"<name>": "<value>"|null, ...}}` — a sparse
///        patch; `null` deletes. Properties absent from it are untouched.
/// @return The passport after the edit, as material_json() returns it.
///
/// @throws HttpError(404) when no passport carries @p guid.
/// @throws HttpError(400) on a malformed body, an empty property name, or a
///         value that is neither a string nor null.
nlohmann::json patch_material(reusex::ProjectDB &db, const std::string &guid,
                              const std::string &body);

} // namespace rux::gui
