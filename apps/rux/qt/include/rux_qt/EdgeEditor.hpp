// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The pose-graph edits of the Database workspace: staged in memory
// (PendingEdgeEdits, rux_qt_core) until "Gem ændringer", then written in ONE
// ProjectDB transaction on the session's GUI-thread connection — deletions
// first, then additions. A failed save keeps every edit pending and says why
// in Danish (read-only, locked by another process, other).

#include <rux_qt/database_logic.hpp>

#include <QObject>
#include <QString>

namespace rux::qt {

class ProjectSession;

class EdgeEditor : public QObject {
  Q_OBJECT
    public:
  explicit EdgeEditor(ProjectSession &session, QObject *parent = nullptr);

  const PendingEdgeEdits &edits() const { return edits_; }
  int pending() const { return edits_.count(); }

  /// A project is open with write access. Edits can be staged anyway; only
  /// saving needs it.
  bool can_save() const;
  /// Why the project cannot be written (Danish), or empty.
  QString read_only_reason() const;

  PendingEdgeEdits::Result add(const EdgeRecord &edge);
  PendingEdgeEdits::Result remove(const EdgeKey &key);
  void discard();

  /// Write every pending edit in one transaction. False on failure, with
  /// last_error() (Danish) and last_error_detail() (the raw message).
  bool save();
  QString last_error() const { return error_; }
  QString last_error_detail() const { return error_detail_; }

  /// Re-read the stored edges (after an open or a reload). Drops pending
  /// edits — callers ask first.
  void reload();

    signals:
  /// The stored or pending edges changed.
  void changed();
  /// A save wrote @p count edits.
  void saved(int count);

    private:
  ProjectSession &session_;
  PendingEdgeEdits edits_;
  QString error_;
  QString error_detail_;
};

} // namespace rux::qt
