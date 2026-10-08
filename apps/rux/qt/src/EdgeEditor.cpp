// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/ProjectSession.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QApplication>
#include <QCursor>

namespace rux::qt {

EdgeEditor::EdgeEditor(ProjectSession &session, QObject *parent)
    : QObject(parent), session_(session) {
  connect(&session_, &ProjectSession::opened, this, &EdgeEditor::reload);
  connect(&session_, &ProjectSession::closed, this, &EdgeEditor::reload);
}

bool EdgeEditor::can_save() const {
  return session_.is_open() && !session_.is_read_only();
}

QString EdgeEditor::read_only_reason() const {
  if (!session_.is_open() || !session_.is_read_only())
    return {};
  return QString::fromStdString(write_error_da(WriteErrorKind::read_only));
}

PendingEdgeEdits::Result EdgeEditor::add(const EdgeRecord &edge) {
  const auto r = edits_.add(edge);
  error_.clear();
  emit changed();
  return r;
}

PendingEdgeEdits::Result EdgeEditor::remove(const EdgeKey &key) {
  const auto r = edits_.remove(key);
  error_.clear();
  emit changed();
  return r;
}

void EdgeEditor::discard() {
  edits_.discard();
  error_.clear();
  error_detail_.clear();
  emit changed();
}

bool EdgeEditor::save() {
  reusex::ProjectDB *db = session_.db();
  if (!db || edits_.empty())
    return edits_.empty();
  error_.clear();
  error_detail_.clear();
  if (session_.is_read_only()) {
    error_ = QString::fromStdString(write_error_da(WriteErrorKind::read_only));
    error_detail_ = session_.read_only_reason();
    emit changed();
    return false;
  }

  const auto ops = edits_.ops();
  // BEGIN IMMEDIATE waits out the 5 s busy timeout on a locked project; the
  // window shows a wait cursor meanwhile (writes stay on the GUI thread's
  // connection, the one the session owns).
  QApplication::setOverrideCursor(Qt::WaitCursor);
  try {
    reusex::ProjectDB::Transaction tx(*db);
    for (const auto &op : ops) {
      const auto &e = op.edge;
      if (op.kind == PendingEdgeEdits::Op::Kind::remove) {
        db->delete_pose_graph_edges(e.key.from, e.key.to, e.key.type);
      } else {
        reusex::ProjectDB::PoseGraphEdge row;
        row.from_node_id = e.key.from;
        row.to_node_id = e.key.to;
        row.edge_type = e.key.type;
        row.residual = e.residual;
        row.weight = e.weight;
        db->add_pose_graph_edge(row);
      }
    }
    tx.commit();
  } catch (const std::exception &ex) {
    QApplication::restoreOverrideCursor();
    error_ =
        QString::fromStdString(write_error_da(classify_write_error(ex.what())));
    error_detail_ = QString::fromUtf8(ex.what());
    emit changed();
    return false;
  }
  QApplication::restoreOverrideCursor();
  const int n = static_cast<int>(ops.size());
  reload(); // the stored rows, exactly as the file now has them
  emit saved(n);
  return true;
}

void EdgeEditor::reload() {
  error_.clear();
  error_detail_.clear();
  std::vector<EdgeRecord> base;
  if (reusex::ProjectDB *db = session_.db()) {
    try {
      for (const auto &e : db->list_pose_graph_edges())
        base.push_back({{e.from_node_id, e.to_node_id, e.edge_type},
                        e.residual,
                        e.weight});
    } catch (const std::exception &ex) {
      error_ = "Posegrafens kanter kunne ikke læses.";
      error_detail_ = QString::fromUtf8(ex.what());
    }
  }
  edits_.set_base(std::move(base));
  emit changed();
}

} // namespace rux::qt
