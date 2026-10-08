// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/ProjectSession.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <rux_qt/background.hpp>

#include <QApplication>
#include <QDialog>
#include <QLabel>
#include <QPointer>
#include <QVBoxLayout>

#include <memory>
#include <thread>

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
  if (!session_.is_open() || edits_.empty() || saving_)
    return false;
  error_.clear();
  error_detail_.clear();
  if (session_.is_read_only()) {
    error_ = QString::fromStdString(write_error_da(WriteErrorKind::read_only));
    error_detail_ = session_.read_only_reason();
    emit changed();
    emit save_finished(false);
    return false;
  }

  const auto snapshot = edits_.ops();
  const unsigned id = ++save_id_;
  saving_ = true;
  emit changed();

  const std::string path = session_.path().toStdString();
  auto work = std::make_shared<BackgroundWork>();
  QPointer<EdgeEditor> guard(this);
  std::thread([path, snapshot, id, guard, work]() mutable {
    bool ok = false;
    QString what;
    try {
      reusex::ProjectDB db(path, /*readOnly=*/false);
      reusex::ProjectDB::Transaction tx(db);
      for (const auto &op : snapshot) {
        const auto &e = op.edge;
        if (op.kind == PendingEdgeEdits::Op::Kind::remove) {
          db.delete_pose_graph_edges(e.key.from, e.key.to, e.key.type);
        } else {
          reusex::ProjectDB::PoseGraphEdge row;
          row.from_node_id = e.key.from;
          row.to_node_id = e.key.to;
          row.edge_type = e.key.type;
          row.residual = e.residual;
          row.weight = e.weight;
          db.add_pose_graph_edge(row);
        }
      }
      tx.commit();
      ok = true;
    } catch (const std::exception &ex) {
      what = QString::fromUtf8(ex.what());
    }
    QMetaObject::invokeMethod(
        qApp,
        [guard, id, ok, what, snapshot] {
          if (guard)
            guard->save_done(id, ok, what, snapshot);
        },
        Qt::QueuedConnection);
    work.reset(); // last: counted until the thread has let go of everything
  }).detach();
  return true;
}

void EdgeEditor::save_done(unsigned id, bool ok, const QString &what,
                           const std::vector<PendingEdgeEdits::Op> &snapshot) {
  if (id != save_id_ || !saving_)
    return; // a reload or a new project dropped this save's state
  saving_ = false;
  if (!ok) {
    error_ = QString::fromStdString(
        write_error_da(classify_write_error(what.toStdString())));
    error_detail_ = what;
    emit changed();
    emit save_finished(false);
    return;
  }
  edits_.commit_saved(snapshot);
  emit changed();
  emit saved(static_cast<int>(snapshot.size()));
  emit save_finished(true);
}

bool EdgeEditor::save_and_wait(QWidget *parent) {
  if (!saving_ && !save())
    return edits_.empty();
  QDialog wait(parent);
  wait.setObjectName("savingDialog");
  wait.setWindowTitle("Gemmer");
  wait.setModal(true);
  auto *l = new QVBoxLayout(&wait);
  auto *text = new QLabel("Gemmer ændringer i posegrafen …");
  text->setObjectName("emptyText");
  l->addWidget(text);
  bool ok = false;
  connect(this, &EdgeEditor::save_finished, &wait, [&](bool result) {
    ok = result;
    wait.accept();
  });
  // A save can finish before exec() starts its loop: the queued delivery
  // only runs inside it, so this cannot miss the signal.
  wait.exec();
  return ok && edits_.empty();
}

void EdgeEditor::reload() {
  saving_ = false; // a running save's result is for the old base: dropped
  ++save_id_;
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
