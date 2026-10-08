// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Gallery pages of the real app shell (Q1): the same AppShell the `rux`
// binary shows, with the project opened synchronously and an in-memory
// recent list, so a screenshot never touches the user's settings.

#include "demo_pages.hpp"

#include <rux_qt/AppShell.hpp>
#include <rux_qt/DatabaseWorkspace.hpp>
#include <rux_qt/EdgeEditor.hpp>
#include <rux_qt/FrameBrowser.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/TableBrowser.hpp>

#include <QAbstractButton>
#include <QButtonGroup>
#include <QDir>
#include <QFile>
#include <QPushButton>
#include <QTemporaryDir>

namespace rux::qt::gallery {
namespace {

/// A recent list with the shot's project first and two that no longer
/// exist, so the "Mangler" state is always on screen.
QStringList demo_recent(const PageContext &ctx) {
  QStringList l;
  if (!ctx.project_path.isEmpty())
    l << ctx.project_path;
  l << QDir::homePath() + "/sager/Kontorhus Valby/project.rux"
    << QDir::homePath() + "/sager/Skolen 2025/scan-02.rux";
  return l;
}

enum class Open { none, project, read_only, junk };

AppShell *make_shell(const PageContext &ctx, Open open) {
  auto *session = new ProjectSession;
  auto *recent = new RecentProjects(demo_recent(ctx));
  auto *shell = new AppShell(*session, *recent);
  session->setParent(shell);
  recent->setParent(shell);
  if (open == Open::project && !ctx.project_path.isEmpty()) {
    session->open_blocking(ctx.project_path);
  } else if (open == Open::read_only && !ctx.project_path.isEmpty()) {
    session->open_blocking(ctx.project_path, /*read_only=*/true);
  } else if (open == Open::junk) {
    // A file that is not a ReUseX project, to show the failure card.
    static QTemporaryDir dir;
    const QString junk = dir.filePath("noter.rux");
    QFile f(junk);
    if (f.open(QIODevice::WriteOnly))
      f.write("Ikke en database, bare noter.\n");
    f.close();
    session->open_blocking(junk);
  }
  return shell;
}

QWidget *make_start(const PageContext &ctx) {
  return make_shell(ctx, Open::project);
}

QWidget *make_start_readonly(const PageContext &ctx) {
  return make_shell(ctx, Open::read_only);
}

QWidget *make_start_empty(const PageContext &ctx) {
  return make_shell(ctx, Open::none);
}

QWidget *make_start_error(const PageContext &ctx) {
  return make_shell(ctx, Open::junk);
}

QWidget *make_shell_empty(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::none);
  shell->show_page(Workspace::database);
  return shell;
}

QWidget *make_shell_project(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->show_page(Workspace::viewer3d);
  return shell;
}

// ---- Database workspace (Q2)

/// The shell on the Database page with A and B a few frames apart in the
/// middle of the capture (frame ids differ per project: use positions).
AppShell *make_db(const PageContext &ctx, Open open = Open::project) {
  auto *shell = make_shell(ctx, open);
  shell->show_page(Workspace::database);
  if (FrameBrowser *f = shell->database()->frames(); !f->pair().empty()) {
    const auto &ids = f->pair().ids();
    const std::size_t a = ids.size() * 3 / 10;
    f->set_a(ids[a]);
    f->set_b(ids[std::min(ids.size() - 1, a + 12)]);
  }
  return shell;
}

/// Click a toolbar layer button of the frame browser ("Dybde", …).
void pick_layer(AppShell *shell, const QString &name) {
  for (auto *b :
       shell->database()->frames()->findChildren<QPushButton *>("segment"))
    if (b->text() == name)
      b->click();
}

QWidget *make_db_frames(const PageContext &ctx) { return make_db(ctx); }

QWidget *make_db_depth(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  pick_layer(shell, "Dybde");
  return shell;
}

QWidget *make_db_confidence(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  pick_layer(shell, "Konfidens");
  return shell;
}

/// Pending edits: an ICP run on the pair and a staged loop closure, plus a
/// staged deletion when the project has stored edges.
QWidget *make_db_pending(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  FrameBrowser *f = shell->database()->frames();
  if (f->pair().empty())
    return shell;
  EdgeEditor &ed = shell->database()->editor();
  if (!ed.edits().base().empty())
    ed.remove(ed.edits().base().front().key);
  for (auto *b : f->findChildren<QPushButton *>())
    if (b->text() == "ICP-forfin")
      b->click();
  ed.add({{f->pair().a_id(), f->pair().b_id(), "loop_closure"}, 0.0, 400.0});
  return shell;
}

/// A save that cannot happen: the project is open read-only.
QWidget *make_db_readonly(const PageContext &ctx) {
  auto *shell = make_db(ctx, Open::read_only);
  FrameBrowser *f = shell->database()->frames();
  if (f->pair().empty())
    return shell;
  shell->database()->editor().add(
      {{f->pair().a_id(), f->pair().b_id(), "loop_closure"}, 0.0, 1.0});
  shell->database()->save_edits();
  return shell;
}

/// A saved edit: stage a loop closure on the pair, then "Gem ændringer" —
/// the stored edge comes back from the file marked "Gemt". WRITES to the
/// project copy.
QWidget *make_db_saved(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  FrameBrowser *f = shell->database()->frames();
  if (f->pair().empty())
    return shell;
  shell->database()->editor().add(
      {{f->pair().a_id(), f->pair().b_id(), "loop_closure"}, 0.0, 250.0});
  shell->database()->save_edits();
  return shell;
}

QWidget *make_db_table(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  shell->database()->open_item(ProjectTree::Kind::table, "sensor_frames");
  if (auto *t = shell->database()->findChild<TableBrowser *>())
    t->select_where(
        "node_id", QString::number(shell->database()->frames()->pair().a_id()));
  return shell;
}

QWidget *make_db_log(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  shell->database()->open_item(ProjectTree::Kind::log);
  return shell;
}

QWidget *make_db_cloud(const PageContext &ctx) {
  auto *shell = make_db(ctx);
  // The semantic label cloud if there is one (its legend), else any cloud.
  shell->database()->open_item(ProjectTree::Kind::cloud, "labels");
  return shell;
}

QWidget *make_palette_open(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->open_palette("pro");
  return shell;
}

QWidget *make_palette_all(const PageContext &ctx) {
  auto *shell = make_shell(ctx, Open::project);
  shell->open_palette();
  return shell;
}

} // namespace

void register_shell_pages() {
  register_page({"start", "Startside med åbent projekt og seneste projekter",
                 make_start});
  register_page({"start-readonly", "Startside med et skrivebeskyttet projekt",
                 make_start_readonly});
  register_page({"start-empty", "Startside uden projekt", make_start_empty});
  register_page(
      {"start-error", "Startside efter en fejlet åbning", make_start_error});
  register_page({"shell-empty", "Skallen uden projekt: Database uden projekt",
                 make_shell_empty});
  register_page({"shell-project", "Skallen med projekt: 3D-pladsholder",
                 make_shell_project});
  register_page({"db-frames",
                 "Database: A/B-billeder med mærkater, kant og filmstrimmel",
                 make_db_frames});
  register_page({"db-depth", "Database: dybdelaget", make_db_depth});
  register_page(
      {"db-confidence", "Database: konfidenslaget", make_db_confidence});
  register_page({"db-pending",
                 "Database: ICP-resultat og ventende kantændringer",
                 make_db_pending});
  register_page({"db-readonly",
                 "Database: gem afvist på et skrivebeskyttet projekt",
                 make_db_readonly});
  register_page({"db-saved",
                 "Database: en gemt kant (skriver til projektkopien)",
                 make_db_saved});
  register_page(
      {"db-table", "Database: tabelvisning af sensor_frames", make_db_table});
  register_page({"db-log", "Database: pipeline-loggen", make_db_log});
  register_page({"db-cloud", "Database: en punktsky i træet og inspektøren",
                 make_db_cloud});
  register_page({"palette-open", "Kommandopaletten med søgningen \"pro\"",
                 make_palette_open});
  register_page({"palette-all", "Kommandopaletten uden søgning (grupperet)",
                 make_palette_all});
}

} // namespace rux::qt::gallery
