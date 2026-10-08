// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// Gallery pages of the real app shell (Q1): the same AppShell the `rux`
// binary shows, with the project opened synchronously and an in-memory
// recent list, so a screenshot never touches the user's settings.

#include "demo_pages.hpp"

#include <rux_qt/AppShell.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/RecentProjects.hpp>

#include <QDir>
#include <QFile>
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
  register_page({"shell-empty", "Skallen uden projekt: Database-pladsholder",
                 make_shell_empty});
  register_page({"shell-project", "Skallen med projekt: 3D-pladsholder",
                 make_shell_project});
  register_page({"palette-open", "Kommandopaletten med søgningen \"pro\"",
                 make_palette_open});
  register_page({"palette-all", "Kommandopaletten uden søgning (grupperet)",
                 make_palette_all});
}

} // namespace rux::qt::gallery
