// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Qt client's window content: a title bar (project, schema status), the
// left nav rail (Start, Database, 3D, Posegraf, Pipeline, Log), a stack of
// workspaces, a collapsible inspector on the right, and the Ctrl+K command
// palette over all of it. A plain QWidget, so the gallery can screenshot it
// without a QMainWindow; MainWindow wraps it for the real app.
//
// The shell owns no project: it is handed the ProjectSession and the recent
// list, and reacts to the session's signals.

#include <rux_qt/CommandPalette.hpp>
#include <rux_qt/workspaces.hpp>

#include <QFrame>
#include <QWidget>

class QAction;
class QLabel;
class QPushButton;
class QStackedWidget;

namespace rux::qt {

class Inspector;
class NavRail;
class ProjectSession;
class RecentProjects;
class StartPage;
class Pill;

class AppShell : public QWidget {
  Q_OBJECT
    public:
  AppShell(ProjectSession &session, RecentProjects &recent,
           QWidget *parent = nullptr);

  void show_page(Workspace w);
  Workspace current_page() const;

  void set_inspector_visible(bool on);
  bool inspector_visible() const;

  CommandPalette *palette() const { return palette_; }
  void open_palette(const QString &query = {});
  /// Every page, action and recent project, as the palette lists them.
  QVector<Command> commands();

  /// Open @p path through the session and remember it in the recent list.
  void open_project(const QString &path, bool read_only = false);
  /// The QFileDialog for .rux files.
  void browse();

    signals:
  void quit_requested();
  void toggle_theme_requested();
  /// The inspector was shown or hidden (MainWindow stores it).
  void inspector_toggled(bool visible);

    protected:
  void dragEnterEvent(QDragEnterEvent *e) override;
  void dragLeaveEvent(QDragLeaveEvent *e) override;
  void dropEvent(QDropEvent *e) override;

    private:
  QWidget *make_title_bar();
  void build_actions();
  void sync_project();
  void refresh_product_label();

  ProjectSession &session_;
  RecentProjects &recent_;

  QLabel *product_ = nullptr;
  QLabel *title_project_ = nullptr;
  QLabel *title_path_ = nullptr;
  Pill *schema_pill_ = nullptr;
  Pill *access_pill_ = nullptr;
  QPushButton *inspector_button_ = nullptr;

  NavRail *rail_ = nullptr;
  QStackedWidget *stack_ = nullptr;
  StartPage *start_ = nullptr;
  QVector<WorkspacePlaceholder *> placeholders_;
  Inspector *inspector_ = nullptr;
  CommandPalette *palette_ = nullptr;

  QAction *open_action_ = nullptr;
  QAction *close_action_ = nullptr;
  QAction *reload_action_ = nullptr;
  QAction *inspector_action_ = nullptr;
  QAction *palette_action_ = nullptr;
  QAction *quit_action_ = nullptr;
  QAction *theme_action_ = nullptr;
};

/// The right-hand context panel. Q1 shows the open project's properties;
/// Q2 puts the current selection here.
class Inspector : public QFrame {
  Q_OBJECT
    public:
  explicit Inspector(ProjectSession &session, QWidget *parent = nullptr);
  void refresh();

    signals:
  void hide_requested();

    private:
  ProjectSession &session_;
  QWidget *body_ = nullptr;
};

} // namespace rux::qt
