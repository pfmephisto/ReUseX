// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The start page: open a project (button, Ctrl+O, drag-and-drop of a .rux),
// the recent list with missing files marked, and — once a project is open —
// its summary card. Shows the loading state while ProjectSession works and a
// Danish explanation when an open fails (with "Åbn skrivebeskyttet" for a
// locked project).

#include <QWidget>

class QBoxLayout;
class QFrame;
class QLabel;
class QVBoxLayout;

namespace rux::qt {

class ProjectSession;
class RecentProjects;

class StartPage : public QWidget {
  Q_OBJECT
    public:
  StartPage(ProjectSession &session, RecentProjects &recent,
            QWidget *parent = nullptr);

  /// Highlight the drop zone while a .rux is dragged over the window.
  void set_drop_active(bool on);

    signals:
  void browse_requested();
  void open_requested(const QString &path, bool read_only);
  void navigate_requested(int page);

    protected:
  void resizeEvent(QResizeEvent *) override;

    private:
  void rebuild_state();
  void rebuild_recent();
  void update_columns();
  QWidget *make_hero(bool compact);
  QWidget *make_loading();
  QWidget *make_error();
  QWidget *make_summary();

  ProjectSession &session_;
  RecentProjects &recent_;
  QBoxLayout *columns_ = nullptr;
  QVBoxLayout *main_col_ = nullptr;
  QVBoxLayout *recent_col_ = nullptr;
  QWidget *main_box_ = nullptr;
  QWidget *recent_box_ = nullptr;
  QFrame *drop_zone_ = nullptr;
  bool drop_active_ = false;
};

} // namespace rux::qt
