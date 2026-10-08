// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The command palette (Ctrl+K): one keyboard-first list over every page,
// action and recent project. An overlay child of the shell — a scrim over the
// window and a card near the top — rather than a separate window, so it
// themes, screenshots and closes with the window.
//
// Typing filters with rux::qt::rank_palette (fuzzy, Danish-folding); the
// matched letters are highlighted. Up/Down/PageUp/PageDown/Home/End move,
// Enter runs, Esc (or a click on the scrim) closes. With an empty query the
// list is grouped (Sider, Handlinger, Seneste projekter).

#include <QPointer>
#include <QString>
#include <QVector>
#include <QWidget>

#include <functional>

class QLabel;
class QLineEdit;
class QListWidget;
class QFrame;

namespace rux::qt {

struct Command {
  QString group;    ///< "Sider" | "Handlinger" | "Seneste projekter"
  QString title;    ///< matched and highlighted
  QString keywords; ///< extra search text (aliases, the path), not shown
  QString subtitle; ///< second line (a path); may be empty
  QString shortcut; ///< e.g. "Ctrl+O"; may be empty
  QString badge;    ///< right-hand tag, e.g. "Mangler"; may be empty
  bool enabled = true;
  std::function<void()> run;
};

class CommandPalette : public QWidget {
  Q_OBJECT
    public:
  /// @param host  the widget the overlay covers (the shell).
  explicit CommandPalette(QWidget *host);

  /// Called on every open to get the current commands.
  void set_provider(std::function<QVector<Command>()> provider);

  void open_palette(const QString &query = {});
  void close_palette();
  bool is_open() const { return isVisible(); }

  /// The visible rows' titles, in order (headers excluded) — for tests.
  QStringList visible_titles() const;
  QString query() const;

    signals:
  void closed();

    protected:
  bool eventFilter(QObject *obj, QEvent *ev) override;
  void paintEvent(QPaintEvent *) override;
  void mousePressEvent(QMouseEvent *) override;
  void resizeEvent(QResizeEvent *) override;

    private:
  void refilter();
  void move_selection(int delta);
  void select_row(int row);
  void run_current();
  void place_card();

  QPointer<QWidget> host_;
  std::function<QVector<Command>()> provider_;
  QVector<Command> commands_;
  QFrame *card_ = nullptr;
  QLineEdit *input_ = nullptr;
  QListWidget *list_ = nullptr;
  QLabel *empty_ = nullptr;
  QLabel *count_ = nullptr;
};

} // namespace rux::qt
