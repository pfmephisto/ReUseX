// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Recent projects, stored in QSettings ("recentProjects") — or only in
// memory, for the gallery and tests, so a screenshot never reads or writes
// the user's settings. The list rules (order, cap, de-duplication) are the
// Qt-free recent_* functions in rux_qt/recent.hpp.

#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

namespace rux::qt {

class RecentProjects : public QObject {
  Q_OBJECT
    public:
  struct Entry {
    QString path;
    bool missing = false;
  };

  /// Backed by the application's QSettings (organisation/application name
  /// set by the app). Loads now; every change is written immediately.
  static RecentProjects *from_settings(QObject *parent = nullptr);
  /// In memory only, starting from @p initial (most recent first).
  explicit RecentProjects(const QStringList &initial = {},
                          QObject *parent = nullptr);

  QStringList paths() const { return list_; }
  /// Each path with whether the file is gone, checked now.
  QVector<Entry> entries() const;

  void add(const QString &path);
  void remove(const QString &path);
  void clear();

    signals:
  void changed();

    private:
  void set(const QStringList &list);
  bool persist_ = false;
  QStringList list_;
};

} // namespace rux::qt
