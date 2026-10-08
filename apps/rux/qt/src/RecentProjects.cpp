// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/RecentProjects.hpp>
#include <rux_qt/recent.hpp>

#include <QDir>
#include <QFileInfo>
#include <QSettings>

namespace rux::qt {
namespace {

constexpr const char *kKey = "recentProjects";

std::vector<std::string> to_std(const QStringList &l) {
  std::vector<std::string> out;
  out.reserve(static_cast<std::size_t>(l.size()));
  for (const QString &s : l)
    out.push_back(s.toStdString());
  return out;
}

QStringList to_qt(const std::vector<std::string> &l) {
  QStringList out;
  for (const auto &s : l)
    out << QString::fromStdString(s);
  return out;
}

std::string cwd() { return QDir::currentPath().toStdString(); }

} // namespace

RecentProjects *RecentProjects::from_settings(QObject *parent) {
  auto *r = new RecentProjects({}, parent);
  r->persist_ = true;
  r->list_ = to_qt(
      recent_sanitise(to_std(QSettings().value(kKey).toStringList()), cwd()));
  return r;
}

RecentProjects::RecentProjects(const QStringList &initial, QObject *parent)
    : QObject(parent), list_(to_qt(recent_sanitise(to_std(initial), cwd()))) {}

QVector<RecentProjects::Entry> RecentProjects::entries() const {
  QVector<Entry> out;
  for (const auto &e : recent_annotate(to_std(list_), [](const std::string &p) {
         return QFileInfo(QString::fromStdString(p)).isFile();
       }))
    out.push_back({QString::fromStdString(e.path), e.missing});
  return out;
}

void RecentProjects::set(const QStringList &list) {
  if (list == list_)
    return;
  list_ = list;
  if (persist_)
    QSettings().setValue(kKey, list_);
  emit changed();
}

void RecentProjects::add(const QString &path) {
  set(to_qt(recent_push(to_std(list_),
                        normalise_project_path(path.toStdString(), cwd()))));
}

void RecentProjects::remove(const QString &path) {
  set(to_qt(recent_remove(to_std(list_), path.toStdString())));
}

void RecentProjects::clear() { set({}); }

} // namespace rux::qt
