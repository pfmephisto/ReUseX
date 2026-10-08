// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

// rux-qt-gallery: render one page of the Qt client, headless to PNG or live
// in a window. The design-studio skill's qt_shot.sh drives it; see
// .claude/skills/design-studio/references/qt-client.md.
//
// Exit codes: 0 ok, 2 bad command line, 3 a token was missing (the PNG is
// still written, with magenta where the token was), 4 project or page could
// not be opened, 5 the PNG could not be written.

#include "demo_pages.hpp"

#include <rux_qt/Theme.hpp>
#include <rux_qt/gallery_args.hpp>
#include <rux_qt/pages.hpp>

#include <reusex/core/ProjectDB.hpp>

#include <QApplication>
#include <QElapsedTimer>
#include <QImage>
#include <QPointer>
#include <QSurfaceFormat>
#include <QThread>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>

#include <cstdio>
#include <memory>

using namespace rux::qt;

namespace {

void pump(int rounds, int sleep_ms) {
  for (int i = 0; i < rounds; ++i) {
    QApplication::processEvents(QEventLoop::AllEvents);
    if (sleep_ms > 0)
      QThread::msleep(static_cast<unsigned long>(sleep_ms));
  }
}

} // namespace

int main(int argc, char **argv) {
  QElapsedTimer clock;
  clock.start();

  const GalleryArgs args = parse_gallery_args(argc, argv);
  if (!args.error.empty()) {
    std::fprintf(stderr, "rux-qt-gallery: %s\n\n%s", args.error.c_str(),
                 gallery_usage().c_str());
    return 2;
  }
  if (args.help) {
    std::fputs(gallery_usage().c_str(), stdout);
    return 0;
  }
  gallery::register_demo_pages();
  if (args.list_pages) {
    for (const auto &p : pages())
      std::printf("%-12s %s\n", qPrintable(p.name), qPrintable(p.description));
    return 0;
  }
  const PageInfo *page = find_page(QString::fromStdString(args.page));
  if (!page) {
    std::fprintf(stderr, "rux-qt-gallery: no page '%s' (see --list-pages)\n",
                 args.page.c_str());
    return 4;
  }

  const bool shot = !args.screenshot.empty();
  // Both must be set before the QApplication exists.
  if (args.scale != 1.0)
    qputenv("QT_SCALE_FACTOR", QByteArray::number(args.scale));
  if (shot && !args.gl && qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM"))
    qputenv("QT_QPA_PLATFORM", "offscreen");
  if (args.gl)
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

  QApplication app(argc, argv);
  QApplication::setApplicationName("rux-qt-gallery");

  Theme::Source source =
      args.dev ? Theme::dev_source(QString::fromStdString(args.style_dir))
               : Theme::embedded_source();
  if (shot)
    source.watch = false;
  theme().apply(args.theme, source);

  std::unique_ptr<reusex::ProjectDB> db;
  if (!args.project.empty()) {
    try {
      db = std::make_unique<reusex::ProjectDB>(args.project);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "rux-qt-gallery: cannot open %s: %s\n",
                   args.project.c_str(), e.what());
      return 4;
    }
  }
  PageContext ctx;
  ctx.db = db.get();
  ctx.project_path = QString::fromStdString(args.project);
  ctx.interactive_3d = args.gl;

  QWidget host;
  host.setObjectName("galleryHost");
  host.setWindowTitle(QString("rux-qt-gallery — %1").arg(page->name));
  auto *layout = new QVBoxLayout(&host);
  layout->setContentsMargins(0, 0, 0, 0);
  QPointer<QWidget> current = page->make(ctx);
  layout->addWidget(current);
  host.resize(args.width, args.height);

  // Dev mode: a token or QSS edit re-applies the stylesheet; rebuilding the
  // page as well picks up the values code reads from tokens (layout spacing,
  // the canvas colour, painted swatches).
  QObject::connect(&theme(), &Theme::changed, &host, [&] {
    if (current)
      current->deleteLater();
    current = page->make(ctx);
    layout->addWidget(current);
  });

  if (!shot) {
    host.show();
    return app.exec();
  }

  if (!args.gl)
    host.setAttribute(Qt::WA_DontShowOnScreen);
  host.show();
  // Coalesced renders (the 3D pane) run on zero-length timers; a GL widget
  // under Xvfb needs real time to get its first frame.
  pump(args.gl ? 30 : 6, args.gl ? 50 : 0);

  const QImage img = host.grab().toImage();
  const bool saved = img.save(QString::fromStdString(args.screenshot));
  const QStringList missing = theme().missing();
  std::fprintf(stderr, "rux-qt-gallery: %s %dx%d (dpr %.1f) in %lld ms%s\n",
               args.screenshot.c_str(), img.width(), img.height(),
               img.devicePixelRatio(), static_cast<long long>(clock.elapsed()),
               missing.isEmpty() ? "" : " — MISSING TOKENS");
  if (!saved) {
    std::fprintf(stderr, "rux-qt-gallery: ERROR could not write %s\n",
                 args.screenshot.c_str());
    return 5;
  }
  if (!missing.isEmpty()) {
    for (const QString &m : missing)
      std::fprintf(stderr, "rux-qt-gallery: ERROR MISSING token %s\n",
                   qPrintable(m));
    return 3;
  }
  return 0;
}
