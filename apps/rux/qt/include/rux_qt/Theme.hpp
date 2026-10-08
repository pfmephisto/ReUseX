// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// The Qt client's theme: tokens.css -> QSS + QPalette, applied to the whole
// application, with hot reload in dev mode.
//
// One design system, two renderers. The values come from
// apps/rux/frontend/src/tokens.css (owned by the Claude Design project) and
// are substituted into styles/app.qss, a QSS template written with the same
// `var(--x)` syntax as the web's CSS Modules. Code never names a colour or a
// size: it asks the Theme for a token (`color("--color-canvas")`).
//
// What QSS cannot express is done in code from the same tokens — see
// references/qt-client.md in the design-studio skill: letter-spacing and
// uppercase (CapsLabel), shadows (skipped; borders carry elevation), motion
// (QPropertyAnimation with --duration-*), and the Fusion-drawn parts QSS does
// not reach (QPalette, set here).

#include <rux_qt/tokens.hpp>

#include <QColor>
#include <QFileSystemWatcher>
#include <QFont>
#include <QObject>
#include <QPalette>
#include <QString>
#include <QStringList>
#include <QTemporaryDir>

#include <memory>

namespace rux::qt {

/// Typeface roles, matching --font-sans / --font-display / --font-mono.
enum class FontRole { sans, display, mono };

class Theme : public QObject {
  Q_OBJECT

    public:
  /// The process-wide theme. Created on first use; apply() must run after
  /// the QApplication exists.
  static Theme &instance();

  /// Where tokens.css and app.qss are read from.
  struct Source {
    /// Read both files from disk (dev mode) instead of the embedded
    /// snapshot; empty = embedded resources.
    QString tokens_css;
    QString qss_template;
    /// Watch the two files and re-apply on every change.
    bool watch = false;
  };

  /// Load @p source for @p mode and apply it to the application: Fusion
  /// style, QPalette, default font and stylesheet. Safe to call again (theme
  /// switch, reload); emits changed().
  void apply(ThemeMode mode, const Source &source);

  /// The source the dev build reads: the live files in the repository.
  static Source dev_source(const QString &style_dir = {});
  /// The embedded snapshot (the release path).
  static Source embedded_source();

  ThemeMode mode() const { return mode_; }

  // --- token lookups (every one records an unknown name in missing()) ------
  /// A colour token. Unknown or unparsable -> magenta.
  QColor color(const QString &token) const;
  /// A length token in logical pixels. Unknown -> 0.
  int px(const QString &token) const;
  /// An em length token (letter-spacing), as a multiple of the font size.
  double em(const QString &token) const;
  /// The raw normalised value; empty if unknown.
  QString value(const QString &token) const;
  /// A font for @p role at the pixel size of @p size_token and the weight of
  /// @p weight_token (e.g. "--font-size-md", "--font-weight-medium").
  QFont font(FontRole role, const QString &size_token,
             const QString &weight_token = "--font-weight-regular") const;

  /// The resolved stylesheet currently applied.
  const QString &stylesheet() const { return stylesheet_; }

  /// Every token name a lookup or the QSS template asked for that tokens.css
  /// does not define, since the last apply(). Screenshot mode exits non-zero
  /// when this is non-empty.
  QStringList missing() const { return missing_; }

    signals:
  /// Emitted after every apply() — including a hot reload — so widgets that
  /// paint from tokens in code can re-read them.
  void changed();

    private:
  Theme() = default;
  void reload();
  /// Tokens painted in code (`--qt-*`), e.g. the checkbox tick image.
  void add_generated_tokens();
  void note_missing(const QString &token) const;

  ThemeMode mode_ = ThemeMode::dark;
  Source source_;
  TokenMap tokens_;
  QString stylesheet_;
  mutable QStringList missing_;
  QFileSystemWatcher *watcher_ = nullptr;
  bool applied_ = false; ///< a theme is on screen (a reload may keep it)
  int retries_ = 0;      ///< waits so far for a deleted style file
  std::unique_ptr<QTemporaryDir> icon_dir_;
};

/// Shorthand for Theme::instance().
inline Theme &theme() { return Theme::instance(); }

/// Re-evaluate a widget's stylesheet after a dynamic property changed (QSS
/// `[active="true"]` selectors are only matched at polish time).
void repolish(class QWidget *w);

} // namespace rux::qt
