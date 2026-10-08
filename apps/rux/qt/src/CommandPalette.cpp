// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/CommandPalette.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/fuzzy.hpp>
#include <rux_qt/widgets.hpp>

#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPainter>
#include <QSet>
#include <QStyledItemDelegate>
#include <QVBoxLayout>

namespace rux::qt {
namespace {

// The modal scrim's strength, as the web's EditDialog
// (color-mix(var(--color-scrim) 62%, transparent)).
constexpr double kScrimAlpha = 0.62;
// Rows shown before the list scrolls.
constexpr int kVisibleRows = 9;

enum Role {
  kIsHeader = Qt::UserRole + 1,
  kCommandIndex,
  kPositions,
  kSubtitle,
  kShortcut,
  kBadge,
  kEnabled,
};

/// Paints one palette row: the title with its matched letters in the accent
/// colour, an optional mono subtitle (a path), and on the right a badge and a
/// shortcut chip. Group headers are caps labels. Every value is a token.
class PaletteDelegate : public QStyledItemDelegate {
    public:
  using QStyledItemDelegate::QStyledItemDelegate;

  QSize sizeHint(const QStyleOptionViewItem &,
                 const QModelIndex &idx) const override {
    const Theme &t = theme();
    if (idx.data(kIsHeader).toBool()) {
      const QFontMetrics fm(
          t.font(FontRole::sans, "--font-size-2xs", "--font-weight-bold"));
      return {0, fm.height() + t.px("--space-4") + t.px("--space-1")};
    }
    const QFontMetrics title(t.font(FontRole::sans, "--font-size-md"));
    int h = title.height() + 2 * t.px("--space-2");
    if (!idx.data(kSubtitle).toString().isEmpty())
      h += QFontMetrics(t.font(FontRole::mono, "--font-size-xs")).height() +
           t.px("--space-0");
    return {0, h};
  }

  void paint(QPainter *p, const QStyleOptionViewItem &opt,
             const QModelIndex &idx) const override {
    const Theme &t = theme();
    p->save();
    p->setRenderHint(QPainter::Antialiasing);
    const int padx = t.px("--space-4");
    const QRect r = opt.rect;

    if (idx.data(kIsHeader).toBool()) {
      QFont f = t.font(FontRole::sans, "--font-size-2xs", "--font-weight-bold");
      f.setCapitalization(QFont::AllUppercase);
      f.setLetterSpacing(QFont::AbsoluteSpacing,
                         t.em("--tracking-wide") * f.pixelSize());
      p->setFont(f);
      p->setPen(t.color("--color-text-faint"));
      p->drawText(r.adjusted(padx, 0, -padx, -t.px("--space-1")),
                  Qt::AlignLeft | Qt::AlignBottom, idx.data().toString());
      p->restore();
      return;
    }

    const bool enabled = idx.data(kEnabled).toBool();
    const bool current = opt.state & QStyle::State_Selected;
    if (current) {
      const QRect bg = r.adjusted(t.px("--space-2"), 0, -t.px("--space-2"), 0);
      p->setPen(Qt::NoPen);
      p->setBrush(t.color("--color-accent-muted"));
      const qreal rad = t.px("--radius-md");
      p->drawRoundedRect(QRectF(bg), rad, rad);
      // An accent tick on the left edge, as the nav rail's current item.
      p->setBrush(t.color("--color-accent"));
      p->drawRect(QRect(bg.left(), bg.top() + t.px("--space-2"),
                        t.px("--space-1") / 2 + 1,
                        bg.height() - 2 * t.px("--space-2")));
    }

    const QFont title_font = t.font(FontRole::sans, "--font-size-md");
    QFont match_font =
        t.font(FontRole::sans, "--font-size-md", "--font-weight-bold");
    const QFont sub_font = t.font(FontRole::mono, "--font-size-xs");
    const QFont chip_font = t.font(FontRole::mono, "--font-size-xs");
    const QFont badge_font =
        t.font(FontRole::sans, "--font-size-xs", "--font-weight-bold");
    const QFontMetrics tfm(title_font), mfm(match_font), sfm(sub_font),
        cfm(chip_font), bfm(badge_font);

    // ---- right side: shortcut chip, then badge, laid out right to left.
    int right = r.right() - padx;
    const int chip_pad = t.px("--space-1");
    const QString shortcut = idx.data(kShortcut).toString();
    const int line_mid = r.top() + t.px("--space-2") + tfm.height() / 2;
    if (!shortcut.isEmpty()) {
      const int w = cfm.horizontalAdvance(shortcut) + 2 * chip_pad + chip_pad;
      const int h = cfm.height() + chip_pad;
      const QRect chip(right - w, line_mid - h / 2, w, h);
      p->setPen(QPen(t.color("--color-border-strong"), 1));
      p->setBrush(t.color("--color-surface-sunken"));
      const qreal rad = t.px("--radius-sm");
      p->drawRoundedRect(QRectF(chip).adjusted(0.5, 0.5, -0.5, -0.5), rad, rad);
      p->setFont(chip_font);
      p->setPen(t.color("--color-text-muted"));
      p->drawText(chip, Qt::AlignCenter, shortcut);
      right = chip.left() - t.px("--space-2");
    }
    const QString badge = idx.data(kBadge).toString();
    if (!badge.isEmpty()) {
      const int w = bfm.horizontalAdvance(badge) + 2 * t.px("--space-2");
      const int h = bfm.height() + chip_pad;
      const QRect pill(right - w, line_mid - h / 2, w, h);
      p->setPen(Qt::NoPen);
      // On the selected row the accent pill would melt into the accent-muted
      // selection fill (dark theme): lift it onto the overlay surface.
      p->setBrush(t.color(!enabled  ? "--tone-crit-bg"
                          : current ? "--color-surface-overlay"
                                    : "--tone-accent-bg"));
      if (current && enabled) {
        p->setPen(QPen(t.color("--color-accent"), 1));
      }
      const qreal rad = t.px("--radius-sm");
      p->drawRoundedRect(QRectF(pill), rad, rad);
      p->setFont(badge_font);
      p->setPen(t.color(enabled ? "--tone-accent-ink" : "--tone-crit-ink"));
      p->drawText(pill, Qt::AlignCenter, badge);
      right = pill.left() - t.px("--space-2");
    }

    // ---- title, with the matched letters emphasised.
    const QString title = idx.data().toString();
    QSet<int> hits;
    for (const QVariant &v : idx.data(kPositions).toList())
      hits.insert(v.toInt());
    const QColor text =
        t.color(enabled ? "--color-text" : "--color-text-faint");
    const QColor hit =
        t.color(enabled ? "--color-accent" : "--color-text-muted");
    int x = r.left() + padx;
    const int base = r.top() + t.px("--space-2") + tfm.ascent();
    const int limit = right - t.px("--space-2");
    const QString ellipsis = QStringLiteral("…");
    for (int i = 0; i < title.size(); ++i) {
      const bool h = hits.contains(i);
      const QFontMetrics &fm = h ? mfm : tfm;
      const int adv = fm.horizontalAdvance(title.at(i));
      if (x + adv > limit - tfm.horizontalAdvance(ellipsis) &&
          i < title.size() - 1) {
        p->setFont(title_font);
        p->setPen(text);
        p->drawText(x, base, ellipsis);
        break;
      }
      p->setFont(h ? match_font : title_font);
      p->setPen(h ? hit : text);
      p->drawText(x, base, QString(title.at(i)));
      x += adv;
    }

    const QString sub = idx.data(kSubtitle).toString();
    if (!sub.isEmpty()) {
      p->setFont(sub_font);
      p->setPen(t.color("--color-text-faint"));
      const int y =
          r.top() + t.px("--space-2") + tfm.height() + t.px("--space-0");
      const QRect sr(r.left() + padx, y, limit - r.left() - padx, sfm.height());
      p->drawText(sr, Qt::AlignLeft | Qt::AlignVCenter,
                  sfm.elidedText(sub, Qt::ElideMiddle, sr.width()));
    }
    p->restore();
  }
};

QLabel *key_hint(const QString &key, const QString &what, QHBoxLayout *row) {
  auto *k = new QLabel(key);
  k->setObjectName("paletteKey");
  auto *w = new QLabel(what);
  w->setObjectName("paletteHint");
  row->addWidget(k);
  row->addWidget(w);
  row->addSpacing(theme().px("--space-3"));
  return w;
}

} // namespace

CommandPalette::CommandPalette(QWidget *host) : QWidget(host), host_(host) {
  setObjectName("paletteOverlay");
  setAttribute(Qt::WA_NoSystemBackground);
  hide();

  card_ = new QFrame(this);
  card_->setObjectName("paletteCard");
  auto *v = new QVBoxLayout(card_);
  v->setContentsMargins(0, 0, 0, 0);
  v->setSpacing(0);

  input_ = new QLineEdit;
  input_->setObjectName("paletteInput");
  input_->setPlaceholderText("Søg efter sider, handlinger og projekter …");
  input_->setClearButtonEnabled(false);
  input_->installEventFilter(this);
  v->addWidget(input_);

  list_ = new QListWidget;
  list_->setObjectName("paletteList");
  list_->setItemDelegate(new PaletteDelegate(list_));
  list_->setFocusPolicy(Qt::NoFocus);
  list_->setUniformItemSizes(false);
  list_->setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
  list_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list_->setSelectionMode(QAbstractItemView::SingleSelection);
  v->addWidget(list_);

  empty_ = new QLabel;
  empty_->setObjectName("paletteEmpty");
  empty_->setAlignment(Qt::AlignCenter);
  empty_->hide();
  v->addWidget(empty_);

  auto *foot = new QFrame;
  foot->setObjectName("paletteFoot");
  auto *f = new QHBoxLayout(foot);
  f->setContentsMargins(theme().px("--space-4"), theme().px("--space-2"),
                        theme().px("--space-4"), theme().px("--space-2"));
  f->setSpacing(theme().px("--space-1"));
  key_hint(QStringLiteral("↑↓"), "vælg", f);
  key_hint(QStringLiteral("↵"), "kør", f);
  key_hint("Esc", "luk", f);
  f->addStretch(1);
  count_ = new QLabel;
  count_->setObjectName("paletteCount");
  f->addWidget(count_);
  v->addWidget(foot);

  connect(input_, &QLineEdit::textChanged, this, [this] { refilter(); });
  connect(list_, &QListWidget::itemClicked, this, [this](QListWidgetItem *it) {
    if (!it->data(kIsHeader).toBool()) {
      list_->setCurrentItem(it);
      run_current();
    }
  });
  if (host_)
    host_->installEventFilter(this);
}

void CommandPalette::set_provider(std::function<QVector<Command>()> provider) {
  provider_ = std::move(provider);
}

QString CommandPalette::query() const { return input_->text(); }

void CommandPalette::open_palette(const QString &query) {
  commands_ = provider_ ? provider_() : QVector<Command>{};
  input_->blockSignals(true);
  input_->setText(query);
  input_->blockSignals(false);
  if (host_)
    setGeometry(host_->rect());
  refilter();
  show();
  raise();
  input_->setFocus(Qt::PopupFocusReason);
  input_->selectAll();
}

void CommandPalette::close_palette() {
  if (!isVisible())
    return;
  hide();
  emit closed();
}

QStringList CommandPalette::visible_titles() const {
  QStringList out;
  for (int i = 0; i < list_->count(); ++i)
    if (!list_->item(i)->data(kIsHeader).toBool())
      out << list_->item(i)->text();
  return out;
}

void CommandPalette::refilter() {
  list_->clear();
  std::vector<PaletteCandidate> cands;
  cands.reserve(static_cast<std::size_t>(commands_.size()));
  for (const Command &c : commands_)
    // Title + keywords, as rux::qt::palette_candidates(): the subtitle (a
    // full path) is shown but not searched.
    cands.push_back({c.title.toStdString(), c.keywords.toStdString()});
  const QString q = input_->text();
  const auto ranked = rank_palette(q.toStdString(), cands);
  const bool grouped = q.trimmed().isEmpty();

  QString last_group;
  int rows = 0;
  for (const PaletteMatch &m : ranked) {
    const Command &c = commands_[static_cast<int>(m.index)];
    if (grouped && c.group != last_group) {
      auto *h = new QListWidgetItem(c.group);
      h->setData(kIsHeader, true);
      h->setFlags(Qt::NoItemFlags);
      list_->addItem(h);
      last_group = c.group;
    }
    auto *it = new QListWidgetItem(c.title);
    it->setData(kIsHeader, false);
    it->setData(kCommandIndex, static_cast<int>(m.index));
    QVariantList pos;
    for (int p : m.positions)
      pos << p;
    it->setData(kPositions, pos);
    it->setData(kSubtitle, c.subtitle);
    it->setData(kShortcut, c.shortcut);
    it->setData(kBadge, c.badge);
    it->setData(kEnabled, c.enabled);
    list_->addItem(it);
    ++rows;
  }

  empty_->setVisible(rows == 0);
  list_->setVisible(rows > 0);
  if (rows == 0)
    empty_->setText(QString("Ingen resultater for “%1”").arg(q.trimmed()));
  count_->setText(rows == 1 ? QString("1 resultat")
                            : QString("%1 resultater").arg(rows));
  select_row(-1);
  move_selection(+1);
  place_card();
}

void CommandPalette::select_row(int row) {
  if (row < 0) {
    list_->setCurrentRow(-1);
    return;
  }
  list_->setCurrentRow(row);
  list_->scrollToItem(list_->item(row));
}

void CommandPalette::move_selection(int delta) {
  const int n = list_->count();
  if (n == 0)
    return;
  int row = list_->currentRow();
  const int step = delta > 0 ? 1 : -1;
  int remaining = std::abs(delta);
  int candidate = row;
  int last_good = row;
  while (remaining > 0) {
    candidate += step;
    if (candidate < 0 || candidate >= n)
      break;
    const auto *it = list_->item(candidate);
    if (it->data(kIsHeader).toBool())
      continue;
    last_good = candidate;
    --remaining;
  }
  // Wrap around at the ends for single steps, like every launcher.
  if (last_good == row && std::abs(delta) == 1 && row >= 0) {
    candidate = step > 0 ? -1 : n;
    do {
      candidate += step;
    } while (candidate >= 0 && candidate < n &&
             list_->item(candidate)->data(kIsHeader).toBool());
    if (candidate >= 0 && candidate < n)
      last_good = candidate;
  }
  if (last_good >= 0)
    select_row(last_good);
}

void CommandPalette::run_current() {
  const auto *it = list_->currentItem();
  if (!it || it->data(kIsHeader).toBool())
    return;
  const int idx = it->data(kCommandIndex).toInt();
  if (idx < 0 || idx >= commands_.size())
    return;
  const Command c = commands_[idx];
  if (!c.enabled)
    return;
  close_palette();
  if (c.run)
    c.run();
}

bool CommandPalette::eventFilter(QObject *obj, QEvent *ev) {
  if (obj == host_ && ev->type() == QEvent::Resize && !isHidden()) {
    setGeometry(host_->rect());
    return false;
  }
  if (obj == input_ && ev->type() == QEvent::KeyPress) {
    auto *k = static_cast<QKeyEvent *>(ev);
    switch (k->key()) {
    case Qt::Key_Down:
      move_selection(+1);
      return true;
    case Qt::Key_Up:
      move_selection(-1);
      return true;
    case Qt::Key_PageDown:
      move_selection(+kVisibleRows - 1);
      return true;
    case Qt::Key_PageUp:
      move_selection(-(kVisibleRows - 1));
      return true;
    case Qt::Key_Return:
    case Qt::Key_Enter:
      run_current();
      return true;
    case Qt::Key_Escape:
      close_palette();
      return true;
    case Qt::Key_Tab:
    case Qt::Key_Backtab:
      return true; // focus stays in the field
    default:
      break;
    }
    if (k->matches(QKeySequence::MoveToStartOfDocument) ||
        (k->key() == Qt::Key_Home && (k->modifiers() & Qt::ControlModifier))) {
      select_row(-1);
      move_selection(+1);
      return true;
    }
  }
  return QWidget::eventFilter(obj, ev);
}

void CommandPalette::paintEvent(QPaintEvent *) {
  QPainter p(this);
  QColor scrim = theme().color("--color-scrim");
  scrim.setAlphaF(kScrimAlpha);
  p.fillRect(rect(), scrim);
}

void CommandPalette::mousePressEvent(QMouseEvent *e) {
  if (!card_->geometry().contains(e->position().toPoint()))
    close_palette();
}

void CommandPalette::resizeEvent(QResizeEvent *) { place_card(); }

void CommandPalette::place_card() {
  const Theme &t = theme();
  const int margin = t.px("--space-5");
  const int want = 2 * t.px("--layout-panel-width") + t.px("--space-7");
  const int w = std::max(0, std::min(want, width() - 2 * margin));

  // List height: the rows themselves, up to kVisibleRows of the tallest
  // kind, and never past the window.
  int rows_h = 0, shown = 0;
  for (int i = 0; i < list_->count() && shown < kVisibleRows; ++i) {
    rows_h += list_->sizeHintForRow(i);
    if (!list_->item(i)->data(kIsHeader).toBool())
      ++shown;
  }
  const int top = std::max(margin, height() / 8);
  const int chrome = input_->sizeHint().height() +
                     card_->layout()->itemAt(3)->sizeHint().height() + 2;
  const int max_list = std::max(0, height() - top - margin - chrome);
  const int list_h = std::min(rows_h + t.px("--space-2"), max_list);
  list_->setFixedHeight(list_h);
  empty_->setFixedHeight(t.px("--space-7") + t.px("--space-5"));
  card_->setFixedWidth(w);
  card_->adjustSize();
  card_->move((width() - w) / 2, top);
}

} // namespace rux::qt
