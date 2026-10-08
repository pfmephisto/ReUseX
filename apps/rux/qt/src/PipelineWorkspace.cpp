// SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
// SPDX-License-Identifier: GPL-3.0-or-later

#include <rux_qt/PipelineWorkspace.hpp>
#include <rux_qt/ProjectSession.hpp>
#include <rux_qt/Theme.hpp>
#include <rux_qt/background.hpp>
#include <rux_qt/pipeline_ui.hpp>
#include <rux_qt/widgets.hpp>
#include <rux_qt/workspace_logic.hpp>

#include <reusex/core/ProjectDB.hpp>
#include <reusex/core/stages.hpp>
#include <reusex/core/validate.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QLocale>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include <atomic>
#include <climits>
#include <cmath>
#include <mutex>

namespace rux::qt {

namespace pl = reusex::pipeline;

namespace {

const QLocale &da() {
  static const QLocale l(QLocale::Danish, QLocale::Denmark);
  return l;
}

QString qs(const std::string &s) { return QString::fromStdString(s); }

/// Decimals a spin box needs to show @p v exactly (2..4).
int decimals_for(double v) {
  const std::string s = format_number(static_cast<float>(v));
  const auto dot = s.find('.');
  const int d =
      dot == std::string::npos ? 0 : static_cast<int>(s.size() - dot - 1);
  return std::clamp(d, 2, 4);
}

QString default_text(const pl::ParameterDescriptor &d) {
  if (const auto *x = std::get_if<double>(&d.default_value))
    return da().toString(*x, 'g', 6);
  if (const auto *x = std::get_if<long long>(&d.default_value))
    return da().toString(*x);
  if (const auto *x = std::get_if<bool>(&d.default_value))
    return *x ? QString("til") : QString("fra");
  if (const auto *x = std::get_if<std::string>(&d.default_value))
    return qs(*x);
  return QString("ingen");
}

} // namespace

/// One form row: the widget for a descriptor, and its pin for a
/// presence-sensitive key.
struct PipelineWorkspace::Field {
  const pl::ParameterDescriptor *d = nullptr;
  QWidget *widget = nullptr;
  QCheckBox *pin = nullptr;

  QJsonValue value() const {
    switch (d->type) {
    case pl::ParameterType::number:
      return static_cast<QDoubleSpinBox *>(widget)->value();
    case pl::ParameterType::integer:
      return static_cast<QSpinBox *>(widget)->value();
    case pl::ParameterType::boolean:
      return static_cast<QCheckBox *>(widget)->isChecked();
    case pl::ParameterType::string:
      if (auto *c = qobject_cast<QComboBox *>(widget))
        return c->currentText();
      return static_cast<QLineEdit *>(widget)->text().trimmed();
    case pl::ParameterType::integer_list: {
      QJsonArray a;
      for (const QString &part : static_cast<QLineEdit *>(widget)->text().split(
               QRegularExpression("[,\\s]+"), Qt::SkipEmptyParts)) {
        bool ok = false;
        const int v = part.toInt(&ok);
        if (ok)
          a.append(v);
      }
      return a;
    }
    }
    return {};
  }

  /// Sent to the stage? Only what differs from the default, and a
  /// presence-sensitive key only when pinned (#214).
  bool included() const {
    if (d->presence_sensitive)
      return pin && pin->isChecked();
    return !is_default(*d, value());
  }

  void set(const QVariant &v) {
    switch (d->type) {
    case pl::ParameterType::number:
      static_cast<QDoubleSpinBox *>(widget)->setValue(v.toDouble());
      break;
    case pl::ParameterType::integer:
      static_cast<QSpinBox *>(widget)->setValue(v.toInt());
      break;
    case pl::ParameterType::boolean:
      static_cast<QCheckBox *>(widget)->setChecked(v.toBool());
      break;
    case pl::ParameterType::string:
      if (auto *c = qobject_cast<QComboBox *>(widget))
        c->setCurrentText(v.toString());
      else
        static_cast<QLineEdit *>(widget)->setText(v.toString());
      break;
    case pl::ParameterType::integer_list:
      static_cast<QLineEdit *>(widget)->setText(v.toString());
      break;
    }
    if (pin)
      pin->setChecked(true);
  }

  void reset() {
    if (pin)
      pin->setChecked(false);
    const auto &def = d->default_value;
    switch (d->type) {
    case pl::ParameterType::number:
      static_cast<QDoubleSpinBox *>(widget)->setValue(std::get<double>(def));
      break;
    case pl::ParameterType::integer:
      static_cast<QSpinBox *>(widget)->setValue(
          static_cast<int>(std::get<long long>(def)));
      break;
    case pl::ParameterType::boolean:
      static_cast<QCheckBox *>(widget)->setChecked(std::get<bool>(def));
      break;
    case pl::ParameterType::string: {
      const std::string *s = std::get_if<std::string>(&def);
      if (auto *c = qobject_cast<QComboBox *>(widget))
        c->setCurrentText(s ? qs(*s) : QString());
      else
        static_cast<QLineEdit *>(widget)->setText(s ? qs(*s) : QString());
      break;
    }
    case pl::ParameterType::integer_list:
      static_cast<QLineEdit *>(widget)->clear();
      break;
    }
  }
};

/// Log lines of the running job, collected on any thread, drained on the
/// GUI thread by a timer.
struct PipelineWorkspace::LogBuffer {
  std::mutex mutex;
  QStringList lines;
  std::atomic<bool> capture{false};
};

PipelineWorkspace::PipelineWorkspace(ProjectSession &session,
                                     pl::StageExecutor executor,
                                     QWidget *parent)
    : QWidget(parent), session_(session), executor_(std::move(executor)),
      log_(std::make_shared<LogBuffer>()) {
  setObjectName("pipelineWorkspace");
  const Theme &t = theme();
  auto *body = new QHBoxLayout(this);
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(0);

  // ---- stage list
  auto *left = new QFrame;
  left->setObjectName("treePane");
  left->setFixedWidth(t.px("--layout-panel-width"));
  auto *ll = new QVBoxLayout(left);
  ll->setContentsMargins(0, t.px("--space-3"), 0, 0);
  ll->setSpacing(t.px("--space-2"));
  auto *eyebrow = new CapsLabel("Trin", "eyebrowSurface", "--tracking-wide");
  eyebrow->setContentsMargins(t.px("--space-4"), 0, 0, 0);
  ll->addWidget(eyebrow);
  stages_ = new QWidget;
  stages_->setObjectName("stageList");
  auto *sl0 = new QVBoxLayout(stages_);
  sl0->setContentsMargins(t.px("--space-2"), 0, t.px("--space-2"), 0);
  sl0->setSpacing(t.px("--space-1"));
  ll->addWidget(stages_);
  ll->addStretch(1);
  body->addWidget(left);

  // ---- centre
  auto *scroll = new QScrollArea;
  scroll_ = scroll;
  scroll->setObjectName("pipelineScroll");
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidgetResizable(true);
  auto *content = new QWidget;
  content->setObjectName("pipelineContent");
  auto *cv = new QVBoxLayout(content);
  const int m = t.px("--space-6");
  cv->setContentsMargins(m, t.px("--space-5"), m, m);
  cv->setSpacing(t.px("--space-4"));

  auto *head = new QHBoxLayout;
  head->setSpacing(t.px("--space-3"));
  title_ = new QLabel;
  title_->setObjectName("stageTitle");
  head->addWidget(title_, 0, Qt::AlignBottom);
  ready_ = new QLabel;
  ready_->setObjectName("stageReady");
  head->addWidget(ready_, 0, Qt::AlignBottom);
  head->addStretch(1);
  cv->addLayout(head);
  blurb_ = new QLabel;
  blurb_->setObjectName("viewSub");
  blurb_->setWordWrap(true);
  cv->addWidget(blurb_);

  auto *params = new Panel("Parametre");
  form_ = new QWidget;
  form_grid_ = new QGridLayout(form_);
  form_grid_->setContentsMargins(0, 0, 0, 0);
  form_grid_->setHorizontalSpacing(t.px("--space-4"));
  form_grid_->setVerticalSpacing(t.px("--space-2"));
  params->body()->addWidget(form_);
  defaults_ = new QPushButton(NavItem::escape_mnemonic("Standardværdier"));
  defaults_->setProperty("kind", "ghost");
  defaults_->setCursor(Qt::PointingHandCursor);
  defaults_->setToolTip("Sæt alle felter tilbage til trinnets standard");
  params->add_header_widget(defaults_);
  cv->addWidget(params);

  auto *actions = new QHBoxLayout;
  actions->setSpacing(t.px("--space-3"));
  run_ = new QPushButton(NavItem::escape_mnemonic("Kør trin"));
  run_->setProperty("kind", "primary");
  run_->setCursor(Qt::PointingHandCursor);
  cancel_ = new QPushButton(NavItem::escape_mnemonic("Annullér"));
  cancel_->setProperty("kind", "secondary");
  cancel_->setCursor(Qt::PointingHandCursor);
  cancel_->setEnabled(false);
  for (QPushButton *b : {run_, cancel_})
    b->setMinimumHeight(b->sizeHint().height());
  actions->addWidget(run_);
  actions->addWidget(cancel_);
  actions->addStretch(1);
  cv->addLayout(actions);

  auto *cli = new Panel("Som rux-kommando");
  copy_ = new QPushButton(NavItem::escape_mnemonic("Kopiér"));
  copy_->setProperty("kind", "secondary");
  copy_->setCursor(Qt::PointingHandCursor);
  copy_->setToolTip("Kopiér kommandoen til udklipsholderen");
  cli->add_header_widget(copy_);
  command_ = new QLabel;
  command_->setObjectName("commandBlock");
  command_->setWordWrap(true);
  command_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  cli->body()->addWidget(command_);
  command_note_ = new QLabel;
  command_note_->setObjectName("layerHint");
  command_note_->setWordWrap(true);
  cli->body()->addWidget(command_note_);
  cv->addWidget(cli);

  auto *job = new Panel("Kørsel");
  job_panel_ = job;
  auto *jh = new QHBoxLayout;
  jh->setSpacing(t.px("--space-3"));
  state_ = new Pill("Ikke startet", "outline");
  jh->addWidget(state_);
  progress_text_ = new QLabel;
  progress_text_->setObjectName("jobMeta");
  jh->addWidget(progress_text_, 1);
  job->body()->addLayout(jh);
  progress_ = new QProgressBar;
  progress_->setTextVisible(false);
  progress_->setRange(0, 1);
  progress_->setValue(0);
  job->body()->addWidget(progress_);
  tail_ = new QPlainTextEdit;
  tail_->setObjectName("logTail");
  tail_->setReadOnly(true);
  tail_->setMaximumBlockCount(2000);
  tail_->setLineWrapMode(QPlainTextEdit::NoWrap);
  tail_->setPlaceholderText("Trinnets log vises her, mens det kører.");
  tail_->setMinimumHeight(t.px("--layout-panel-width") / 2);
  job->body()->addWidget(tail_);
  cv->addWidget(job);
  cv->addStretch(1);

  scroll->setWidget(content);
  body->addWidget(scroll, 1);

  // Log tap: lines while a job runs, drained every few frames.
  std::weak_ptr<LogBuffer> weak = log_;
  log_token_ = add_log_listener([weak](int level, std::string_view msg) {
    auto buf = weak.lock();
    if (!buf || !buf->capture.load() || level < 2) // info and up
      return;
    static const char *tags[] = {"T", "D", "I", "W", "E", "C", ""};
    std::lock_guard lock(buf->mutex);
    buf->lines << QString("%1  %2")
                      .arg(tags[std::clamp(level, 0, 6)])
                      .arg(QString::fromUtf8(msg.data(),
                                             static_cast<int>(msg.size())));
  });
  log_timer_ = new QTimer(this);
  log_timer_->setInterval(120);
  connect(log_timer_, &QTimer::timeout, this, &PipelineWorkspace::drain_log);
  log_timer_->start();

  connect(run_, &QPushButton::clicked, this, [this] { run(); });
  connect(cancel_, &QPushButton::clicked, this, [this] {
    if (runner_ && !job_id_.empty())
      runner_->cancel(job_id_);
  });
  connect(defaults_, &QPushButton::clicked, this, [this] {
    for (auto &f : fields_)
      f->reset();
    refresh_command();
  });
  connect(copy_, &QPushButton::clicked, this, [this] {
    QApplication::clipboard()->setText(QString::fromStdString(command().text));
    copy_->setText("Kopieret");
    QTimer::singleShot(1500, copy_, [this] { copy_->setText("Kopiér"); });
  });
  connect(&session_, &ProjectSession::state_changed, this,
          &PipelineWorkspace::reset);
  reset();
}

PipelineWorkspace::~PipelineWorkspace() {
  remove_log_listener(log_token_);
  log_->capture = false;
  runner_.reset(); // cancels and joins a running job
  busy_.reset();
}

pl::JobRunner &PipelineWorkspace::runner() {
  const std::string path = session_.path().toStdString();
  if (!runner_ || runner_path_ != path) {
    runner_.reset(); // cancels and joins anything the old project ran
    runner_ = std::make_unique<pl::JobRunner>(
        path, executor_ ? executor_ : pl::default_stage_executor());
    runner_path_ = path;
    QPointer<PipelineWorkspace> guard(this);
    runner_->add_listener([guard](const pl::JobEvent &e) {
      QMetaObject::invokeMethod(
          qApp,
          [guard, e] {
            if (guard)
              guard->on_event(e);
          },
          Qt::QueuedConnection);
    });
  }
  return *runner_;
}

void PipelineWorkspace::reset() {
  rebuild_stages();
  // A reload of the same project (after a run) keeps what was typed.
  const QString path = session_.path();
  if (session_.state() == ProjectSession::State::loading ||
      (path == form_path_ && !fields_.empty())) {
    refresh_readiness();
    refresh_command();
    return;
  }
  form_path_ = path;
  rebuild_form();
}

void PipelineWorkspace::rebuild_stages() {
  const Theme &t = theme();
  auto *list = static_cast<QVBoxLayout *>(stages_->layout());
  while (QLayoutItem *it = list->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }
  std::vector<reusex::ProjectDB::PipelineLogEntry> log;
  if (const auto *db = session_.db()) {
    try {
      log = db->pipeline_log();
    } catch (const std::exception &) {
    }
  }
  for (const auto &name : pl::job_stage_names()) {
    const auto s = *pl::parse_job_stage(name);
    // A row is a button holding labels (qt-client.md: give it a minimum
    // height from its layout).
    auto *row = new QPushButton;
    row->setObjectName("stageRow");
    row->setCheckable(true);
    row->setChecked(s == stage_);
    row->setCursor(Qt::PointingHandCursor);
    row->setProperty("stage", static_cast<int>(s));
    auto *h = new QHBoxLayout(row);
    h->setContentsMargins(t.px("--space-3"), t.px("--space-2"),
                          t.px("--space-2"), t.px("--space-2"));
    h->setSpacing(t.px("--space-2"));
    auto *texts = new QVBoxLayout;
    texts->setSpacing(0);
    auto *n = new QLabel(stage_name_da(s));
    n->setObjectName("stageRowName");
    texts->addWidget(n);
    auto *c = new QLabel(QString("rux %1").arg(
        s == pl::JobStage::optimize ? QString("optimize")
                                    : QString("create %1").arg(qs(name))));
    c->setObjectName("stageRowCmd");
    texts->addWidget(c);
    h->addLayout(texts, 1);
    // The last run of this stage, from pipeline_log.
    const std::string log_name(pl::pipeline_log_name(s));
    const reusex::ProjectDB::PipelineLogEntry *last = nullptr;
    for (const auto &e : log)
      if (e.stage == log_name && (!last || e.id > last->id))
        last = &e;
    if (last) {
      const bool ok = last->status == "success";
      const bool failed = last->status == "failed";
      auto *pill = new Pill(ok       ? "Kørt"
                            : failed ? "Fejlet"
                                     : "Afbrudt",
                            ok       ? "good"
                            : failed ? "crit"
                                     : "wait");
      pill->setToolTip(QString("Sidste kørsel: #%1").arg(last->id));
      h->addWidget(pill, 0, Qt::AlignVCenter);
    }
    for (QWidget *l : row->findChildren<QWidget *>())
      l->setAttribute(Qt::WA_TransparentForMouseEvents);
    row->setMinimumHeight(h->sizeHint().height());
    row->setEnabled(session_.is_open());
    connect(row, &QPushButton::clicked, this, [this, s] { select_stage(s); });
    list->addWidget(row);
  }
}

void PipelineWorkspace::select_stage(pl::JobStage stage) {
  stage_ = stage;
  for (auto *b : stages_->findChildren<QPushButton *>("stageRow"))
    b->setChecked(b->property("stage").toInt() == static_cast<int>(stage));
  rebuild_form();
}

void PipelineWorkspace::rebuild_form() {
  const Theme &t = theme();
  title_->setText(stage_name_da(stage_));
  blurb_->setText(stage_blurb_da(stage_));
  while (QLayoutItem *it = form_grid_->takeAt(0)) {
    if (QWidget *w = it->widget()) {
      w->hide();
      w->deleteLater();
    }
    delete it;
  }
  fields_.clear();
  int row = 0;
  for (const auto &d : pl::stage_parameters(stage_)) {
    auto f = std::make_unique<Field>();
    f->d = &d;
    const QString tip = QString::fromStdString(d.description);
    auto *label = new QLabel(parameter_label_da(stage_, d));
    label->setObjectName("paramLabel");
    label->setToolTip(tip);
    QWidget *w = nullptr;
    switch (d.type) {
    case pl::ParameterType::number: {
      auto *s = new QDoubleSpinBox;
      s->setLocale(da());
      const double def = std::get<double>(d.default_value);
      s->setDecimals(decimals_for(def));
      s->setRange(d.minimum.value_or(def >= 0 ? 0.0 : -1e9),
                  d.maximum.value_or(1e9));
      s->setSingleStep(
          def != 0 ? std::pow(10.0, std::floor(std::log10(std::abs(def)))) / 2
                   : 0.1);
      s->setValue(def);
      connect(s, &QDoubleSpinBox::valueChanged, this,
              [this] { refresh_command(); });
      w = s;
      break;
    }
    case pl::ParameterType::integer: {
      auto *s = new QSpinBox;
      s->setLocale(da());
      s->setRange(static_cast<int>(d.minimum.value_or(INT_MIN / 2)),
                  static_cast<int>(d.maximum.value_or(INT_MAX)));
      s->setValue(static_cast<int>(std::get<long long>(d.default_value)));
      connect(s, &QSpinBox::valueChanged, this, [this] { refresh_command(); });
      w = s;
      break;
    }
    case pl::ParameterType::boolean: {
      auto *c = new QCheckBox;
      c->setChecked(std::get<bool>(d.default_value));
      connect(c, &QCheckBox::toggled, this, [this] { refresh_command(); });
      w = c;
      break;
    }
    case pl::ParameterType::string: {
      if (d.key == "solver") {
        auto *c = new QComboBox;
        c->addItems({"auto", "highs", "cuopt"});
        c->setCurrentText(qs(std::get<std::string>(d.default_value)));
        connect(c, &QComboBox::currentTextChanged, this,
                [this] { refresh_command(); });
        w = c;
      } else {
        auto *e = new QLineEdit;
        if (const auto *s = std::get_if<std::string>(&d.default_value))
          e->setText(qs(*s));
        if (d.key == "filter")
          e->setPlaceholderText("f.eks. rooms == 3");
        connect(e, &QLineEdit::textChanged, this,
                [this] { refresh_command(); });
        w = e;
      }
      break;
    }
    case pl::ParameterType::integer_list: {
      auto *e = new QLineEdit;
      e->setPlaceholderText("alle — eller f.eks. 1, 4, 7");
      connect(e, &QLineEdit::textChanged, this, [this] { refresh_command(); });
      w = e;
      break;
    }
    }
    w->setObjectName("paramField");
    w->setMaximumWidth(t.px("--layout-panel-width"));
    if (auto *sb = qobject_cast<QAbstractSpinBox *>(w))
      sb->setButtonSymbols(QAbstractSpinBox::NoButtons);
    w->setToolTip(tip);
    f->widget = w;
    form_grid_->addWidget(label, row, 0);
    form_grid_->addWidget(w, row, 1);
    if (d.presence_sensitive) {
      // Pinned: sent, which switches adaptive derivation off for this one
      // parameter (#214). Unpinned: the stage derives it from the noise.
      auto *pin = new QCheckBox(NavItem::escape_mnemonic("Fastlås"));
      pin->setObjectName("paramPin");
      pin->setToolTip("Send værdien. Ellers udleder trinnet den selv af "
                      "skyens støj (adaptiv).");
      w->setEnabled(false);
      connect(pin, &QCheckBox::toggled, w, &QWidget::setEnabled);
      connect(pin, &QCheckBox::toggled, this, [this] { refresh_command(); });
      f->pin = pin;
      form_grid_->addWidget(pin, row, 2);
    } else {
      auto *hint = new QLabel(QString("standard %1").arg(default_text(d)));
      hint->setObjectName("paramDefault");
      form_grid_->addWidget(hint, row, 2);
    }
    ++row;
    fields_.push_back(std::move(f));
  }
  form_grid_->setColumnStretch(3, 1);
  form_grid_->setColumnMinimumWidth(1, t.px("--layout-panel-width") / 2);
  refresh_readiness();
  refresh_command();
}

void PipelineWorkspace::refresh_readiness() {
  const reusex::ProjectDB *db = session_.db();
  bool ready = false;
  QString text;
  if (!db) {
    text = "Intet projekt åbent";
  } else if (session_.is_read_only()) {
    text = "Projektet er skrivebeskyttet";
  } else {
    try {
      const auto report =
          reusex::core::validate_stage(*db, contract_stage(stage_));
      ready = report.ok();
      if (!ready) {
        QStringList missing;
        QString hint;
        for (const auto &i : report.issues) {
          if (i.severity != reusex::core::ValidationSeverity::error)
            continue;
          if (!i.artifact.empty())
            missing << qs(i.artifact);
          if (hint.isEmpty() && !i.commands.empty())
            hint = qs(i.commands.front());
        }
        missing.removeDuplicates();
        text = QString("Mangler %1").arg(missing.join(", "));
        if (!hint.isEmpty())
          text += QString(" — kør først %1").arg(hint);
      }
    } catch (const std::exception &e) {
      text = QString::fromUtf8(e.what());
    }
  }
  ready_->setText(ready ? QString("Klar til at køre") : text);
  ready_->setProperty("ready", ready);
  repolish(ready_);
  run_->setEnabled(ready && !is_running());
  run_->setToolTip(ready ? QString() : text);
}

QJsonObject PipelineWorkspace::parameters() const {
  QJsonObject o;
  for (const auto &f : fields_)
    if (f->included())
      o.insert(qs(f->d->key), f->value());
  return o;
}

CliCommand PipelineWorkspace::command() const {
  std::vector<CliParam> params;
  for (const auto &f : fields_)
    if (f->included())
      if (auto p = cli_param(*f->d, f->value()))
        params.push_back(*p);
  return build_cli_command(pl::to_string(stage_), session_.path().toStdString(),
                           params);
}

void PipelineWorkspace::refresh_command() {
  const CliCommand cmd = command();
  command_->setText(QString::fromStdString(cmd.text));
  QStringList notes;
  if (!cmd.unmapped.empty()) {
    QStringList keys;
    for (const auto &k : cmd.unmapped)
      keys << qs(k);
    notes << QString("%1 har intet flag i rux — kommandoen bruger trinnets "
                     "standard for det.")
                 .arg(keys.join(", "));
  }
  if (stage_ == pl::JobStage::mesh)
    notes << "rux create mesh løser selv cellekomplekset; flagene svarer til "
             "trinnets parametre her.";
  command_note_->setText(notes.join(" "));
  command_note_->setVisible(!notes.isEmpty());
}

bool PipelineWorkspace::set_field(const QString &key, const QVariant &value) {
  for (auto &f : fields_)
    if (qs(f->d->key) == key) {
      f->set(value);
      refresh_command();
      return true;
    }
  return false;
}

bool PipelineWorkspace::is_running() const { return !job_id_.empty(); }

bool PipelineWorkspace::run() {
  refresh_readiness();
  if (!run_->isEnabled())
    return false;
  const QString params = QString::fromUtf8(
      QJsonDocument(parameters()).toJson(QJsonDocument::Compact));
  try {
    tail_->clear();
    {
      std::lock_guard lock(log_->mutex);
      log_->lines.clear();
    }
    log_->capture = true;
    job_stage_ = stage_;
    job_id_ = runner().submit(stage_, params.toStdString());
    busy_ = std::make_unique<BackgroundWork>();
  } catch (const std::exception &e) {
    log_->capture = false;
    state_->setText("Fejlet");
    state_->setProperty("tone", "crit");
    repolish(state_);
    progress_text_->setText(QString::fromUtf8(e.what()));
    return false;
  }
  // Bring the run into view: its progress and log are what matter now.
  QTimer::singleShot(0, this, [this] {
    scroll_->ensureWidgetVisible(job_panel_, 0, theme().px("--space-6"));
  });
  state_->setText("I kø");
  state_->setProperty("tone", "wait");
  repolish(state_);
  progress_->setProperty("state", QVariant());
  repolish(progress_);
  progress_->setRange(0, 0);
  progress_text_->setText(
      QString("%1 · %2").arg(stage_name_da(stage_)).arg(params));
  run_->setEnabled(false);
  const bool can_cancel = pl::stage_supports_cancellation(stage_);
  cancel_->setEnabled(true);
  cancel_->setToolTip(
      can_cancel ? QString("Stop trinnet ved næste kontrolpunkt")
                 : QString("Dette trin stopper først mellem to "
                           "skridt; en kørende løsning afbrydes ikke"));
  return true;
}

void PipelineWorkspace::on_event(const pl::JobEvent &e) {
  if (e.job.id != job_id_)
    return;
  const pl::JobRecord &j = e.job;
  switch (j.status) {
  case pl::JobStatus::queued:
    break;
  case pl::JobStatus::running: {
    state_->setText("Kører");
    state_->setProperty("tone", "accent");
    repolish(state_);
    if (j.progress_total > 0) {
      progress_->setRange(0, static_cast<int>(std::min<std::size_t>(
                                 j.progress_total, INT_MAX)));
      progress_->setValue(
          static_cast<int>(std::min<std::size_t>(j.progress_current, INT_MAX)));
      progress_text_->setText(
          QString("%1 · %2 af %3")
              .arg(QString::fromUtf8(
                  reusex::core::to_string(j.progress_stage).data()))
              .arg(format_count(j.progress_current))
              .arg(format_count(j.progress_total)));
    } else {
      progress_->setRange(0, 0);
      if (j.progress_stage != reusex::core::Stage::idle)
        progress_text_->setText(QString::fromStdString(
            std::string(reusex::core::to_string(j.progress_stage))));
    }
    break;
  }
  case pl::JobStatus::succeeded:
  case pl::JobStatus::failed:
  case pl::JobStatus::cancelled: {
    drain_log();
    log_->capture = false;
    const bool ok = j.status == pl::JobStatus::succeeded;
    const bool failed = j.status == pl::JobStatus::failed;
    state_->setText(ok ? "Gennemført" : failed ? "Fejlet" : "Annulleret");
    state_->setProperty("tone", ok ? "good" : failed ? "crit" : "wait");
    repolish(state_);
    progress_->setRange(0, 1);
    progress_->setValue(1);
    progress_->setProperty("state", ok ? "succeeded" : "failed");
    repolish(progress_);
    progress_text_->setText(ok ? qs(j.result_summary) : qs(j.error));
    cancel_->setEnabled(false);
    job_id_.clear();
    busy_.reset();

    Selection s;
    s.kind = "Kørsel";
    s.title = stage_name_da(job_stage_);
    s.subtitle = QString("I programmet · job %1").arg(qs(j.id).left(8));
    s.pill = state_->text();
    s.pill_tone = ok ? "good" : failed ? "crit" : "wait";
    SelectionSection res{"Resultat", {}, {}};
    if (!j.started_at.empty())
      res.rows.push_back({"Startet", qs(j.started_at)});
    if (!j.finished_at.empty())
      res.rows.push_back({"Afsluttet", qs(j.finished_at)});
    for (const auto &o : j.result_outputs)
      res.rows.push_back({qs(o.name),
                          o.count >= 0
                              ? format_count(static_cast<qulonglong>(o.count))
                              : qs(o.kind),
                          SelectionRow::Style::name});
    s.sections.push_back(res);
    s.sections.push_back(
        {"Besked", {}, ok ? qs(j.result_summary) : qs(j.error)});
    s.sections.push_back(
        {"Som rux-kommando",
         {},
         QString::fromStdString(
             cli_for_parameters(job_stage_, qs(j.parameters), session_.path())
                 .text)});
    selection_ = s;
    emit selection_changed(selection_);
    if (ok)
      emit project_changed();
    rebuild_stages();
    refresh_readiness();
    break;
  }
  }
}

void PipelineWorkspace::drain_log() {
  QStringList lines;
  {
    std::lock_guard lock(log_->mutex);
    lines.swap(log_->lines);
  }
  for (const QString &l : lines)
    tail_->appendPlainText(l);
}

} // namespace rux::qt
