#include "JointControlPanel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QSaveFile>
#include <QSlider>
#include <QStandardPaths>
#include <QVBoxLayout>
#include <cmath>

namespace vrmc {

JointControlPanel::JointControlPanel(QWidget* parent) : QWidget(parent)
{
    m_label = new QLabel(tr("(no slave selected)"), this);

    /* Readout strip: three labels rendered in a monospaced row right
     * under the slave label. Order matches the cause-and-effect chain:
     *   - state: what the slave is reporting (statusword decoded)
     *   - cw:    what the master is currently telling it (cached on
     *            this side because cia402_master doesn't expose a
     *            get_controlword and we don't echo via TPDO)
     *   - mode:  what we asked for (requested mode; mode_actual would
     *            need a TPDO extension)
     */
    m_state      = new QLabel(tr("state: —"),  this);
    m_cwLabel    = new QLabel(tr("cw: —"),     this);
    m_modeLabel  = new QLabel(tr("mode: —"),   this);
    m_errorLabel = new QLabel(QString(),       this);   /* hidden until error_code != 0 */
    m_trackLabel = new QLabel(QString(),       this);   /* hidden until pdo fresh       */
    {
        QFont f = m_state->font();
        f.setFamily(QStringLiteral("monospace"));
        for (auto* l : { m_state, m_cwLabel, m_modeLabel,
                         m_errorLabel, m_trackLabel }){ l->setFont(f); }
    }
    m_errorLabel->setVisible(false);
    m_trackLabel->setVisible(false);

    m_bringup     = new QPushButton(tr("Bringup"),    this);
    m_enable      = new QPushButton(tr("Enable"),     this);
    m_disable     = new QPushButton(tr("Disable"),    this);
    m_quickStop   = new QPushButton(tr("Quick Stop"), this);
    m_faultReset  = new QPushButton(tr("Fault reset"),this);
    m_brake       = new QPushButton(tr("Brake"),      this);
    m_brake->setCheckable(true);

    /* Quick Stop is highlighted amber so it reads as "controlled stop"
     * — different from the red E-STOP in the top toolbar (which is the
     * panic kill that disarms PDO + walks every slave to Disable). */
    m_quickStop->setStyleSheet(QStringLiteral(
        "QPushButton { background-color: #d68000; color: white; font-weight: 600; }"
        "QPushButton:disabled { background-color: #5e3a00; color: #aaa; }"));

    /* Brake reads as "held / not held"; checked = engaged (red border). */
    m_brake->setStyleSheet(QStringLiteral(
        "QPushButton:checked { background-color: #b03030; color: white; font-weight: 600; }"
        "QPushButton:disabled { color: #888; }"));

    m_modeCombo = new QComboBox(this);
    m_modeCombo->addItem(tr("NONE"),     int(Mode::None));
    m_modeCombo->addItem(tr("TORQUE"),   int(Mode::Torque));
    m_modeCombo->addItem(tr("VELOCITY"), int(Mode::Velocity));
    m_modeCombo->addItem(tr("POSITION"), int(Mode::Position));
    /* Default to POSITION — most common diagnostic workflow. Without
     * this the combo defaults to NONE, the drive's 0x6060 never gets
     * written when the user clicks Bringup, vmotor.mode stays at
     * whatever leftover value it had, and Position setpoints don't
     * actually steer the motor. */
    m_modeCombo->setCurrentIndex(m_modeCombo->findData(int(Mode::Position)));

    m_targetCombo = new QComboBox(this);
    m_targetCombo->addItem(tr("Position (rad)"),   int(TargetKind::Position));
    m_targetCombo->addItem(tr("Velocity (rad/s)"), int(TargetKind::Velocity));
    m_targetCombo->addItem(tr("Torque (Nm)"),      int(TargetKind::Torque));

    m_valueSpin = new QDoubleSpinBox(this);
    m_valueSpin->setRange(-10000.0, 10000.0);
    m_valueSpin->setDecimals(8);
    m_valueSpin->setSingleStep(0.01);

    m_sendBtn = new QPushButton(tr("Send"), this);

    auto* powerRow = new QHBoxLayout;
    powerRow->addWidget(m_bringup);
    powerRow->addWidget(m_enable);
    powerRow->addWidget(m_disable);
    powerRow->addWidget(m_quickStop);
    powerRow->addWidget(m_faultReset);
    powerRow->addWidget(m_brake);
    powerRow->addStretch();

    auto* modeBox = new QGroupBox(tr("Mode"), this);
    {
        auto* l = new QHBoxLayout(modeBox);
        l->addWidget(m_modeCombo);
        auto* apply = new QPushButton(tr("Apply"), modeBox);
        l->addWidget(apply);
        l->addStretch();
        connect(apply, &QPushButton::clicked, this, [this]{
            if (m_idx < 0){ return; }
            emit modeRequested(m_idx,
                static_cast<Mode>(m_modeCombo->currentData().toInt()));
        });
    }

    /* Preset value buttons — set spinbox + Send in one click. The
     * default set targets POSITION-mode rad values; in TORQUE or
     * VELOCITY mode they still apply but mean a different unit (and
     * `Home` is always 0 by convention). */
    m_preset0    = new QPushButton(QStringLiteral("0"),     this);
    m_presetQp   = new QPushButton(QStringLiteral("+π/4"),  this);
    m_presetQn   = new QPushButton(QStringLiteral("−π/4"),  this);
    m_presetHome = new QPushButton(tr("Home"),              this);
    for (auto* b : { m_preset0, m_presetQp, m_presetQn, m_presetHome }){
        b->setMaximumWidth(60);
    }

    /* Jog row — incremental ± of the current spinbox value, then Send.
     * Step defaults to 0.1 (rad / rad/s / Nm depending on mode). */
    m_jogMinus = new QPushButton(QStringLiteral("−"), this);
    m_jogPlus  = new QPushButton(QStringLiteral("+"), this);
    m_jogStep  = new QDoubleSpinBox(this);
    m_jogStep->setRange(0.001, 100.0);
    m_jogStep->setDecimals(3);
    m_jogStep->setSingleStep(0.01);
    m_jogStep->setValue(0.1);
    for (auto* b : { m_jogMinus, m_jogPlus }){
        b->setMaximumWidth(40);
        b->setStyleSheet(QStringLiteral("QPushButton { font-weight: 700; font-size: 14px; }"));
    }

    /* Slider + live-stream checkbox — operator can sweep the value
     * smoothly. Slider range is fixed per target kind (resyncSliderRange
     * recomputes on mode change). int-backed at 1/1000 of a unit so a
     * full ±π position sweep is ~6300 ticks. */
    m_setpointSlider = new QSlider(Qt::Horizontal, this);
    m_setpointSlider->setRange(-3141, 3141);                /* default Position range */
    m_setpointSlider->setSingleStep(10);
    m_liveStream = new QCheckBox(tr("stream live"), this);
    m_liveStream->setToolTip(tr("Send setpoint on every slider drag, "
                                "not just on Send."));

    /* Named-preset combobox (Phase C.5). Persists the (mode,
     * target_kind, value) triple under a user-typed name in
     * ~/.config/vrmc/control_presets.json. */
    m_presetCombo   = new QComboBox(this);
    m_presetSaveBtn = new QPushButton(tr("Save…"), this);
    m_presetDelBtn  = new QPushButton(tr("Delete"), this);

    auto* tgtBox = new QGroupBox(tr("Setpoint"), this);
    {
        auto* l = new QGridLayout(tgtBox);
        l->addWidget(m_targetCombo, 0, 0);
        l->addWidget(m_valueSpin,   0, 1);
        l->addWidget(m_sendBtn,     0, 2);
        connect(m_sendBtn, &QPushButton::clicked, this, &JointControlPanel::emitTarget);

        /* Slider row spans 3 columns under the combo+spinbox+Send. */
        auto* sliderRow = new QHBoxLayout;
        sliderRow->addWidget(m_setpointSlider, /*stretch=*/1);
        sliderRow->addWidget(m_liveStream);
        l->addLayout(sliderRow, 1, 0, 1, 3);

        /* Preset row sits under the slider. */
        auto* presetsRow = new QHBoxLayout;
        presetsRow->addWidget(new QLabel(tr("Presets:"), tgtBox));
        presetsRow->addWidget(m_preset0);
        presetsRow->addWidget(m_presetQp);
        presetsRow->addWidget(m_presetQn);
        presetsRow->addWidget(m_presetHome);
        presetsRow->addStretch();
        l->addLayout(presetsRow, 2, 0, 1, 3);

        /* Jog row underneath. */
        auto* jogRow = new QHBoxLayout;
        jogRow->addWidget(new QLabel(tr("Jog:"), tgtBox));
        jogRow->addWidget(m_jogMinus);
        jogRow->addWidget(m_jogStep);
        jogRow->addWidget(m_jogPlus);
        jogRow->addStretch();
        l->addLayout(jogRow, 3, 0, 1, 3);

        /* Named-preset row. */
        auto* savedRow = new QHBoxLayout;
        savedRow->addWidget(new QLabel(tr("Saved:"), tgtBox));
        savedRow->addWidget(m_presetCombo, /*stretch=*/1);
        savedRow->addWidget(m_presetSaveBtn);
        savedRow->addWidget(m_presetDelBtn);
        l->addLayout(savedRow, 4, 0, 1, 3);
    }

    /* Slider ↔ spinbox bidirectional sync. */
    connect(m_setpointSlider, &QSlider::valueChanged,
            this, &JointControlPanel::onSliderChanged);
    connect(m_valueSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &JointControlPanel::onSpinboxChanged);

    /* Named-preset wiring. */
    connect(m_presetCombo,
            QOverload<int>::of(&QComboBox::activated),
            this, [this](int){
        if (m_presetCombo->currentData().isNull()){ return; }
        const auto v = m_presetCombo->currentData().toMap();
        const Mode mode = static_cast<Mode>(v.value("mode", 0).toInt());
        const TargetKind kind = static_cast<TargetKind>(
            v.value("target_kind", 0).toInt());
        const double value = v.value("value", 0.0).toDouble();
        /* Apply mode (and unit suffix) first, then push value + send. */
        const int mi = m_modeCombo->findData(int(mode));
        if (mi >= 0){ m_modeCombo->setCurrentIndex(mi); }
        const int ti = m_targetCombo->findData(int(kind));
        if (ti >= 0){ m_targetCombo->setCurrentIndex(ti); }
        m_valueSpin->setValue(value);
        sendTarget(value);
    });
    connect(m_presetSaveBtn, &QPushButton::clicked, this, [this]{
        bool ok = false;
        const QString name = QInputDialog::getText(
            this, tr("Save preset"), tr("Name:"),
            QLineEdit::Normal, QString(), &ok);
        if (!ok || name.isEmpty()){ return; }
        QVariantMap rec;
        rec["mode"]        = m_modeCombo  ->currentData().toInt();
        rec["target_kind"] = m_targetCombo->currentData().toInt();
        rec["value"]       = m_valueSpin  ->value();
        /* Replace if existing, else append. */
        const int existing = m_presetCombo->findText(name);
        if (existing >= 0){
            m_presetCombo->setItemData(existing, rec);
        } else {
            m_presetCombo->addItem(name, rec);
            m_presetCombo->setCurrentIndex(m_presetCombo->count() - 1);
        }
        savePresetStore();
    });
    connect(m_presetDelBtn, &QPushButton::clicked, this, [this]{
        const int i = m_presetCombo->currentIndex();
        if (i < 0){ return; }
        m_presetCombo->removeItem(i);
        savePresetStore();
    });
    loadPresetStore();

    /* Preset button wiring: each preset value is canonical-rad and we
     * convert into whatever the spinbox is currently displaying (rad
     * or deg). Set spinbox and route through sendTarget which converts
     * back to rad before emitting. 0 / Home are unit-agnostic. */
    const double kQuarterPi = M_PI / 4.0;
    connect(m_preset0,    &QPushButton::clicked, this, [this]{
        m_valueSpin->setValue(0.0);  sendTarget(0.0);
    });
    connect(m_presetQp,   &QPushButton::clicked, this, [this, kQuarterPi]{
        const double v = +kQuarterPi * displayFactor();
        m_valueSpin->setValue(v);  sendTarget(v);
    });
    connect(m_presetQn,   &QPushButton::clicked, this, [this, kQuarterPi]{
        const double v = -kQuarterPi * displayFactor();
        m_valueSpin->setValue(v);  sendTarget(v);
    });
    connect(m_presetHome, &QPushButton::clicked, this, [this]{
        m_valueSpin->setValue(0.0);  sendTarget(0.0);
    });

    /* Jog wiring: increment/decrement spinbox by step and send. */
    connect(m_jogMinus, &QPushButton::clicked, this, [this]{
        const double v = m_valueSpin->value() - m_jogStep->value();
        m_valueSpin->setValue(v);
        sendTarget(v);
    });
    connect(m_jogPlus, &QPushButton::clicked, this, [this]{
        const double v = m_valueSpin->value() + m_jogStep->value();
        m_valueSpin->setValue(v);
        sendTarget(v);
    });

    /* Mode-aware setpoint kind + unit. When the operator picks a new
     * mode (POSITION / VELOCITY / TORQUE), automatically switch the
     * target combo to the matching kind and update the spinbox suffix
     * so unit labelling stays consistent. NONE leaves things as-is. */
    connect(m_modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int){ syncTargetKindToMode(); refreshReadout(); });

    /* Readout strip — sits between the slave label and the power
     * buttons so it's the first thing the eye lands on after picking a
     * slave. Two rows: row 1 = always-on state / cw / mode; row 2 =
     * conditional error + tracking-error labels (hidden when clean)
     * plus a checkbox that flips the Δq display between rad and deg. */
    m_degToggle = new QCheckBox(tr("deg"), this);
    m_degToggle->setToolTip(tr("Display setpoint, slider, presets, and "
                               "Δq in degrees instead of radians "
                               "(Position mode only)."));
    connect(m_degToggle, &QCheckBox::toggled,
            this, &JointControlPanel::onDegToggled);

    auto* readoutRow = new QHBoxLayout;
    readoutRow->setSpacing(18);
    readoutRow->addWidget(m_state);
    readoutRow->addWidget(m_cwLabel);
    readoutRow->addWidget(m_modeLabel);
    readoutRow->addStretch();
    auto* readoutRow2 = new QHBoxLayout;
    readoutRow2->setSpacing(18);
    readoutRow2->addWidget(m_errorLabel);
    readoutRow2->addWidget(m_trackLabel);
    readoutRow2->addStretch();
    readoutRow2->addWidget(m_degToggle);

    /* Manual CiA-402 walk — debug aid. Each button sends a single
     * controlword via PDO without going through cia402_master, so the
     * operator can step Shutdown → SwitchOn → EnableOp by hand while
     * investigating why the automatic walk fails. Collapsed into a
     * GroupBox so it doesn't shout for attention. */
    m_walkShutdown = new QPushButton(tr("Shutdown 0x06"),  this);
    m_walkSwitchOn = new QPushButton(tr("SwitchOn 0x07"),  this);
    m_walkEnableOp = new QPushButton(tr("EnableOp 0x0F"),  this);
    m_walkDisableV = new QPushButton(tr("DisableV 0x00"),  this);
    auto* walkBox = new QGroupBox(tr("Manual CiA-402 walk (debug)"), this);
    walkBox->setCheckable(true);
    walkBox->setChecked(false);
    {
        auto* l = new QHBoxLayout(walkBox);
        l->addWidget(m_walkShutdown);
        l->addWidget(m_walkSwitchOn);
        l->addWidget(m_walkEnableOp);
        l->addWidget(m_walkDisableV);
        l->addStretch();
        /* When the GroupBox is unchecked, hide all children so it
         * collapses to a single header line. */
        auto applyVis = [walkBox]{
            const bool on = walkBox->isChecked();
            for (auto* c : walkBox->findChildren<QWidget*>()){ c->setVisible(on); }
        };
        connect(walkBox, &QGroupBox::toggled, this, applyVis);
        applyVis();
    }
    connect(m_walkShutdown, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit walkControlwordRequested(m_idx, 0x0006);
    });
    connect(m_walkSwitchOn, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit walkControlwordRequested(m_idx, 0x0007);
    });
    connect(m_walkEnableOp, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit walkControlwordRequested(m_idx, 0x000F);
    });
    connect(m_walkDisableV, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit walkControlwordRequested(m_idx, 0x0000);
    });

    /* Open-loop V/f. Two engines, single OD entry (unified at 0x2030):
     *   FOC (0)  -> 0x2030:01 type=2 + freq/amp + 0x6060=-2; needs Enable
     *               (Start handles it). BSP (1) -> 0x2030:01 type=1 +
     *               freq/amp drives bsp_vf directly; auto-disables FOC.
     *               BOTH take a per-unit amplitude (modulation depth of
     *               Vbus), so the level field is uniform. Engine locked
     *               while running. */
    m_vfEngine = new QComboBox(this);
    m_vfEngine->addItem(tr("FOC (mode -2)"), 0);
    m_vfEngine->addItem(tr("BSP generator"), 1);
    m_vfFreq = new QDoubleSpinBox(this);
    m_vfFreq->setRange(-1000.0, 1000.0);
    m_vfFreq->setDecimals(2);
    m_vfFreq->setSingleStep(1.0);
    m_vfFreq->setSuffix(tr(" Hz"));
    m_vfFreq->setToolTip(tr("Electrical frequency (signed; negative = reverse)."));
    m_vfLevel = new QDoubleSpinBox(this);
    m_vfLevel->setRange(0.0, 0.5);
    m_vfLevel->setDecimals(3);
    m_vfLevel->setSingleStep(0.01);
    m_vfLevel->setSuffix(tr(" pu"));
    m_vfLevel->setToolTip(tr("Amplitude — per-unit modulation depth of Vbus "
                             "(≤ 0.4), same for both engines. Start small."));
    m_vfLevelLabel = new QLabel(tr("Amp"), this);
    m_vfStart = new QPushButton(tr("Start V/f"), this);
    m_vfStop  = new QPushButton(tr("Stop"),      this);

    auto* vfBox = new QGroupBox(tr("Open-loop V/f"), this);
    {
        auto* l = new QGridLayout(vfBox);
        l->addWidget(new QLabel(tr("Engine"), vfBox), 0, 0);
        l->addWidget(m_vfEngine,                       0, 1, 1, 3);
        l->addWidget(new QLabel(tr("Freq"), vfBox),    1, 0);
        l->addWidget(m_vfFreq,                          1, 1);
        l->addWidget(m_vfLevelLabel,                    1, 2);
        l->addWidget(m_vfLevel,                         1, 3);
        l->addWidget(m_vfStart,                         2, 0, 1, 2);
        l->addWidget(m_vfStop,                          2, 2, 1, 2);
    }
    connect(m_vfStart, &QPushButton::clicked, this, [this]{
        if (m_idx < 0){ return; }
        const int engine = m_vfEngine->currentData().toInt();
        m_vfRunning = true;
        m_vfEngine->setEnabled(false);                 /* lock engine while running */
        /* FOC V/f needs OPERATION_ENABLED. Bring up FIRST (bringupOne walks
         * the controlword only — it does NOT write 0x6060), THEN start V/f so
         * mode -2 is written LAST and the mode combo can't clobber it.
         * (Using the Bringup *button* instead would re-apply the combo mode
         * and overwrite -2.) BSP needs no enable — it's the raw generator. */
        if (engine == 0){
            emit bringupRequested(m_idx, 2000);
        }
        emit vfStartRequested(m_idx, engine, m_vfFreq->value(), m_vfLevel->value());
    });
    connect(m_vfStop, &QPushButton::clicked, this, [this]{
        if (m_idx < 0){ return; }
        const int engine = m_vfEngine->currentData().toInt();
        m_vfRunning = false;
        m_vfEngine->setEnabled(true);
        emit vfStopRequested(m_idx, engine);
        if (engine == 0){ emit disableRequested(m_idx); }  /* undo the auto-enable */
    });
    auto vfLive = [this]{
        if (m_vfRunning && m_idx >= 0){
            emit vfSetpointChanged(m_idx, m_vfEngine->currentData().toInt(),
                                   m_vfFreq->value(), m_vfLevel->value());
        }
    };
    connect(m_vfFreq,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vfLive);
    connect(m_vfLevel, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, vfLive);

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(readoutRow);
    root->addLayout(readoutRow2);
    root->addLayout(powerRow);
    root->addWidget(walkBox);     /* Manual CiA-402: below Bringup, above Mode */
    root->addWidget(modeBox);
    root->addWidget(tgtBox);
    root->addWidget(vfBox);
    root->addStretch();

    connect(m_bringup,    &QPushButton::clicked, this, [this]{
        if (m_idx < 0){ return; }
        /* Push the currently-selected mode to the slave first so 0x6060
         * is set BEFORE the bringup walk takes the drive to
         * OPERATION_ENABLED — many drives reject mode changes once in
         * OP_EN, and vmotor.mode on the sim must be set or the
         * setpoint won't actually steer the motor (default vmotor mode
         * is NONE → motor coasts with stale omega). */
        emit modeRequested(m_idx,
            static_cast<Mode>(m_modeCombo->currentData().toInt()));
        emit bringupRequested(m_idx, 2000);
    });
    connect(m_enable,     &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit enableRequested(m_idx);
    });
    connect(m_disable,    &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit disableRequested(m_idx);
    });
    connect(m_quickStop,  &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit quickStopRequested(m_idx);
    });
    connect(m_faultReset, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit faultResetRequested(m_idx);
    });
    connect(m_brake, &QPushButton::toggled, this, [this](bool on){
        if (m_idx >= 0){
            emit brakeRequested(m_idx, on);
            m_brake->setText(on ? tr("Brake (ON)") : tr("Brake"));
        }
    });

    setActiveSlave(-1);
}

void JointControlPanel::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    m_hasSnap = false;
    m_snap = {};
    m_cwCached = 0;
    /* Invalidate caches so the next snapshot rebuilds buttons + restyles
     * the state badge for the newly-active slave. */
    m_lastBtnState     = -2;
    m_lastBtnPdoFresh  = false;
    m_lastReadoutState = -1;
    m_lastErrorVisible = false;
    if (m_brake && m_brake->isChecked()){
        QSignalBlocker block(m_brake);
        m_brake->setChecked(false);
        m_brake->setText(tr("Brake"));
    }
    if (idx < 0){
        m_label->setText(tr("(select a slave above)"));
    } else {
        m_label->setText(tr("Slave %1  —  %2").arg(idx).arg(name));
    }
    refreshReadout();
    refreshButtons();
}

void JointControlPanel::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }
    for (const auto& s : snaps){
        if (s.idx == m_idx){
            m_snap    = s;
            m_hasSnap = true;
            refreshReadout();
            /* Buttons only flip on state-machine transitions or first
             * PDO freshness — gate refreshButtons on those edges to
             * avoid 30 Hz × ~20 setEnabled calls thrashing the UI
             * thread (the "lag after a while" backlog). */
            const int  newState = stateCode(s.statusword);
            const bool fresh    = s.pdoFresh;
            if (newState != m_lastBtnState || fresh != m_lastBtnPdoFresh){
                m_lastBtnState    = newState;
                m_lastBtnPdoFresh = fresh;
                refreshButtons();
            }
            return;
        }
    }
}

void JointControlPanel::onControlwordCached(int idx, uint16_t cw)
{
    if (idx != m_idx){ return; }
    m_cwCached = cw;
    refreshReadout();
}

void JointControlPanel::emitTarget()
{
    sendTarget(m_valueSpin->value());
}

void JointControlPanel::sendTarget(double value)
{
    if (m_idx < 0){ return; }
    /* Spinbox / slider hold values in the current display unit (rad
     * by default, degrees when deg-mode is on and the target kind is
     * Position). The wire always expects rad — convert before
     * emitting. Velocity (rad/s) and Torque (Nm) ignore the deg
     * toggle. */
    const auto kind = static_cast<TargetKind>(m_targetCombo->currentData().toInt());
    double rad = value;
    if (kind == TargetKind::Position && isDegMode()){
        rad = value * M_PI / 180.0;
    }
    emit targetRequested(m_idx, kind, static_cast<float>(rad));
}

bool JointControlPanel::isDegMode() const
{
    if (!m_degToggle || !m_degToggle->isChecked()){ return false; }
    if (!m_targetCombo){ return false; }
    const auto kind = static_cast<TargetKind>(m_targetCombo->currentData().toInt());
    return kind == TargetKind::Position;
}

double JointControlPanel::displayFactor() const
{
    return isDegMode() ? (180.0 / M_PI) : 1.0;
}

void JointControlPanel::onDegToggled(bool checked)
{
    /* deg only applies to Position. For Velocity/Torque, just refresh
     * the readout strip (Δq is still meaningful) and bail. */
    const auto kind = static_cast<TargetKind>(m_targetCombo->currentData().toInt());
    if (kind != TargetKind::Position){
        refreshReadout();
        return;
    }
    /* Convert the currently-displayed value into the new unit so the
     * operator sees the same physical setpoint after the toggle. */
    const double current   = m_valueSpin->value();
    const double converted = checked ? current * 180.0 / M_PI
                                     : current * M_PI / 180.0;
    m_valueSpin->blockSignals(true);
    /* Spinbox range needs to expand for deg (±180) and tighten for
     * rad (±π ≈ ±3.142). Wider than ±π is fine for rad — we just want
     * sensible defaults for the operator's typing. */
    if (checked){
        m_valueSpin->setRange(-360.0, 360.0);   /* deg, allow ±1 rev */
        m_valueSpin->setSuffix(QStringLiteral(" °"));
        m_valueSpin->setSingleStep(1.0);
    } else {
        m_valueSpin->setRange(-10000.0, 10000.0);
        m_valueSpin->setSuffix(QStringLiteral(" rad"));
        m_valueSpin->setSingleStep(0.01);
    }
    resyncSliderRange();
    m_valueSpin->setValue(converted);
    m_valueSpin->blockSignals(false);
    /* Slider needs an explicit re-sync since we suppressed the
     * spinbox's valueChanged. */
    onSpinboxChanged(converted);
    refreshReadout();
}

void JointControlPanel::syncTargetKindToMode()
{
    const auto mode = static_cast<Mode>(m_modeCombo->currentData().toInt());
    int desired = -1;
    QString suffix;
    switch (mode){
    case Mode::Position: desired = int(TargetKind::Position); suffix = QStringLiteral(" rad");   break;
    case Mode::Velocity: desired = int(TargetKind::Velocity); suffix = QStringLiteral(" rad/s"); break;
    case Mode::Torque:   desired = int(TargetKind::Torque);   suffix = QStringLiteral(" Nm");    break;
    case Mode::None:     return;   /* NONE leaves user's choice intact */
    }
    const int found = m_targetCombo->findData(desired);
    if (found >= 0){ m_targetCombo->setCurrentIndex(found); }

    /* Force deg off when leaving Position — degrees aren't meaningful
     * for rad/s or Nm. Block the signal so we don't re-enter the
     * conversion dance with whatever value is in the spinbox. */
    if (mode != Mode::Position && m_degToggle && m_degToggle->isChecked()){
        m_degToggle->blockSignals(true);
        m_degToggle->setChecked(false);
        m_degToggle->blockSignals(false);
        m_valueSpin->setRange(-1000.0, 1000.0);
        m_valueSpin->setSingleStep(0.01);
    }
    /* Spinbox suffix: deg if active, otherwise the natural rad/s/Nm
     * suffix computed above. */
    m_valueSpin->setSuffix(isDegMode() ? QStringLiteral(" °") : suffix);
    resyncSliderRange();
}

void JointControlPanel::resyncSliderRange()
{
    if (!m_setpointSlider){ return; }
    const auto kind = static_cast<TargetKind>(m_targetCombo->currentData().toInt());
    /* int slider ticks at 1 / 1000 of one unit of the current display.
     * Sliding kind decides the natural sweep window. Position in deg
     * uses ±180 (full half-turn) instead of ±π. */
    int lo = -3142, hi = 3142;
    switch (kind){
    case TargetKind::Position:
        if (isDegMode()){ lo = -180000; hi = 180000; }
        else            { lo = -3142;   hi = 3142;   }
        break;
    case TargetKind::Velocity: lo = -10000; hi = 10000; break;   /* ±10 rad/s */
    case TargetKind::Torque:   lo = -1000;  hi = 1000;  break;   /* ±1  Nm    */
    }
    m_setpointSlider->blockSignals(true);
    m_setpointSlider->setRange(lo, hi);
    /* Re-snap the slider to whatever the spinbox holds, clamped. */
    const double v = m_valueSpin->value();
    const int    s = std::clamp(static_cast<int>(v * 1000.0), lo, hi);
    m_setpointSlider->setValue(s);
    m_setpointSlider->blockSignals(false);
}

void JointControlPanel::onSliderChanged(int v)
{
    if (m_syncingFromSpin){ return; }
    m_syncingFromSlider = true;
    const double f = v / 1000.0;
    m_valueSpin->setValue(f);
    m_syncingFromSlider = false;
    if (m_liveStream && m_liveStream->isChecked()){
        sendTarget(f);
    }
}

void JointControlPanel::onSpinboxChanged(double v)
{
    if (m_syncingFromSlider){ return; }
    if (!m_setpointSlider){ return; }
    m_syncingFromSpin = true;
    const int s = std::clamp(static_cast<int>(v * 1000.0),
                             m_setpointSlider->minimum(),
                             m_setpointSlider->maximum());
    m_setpointSlider->setValue(s);
    m_syncingFromSpin = false;
}

QString JointControlPanel::decodeErrorCode(uint16_t code)
{
    if (code == 0){ return QString(); }
    /* DS-402 §8.2 common codes (group prefix → name). Specific
     * codes override the prefix lookup. Manufacturer codes (0xFF00+)
     * fall through to "manufacturer". */
    struct Entry { uint16_t code; const char* name; };
    static const Entry kSpecific[] = {
        {0x2310, "over-current (continuous)"},
        {0x2320, "short-circuit"},
        {0x2330, "earth leakage"},
        {0x3210, "DC link over-voltage"},
        {0x3220, "DC link under-voltage"},
        {0x4310, "drive over-temperature"},
        {0x4210, "device over-temperature"},
        {0x5520, "ROM/flash error"},
        {0x6010, "internal software error"},
        {0x7305, "resolver fault"},
        {0x7310, "incremental sensor fault"},
        {0x7320, "absolute sensor fault"},
        {0x8110, "CAN overrun"},
        {0x8210, "PDO length error"},
        {0x8611, "following error"},
        {0x8612, "reference limit reached"},
    };
    for (const auto& e : kSpecific){
        if (e.code == code){
            return QStringLiteral("0x%1 %2")
                .arg(code, 4, 16, QChar('0'))
                .arg(QString::fromLatin1(e.name));
        }
    }
    /* Group prefix fallback. */
    const char* group = nullptr;
    switch (code & 0xF000){
    case 0x1000: group = "generic"; break;
    case 0x2000: group = "current"; break;
    case 0x3000: group = "voltage"; break;
    case 0x4000: group = "temperature"; break;
    case 0x5000: group = "hardware"; break;
    case 0x6000: group = "software"; break;
    case 0x7000: group = "sensor"; break;
    case 0x8000: group = "monitoring/control"; break;
    case 0xF000: group = "additional"; break;
    default: break;
    }
    if (code >= 0xFF00){
        return QStringLiteral("0x%1 manufacturer")
            .arg(code, 4, 16, QChar('0'));
    }
    return QStringLiteral("0x%1 %2")
        .arg(code, 4, 16, QChar('0'))
        .arg(group ? QString::fromLatin1(group) : QStringLiteral("unknown"));
}

QString JointControlPanel::presetStorePath()
{
    const QString dir = QStandardPaths::writableLocation(
        QStandardPaths::AppConfigLocation);
    return dir + QStringLiteral("/control_presets.json");
}

void JointControlPanel::loadPresetStore()
{
    QFile f(presetStorePath());
    if (!f.exists() || !f.open(QIODevice::ReadOnly)){ return; }
    const auto doc = QJsonDocument::fromJson(f.readAll());
    if (!doc.isArray()){ return; }
    m_presetCombo->blockSignals(true);
    m_presetCombo->clear();
    for (const auto& v : doc.array()){
        const QJsonObject o = v.toObject();
        QVariantMap rec;
        rec["mode"]        = o.value("mode").toInt();
        rec["target_kind"] = o.value("target_kind").toInt();
        rec["value"]       = o.value("value").toDouble();
        m_presetCombo->addItem(o.value("name").toString(), rec);
    }
    m_presetCombo->setCurrentIndex(-1);
    m_presetCombo->blockSignals(false);
}

void JointControlPanel::savePresetStore()
{
    const QString path = presetStorePath();
    QDir().mkpath(QFileInfo(path).path());
    QJsonArray arr;
    for (int i = 0; i < m_presetCombo->count(); ++i){
        const QVariantMap rec = m_presetCombo->itemData(i).toMap();
        QJsonObject o;
        o["name"]        = m_presetCombo->itemText(i);
        o["mode"]        = rec.value("mode").toInt();
        o["target_kind"] = rec.value("target_kind").toInt();
        o["value"]       = rec.value("value").toDouble();
        arr.append(o);
    }
    QSaveFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){ return; }
    f.write(QJsonDocument(arr).toJson(QJsonDocument::Indented));
    f.commit();
}

QString JointControlPanel::stateName(uint16_t sw)
{
    /* CiA-402 statusword bit pattern decoding (DS-402 §6.3.1).
     * Masks 0x4F / 0x6F isolate the state-defining bits. */
    if      ((sw & 0x4F) == 0x00) return QStringLiteral("NOT_READY");
    else if ((sw & 0x4F) == 0x40) return QStringLiteral("SWITCH_ON_DISABLED");
    else if ((sw & 0x6F) == 0x21) return QStringLiteral("READY_TO_SWITCH_ON");
    else if ((sw & 0x6F) == 0x23) return QStringLiteral("SWITCHED_ON");
    else if ((sw & 0x6F) == 0x27) return QStringLiteral("OPERATION_ENABLED");
    else if ((sw & 0x6F) == 0x07) return QStringLiteral("QUICK_STOP_ACTIVE");
    else if ((sw & 0x4F) == 0x0F) return QStringLiteral("FAULT_REACTION_ACTIVE");
    else if ((sw & 0x4F) == 0x08) return QStringLiteral("FAULT");
    else                          return QStringLiteral("UNKNOWN");
}

int JointControlPanel::stateCode(uint16_t sw)
{
    if      ((sw & 0x4F) == 0x00) return 0;
    else if ((sw & 0x4F) == 0x40) return 1;
    else if ((sw & 0x6F) == 0x21) return 2;
    else if ((sw & 0x6F) == 0x23) return 3;
    else if ((sw & 0x6F) == 0x27) return 4;
    else if ((sw & 0x6F) == 0x07) return 5;
    else if ((sw & 0x4F) == 0x0F) return 6;
    else if ((sw & 0x4F) == 0x08) return 7;
    else                          return 99;
}

void JointControlPanel::refreshReadout()
{
    if (m_idx < 0){
        m_state    ->setText(tr("state: —"));
        m_cwLabel  ->setText(tr("cw: —"));
        m_modeLabel->setText(tr("mode: —"));
        return;
    }

    /* state column: show name + paint background red on FAULT, amber
     * on QUICK_STOP, green on OPERATION_ENABLED, neutral otherwise.
     * pdoFresh gate avoids showing a stale "NOT_READY" before any
     * TPDO has actually landed. */
    if (m_hasSnap && m_snap.pdoFresh){
        const int code = stateCode(m_snap.statusword);
        const QString name = stateName(m_snap.statusword);
        m_state->setText(tr("state: %1").arg(name));
        /* setStyleSheet only on state change — same UI-thread thrash
         * fix as SlavePickerBar. */
        if (code != m_lastReadoutState){
            m_lastReadoutState = code;
            QString colour;
            switch (code){
            case 4:  colour = QStringLiteral("#1a6e1a"); break;  /* OPERATION_ENABLED — green */
            case 5:  colour = QStringLiteral("#8a5a00"); break;  /* QUICK_STOP_ACTIVE — amber */
            case 6:
            case 7:  colour = QStringLiteral("#8a1a1a"); break;  /* FAULT* — red             */
            default: colour = QStringLiteral("#222222"); break;
            }
            m_state->setStyleSheet(QStringLiteral("color: white; background-color: %1; "
                                                  "padding: 2px 8px; border-radius: 3px;")
                                   .arg(colour));
        }
    } else {
        m_state->setText(tr("state: (waiting for telemetry)"));
        if (m_lastReadoutState != -1){
            m_lastReadoutState = -1;
            m_state->setStyleSheet(QString());
        }
    }

    m_cwLabel  ->setText(tr("cw: 0x%1").arg(m_cwCached, 4, 16, QChar('0')));
    m_modeLabel->setText(tr("mode: %1").arg(
        m_modeCombo->currentText()));

    /* Error code label — appear only when non-zero. Red background for
     * any reported error. setStyleSheet only on visibility transition;
     * the text can update freely. */
    const bool hasError = m_hasSnap && m_snap.pdoFresh && m_snap.errorCode != 0;
    if (hasError){
        m_errorLabel->setText(tr("error: %1").arg(decodeErrorCode(m_snap.errorCode)));
        if (!m_lastErrorVisible){
            m_lastErrorVisible = true;
            m_errorLabel->setVisible(true);
            m_errorLabel->setStyleSheet(QStringLiteral(
                "color: white; background-color: #8a1a1a; "
                "padding: 2px 8px; border-radius: 3px;"));
        }
    } else if (m_lastErrorVisible){
        m_lastErrorVisible = false;
        m_errorLabel->setVisible(false);
    }

    /* Tracking error — show even when small; amber when above an
     * arbitrary "look at me" threshold (0.05 in the current command
     * channel's natural units). trackingError is already computed by
     * MasterWorker; we just present it. */
    if (m_hasSnap && m_snap.pdoFresh){
        const double dq = static_cast<double>(m_snap.trackingError);
        const auto kind = static_cast<TargetKind>(
            m_targetCombo->currentData().toInt());
        /* Δq display unit follows the `deg` checkbox, but only when
         * we're in Position mode (degrees aren't meaningful for
         * velocity/torque). For Velocity/Torque, always print raw. */
        const bool inDeg = m_degToggle && m_degToggle->isChecked()
                           && kind == TargetKind::Position;
        const double shown   = inDeg ? (dq * 180.0 / M_PI) : dq;
        const char*  unit    = inDeg ? "°" : (kind == TargetKind::Position ? " rad"
                                            : kind == TargetKind::Velocity ? " rad/s"
                                                                           : " Nm");
        const double warnThr = inDeg ? 2.8 : 0.05;   /* ≈0.05 rad in deg */
        m_trackLabel->setVisible(true);
        m_trackLabel->setText(tr("Δq: %1%2%3")
            .arg(shown >= 0 ? QStringLiteral("+") : QString())
            .arg(shown, 0, 'f', inDeg ? 2 : 4)
            .arg(QString::fromLatin1(unit)));
        if (std::fabs(shown) > warnThr){
            m_trackLabel->setStyleSheet(QStringLiteral(
                "color: white; background-color: #8a5a00; "
                "padding: 2px 8px; border-radius: 3px;"));
        } else {
            m_trackLabel->setStyleSheet(QString());
        }
    } else {
        m_trackLabel->setVisible(false);
    }
}

void JointControlPanel::refreshButtons()
{
    const bool slaveSelected = (m_idx >= 0);
    /* Default: everything disabled when no slave. */
    if (!slaveSelected){
        for (QWidget* w : { static_cast<QWidget*>(m_bringup),
                            static_cast<QWidget*>(m_enable),
                            static_cast<QWidget*>(m_disable),
                            static_cast<QWidget*>(m_quickStop),
                            static_cast<QWidget*>(m_faultReset),
                            static_cast<QWidget*>(m_brake),
                            static_cast<QWidget*>(m_modeCombo),
                            static_cast<QWidget*>(m_targetCombo),
                            static_cast<QWidget*>(m_valueSpin),
                            static_cast<QWidget*>(m_sendBtn),
                            static_cast<QWidget*>(m_preset0),
                            static_cast<QWidget*>(m_presetQp),
                            static_cast<QWidget*>(m_presetQn),
                            static_cast<QWidget*>(m_presetHome),
                            static_cast<QWidget*>(m_jogMinus),
                            static_cast<QWidget*>(m_jogPlus),
                            static_cast<QWidget*>(m_jogStep),
                            static_cast<QWidget*>(m_setpointSlider),
                            static_cast<QWidget*>(m_liveStream),
                            static_cast<QWidget*>(m_walkShutdown),
                            static_cast<QWidget*>(m_walkSwitchOn),
                            static_cast<QWidget*>(m_walkEnableOp),
                            static_cast<QWidget*>(m_walkDisableV),
                            static_cast<QWidget*>(m_presetCombo),
                            static_cast<QWidget*>(m_presetSaveBtn),
                            static_cast<QWidget*>(m_presetDelBtn) }){
            w->setEnabled(false);
        }
        return;
    }
    /* Slave selected but no PDO yet — allow Bringup only, lock the
     * rest until we know what state the drive is in. */
    if (!m_hasSnap || !m_snap.pdoFresh){
        m_bringup   ->setEnabled(true);
        m_enable    ->setEnabled(false);
        m_disable   ->setEnabled(false);
        m_quickStop ->setEnabled(false);
        m_faultReset->setEnabled(false);
        m_brake     ->setEnabled(false);
        m_modeCombo ->setEnabled(false);
        m_targetCombo->setEnabled(false);
        m_valueSpin ->setEnabled(false);
        m_sendBtn   ->setEnabled(false);
        for (QWidget* w : { static_cast<QWidget*>(m_preset0),
                            static_cast<QWidget*>(m_presetQp),
                            static_cast<QWidget*>(m_presetQn),
                            static_cast<QWidget*>(m_presetHome),
                            static_cast<QWidget*>(m_jogMinus),
                            static_cast<QWidget*>(m_jogPlus),
                            static_cast<QWidget*>(m_jogStep),
                            static_cast<QWidget*>(m_setpointSlider),
                            static_cast<QWidget*>(m_liveStream),
                            static_cast<QWidget*>(m_presetCombo),
                            static_cast<QWidget*>(m_presetSaveBtn),
                            static_cast<QWidget*>(m_presetDelBtn) }){
            w->setEnabled(false);
        }
        /* Manual walk buttons stay live as debug aid even without
         * pdo fresh — that's the whole point of having them. */
        for (QWidget* w : { static_cast<QWidget*>(m_walkShutdown),
                            static_cast<QWidget*>(m_walkSwitchOn),
                            static_cast<QWidget*>(m_walkEnableOp),
                            static_cast<QWidget*>(m_walkDisableV) }){
            w->setEnabled(true);
        }
        return;
    }
    /* State-aware gating based on cached CiA-402 state. Encodes the
     * legal transitions so the operator can't click an action that's
     * a no-op or actively harmful in the current state. */
    const int code = stateCode(m_snap.statusword);
    const bool isFault    = (code == 6 || code == 7);
    const bool isQuickStop= (code == 5);
    const bool isOpEnabled= (code == 4);

    /* Bringup walks any state up to OPERATION_ENABLED. Only point of
     * blocking it is when we're already there. */
    m_bringup->setEnabled(!isOpEnabled);
    /* Enable assumes you're already past the fault path. From FAULT
     * you must Fault Reset first. */
    m_enable->setEnabled(!isFault && !isOpEnabled);
    /* Disable is always safe except when already disabled. */
    m_disable->setEnabled(isOpEnabled || isQuickStop);
    /* Quick Stop only makes sense while running. */
    m_quickStop->setEnabled(isOpEnabled);
    /* Fault Reset only applies in FAULT. */
    m_faultReset->setEnabled(isFault);
    /* Brake (Halt bit) only meaningful while running. Auto-release
     * the toggle if the drive leaves OPERATION_ENABLED so the button
     * label / cached bit don't mis-represent reality after a Disable
     * or Quick Stop. */
    m_brake->setEnabled(isOpEnabled);
    if (!isOpEnabled && m_brake->isChecked()){
        QSignalBlocker block(m_brake);
        m_brake->setChecked(false);
        m_brake->setText(tr("Brake"));
    }
    /* Mode + setpoint only useful when drive is enabled and tracking. */
    const bool canCommand = isOpEnabled;
    m_modeCombo  ->setEnabled(canCommand);
    m_targetCombo->setEnabled(canCommand);
    m_valueSpin  ->setEnabled(canCommand);
    m_sendBtn    ->setEnabled(canCommand);
    for (QWidget* w : { static_cast<QWidget*>(m_preset0),
                        static_cast<QWidget*>(m_presetQp),
                        static_cast<QWidget*>(m_presetQn),
                        static_cast<QWidget*>(m_presetHome),
                        static_cast<QWidget*>(m_jogMinus),
                        static_cast<QWidget*>(m_jogPlus),
                        static_cast<QWidget*>(m_jogStep),
                        static_cast<QWidget*>(m_setpointSlider),
                        static_cast<QWidget*>(m_liveStream),
                        static_cast<QWidget*>(m_presetCombo),
                        static_cast<QWidget*>(m_presetSaveBtn),
                        static_cast<QWidget*>(m_presetDelBtn) }){
        w->setEnabled(canCommand);
    }
    /* Manual walk: always live so the operator can step into / out of
     * states the regular buttons forbid. */
    for (QWidget* w : { static_cast<QWidget*>(m_walkShutdown),
                        static_cast<QWidget*>(m_walkSwitchOn),
                        static_cast<QWidget*>(m_walkEnableOp),
                        static_cast<QWidget*>(m_walkDisableV) }){
        w->setEnabled(true);
    }
}

}  // namespace vrmc
