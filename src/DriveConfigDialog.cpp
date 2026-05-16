#include "DriveConfigDialog.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QVBoxLayout>

#include <limits>

namespace vrmc {

namespace {

QSpinBox* makeSpin(int minVal, int maxVal, int value,
                   const QString& suffix = QString())
{
    auto* s = new QSpinBox;
    s->setRange(minVal, maxVal);
    s->setValue(value);
    s->setAlignment(Qt::AlignRight);
    if (!suffix.isEmpty()){
        s->setSuffix(QStringLiteral(" ") + suffix);
    }
    return s;
}

/* The CiA 402 §7 table of standard homing methods, with the ones most
 * real drives actually implement. Index is 0x6098 value. */
struct HomingMethod {
    int8_t      value;
    const char* label;
};

/* Labels authored in plain ASCII on purpose — earlier revisions used
 * U+2014 EM DASH and U+2212 MINUS SIGN which tripped up
 * QString::fromLatin1 below and rendered as mojibake on the combobox. */
static const HomingMethod kHomingMethods[] = {
    {   0, "0  - no homing"                          },
    {   1, "1  - negative limit + index"             },
    {   2, "2  - positive limit + index"             },
    {   7, "7  - pos limit switch + index"           },
    {  11, "11 - neg limit switch + index"           },
    {  17, "17 - negative limit (no index)"          },
    {  18, "18 - positive limit (no index)"          },
    {  23, "23 - pos limit switch (no index)"        },
    {  27, "27 - neg limit switch (no index)"        },
    {  33, "33 - current position (next index)"      },
    {  34, "34 - current position (prev index)"      },
    {  35, "35 - current position as home"           },
    {  37, "37 - current position as home (v2)"      },
    { -1,  "-1 - hard-stop negative"                 },
    { -2,  "-2 - hard-stop positive"                 },
};

}  // namespace


DriveConfigDialog::DriveConfigDialog(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Configure drive"));
    /* Don't hard-code a size — the dialog now hosts 6 tabs and the
     * Manufacturer tab in particular needs more room than the original
     * 520×520. adjustSize() (called at end of constructor + on every
     * show via showEvent) lets the layout compute its natural footprint. */

    m_header = new QLabel(tr("No slave selected."), this);
    m_header->setStyleSheet(QStringLiteral("font-weight: 600;"));

    auto* tabs = new QTabWidget(this);
    auto* tHoming  = new QWidget;
    auto* tMotion  = new QWidget;
    auto* tProtect = new QWidget;
    auto* tEncoder = new QWidget;
    auto* tManuf   = new QWidget;
    auto* tCustom  = new QWidget;
    buildHomingTab      (tHoming);
    buildMotionTab      (tMotion);
    buildProtectTab     (tProtect);
    buildEncoderTab     (tEncoder);
    buildManufacturerTab(tManuf);
    buildCustomTab      (tCustom);
    tabs->addTab(tHoming,  tr("Homing"));
    tabs->addTab(tMotion,  tr("Motion profile"));
    tabs->addTab(tProtect, tr("Protection"));
    tabs->addTab(tEncoder, tr("Encoder"));
    tabs->addTab(tManuf,   tr("Manufacturer"));
    tabs->addTab(tCustom,  tr("Custom SDO"));

    m_readBtn  = new QPushButton(tr("Read from drive"), this);
    m_applyBtn = new QPushButton(tr("Apply"), this);
    connect(m_readBtn,  &QPushButton::clicked,
            this,       &DriveConfigDialog::onReadClicked);
    connect(m_applyBtn, &QPushButton::clicked,
            this,       &DriveConfigDialog::onApplyClicked);

    m_buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(m_buttons, &QDialogButtonBox::accepted, this, [this]{
        onApplyClicked();
        accept();
    });
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* actionRow = new QHBoxLayout;
    actionRow->addWidget(m_readBtn);
    actionRow->addWidget(m_applyBtn);
    actionRow->addStretch(1);

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_header);
    root->addWidget(tabs, 1);
    root->addLayout(actionRow);
    root->addWidget(m_buttons);

    setConfig(DriveConfig{});
    /* Ask the layout for its natural footprint so all tabs (and the
     * widest one, "Fault thresholds", in particular) are fully visible
     * the moment the dialog is shown. Without this the dialog comes up
     * at QDialog's default tiny size and the operator has to drag a
     * corner before they can see the spinboxes. */
    adjustSize();
}

void DriveConfigDialog::showEvent(QShowEvent* e)
{
    QDialog::showEvent(e);
    /* Re-fit on every show in case the tab contents were modified
     * between opens (e.g. the operator typed a long value into the
     * Custom-SDO result label which then asked for more width). */
    adjustSize();
}

void DriveConfigDialog::buildHomingTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    auto* formWrap = new QWidget(host);
    auto* form = new QFormLayout(formWrap);
    form->setContentsMargins(0, 0, 0, 0);
    m_homingMethod = new QComboBox;
    for (const auto& m : kHomingMethods){
        /* fromUtf8 handles both ASCII and any future multibyte glyphs;
         * fromLatin1 silently mangled the em-dash variants. */
        m_homingMethod->addItem(QString::fromUtf8(m.label), int(m.value));
    }
    m_homeOffset  = makeSpin(std::numeric_limits<int>::min(),
                             std::numeric_limits<int>::max(), 0, tr("counts"));
    m_homingFast  = makeSpin(0, 1'000'000, 100, tr("cnt/s"));
    m_homingSlow  = makeSpin(0, 1'000'000,  10, tr("cnt/s"));
    m_homingAccel = makeSpin(0, 1'000'000, 1000, tr("cnt/s²"));

    form->addRow(tr("Method (0x6098)"),       m_homingMethod);
    form->addRow(tr("Home offset (0x607C)"),  m_homeOffset);
    form->addRow(tr("Switch search speed (0x6099:1)"), m_homingFast);
    form->addRow(tr("Zero search speed  (0x6099:2)"),  m_homingSlow);
    form->addRow(tr("Homing acceleration (0x609A)"),   m_homingAccel);
    root->addWidget(formWrap);

    /* Commissioning actions live below the form — writing the config
     * tweaks the params, these buttons actually kick things off. */
    m_startHomingBtn = new QPushButton(tr("Start Homing"), host);
    m_startHomingBtn->setToolTip(
        tr("Switch drive to mode 6 and pulse controlword bit 4 to start\n"
           "the homing procedure. Requires the drive to be Enabled first."));
    m_zeroEncBtn = new QPushButton(tr("Zero encoder here"), host);
    m_zeroEncBtn->setToolTip(
        tr("Capture position_actual (0x6064) and write it into\n"
           "home_offset (0x607C). Makes \"here\" the new origin."));
    connect(m_startHomingBtn, &QPushButton::clicked,
            this,             &DriveConfigDialog::onStartHomingClicked);
    connect(m_zeroEncBtn,     &QPushButton::clicked,
            this,             &DriveConfigDialog::onZeroEncoderClicked);

    auto* actRow = new QHBoxLayout;
    actRow->addWidget(m_startHomingBtn);
    actRow->addWidget(m_zeroEncBtn);
    actRow->addStretch(1);
    root->addLayout(actRow);

    m_homingState = new QLabel(tr("Homing: —"), host);
    m_homingState->setStyleSheet(QStringLiteral(
        "QLabel { padding: 6px 8px; border: 1px solid #666;"
        "         border-radius: 4px; background: #1a1a1a; color: #ddd; }"));
    root->addWidget(m_homingState);
    root->addStretch(1);
}

void DriveConfigDialog::buildMotionTab(QWidget* host)
{
    auto* form = new QFormLayout(host);
    m_profileVel     = makeSpin(0, 1'000'000, 1000, tr("cnt/s"));
    m_profileAccel   = makeSpin(0, 1'000'000, 5000, tr("cnt/s²"));
    m_profileDecel   = makeSpin(0, 1'000'000, 5000, tr("cnt/s²"));
    m_quickstopDecel = makeSpin(0, 1'000'000, 10000, tr("cnt/s²"));
    form->addRow(tr("Profile velocity (0x6081)"),     m_profileVel);
    form->addRow(tr("Profile acceleration (0x6083)"), m_profileAccel);
    form->addRow(tr("Profile deceleration (0x6084)"), m_profileDecel);
    form->addRow(tr("Quickstop deceleration (0x6085)"), m_quickstopDecel);
}

void DriveConfigDialog::buildProtectTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    auto* formWrap = new QWidget(host);
    auto* form = new QFormLayout(formWrap);
    form->setContentsMargins(0, 0, 0, 0);
    m_followingError = makeSpin(0, 1'000'000, 5000, tr("counts"));
    m_followingMs    = makeSpin(0, 60000,      100, tr("ms"));
    m_posMin         = makeSpin(std::numeric_limits<int>::min(),
                                std::numeric_limits<int>::max(),
                                std::numeric_limits<int>::min() + 1, tr("counts"));
    m_posMax         = makeSpin(std::numeric_limits<int>::min(),
                                std::numeric_limits<int>::max(),
                                std::numeric_limits<int>::max(),    tr("counts"));
    m_maxSpeed       = makeSpin(0, 1'000'000, 10000, tr("cnt/s"));
    m_maxTorque      = makeSpin(0, 10000,     2000, tr("‰ rated"));
    m_ratedTorque    = makeSpin(0, std::numeric_limits<int>::max(),
                                0, tr("mNm"));
    /* Current actual is read-only (0x6078). Drive populates on Read;
     * write pass skips it. Rendered as a label, not a spinbox, so the
     * operator can't accidentally type into it. */
    m_currentActual  = new QLabel(QStringLiteral("—"), host);
    m_currentActual->setStyleSheet(QStringLiteral(
        "QLabel { font-family: monospace; padding: 2px 8px; "
        "         border: 1px solid #555; border-radius: 3px; "
        "         background: #1a1a1a; color: #cfd8e3; }"));

    form->addRow(tr("Max following error (0x6065)"), m_followingError);
    form->addRow(tr("Following error time (0x6066)"), m_followingMs);
    form->addRow(tr("Pos limit min (0x607D:1)"),      m_posMin);
    form->addRow(tr("Pos limit max (0x607D:2)"),      m_posMax);
    form->addRow(tr("Max motor speed (0x6080)"),      m_maxSpeed);
    form->addRow(tr("Max torque (0x6072)"),           m_maxTorque);
    form->addRow(tr("Rated torque (0x6076)"),         m_ratedTorque);
    form->addRow(tr("Current actual (0x6078, RO)"),   m_currentActual);
    root->addWidget(formWrap);

    m_zeroTorqueBtn = new QPushButton(tr("Zero torque here"), host);
    m_zeroTorqueBtn->setToolTip(
        tr("Capture torque_actual (0x6077) and write it into\n"
           "torque_offset (0x60B2) so the current loading reads as 0."));
    connect(m_zeroTorqueBtn, &QPushButton::clicked,
            this,            &DriveConfigDialog::onZeroTorqueClicked);
    auto* row = new QHBoxLayout;
    row->addWidget(m_zeroTorqueBtn);
    row->addStretch(1);
    root->addLayout(row);

    m_torqueState = new QLabel(tr("Torque: —"), host);
    m_torqueState->setStyleSheet(QStringLiteral(
        "QLabel { padding: 6px 8px; border: 1px solid #666;"
        "         border-radius: 4px; background: #1a1a1a; color: #ddd; }"));
    root->addWidget(m_torqueState);
    root->addStretch(1);
}

void DriveConfigDialog::buildEncoderTab(QWidget* host)
{
    auto* form = new QFormLayout(host);
    m_encRes  = makeSpin(1, 1'000'000'000, 16384, tr("counts/rev"));
    m_gearNum = makeSpin(1, 1'000'000, 1);
    m_gearDen = makeSpin(1, 1'000'000, 1);
    m_feedNum = makeSpin(1, 1'000'000, 1);
    m_feedDen = makeSpin(1, 1'000'000, 1);

    form->addRow(tr("Encoder resolution (0x608F:1)"), m_encRes);
    form->addRow(tr("Gear numerator (0x6091:1)"),      m_gearNum);
    form->addRow(tr("Gear denominator (0x6091:2)"),    m_gearDen);
    form->addRow(tr("Feed const numerator (0x6092:1)"), m_feedNum);
    form->addRow(tr("Feed const denominator (0x6092:2)"), m_feedDen);
}

/* ------------------------------------------------------------ Manufacturer
 *
 * Vendor-defined OD entries — Node ID, per-loop gains via the manufacturer
 * range (parallel to the Gains tab which uses motor_drive_intf abstractions),
 * per-phase current-sensor calibration, and the over-current trip threshold.
 *
 * All indices below are placeholders for a typical FOC drive layout. If
 * your hardware uses different indices, override at the Custom-SDO tab
 * or edit driveFields() in MasterWorker.cpp. Slaves that don't expose
 * an index will silently abort the read with 0x06020000 and the
 * spinbox will remain at 0. */
void DriveConfigDialog::buildManufacturerTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    auto makeDoubleSpin = [host](double lo, double hi, double v, int dec){
        auto* s = new QDoubleSpinBox(host);
        s->setRange(lo, hi);
        s->setDecimals(dec);
        s->setValue(v);
        s->setAlignment(Qt::AlignRight);
        return s;
    };

    /* --- Node + identity --- */
    auto* idBox = new QGroupBox(tr("Identity"), host);
    {
        auto* form = new QFormLayout(idBox);
        m_mfNodeId = makeSpin(1, 127, 5);  /* CANopen node IDs are 1..127 */
        form->addRow(tr("Node ID (0x2000:00)"), m_mfNodeId);
    }
    root->addWidget(idBox);

    /* --- Per-loop gains (manufacturer parallel path to Gains tab) --- */
    auto* gainBox = new QGroupBox(tr("Loop gains (manufacturer)"), host);
    {
        auto* form = new QFormLayout(gainBox);
        m_mfCurKp = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        m_mfCurKi = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        m_mfVelKp = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        m_mfVelKi = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        m_mfPosKp = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        m_mfPosKi = makeDoubleSpin(0.0, 1e6, 0.0, 4);
        form->addRow(tr("Current Kp (0x2010)"), m_mfCurKp);
        form->addRow(tr("Current Ki (0x2011)"), m_mfCurKi);
        form->addRow(tr("Velocity Kp (0x2020)"), m_mfVelKp);
        form->addRow(tr("Velocity Ki (0x2021)"), m_mfVelKi);
        form->addRow(tr("Position Kp (0x2030)"), m_mfPosKp);
        form->addRow(tr("Position Ki (0x2031)"), m_mfPosKi);
    }
    root->addWidget(gainBox);

    /* --- Per-phase current sensor calibration --- */
    auto* phaseBox = new QGroupBox(tr("Current sensor calibration"), host);
    {
        auto* form = new QFormLayout(phaseBox);
        m_mfCurOffA = makeSpin(-32768, 32767, 0, tr("counts"));
        m_mfCurOffB = makeSpin(-32768, 32767, 0, tr("counts"));
        m_mfCurOffC = makeSpin(-32768, 32767, 0, tr("counts"));
        m_mfCurGainA = makeSpin(0, 65535, 0);
        m_mfCurGainB = makeSpin(0, 65535, 0);
        m_mfCurGainC = makeSpin(0, 65535, 0);
        form->addRow(tr("Current offset A (0x2040:1)"), m_mfCurOffA);
        form->addRow(tr("Current offset B (0x2040:2)"), m_mfCurOffB);
        form->addRow(tr("Current offset C (0x2040:3)"), m_mfCurOffC);
        form->addRow(tr("Current gain A (0x2041:1)"),   m_mfCurGainA);
        form->addRow(tr("Current gain B (0x2041:2)"),   m_mfCurGainB);
        form->addRow(tr("Current gain C (0x2041:3)"),   m_mfCurGainC);
    }
    root->addWidget(phaseBox);

    /* --- Trip thresholds (10 sub-indices under 0x2050) --- */
    auto* faultBox = new QGroupBox(tr("Fault thresholds (0x2050)"), host);
    {
        auto* form = new QFormLayout(faultBox);
        const int kMaxU32 = std::numeric_limits<int>::max();
        m_mfFltOverCur     = makeSpin(0, kMaxU32, 0, tr("mA"));
        m_mfFltOverLoad    = makeSpin(0, 1000,    0, tr("% rated"));
        m_mfFltOverLoadMs  = makeSpin(0, kMaxU32, 0, tr("ms"));
        m_mfFltLossPhase   = makeSpin(0, kMaxU32, 0, tr("mA"));
        m_mfFltLossPhaseMs = makeSpin(0, kMaxU32, 0, tr("ms"));
        m_mfFltUnbalance   = makeSpin(0, kMaxU32, 0, tr("mA"));
        m_mfFltStallMs     = makeSpin(0, kMaxU32, 0, tr("ms"));
        m_mfFltOverVolt    = makeSpin(0, kMaxU32, 0, tr("mV"));
        m_mfFltUnderVolt   = makeSpin(0, kMaxU32, 0, tr("mV"));
        /* Temperature is signed °C × 10 (so -40.0°C = -400). */
        m_mfFltOverTemp    = makeSpin(-3000, 3000, 0, tr("°C × 10"));

        form->addRow(tr("Over current (:01)"),         m_mfFltOverCur);
        form->addRow(tr("Over load (:02)"),            m_mfFltOverLoad);
        form->addRow(tr("Over load time (:03)"),       m_mfFltOverLoadMs);
        form->addRow(tr("Current loss phase (:04)"),   m_mfFltLossPhase);
        form->addRow(tr("Current loss phase time (:05)"), m_mfFltLossPhaseMs);
        form->addRow(tr("Unbalance current (:06)"),    m_mfFltUnbalance);
        form->addRow(tr("Stall time (:07)"),           m_mfFltStallMs);
        form->addRow(tr("Over voltage (:08)"),         m_mfFltOverVolt);
        form->addRow(tr("Under voltage (:09)"),        m_mfFltUnderVolt);
        form->addRow(tr("Over temperature (:0A)"),     m_mfFltOverTemp);
    }
    root->addWidget(faultBox);
    root->addStretch(1);
}

/* ---------------------------------------------------------------- Custom SDO
 *
 * Free-form OD poke. The operator picks an index, sub-index, datatype,
 * and a write value, then Reads or Writes via an expedited SDO. Output
 * pane shows the wire bytes (little-endian hex) on success or the SDO
 * abort code on failure. Most useful for one-off pokes during
 * commissioning of a new drive whose vendor-specific OD entries aren't
 * yet exposed in the dedicated tabs. */
void DriveConfigDialog::buildCustomTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    auto* form = new QFormLayout;
    m_customIdx = new QSpinBox;
    m_customIdx->setRange(0, 0xFFFF);
    m_customIdx->setDisplayIntegerBase(16);
    m_customIdx->setPrefix(QStringLiteral("0x"));
    m_customIdx->setValue(0x6040);                  /* default: controlword */

    m_customSub = new QSpinBox;
    m_customSub->setRange(0, 0xFF);
    m_customSub->setDisplayIntegerBase(16);
    m_customSub->setPrefix(QStringLiteral("0x"));
    m_customSub->setValue(0x00);

    m_customType = new QComboBox;
    /* (userData = byte length). Signed types just affect display; the
     * wire is identical to unsigned. */
    m_customType->addItem(QStringLiteral("uint8  (1)"),  1);
    m_customType->addItem(QStringLiteral("uint16 (2)"),  2);
    m_customType->addItem(QStringLiteral("uint32 (4)"),  4);
    m_customType->addItem(QStringLiteral("int8   (1)"),  1);
    m_customType->addItem(QStringLiteral("int16  (2)"),  2);
    m_customType->addItem(QStringLiteral("int32  (4)"),  4);
    m_customType->setCurrentIndex(1);               /* uint16 by default */

    m_customValue = new QSpinBox;
    m_customValue->setRange(std::numeric_limits<int>::min(),
                            std::numeric_limits<int>::max());
    m_customValue->setValue(0);
    m_customValue->setToolTip(tr("Write payload (decimal). Encoded as "
                                 "little-endian according to the chosen "
                                 "data type."));

    form->addRow(tr("Index"),     m_customIdx);
    form->addRow(tr("Sub-index"), m_customSub);
    form->addRow(tr("Data type"), m_customType);
    form->addRow(tr("Value"),     m_customValue);
    root->addLayout(form);

    m_customReadBtn  = new QPushButton(tr("Read"),  host);
    m_customWriteBtn = new QPushButton(tr("Write"), host);
    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(m_customReadBtn);
    btnRow->addWidget(m_customWriteBtn);
    btnRow->addStretch();
    root->addLayout(btnRow);

    m_customResult = new QLabel(tr("(no SDO yet)"), host);
    m_customResult->setWordWrap(true);
    m_customResult->setStyleSheet(QStringLiteral(
        "QLabel { font-family: monospace; padding: 6px 8px; "
        "         border: 1px solid #555; border-radius: 4px; "
        "         background: #1a1a1a; color: #cfd8e3; }"));
    root->addWidget(m_customResult);
    root->addStretch(1);

    connect(m_customReadBtn, &QPushButton::clicked, this, [this]{
        if (m_slaveIdx < 0){ return; }
        const int bytes = m_customType->currentData().toInt();
        emit customSdoReadRequested(m_slaveIdx,
                                    static_cast<uint16_t>(m_customIdx->value()),
                                    static_cast<uint8_t>(m_customSub->value()),
                                    bytes);
        m_customResult->setText(tr("(reading…)"));
    });
    connect(m_customWriteBtn, &QPushButton::clicked, this, [this]{
        if (m_slaveIdx < 0){ return; }
        const int bytes = m_customType->currentData().toInt();
        /* Encode value LE into a QByteArray sized to the datatype. */
        QByteArray payload(bytes, '\0');
        const int64_t v = static_cast<int64_t>(m_customValue->value());
        for (int i = 0; i < bytes; ++i){
            payload[i] = char((v >> (8 * i)) & 0xFF);
        }
        emit customSdoWriteRequested(m_slaveIdx,
                                     static_cast<uint16_t>(m_customIdx->value()),
                                     static_cast<uint8_t>(m_customSub->value()),
                                     payload);
        m_customResult->setText(tr("(writing %1 bytes…)").arg(bytes));
    });
}

void DriveConfigDialog::onCustomSdoDone(int idx, bool isWrite,
                                        uint16_t odIdx, uint8_t sub,
                                        bool ok, const QString& valueDecoded,
                                        const QString& message)
{
    /* Result lines are intentionally verbose so the operator can copy
     * them into a bug report without losing context. */
    if (idx != m_slaveIdx || !m_customResult){ return; }
    const QString head = QStringLiteral("0x%1.%2 %3")
        .arg(odIdx, 4, 16, QChar('0'))
        .arg(sub,   2, 16, QChar('0'))
        .arg(isWrite ? QStringLiteral("WRITE") : QStringLiteral("READ"));
    if (ok){
        const QString body = isWrite
            ? tr("OK")
            : tr("OK  →  %1").arg(valueDecoded);
        m_customResult->setStyleSheet(QStringLiteral(
            "QLabel { font-family: monospace; padding: 6px 8px; "
            "         border: 1px solid #3c6e3c; border-radius: 4px; "
            "         background: #14301a; color: #aed8ae; }"));
        m_customResult->setText(QStringLiteral("%1: %2").arg(head, body));
    } else {
        m_customResult->setStyleSheet(QStringLiteral(
            "QLabel { font-family: monospace; padding: 6px 8px; "
            "         border: 1px solid #8a1a1a; border-radius: 4px; "
            "         background: #2a1010; color: #ff9090; }"));
        m_customResult->setText(QStringLiteral("%1 FAILED: %2").arg(head, message));
    }
}

void DriveConfigDialog::setSlaveContext(int idx, const QString& name)
{
    m_slaveIdx  = idx;
    m_slaveName = name;
    if (idx < 0){
        m_header->setText(tr("No slave selected."));
        m_readBtn->setEnabled(false);
        m_applyBtn->setEnabled(false);
        m_buttons->button(QDialogButtonBox::Ok)->setEnabled(false);
    } else {
        m_header->setText(tr("Slave #%1 — %2")
                              .arg(idx)
                              .arg(name.isEmpty() ? tr("(unnamed)") : name));
        m_readBtn->setEnabled(true);
        m_applyBtn->setEnabled(true);
        m_buttons->button(QDialogButtonBox::Ok)->setEnabled(true);
    }
}

void DriveConfigDialog::setConfig(const DriveConfig& cfg)
{
    const int methodPos = m_homingMethod->findData(int(cfg.homing_method));
    m_homingMethod->setCurrentIndex(methodPos < 0 ? 0 : methodPos);
    m_homeOffset     ->setValue(int(cfg.home_offset));
    m_homingFast     ->setValue(int(cfg.homing_speed_fast));
    m_homingSlow     ->setValue(int(cfg.homing_speed_slow));
    m_homingAccel    ->setValue(int(cfg.homing_accel));

    m_profileVel     ->setValue(int(cfg.profile_velocity));
    m_profileAccel   ->setValue(int(cfg.profile_accel));
    m_profileDecel   ->setValue(int(cfg.profile_decel));
    m_quickstopDecel ->setValue(int(cfg.quickstop_decel));

    m_followingError ->setValue(int(cfg.following_error));
    m_followingMs    ->setValue(int(cfg.following_error_ms));
    m_posMin         ->setValue(int(cfg.pos_limit_min));
    m_posMax         ->setValue(int(cfg.pos_limit_max));
    m_maxSpeed       ->setValue(int(cfg.max_motor_speed));
    m_maxTorque      ->setValue(int(cfg.max_torque));
    m_ratedTorque    ->setValue(int(cfg.rated_torque));
    /* Render current_actual as `±N ‰  (±N.NNN A)` when rated torque is
     * also known — otherwise just the raw per-mille reading. */
    if (m_currentActual){
        const int per_mille = cfg.current_actual;
        m_currentActual->setText(QStringLiteral("%1%2 ‰")
            .arg(per_mille >= 0 ? QStringLiteral("+") : QString())
            .arg(per_mille));
    }

    m_encRes         ->setValue(int(cfg.enc_resolution));
    m_gearNum        ->setValue(int(cfg.gear_num));
    m_gearDen        ->setValue(int(cfg.gear_den));
    m_feedNum        ->setValue(int(cfg.feed_const_num));
    m_feedDen        ->setValue(int(cfg.feed_const_den));

    /* Manufacturer-range fields. */
    if (m_mfNodeId)     m_mfNodeId    ->setValue(int(cfg.node_id));
    if (m_mfCurKp)      m_mfCurKp     ->setValue(cfg.manuf_cur_kp);
    if (m_mfCurKi)      m_mfCurKi     ->setValue(cfg.manuf_cur_ki);
    if (m_mfVelKp)      m_mfVelKp     ->setValue(cfg.manuf_vel_kp);
    if (m_mfVelKi)      m_mfVelKi     ->setValue(cfg.manuf_vel_ki);
    if (m_mfPosKp)      m_mfPosKp     ->setValue(cfg.manuf_pos_kp);
    if (m_mfPosKi)      m_mfPosKi     ->setValue(cfg.manuf_pos_ki);
    if (m_mfCurOffA)    m_mfCurOffA   ->setValue(int(cfg.current_offset_a));
    if (m_mfCurOffB)    m_mfCurOffB   ->setValue(int(cfg.current_offset_b));
    if (m_mfCurOffC)    m_mfCurOffC   ->setValue(int(cfg.current_offset_c));
    if (m_mfCurGainA)   m_mfCurGainA  ->setValue(int(cfg.current_gain_a));
    if (m_mfCurGainB)   m_mfCurGainB  ->setValue(int(cfg.current_gain_b));
    if (m_mfCurGainC)   m_mfCurGainC  ->setValue(int(cfg.current_gain_c));
    if (m_mfFltOverCur)     m_mfFltOverCur    ->setValue(int(cfg.over_current));
    if (m_mfFltOverLoad)    m_mfFltOverLoad   ->setValue(int(cfg.over_load));
    if (m_mfFltOverLoadMs)  m_mfFltOverLoadMs ->setValue(int(cfg.over_load_ms));
    if (m_mfFltLossPhase)   m_mfFltLossPhase  ->setValue(int(cfg.current_loss_phase));
    if (m_mfFltLossPhaseMs) m_mfFltLossPhaseMs->setValue(int(cfg.current_loss_phase_ms));
    if (m_mfFltUnbalance)   m_mfFltUnbalance  ->setValue(int(cfg.unbalance_current));
    if (m_mfFltStallMs)     m_mfFltStallMs    ->setValue(int(cfg.stall_ms));
    if (m_mfFltOverVolt)    m_mfFltOverVolt   ->setValue(int(cfg.over_voltage));
    if (m_mfFltUnderVolt)   m_mfFltUnderVolt  ->setValue(int(cfg.under_voltage));
    if (m_mfFltOverTemp)    m_mfFltOverTemp   ->setValue(int(cfg.over_temperature));
}

DriveConfig DriveConfigDialog::config() const
{
    DriveConfig c;
    c.homing_method      = int8_t(m_homingMethod->currentData().toInt());
    c.home_offset        = int32_t(m_homeOffset->value());
    c.homing_speed_fast  = uint32_t(m_homingFast->value());
    c.homing_speed_slow  = uint32_t(m_homingSlow->value());
    c.homing_accel       = uint32_t(m_homingAccel->value());

    c.profile_velocity   = uint32_t(m_profileVel->value());
    c.profile_accel      = uint32_t(m_profileAccel->value());
    c.profile_decel      = uint32_t(m_profileDecel->value());
    c.quickstop_decel    = uint32_t(m_quickstopDecel->value());

    c.following_error    = uint32_t(m_followingError->value());
    c.following_error_ms = uint16_t(m_followingMs->value());
    c.pos_limit_min      = int32_t(m_posMin->value());
    c.pos_limit_max      = int32_t(m_posMax->value());
    c.max_motor_speed    = uint32_t(m_maxSpeed->value());
    c.max_torque         = uint16_t(m_maxTorque->value());
    c.rated_torque       = uint32_t(m_ratedTorque->value());
    /* current_actual is RO; preserve a 0 so write path skips it. */
    c.current_actual     = 0;

    c.enc_resolution     = uint32_t(m_encRes->value());
    c.gear_num           = uint32_t(m_gearNum->value());
    c.gear_den           = uint32_t(m_gearDen->value());
    c.feed_const_num     = uint32_t(m_feedNum->value());
    c.feed_const_den     = uint32_t(m_feedDen->value());

    /* Manufacturer-range fields (0x20xx). */
    if (m_mfNodeId)     c.node_id          = uint8_t (m_mfNodeId->value());
    if (m_mfCurKp)      c.manuf_cur_kp     = float   (m_mfCurKp->value());
    if (m_mfCurKi)      c.manuf_cur_ki     = float   (m_mfCurKi->value());
    if (m_mfVelKp)      c.manuf_vel_kp     = float   (m_mfVelKp->value());
    if (m_mfVelKi)      c.manuf_vel_ki     = float   (m_mfVelKi->value());
    if (m_mfPosKp)      c.manuf_pos_kp     = float   (m_mfPosKp->value());
    if (m_mfPosKi)      c.manuf_pos_ki     = float   (m_mfPosKi->value());
    if (m_mfCurOffA)    c.current_offset_a = int16_t (m_mfCurOffA->value());
    if (m_mfCurOffB)    c.current_offset_b = int16_t (m_mfCurOffB->value());
    if (m_mfCurOffC)    c.current_offset_c = int16_t (m_mfCurOffC->value());
    if (m_mfCurGainA)   c.current_gain_a   = uint16_t(m_mfCurGainA->value());
    if (m_mfCurGainB)   c.current_gain_b   = uint16_t(m_mfCurGainB->value());
    if (m_mfCurGainC)   c.current_gain_c   = uint16_t(m_mfCurGainC->value());
    if (m_mfFltOverCur)     c.over_current           = uint32_t(m_mfFltOverCur->value());
    if (m_mfFltOverLoad)    c.over_load              = uint16_t(m_mfFltOverLoad->value());
    if (m_mfFltOverLoadMs)  c.over_load_ms           = uint32_t(m_mfFltOverLoadMs->value());
    if (m_mfFltLossPhase)   c.current_loss_phase     = uint32_t(m_mfFltLossPhase->value());
    if (m_mfFltLossPhaseMs) c.current_loss_phase_ms  = uint32_t(m_mfFltLossPhaseMs->value());
    if (m_mfFltUnbalance)   c.unbalance_current      = uint32_t(m_mfFltUnbalance->value());
    if (m_mfFltStallMs)     c.stall_ms               = uint32_t(m_mfFltStallMs->value());
    if (m_mfFltOverVolt)    c.over_voltage           = uint32_t(m_mfFltOverVolt->value());
    if (m_mfFltUnderVolt)   c.under_voltage          = uint32_t(m_mfFltUnderVolt->value());
    if (m_mfFltOverTemp)    c.over_temperature       = int16_t (m_mfFltOverTemp->value());
    return c;
}

void DriveConfigDialog::onReadClicked()
{
    if (m_slaveIdx < 0){ return; }
    emit readRequested(m_slaveIdx);
}

void DriveConfigDialog::onApplyClicked()
{
    if (m_slaveIdx < 0){ return; }
    emit applyRequested(m_slaveIdx, config());
}

void DriveConfigDialog::onReadResult(int idx, const DriveConfig& cfg,
                                     bool ok, const QString& msg)
{
    if (idx != m_slaveIdx){ return; }
    setConfig(cfg);
    if (!ok){
        /* Show the worker's guidance verbatim — it already distinguishes
         * "early bail" (drive unreachable / doesn't expose these OD
         * entries) from "partial" (real drive, some missing objects). */
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("Could not read from device"));
        box.setText(tr("The drive did not return configuration values."));
        box.setInformativeText(msg.isEmpty()
            ? tr("No detail reported. Check the slave is online and "
                 "responsive on the Control tab.")
            : msg);
        box.setStandardButtons(QMessageBox::Ok);
        box.exec();
    } else if (!msg.isEmpty()){
        /* Partial read success — informational, not a blocker. */
        QMessageBox::information(this, tr("Partial read"),
            tr("Some fields could not be read. They kept their default "
               "values; the rest reflect what's on the drive.\n\n%1")
                .arg(msg));
    }
}

void DriveConfigDialog::onWriteResult(int idx, bool ok, const QString& msg)
{
    if (idx != m_slaveIdx){ return; }
    if (ok){
        QMessageBox::information(this, tr("Drive config written"),
            tr("All fields written successfully."));
    } else {
        QMessageBox::warning(this, tr("Drive config — partial write"),
            tr("Some fields failed. Check the log for details.\n\n%1").arg(msg));
    }
}

void DriveConfigDialog::onStartHomingClicked()
{
    if (m_slaveIdx < 0){ return; }
    m_homingState->setText(tr("Homing: started — waiting for attained bit…"));
    emit startHomingRequested(m_slaveIdx);
}

void DriveConfigDialog::onZeroEncoderClicked()
{
    if (m_slaveIdx < 0){ return; }
    emit zeroEncoderRequested(m_slaveIdx);
}

void DriveConfigDialog::onZeroTorqueClicked()
{
    if (m_slaveIdx < 0){ return; }
    emit zeroTorqueRequested(m_slaveIdx);
}

void DriveConfigDialog::onCalibrationDone(int idx, const QString& what,
                                          qint64 raw, bool ok,
                                          const QString& msg)
{
    if (idx != m_slaveIdx){ return; }
    if (what == QLatin1String("encoder-zero")){
        if (ok){
            m_homeOffset->setValue(int(raw));
            m_homingState->setText(tr("Encoder zeroed — home_offset = %1 cnt")
                                       .arg(qlonglong(raw)));
        } else {
            m_homingState->setText(tr("Encoder zero failed: %1").arg(msg));
        }
    } else if (what == QLatin1String("torque-zero")){
        if (ok){
            m_torqueState->setText(
                tr("Torque offset set — torque_offset = %1 ‰ rated")
                    .arg(qlonglong(raw)));
        } else {
            m_torqueState->setText(tr("Torque zero failed: %1").arg(msg));
        }
    }
}

void DriveConfigDialog::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_slaveIdx < 0){ return; }
    const SlaveSnapshot* s = nullptr;
    for (const auto& x : snaps){
        if (x.idx == m_slaveIdx){ s = &x; break; }
    }
    if (!s){ return; }

    /* Homing status decode (CiA 402 §10 statusword bits in homing mode):
     *   bit 12 = homing attained     bit 13 = homing error
     *   bit 10 = target reached     */
    if (s->pdoFresh){
        const bool attained = (s->statusword & (1u << 12)) != 0;
        const bool errBit   = (s->statusword & (1u << 13)) != 0;
        const bool reached  = (s->statusword & (1u << 10)) != 0;
        QString line = tr("Statusword 0x%1")
                           .arg(s->statusword, 4, 16, QChar('0'));
        if (errBit){
            line += tr(" — homing ERROR");
        } else if (attained){
            line += tr(" — homing attained%1")
                        .arg(reached ? tr(" · target reached") : QString());
        } else {
            line += tr(" — in progress…");
        }
        m_homingState->setText(line);
        m_torqueState->setText(tr("Torque actual %1 ‰ rated")
                                   .arg(int(s->torque / 0.5 * 1000.0)));
    }
}

}  // namespace vrmc
