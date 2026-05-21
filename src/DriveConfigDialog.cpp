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
#include <QTabWidget>
#include <QVBoxLayout>

#include <cmath>
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

QDoubleSpinBox* makeDSpin(double minVal, double maxVal, int decimals,
                          const QString& suffix = QString())
{
    auto* s = new QDoubleSpinBox;
    s->setRange(minVal, maxVal);
    s->setDecimals(decimals);
    s->setValue(0.0);
    s->setAlignment(Qt::AlignRight);
    if (!suffix.isEmpty()){
        s->setSuffix(QStringLiteral(" ") + suffix);
    }
    return s;
}

/* ---- SI <-> CiA-402 wire-unit conversion ---------------------------- *
 *
 * The dialog presents everything in SI; the DriveConfig blob (and the
 * wire) stay in CiA-402 units. Position/velocity assume a fixed encoder
 * resolution -- this must match the drive (board: 14-bit, 16384/rev).
 * Torque/current scale by the live rated values read from the drive. */
constexpr double kTwoPi          = 6.283185307179586;
constexpr double kIncPerRevDflt  = 16384.0;   /* fallback if 0x608F unread */

inline double incToRad (double inc, double cpr) { return inc * kTwoPi / cpr; }
inline double radToInc (double rad, double cpr) { return rad * cpr / kTwoPi; }
inline double rpmToRadS(double rpm) { return rpm * kTwoPi / 60.0; }
inline double radSToRpm(double rs ) { return rs  * 60.0 / kTwoPi; }
/* per-mille of rated <-> physical, given the rated quantity (Nm or A). */
inline double milliToPhys(double milli, double rated){ return milli / 1000.0 * rated; }
inline double physToMilli(double phys,  double rated){
    return (rated != 0.0) ? (phys / rated * 1000.0) : 0.0;
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
    auto* tManuf   = new QWidget;
    auto* tCustom  = new QWidget;
    buildHomingTab      (tHoming);
    buildMotionTab      (tMotion);
    buildProtectTab     (tProtect);
    buildManufacturerTab(tManuf);
    buildCustomTab      (tCustom);
    tabs->addTab(tHoming,  tr("Homing"));
    tabs->addTab(tMotion,  tr("Motion profile"));
    tabs->addTab(tProtect, tr("Protection"));
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
    m_tabs = tabs;
    /* Pre-size every tab page to the maximum sizeHint across all tabs
     * so that the QTabWidget's own sizeHint (which is just the current
     * page's) reflects the worst-case footprint. Without this, the
     * first-shown tab (Homing) is small → adjustSize() picks a tiny
     * dialog → switching to Manufacturer / Custom SDO clips the
     * spinboxes until the user manually drags the corner. */
    fitTabsToLargest();
    adjustSize();
}

void DriveConfigDialog::showEvent(QShowEvent* e)
{
    QDialog::showEvent(e);
    fitTabsToLargest();
    adjustSize();
}

void DriveConfigDialog::fitTabsToLargest()
{
    /* QTabWidget's sizeHint is unreliable on first show: non-current
     * pages live in a lazy QStackedWidget that doesn't always realise
     * their layout in time for adjustSize() to read correctly. So we
     * just hardcode a footprint sized to comfortably hold the widest
     * tab — Manufacturer, with 4 group boxes incl. the 10-field Fault
     * thresholds matrix. Users can shrink it later if they want. */
    constexpr int kMinW = 720;
    constexpr int kMinH = 880;
    setMinimumSize(kMinW, kMinH);
    if (size().width() < kMinW || size().height() < kMinH){
        resize(kMinW, kMinH);
    }
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
    m_homeOffset  = makeDSpin(-1e6, 1e6, 4, tr("rad"));
    m_homingFast  = makeDSpin(0, 1e4, 3, tr("rad/s"));
    m_homingSlow  = makeDSpin(0, 1e4, 3, tr("rad/s"));
    m_homingAccel = makeDSpin(0, 1e6, 1, tr("rad/s²"));

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
    m_profileVel     = makeDSpin(0, 1e4, 3, tr("rad/s"));
    m_profileAccel   = makeDSpin(0, 1e6, 1, tr("rad/s²"));
    m_profileDecel   = makeDSpin(0, 1e6, 1, tr("rad/s²"));
    m_quickstopDecel = makeDSpin(0, 1e6, 1, tr("rad/s²"));
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
    m_followingError = makeDSpin(0, 1e6, 4, tr("rad"));
    m_followingMs    = makeSpin(0, 60000, 100, tr("ms"));
    m_posMin         = makeDSpin(-1e6, 1e6, 4, tr("rad"));
    m_posMax         = makeDSpin(-1e6, 1e6, 4, tr("rad"));
    m_maxSpeed       = makeDSpin(0, 1e4, 3, tr("rad/s"));
    m_maxTorque      = makeDSpin(0, 1e4, 4, tr("Nm"));
    m_ratedTorque    = makeDSpin(0, 1e4, 4, tr("Nm"));
    /* 0x6076 is derived (= Kt * rated current) and read-only on the drive;
     * show it but don't let the user edit/write it. */
    m_ratedTorque->setReadOnly(true);
    m_ratedTorque->setButtonSymbols(QAbstractSpinBox::NoButtons);
    m_ratedCurrent   = makeDSpin(0, 1e4, 3, tr("A"));
    /* Protection: stall trip (vendor 0x2050). Current actual moved to
     * the telemetry graph; encoder resolution moved to Manufacturer. */
    m_stallCurrent = makeSpin(0, 1'000'000, 0, tr("mA"));
    m_stallTime    = makeSpin(0, 60000,     0, tr("ms"));

    form->addRow(tr("Max following error (0x6065)"), m_followingError);
    form->addRow(tr("Following error time (0x6066)"), m_followingMs);
    form->addRow(tr("Pos limit min (0x607D:1)"),      m_posMin);
    form->addRow(tr("Pos limit max (0x607D:2)"),      m_posMax);
    form->addRow(tr("Max motor speed (0x6080)"),      m_maxSpeed);
    form->addRow(tr("Max torque (0x6072)"),           m_maxTorque);
    form->addRow(tr("Rated torque (0x6076, derived)"), m_ratedTorque);
    form->addRow(tr("Rated current (0x6075)"),        m_ratedCurrent);
    form->addRow(tr("Stall current (0x2050:1)"),      m_stallCurrent);
    form->addRow(tr("Stall time (0x2050:2)"),         m_stallTime);
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

/* ------------------------------------------------------------ Manufacturer
 *
 * Vendor-range (0x20xx) objects: Node ID (0x2000:01), per-phase current
 * sensor calibration (offsets 0x2040 + gains 0x2041, RW), Hall offset
 * (0x2060), and the motor-profile electrical record (0x2070). PID loop
 * gains are on the Gains tab (standard 0x60F6/0x60F9/0x60FB). */
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
        form->addRow(tr("Node ID (0x2000:01)"), m_mfNodeId);
    }
    root->addWidget(idBox);

    /* --- Per-phase current sensor calibration (writable) --- */
    auto* phaseBox = new QGroupBox(tr("Current sensor calibration"), host);
    {
        auto* form = new QFormLayout(phaseBox);
        m_mfCurOffA  = makeDoubleSpin(-1e6, 1e6, 0.0, 4);
        m_mfCurOffB  = makeDoubleSpin(-1e6, 1e6, 0.0, 4);
        m_mfCurOffC  = makeDoubleSpin(-1e6, 1e6, 0.0, 4);
        m_mfCurGainA = makeDoubleSpin(0.0, 1e6, 1.0, 4);
        m_mfCurGainB = makeDoubleSpin(0.0, 1e6, 1.0, 4);
        m_mfCurGainC = makeDoubleSpin(0.0, 1e6, 1.0, 4);
        form->addRow(tr("Current offset A (0x2040:1)"), m_mfCurOffA);
        form->addRow(tr("Current offset B (0x2040:2)"), m_mfCurOffB);
        form->addRow(tr("Current offset C (0x2040:3)"), m_mfCurOffC);
        form->addRow(tr("Current gain A (0x2041:1)"),   m_mfCurGainA);
        form->addRow(tr("Current gain B (0x2041:2)"),   m_mfCurGainB);
        form->addRow(tr("Current gain C (0x2041:3)"),   m_mfCurGainC);
    }
    root->addWidget(phaseBox);

    /* --- Commutation + encoder resolution --- */
    auto* commBox = new QGroupBox(tr("Commutation / encoder"), host);
    {
        auto* form = new QFormLayout(commBox);
        m_mfHallOffset = makeDoubleSpin(-7.0, 7.0, 0.0, 5);
        m_mfHallOffset->setSuffix(tr(" rad"));
        m_encInc  = makeSpin(1, 1'000'000'000, 16384, tr("inc"));
        m_encRevs = makeSpin(1, 1'000'000,     1,     tr("rev"));
        form->addRow(tr("Hall offset (0x2060)"),          m_mfHallOffset);
        form->addRow(tr("Encoder increments (0x608F:1)"), m_encInc);
        form->addRow(tr("Motor revolutions (0x608F:2)"),  m_encRevs);
    }
    root->addWidget(commBox);

    auto* note = new QLabel(
        tr("Motor profile (type, Rs, Ls, flux, …) is on the <b>Motor "
           "Profile</b> editor — its OK writes 0x2070 to the selected "
           "drive. PID loop gains are on the <b>Gains</b> tab "
           "(0x60F6 / 0x60F9 / 0x60FB)."), host);
    note->setWordWrap(true);
    note->setStyleSheet(QStringLiteral("QLabel { color: #9aa; padding: 4px; }"));
    root->addWidget(note);
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
    /* CiA wire units -> SI for display. Torque/current use the live
     * rated values; position/velocity use the drive's reported encoder
     * resolution (0x608F), falling back to 16384 inc/rev. */
    const double rated_nm = cfg.rated_torque  / 1000.0;   /* mNm -> Nm */
    const double rated_a  = cfg.rated_current / 1000.0;   /* mA  -> A  */
    m_countsPerRev = (cfg.enc_increments != 0u && cfg.enc_motor_revs != 0u)
                   ? (double(cfg.enc_increments) / double(cfg.enc_motor_revs))
                   : kIncPerRevDflt;
    const double cpr = m_countsPerRev;
    if (m_encInc)  m_encInc ->setValue(static_cast<int>(cfg.enc_increments));
    if (m_encRevs) m_encRevs->setValue(static_cast<int>(cfg.enc_motor_revs));

    m_homeOffset     ->setValue(incToRad (int32_t(cfg.home_offset), cpr));
    m_homingFast     ->setValue(incToRad (cfg.homing_speed_fast, cpr));
    m_homingSlow     ->setValue(incToRad (cfg.homing_speed_slow, cpr));
    m_homingAccel    ->setValue(incToRad (cfg.homing_accel, cpr));

    m_profileVel     ->setValue(incToRad (cfg.profile_velocity, cpr));
    m_profileAccel   ->setValue(incToRad (cfg.profile_accel, cpr));
    m_profileDecel   ->setValue(incToRad (cfg.profile_decel, cpr));
    m_quickstopDecel ->setValue(incToRad (cfg.quickstop_decel, cpr));

    m_followingError ->setValue(incToRad (cfg.following_error, cpr));
    m_followingMs    ->setValue(int(cfg.following_error_ms));
    m_posMin         ->setValue(incToRad (int32_t(cfg.pos_limit_min), cpr));
    m_posMax         ->setValue(incToRad (int32_t(cfg.pos_limit_max), cpr));
    m_maxSpeed       ->setValue(rpmToRadS(cfg.max_motor_speed));
    m_maxTorque      ->setValue(milliToPhys(cfg.max_torque, rated_nm));
    m_ratedTorque    ->setValue(rated_nm);
    if (m_ratedCurrent) m_ratedCurrent->setValue(rated_a);
    if (m_stallCurrent) m_stallCurrent->setValue(int(cfg.stall_current));
    if (m_stallTime)    m_stallTime   ->setValue(int(cfg.stall_time));

    /* Manufacturer-range fields. */
    if (m_mfNodeId)     m_mfNodeId    ->setValue(int(cfg.node_id));
    if (m_mfCurOffA)    m_mfCurOffA   ->setValue(double(cfg.current_offset_a));
    if (m_mfCurOffB)    m_mfCurOffB   ->setValue(double(cfg.current_offset_b));
    if (m_mfCurOffC)    m_mfCurOffC   ->setValue(double(cfg.current_offset_c));
    if (m_mfCurGainA)   m_mfCurGainA  ->setValue(double(cfg.current_gain_a));
    if (m_mfCurGainB)   m_mfCurGainB  ->setValue(double(cfg.current_gain_b));
    if (m_mfCurGainC)   m_mfCurGainC  ->setValue(double(cfg.current_gain_c));
    if (m_mfHallOffset) m_mfHallOffset->setValue(double(cfg.hall_offset));
}

DriveConfig DriveConfigDialog::config() const
{
    DriveConfig c;
    /* SI (dialog) -> CiA wire units. Rated torque/current are harvested
     * first so the per-mille max-torque conversion uses the new ratings. */
    const double rated_nm = m_ratedTorque->value();              /* Nm */
    const double rated_a  = m_ratedCurrent ? m_ratedCurrent->value() : 0.0;
    /* Use the live (possibly edited) encoder resolution for SI<->inc. */
    if (m_encInc)  c.enc_increments = static_cast<uint32_t>(m_encInc->value());
    if (m_encRevs) c.enc_motor_revs = static_cast<uint32_t>(m_encRevs->value());
    const double cpr = (c.enc_increments != 0u && c.enc_motor_revs != 0u)
                     ? (double(c.enc_increments) / double(c.enc_motor_revs))
                     : m_countsPerRev;

    c.homing_method      = int8_t(m_homingMethod->currentData().toInt());
    c.home_offset        = int32_t (lround(radToInc(m_homeOffset->value(), cpr)));
    c.homing_speed_fast  = uint32_t(lround(radToInc(m_homingFast->value(), cpr)));
    c.homing_speed_slow  = uint32_t(lround(radToInc(m_homingSlow->value(), cpr)));
    c.homing_accel       = uint32_t(lround(radToInc(m_homingAccel->value(), cpr)));

    c.profile_velocity   = uint32_t(lround(radToInc(m_profileVel->value(), cpr)));
    c.profile_accel      = uint32_t(lround(radToInc(m_profileAccel->value(), cpr)));
    c.profile_decel      = uint32_t(lround(radToInc(m_profileDecel->value(), cpr)));
    c.quickstop_decel    = uint32_t(lround(radToInc(m_quickstopDecel->value(), cpr)));

    c.following_error    = uint32_t(lround(radToInc(m_followingError->value(), cpr)));
    c.following_error_ms = uint16_t(m_followingMs->value());
    c.pos_limit_min      = int32_t (lround(radToInc(m_posMin->value(), cpr)));
    c.pos_limit_max      = int32_t (lround(radToInc(m_posMax->value(), cpr)));
    c.max_motor_speed    = uint32_t(lround(radSToRpm(m_maxSpeed->value())));
    c.max_torque         = uint16_t(lround(physToMilli(m_maxTorque->value(), rated_nm)));
    /* c.rated_torque (0x6076) is NOT set: it is derived/read-only on the
     * drive. rated_nm above is still used for the max-torque per-mille scale. */
    if (m_ratedCurrent) c.rated_current = uint32_t(lround(rated_a * 1000.0)); /* A -> mA */
    if (m_stallCurrent) c.stall_current = uint32_t(m_stallCurrent->value());
    if (m_stallTime)    c.stall_time    = uint32_t(m_stallTime->value());

    /* Manufacturer-range fields (0x20xx), all RW. */
    if (m_mfNodeId)     c.node_id          = uint8_t (m_mfNodeId->value());
    if (m_mfCurOffA)    c.current_offset_a = float (m_mfCurOffA->value());
    if (m_mfCurOffB)    c.current_offset_b = float (m_mfCurOffB->value());
    if (m_mfCurOffC)    c.current_offset_c = float (m_mfCurOffC->value());
    if (m_mfCurGainA)   c.current_gain_a   = float (m_mfCurGainA->value());
    if (m_mfCurGainB)   c.current_gain_b   = float (m_mfCurGainB->value());
    if (m_mfCurGainC)   c.current_gain_c   = float (m_mfCurGainC->value());
    if (m_mfHallOffset) c.hall_offset      = float (m_mfHallOffset->value());
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
