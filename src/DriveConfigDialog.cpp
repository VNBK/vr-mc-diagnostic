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
    auto* tApp     = new QWidget;
    auto* tMotor   = new QWidget;
    auto* tRuntime = new QWidget;
    auto* tCustom  = new QWidget;
    auto* tStorage = new QWidget;
    buildAppTab    (tApp);
    buildMotorTab  (tMotor);
    buildRuntimeTab(tRuntime);
    buildCustomTab (tCustom);
    buildStorageTab(tStorage);
    tabs->addTab(tApp,     tr("App parameters"));
    tabs->addTab(tMotor,   tr("Motor parameters"));
    tabs->addTab(tRuntime, tr("Runtime"));
    tabs->addTab(tCustom,  tr("Custom SDO"));
    tabs->addTab(tStorage, tr("Storage"));

    m_readBtn = new QPushButton(tr("Read from drive"), this);
    m_readBtn->setToolTip(tr("Re-fetch every field from the drive's OD."));
    m_saveBtn = new QPushButton(tr("Save"), this);
    m_saveBtn->setToolTip(
        tr("Write all fields to the drive (live) AND commit the App + Motor "
           "blobs to flash (0x1010:01 = save). One click, fully persisted."));
    connect(m_readBtn, &QPushButton::clicked,
            this,      &DriveConfigDialog::onReadClicked);
    connect(m_saveBtn, &QPushButton::clicked,
            this,      &DriveConfigDialog::onSaveClicked);

    m_buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(m_buttons, &QDialogButtonBox::accepted, this, [this]{
        onSaveClicked();   /* OK = Save + close */
        accept();
    });
    connect(m_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* actionRow = new QHBoxLayout;
    actionRow->addWidget(m_readBtn);
    actionRow->addWidget(m_saveBtn);
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

/* ============================================================ App tab
 *
 * All fields here are persisted in the app_params_t blob (data-flash).
 * Save = Storage tab :04 (app-only) OR :01 (all). Layout matches the
 * board struct: identity / mechanical / sensor cal. Commissioning
 * actions that mutate App-blob fields (zero-encoder writes 0x607C)
 * live with the field they modify. */
void DriveConfigDialog::buildAppTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    /* --- Identity --- */
    auto* idBox = new QGroupBox(tr("Identity"), host);
    {
        auto* form = new QFormLayout(idBox);
        m_mfNodeId = makeSpin(1, 127, 5);
        form->addRow(tr("Node ID (0x2000:01)"), m_mfNodeId);
    }
    root->addWidget(idBox);

    /* --- Mechanical (home offset, gear, encoder) --- */
    auto* mechBox = new QGroupBox(tr("Mechanical"), host);
    {
        auto* form = new QFormLayout(mechBox);
        m_homeOffset = makeDSpin(-1e6, 1e6, 4, tr("rad"));
        m_encInc     = makeSpin(1, 1'000'000'000, 16384, tr("inc"));
        m_encRevs    = makeSpin(1, 1'000'000,     1,     tr("rev"));
        m_gearMotor  = makeSpin(1, 1'000'000, 1, tr("rev"));
        m_gearShaft  = makeSpin(1, 1'000'000, 1, tr("rev"));

        form->addRow(tr("Home offset (0x607C)"),          m_homeOffset);
        form->addRow(tr("Gear motor revs (0x6091:1)"),    m_gearMotor);
        form->addRow(tr("Gear shaft revs (0x6091:2)"),    m_gearShaft);
        form->addRow(tr("Encoder increments (0x608F:1)"), m_encInc);
        form->addRow(tr("Motor revolutions (0x608F:2)"),  m_encRevs);

        m_zeroEncBtn = new QPushButton(tr("Zero encoder here"), mechBox);
        m_zeroEncBtn->setToolTip(
            tr("Capture position_actual (0x6064) and write it into\n"
               "home_offset (0x607C). Makes \"here\" the new origin."));
        connect(m_zeroEncBtn, &QPushButton::clicked,
                this,         &DriveConfigDialog::onZeroEncoderClicked);
        auto* row = new QHBoxLayout;
        row->addWidget(m_zeroEncBtn);
        row->addStretch(1);
        form->addRow(QString(), row);
    }
    root->addWidget(mechBox);

    /* --- Sensor calibration --- */
    auto* sensBox = new QGroupBox(tr("Sensor calibration"), host);
    {
        auto* form = new QFormLayout(sensBox);
        m_mfHallOffset = new QDoubleSpinBox(sensBox);
        m_mfHallOffset->setRange(-7.0, 7.0);
        m_mfHallOffset->setDecimals(5);
        m_mfHallOffset->setValue(0.0);
        m_mfHallOffset->setAlignment(Qt::AlignRight);
        m_mfHallOffset->setSuffix(tr(" rad"));
        form->addRow(tr("Hall offset (0x2060)"), m_mfHallOffset);

        auto* tmagNote = new QLabel(
            tr("TMAG6180 sin/cos cal is run as a one-shot via "
               "0x2031:01 = 3 (Custom SDO tab), not edited per-field here."),
            sensBox);
        tmagNote->setWordWrap(true);
        tmagNote->setStyleSheet(QStringLiteral(
            "QLabel { color: #9aa; padding: 2px 4px; }"));
        form->addRow(QString(), tmagNote);
    }
    root->addWidget(sensBox);

    auto* hint = new QLabel(
        tr("💾 Persisted in <b>app_params</b> blob (data-flash). "
           "Save via Storage tab → <i>Save App (:04)</i> or "
           "<i>Save All (:01)</i>, or just click <b>Save</b> below."),
        host);
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral(
        "QLabel { color: #9ad8a8; padding: 4px; border-top: 1px solid #333; }"));
    root->addWidget(hint);
    root->addStretch(1);
}

/* ============================================================ Motor tab
 *
 * All fields here are persisted in the motor_drive_params_t blob
 * (code-flash). Save = Storage tab :05 (motor-only) OR :01 (all).
 * Layout: limits / rated / current sense / fault thresholds. Cross-
 * references to Motor Profile editor + Gains tab at the bottom. */
void DriveConfigDialog::buildMotorTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    /* --- Motion limits --- */
    auto* limBox = new QGroupBox(tr("Motion limits"), host);
    {
        auto* form = new QFormLayout(limBox);
        m_posMin    = makeDSpin(-1e6, 1e6, 4, tr("rad"));
        m_posMax    = makeDSpin(-1e6, 1e6, 4, tr("rad"));
        m_maxSpeed  = makeDSpin(0, 1e4, 3, tr("rad/s"));
        m_maxTorque = makeDSpin(0, 1e4, 4, tr("Nm"));
        form->addRow(tr("Pos limit min (0x607D:1)"), m_posMin);
        form->addRow(tr("Pos limit max (0x607D:2)"), m_posMax);
        form->addRow(tr("Max motor speed (0x6080)"), m_maxSpeed);
        form->addRow(tr("Max torque (0x6072)"),      m_maxTorque);
    }
    root->addWidget(limBox);

    /* --- Rated values --- */
    auto* rateBox = new QGroupBox(tr("Rated values"), host);
    {
        auto* form = new QFormLayout(rateBox);
        m_ratedCurrent = makeDSpin(0, 1e4, 3, tr("A"));
        m_ratedTorque  = makeDSpin(0, 1e4, 4, tr("Nm"));
        /* 0x6076 is derived (= Kt * rated_current) and RO on the drive. */
        m_ratedTorque->setReadOnly(true);
        m_ratedTorque->setButtonSymbols(QAbstractSpinBox::NoButtons);
        form->addRow(tr("Rated current (0x6075)"),         m_ratedCurrent);
        form->addRow(tr("Rated torque (0x6076, derived)"), m_ratedTorque);

        m_zeroTorqueBtn = new QPushButton(tr("Zero torque here"), rateBox);
        m_zeroTorqueBtn->setToolTip(
            tr("Capture torque_actual (0x6077) and write it into\n"
               "torque_offset (0x60B2) so the current loading reads as 0."));
        connect(m_zeroTorqueBtn, &QPushButton::clicked,
                this,            &DriveConfigDialog::onZeroTorqueClicked);
        auto* row = new QHBoxLayout;
        row->addWidget(m_zeroTorqueBtn);
        row->addStretch(1);
        form->addRow(QString(), row);

        m_torqueState = new QLabel(tr("Torque: —"), rateBox);
        m_torqueState->setStyleSheet(QStringLiteral(
            "QLabel { padding: 6px 8px; border: 1px solid #666;"
            "         border-radius: 4px; background: #1a1a1a; color: #ddd; }"));
        form->addRow(QString(), m_torqueState);
    }
    root->addWidget(rateBox);

    /* --- Current sensor calibration --- */
    auto* csBox = new QGroupBox(tr("Current sensor calibration"), host);
    {
        auto makeCS = [csBox](double lo, double hi, double v, int dec){
            auto* s = new QDoubleSpinBox(csBox);
            s->setRange(lo, hi);
            s->setDecimals(dec);
            s->setValue(v);
            s->setAlignment(Qt::AlignRight);
            return s;
        };
        auto* form = new QFormLayout(csBox);
        m_mfCurOffA  = makeCS(-1e6, 1e6, 0.0, 4);
        m_mfCurOffB  = makeCS(-1e6, 1e6, 0.0, 4);
        m_mfCurOffC  = makeCS(-1e6, 1e6, 0.0, 4);
        m_mfCurGainA = makeCS(0.0, 1e6, 1.0, 4);
        m_mfCurGainB = makeCS(0.0, 1e6, 1.0, 4);
        m_mfCurGainC = makeCS(0.0, 1e6, 1.0, 4);
        form->addRow(tr("Current offset A (0x2040:1)"), m_mfCurOffA);
        form->addRow(tr("Current offset B (0x2040:2)"), m_mfCurOffB);
        form->addRow(tr("Current offset C (0x2040:3)"), m_mfCurOffC);
        form->addRow(tr("Current gain A (0x2041:1)"),   m_mfCurGainA);
        form->addRow(tr("Current gain B (0x2041:2)"),   m_mfCurGainB);
        form->addRow(tr("Current gain C (0x2041:3)"),   m_mfCurGainC);
    }
    root->addWidget(csBox);

    /* --- Fault thresholds --- */
    auto* faultBox = new QGroupBox(tr("Fault thresholds"), host);
    {
        auto* form = new QFormLayout(faultBox);
        m_stallCurrent = makeSpin(0, 1'000'000, 0, tr("mA"));
        m_stallTime    = makeSpin(0, 60000,     0, tr("ms"));
        form->addRow(tr("Stall current (0x2050:1)"), m_stallCurrent);
        form->addRow(tr("Stall time (0x2050:2)"),    m_stallTime);
    }
    root->addWidget(faultBox);

    auto* xref = new QLabel(
        tr("Related editors that also write to this blob:<br>"
           "• <b>Motor Profile</b> editor → writes 0x2070 (Rs, Ls, Kt, J, …)<br>"
           "• <b>Gains</b> tab → writes 0x60F6 / 0x60F9 / 0x60FB"),
        host);
    xref->setWordWrap(true);
    xref->setStyleSheet(QStringLiteral("QLabel { color: #9aa; padding: 4px; }"));
    root->addWidget(xref);

    auto* hint = new QLabel(
        tr("💾 Persisted in <b>motor_drive_params</b> blob (code-flash). "
           "Save via Storage tab → <i>Save Motor (:05)</i> or "
           "<i>Save All (:01)</i>, or just click <b>Save</b> below."),
        host);
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral(
        "QLabel { color: #9ad8a8; padding: 4px; border-top: 1px solid #333; }"));
    root->addWidget(hint);
    root->addStretch(1);
}

/* ============================================================ Runtime tab
 *
 * Fields here are NOT persisted in any flash blob — they live in the
 * drive's OD RAM only, and reset to defaults on power-cycle / NMT
 * reset. Includes the Homing FSM start action (also transient). */
void DriveConfigDialog::buildRuntimeTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    /* --- Homing --- */
    auto* homeBox = new QGroupBox(tr("Homing"), host);
    {
        auto* form = new QFormLayout(homeBox);
        m_homingMethod = new QComboBox(homeBox);
        for (const auto& m : kHomingMethods){
            m_homingMethod->addItem(QString::fromUtf8(m.label), int(m.value));
        }
        m_homingFast  = makeDSpin(0, 1e4, 3, tr("rad/s"));
        m_homingSlow  = makeDSpin(0, 1e4, 3, tr("rad/s"));
        m_homingAccel = makeDSpin(0, 1e6, 1, tr("rad/s²"));
        form->addRow(tr("Method (0x6098)"),                m_homingMethod);
        form->addRow(tr("Switch search speed (0x6099:1)"), m_homingFast);
        form->addRow(tr("Zero search speed  (0x6099:2)"),  m_homingSlow);
        form->addRow(tr("Homing acceleration (0x609A)"),   m_homingAccel);

        m_startHomingBtn = new QPushButton(tr("Start Homing"), homeBox);
        m_startHomingBtn->setToolTip(
            tr("Switch drive to mode 6 and pulse controlword bit 4 to start\n"
               "the homing procedure. Requires the drive to be Enabled first."));
        connect(m_startHomingBtn, &QPushButton::clicked,
                this,             &DriveConfigDialog::onStartHomingClicked);
        auto* row = new QHBoxLayout;
        row->addWidget(m_startHomingBtn);
        row->addStretch(1);
        form->addRow(QString(), row);

        m_homingState = new QLabel(tr("Homing: —"), homeBox);
        m_homingState->setStyleSheet(QStringLiteral(
            "QLabel { padding: 6px 8px; border: 1px solid #666;"
            "         border-radius: 4px; background: #1a1a1a; color: #ddd; }"));
        form->addRow(QString(), m_homingState);
    }
    root->addWidget(homeBox);

    /* --- Motion profile --- */
    auto* motBox = new QGroupBox(tr("Motion profile"), host);
    {
        auto* form = new QFormLayout(motBox);
        m_profileVel     = makeDSpin(0, 1e4, 3, tr("rad/s"));
        m_profileAccel   = makeDSpin(0, 1e6, 1, tr("rad/s²"));
        m_profileDecel   = makeDSpin(0, 1e6, 1, tr("rad/s²"));
        m_quickstopDecel = makeDSpin(0, 1e6, 1, tr("rad/s²"));
        form->addRow(tr("Profile velocity (0x6081)"),       m_profileVel);
        form->addRow(tr("Profile acceleration (0x6083)"),   m_profileAccel);
        form->addRow(tr("Profile deceleration (0x6084)"),   m_profileDecel);
        form->addRow(tr("Quickstop deceleration (0x6085)"), m_quickstopDecel);
    }
    root->addWidget(motBox);

    /* --- Following error --- */
    auto* feBox = new QGroupBox(tr("Following error"), host);
    {
        auto* form = new QFormLayout(feBox);
        m_followingError = makeDSpin(0, 1e6, 4, tr("rad"));
        m_followingMs    = makeSpin(0, 60000, 100, tr("ms"));
        form->addRow(tr("Max following error (0x6065)"), m_followingError);
        form->addRow(tr("Following error time (0x6066)"), m_followingMs);
    }
    root->addWidget(feBox);

    auto* hint = new QLabel(
        tr("⚠ <b>Not persisted</b>. These OD entries live in the drive's "
           "RAM only and reset to defaults on the next power-cycle / NMT "
           "reset. <b>Save</b> writes them live but they will NOT survive "
           "a reboot."),
        host);
    hint->setWordWrap(true);
    hint->setStyleSheet(QStringLiteral(
        "QLabel { color: #d8c89a; padding: 4px; border-top: 1px solid #333; }"));
    root->addWidget(hint);
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
    if (idx != m_slaveIdx){ return; }

    /* 0x1010 / 0x1011 commands land on the Storage tab; everything else
     * on the Custom SDO tab. The two paths share the customSdoDone
     * signal because the Storage tab piggybacks customSdoWrite under
     * the hood (same wire shape, same worker slot). */
    const bool isStorage = (odIdx == 0x1010 || odIdx == 0x1011);
    QLabel* target = isStorage ? m_storageStatus : m_customResult;
    if (!target) return;

    const QString head = QStringLiteral("0x%1.%2 %3")
        .arg(odIdx, 4, 16, QChar('0'))
        .arg(sub,   2, 16, QChar('0'))
        .arg(isWrite ? QStringLiteral("WRITE") : QStringLiteral("READ"));
    if (ok){
        const QString body = isWrite
            ? (isStorage && odIdx == 0x1011
                  ? tr("OK — blob erased; power-cycle to load defaults.")
                  : tr("OK"))
            : tr("OK  →  %1").arg(valueDecoded);
        target->setStyleSheet(QStringLiteral(
            "QLabel { font-family: monospace; padding: 6px 8px; "
            "         border: 1px solid #3c6e3c; border-radius: 4px; "
            "         background: #14301a; color: #aed8ae; }"));
        target->setText(QStringLiteral("%1: %2").arg(head, body));
    } else {
        target->setStyleSheet(QStringLiteral(
            "QLabel { font-family: monospace; padding: 6px 8px; "
            "         border: 1px solid #8a1a1a; border-radius: 4px; "
            "         background: #2a1010; color: #ff9090; }"));
        target->setText(QStringLiteral("%1 FAILED: %2").arg(head, message));
    }
}

/* ---------------------------------------------------------------- Storage
 *
 * CiA-301 0x1010 (Save) / 0x1011 (Restore) buttons. Each button writes the
 * spec magic value LE-encoded ("save" = 0x65766173, "load" = 0x64616F6C)
 * to the matching subindex; board-side dispatch fans out to one or both
 * persistence callbacks:
 *
 *   :01  both blobs (app_params + motor_drive_params)
 *   :04  app_params only
 *   :05  motor_drive_params only
 *
 * Restore wipes the blob -- defaults only take effect after a power-cycle
 * / NMT reset. The Status label below paints success / failure with the
 * last command's OD address and SDO outcome. */
void DriveConfigDialog::buildStorageTab(QWidget* host)
{
    auto* root = new QVBoxLayout(host);

    auto makeBtn = [host](const QString& text, uint16_t od, uint8_t sub){
        auto* b = new QPushButton(text, host);
        b->setProperty("od",  od);
        b->setProperty("sub", sub);
        b->setMinimumWidth(160);
        return b;
    };

    /* --- Save group --- */
    auto* saveBox = new QGroupBox(tr("Save to flash (0x1010)"), host);
    {
        auto* col = new QVBoxLayout(saveBox);
        auto* intro = new QLabel(
            tr("Commit the live OD values to flash. Synchronous: blocks "
               "the slave's SDO server for ~20 ms per blob; the control "
               "loop keeps running."), saveBox);
        intro->setWordWrap(true);
        intro->setStyleSheet(QStringLiteral("QLabel { color: #9aa; }"));
        col->addWidget(intro);

        m_saveAllBtn   = makeBtn(tr("Save All (:01)"),         0x1010, 0x01);
        m_saveAppBtn   = makeBtn(tr("Save App params (:04)"),  0x1010, 0x04);
        m_saveMotorBtn = makeBtn(tr("Save Motor params (:05)"), 0x1010, 0x05);

        auto* row = new QHBoxLayout;
        row->addWidget(m_saveAllBtn);
        row->addWidget(m_saveAppBtn);
        row->addWidget(m_saveMotorBtn);
        row->addStretch(1);
        col->addLayout(row);
    }
    root->addWidget(saveBox);

    /* --- Restore group --- */
    auto* restoreBox = new QGroupBox(tr("Restore defaults (0x1011)"), host);
    {
        auto* col = new QVBoxLayout(restoreBox);
        auto* intro = new QLabel(
            tr("Erase the flash blob. The live motor keeps its current "
               "gains; <b>factory defaults take effect on the next "
               "power-on or NMT reset</b>."), restoreBox);
        intro->setWordWrap(true);
        intro->setStyleSheet(QStringLiteral("QLabel { color: #c9a; }"));
        col->addWidget(intro);

        m_restoreAllBtn   = makeBtn(tr("Restore All (:01)"),         0x1011, 0x01);
        m_restoreAppBtn   = makeBtn(tr("Restore App params (:04)"),  0x1011, 0x04);
        m_restoreMotorBtn = makeBtn(tr("Restore Motor params (:05)"), 0x1011, 0x05);

        auto* row = new QHBoxLayout;
        row->addWidget(m_restoreAllBtn);
        row->addWidget(m_restoreAppBtn);
        row->addWidget(m_restoreMotorBtn);
        row->addStretch(1);
        col->addLayout(row);
    }
    root->addWidget(restoreBox);

    /* --- Status line --- */
    m_storageStatus = new QLabel(tr("Ready."), host);
    m_storageStatus->setWordWrap(true);
    m_storageStatus->setStyleSheet(QStringLiteral(
        "QLabel { font-family: monospace; padding: 6px 8px; "
        "         border: 1px solid #444; border-radius: 4px; "
        "         background: #1a1a1a; color: #ccc; }"));
    root->addWidget(m_storageStatus);
    root->addStretch(1);

    /* Restore buttons get a confirm dialog before firing -- this is the
     * destructive path. Save buttons fire immediately. */
    auto fire = [this](QPushButton* b, uint32_t magic, bool destructive){
        connect(b, &QPushButton::clicked, this, [this, b, magic, destructive]{
            if (m_slaveIdx < 0){
                m_storageStatus->setText(tr("Select a slave first."));
                return;
            }
            const uint16_t od  = uint16_t(b->property("od").toUInt());
            const uint8_t  sub = uint8_t (b->property("sub").toUInt());
            if (destructive){
                const auto ans = QMessageBox::warning(
                    this, tr("Restore defaults"),
                    tr("This will erase the parameter blob on slave %1.\n\n"
                       "The live motor keeps its current gains; defaults take "
                       "effect on the next power-cycle / NMT reset.\n\n"
                       "Continue?").arg(m_slaveIdx),
                    QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
                if (ans != QMessageBox::Ok) return;
            }
            QByteArray payload(4, '\0');
            payload[0] = char( magic        & 0xFFu);
            payload[1] = char((magic >> 8 ) & 0xFFu);
            payload[2] = char((magic >> 16) & 0xFFu);
            payload[3] = char((magic >> 24) & 0xFFu);
            m_storageStatus->setText(
                tr("Sending 0x%1:%2 …")
                    .arg(od, 4, 16, QChar('0')).arg(sub));
            emit storageCommandRequested(m_slaveIdx, od, sub, payload);
        });
    };
    fire(m_saveAllBtn,      0x65766173u, /*destructive=*/false);
    fire(m_saveAppBtn,      0x65766173u, false);
    fire(m_saveMotorBtn,    0x65766173u, false);
    fire(m_restoreAllBtn,   0x64616F6Cu, /*destructive=*/true);
    fire(m_restoreAppBtn,   0x64616F6Cu, true);
    fire(m_restoreMotorBtn, 0x64616F6Cu, true);
}

void DriveConfigDialog::setSlaveContext(int idx, const QString& name)
{
    m_slaveIdx  = idx;
    m_slaveName = name;
    if (idx < 0){
        m_header->setText(tr("No slave selected."));
        m_readBtn->setEnabled(false);
        m_saveBtn->setEnabled(false);
        m_buttons->button(QDialogButtonBox::Ok)->setEnabled(false);
    } else {
        m_header->setText(tr("Slave #%1 — %2")
                              .arg(idx)
                              .arg(name.isEmpty() ? tr("(unnamed)") : name));
        m_readBtn->setEnabled(true);
        m_saveBtn->setEnabled(true);
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
    if (m_gearMotor) m_gearMotor->setValue(static_cast<int>(cfg.gear_ratio_motor_revs));
    if (m_gearShaft) m_gearShaft->setValue(static_cast<int>(cfg.gear_ratio_shaft_revs));

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
    if (m_gearMotor) c.gear_ratio_motor_revs = static_cast<uint32_t>(m_gearMotor->value());
    if (m_gearShaft) c.gear_ratio_shaft_revs = static_cast<uint32_t>(m_gearShaft->value());
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

void DriveConfigDialog::onSaveClicked()
{
    if (m_slaveIdx < 0){ return; }

    /* 1) Live commit: write every OD field via writeDriveConfig. The
     *    worker uses Qt::QueuedConnection + an internal mutex, so the
     *    follow-up storage SDO below queues behind this batch and won't
     *    overlap. */
    emit applyRequested(m_slaveIdx, config());

    /* 2) Flash commit: fire 0x1010:01 = "save" so the just-written
     *    values land in flash and survive reboot. Same magic value the
     *    Storage tab's "Save All" button uses. */
    QByteArray payload(4, '\0');
    constexpr uint32_t kMagicSave = 0x65766173u;   /* "save" LE on wire */
    payload[0] = char( kMagicSave        & 0xFFu);
    payload[1] = char((kMagicSave >> 8 ) & 0xFFu);
    payload[2] = char((kMagicSave >> 16) & 0xFFu);
    payload[3] = char((kMagicSave >> 24) & 0xFFu);
    emit storageCommandRequested(m_slaveIdx, 0x1010, 0x01, payload);

    if (m_storageStatus){
        m_storageStatus->setText(tr("Save: writing OD + 0x1010:01 …"));
    }
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
