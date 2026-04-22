#include "DriveConfigDialog.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
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
    resize(520, 520);

    m_header = new QLabel(tr("No slave selected."), this);
    m_header->setStyleSheet(QStringLiteral("font-weight: 600;"));

    auto* tabs = new QTabWidget(this);
    auto* tHoming  = new QWidget;
    auto* tMotion  = new QWidget;
    auto* tProtect = new QWidget;
    auto* tEncoder = new QWidget;
    buildHomingTab (tHoming);
    buildMotionTab (tMotion);
    buildProtectTab(tProtect);
    buildEncoderTab(tEncoder);
    tabs->addTab(tHoming,  tr("Homing"));
    tabs->addTab(tMotion,  tr("Motion profile"));
    tabs->addTab(tProtect, tr("Protection"));
    tabs->addTab(tEncoder, tr("Encoder"));

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

    form->addRow(tr("Max following error (0x6065)"), m_followingError);
    form->addRow(tr("Following error time (0x6066)"), m_followingMs);
    form->addRow(tr("Pos limit min (0x607D:1)"),      m_posMin);
    form->addRow(tr("Pos limit max (0x607D:2)"),      m_posMax);
    form->addRow(tr("Max motor speed (0x6080)"),      m_maxSpeed);
    form->addRow(tr("Max torque (0x6072)"),           m_maxTorque);
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

    m_encRes         ->setValue(int(cfg.enc_resolution));
    m_gearNum        ->setValue(int(cfg.gear_num));
    m_gearDen        ->setValue(int(cfg.gear_den));
    m_feedNum        ->setValue(int(cfg.feed_const_num));
    m_feedDen        ->setValue(int(cfg.feed_const_den));
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

    c.enc_resolution     = uint32_t(m_encRes->value());
    c.gear_num           = uint32_t(m_gearNum->value());
    c.gear_den           = uint32_t(m_gearDen->value());
    c.feed_const_num     = uint32_t(m_feedNum->value());
    c.feed_const_den     = uint32_t(m_feedDen->value());
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
