#include "RunInDialog.hpp"

#include <QVBoxLayout>
#include <QFormLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <QLabel>

#include <cmath>

namespace vrmc {

/* rpm -> rad/s. rad/s = rpm * 2*pi / 60. */
static constexpr double kRpmToRadS = 2.0 * M_PI / 60.0;
static constexpr double kMaxRpm    = 100000.0;

RunInDialog::RunInDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Run-in (Roda)"));
    setModal(false);

    auto* form = new QFormLayout;

    m_speed = new QDoubleSpinBox;
    m_speed->setRange(0.0, kMaxRpm);
    m_speed->setDecimals(0);
    m_speed->setSuffix(tr(" rpm"));
    m_speed->setValue(300.0);
    form->addRow(tr("Speed:"), m_speed);

    m_unitRadS = new QCheckBox(tr("Speed in rad/s"));
    form->addRow(QString(), m_unitRadS);

    m_fwdSec = new QSpinBox;
    m_fwdSec->setRange(1, 36000);
    m_fwdSec->setSuffix(tr(" s"));
    m_fwdSec->setValue(10);
    form->addRow(tr("Forward time:"), m_fwdSec);

    m_revSec = new QSpinBox;
    m_revSec->setRange(1, 36000);
    m_revSec->setSuffix(tr(" s"));
    m_revSec->setValue(10);
    form->addRow(tr("Reverse time:"), m_revSec);

    m_totalSec = new QSpinBox;
    m_totalSec->setRange(0, 86400);
    m_totalSec->setSuffix(tr(" s"));
    m_totalSec->setValue(300);
    m_totalSec->setSpecialValueText(tr("until Stop"));   /* shown for value 0 */
    form->addRow(tr("Total run time:"), m_totalSec);

    m_button = new QPushButton(tr("Start"));
    m_status = new QLabel(tr("idle"));
    m_status->setWordWrap(true);

    auto* lay = new QVBoxLayout(this);
    lay->addLayout(form);
    lay->addWidget(m_button);
    lay->addWidget(m_status);

    connect(m_button, &QPushButton::clicked, this, &RunInDialog::onButton);
    connect(m_unitRadS, &QCheckBox::toggled, this, &RunInDialog::onUnitToggled);
}

void RunInDialog::onButton()
{
    if (!m_running){
        /* The worker always expects rpm; convert back when in rad/s. */
        double rpm = m_speed->value();
        if (m_unitRadS->isChecked()){ rpm /= kRpmToRadS; }
        emit startRequested(rpm, m_fwdSec->value(),
                             m_revSec->value(), m_totalSec->value());
    } else {
        emit stopRequested();
    }
}

void RunInDialog::onUnitToggled(bool radS)
{
    const double cur = m_speed->value();
    /* Widen the range and decimals BEFORE writing the converted value so
     * it isn't clamped/rounded on the way in. */
    if (radS){
        m_speed->setDecimals(2);
        m_speed->setRange(0.0, kMaxRpm * kRpmToRadS);
        m_speed->setSuffix(tr(" rad/s"));
        m_speed->setValue(cur * kRpmToRadS);
    } else {
        m_speed->setRange(0.0, kMaxRpm);
        m_speed->setDecimals(0);
        m_speed->setSuffix(tr(" rpm"));
        m_speed->setValue(cur / kRpmToRadS);
    }
}

void RunInDialog::setRunning(bool running, const QString& status)
{
    m_running = running;
    m_button->setText(running ? tr("Stop") : tr("Start"));
    m_unitRadS->setEnabled(!running);
    m_speed   ->setEnabled(!running);
    m_fwdSec  ->setEnabled(!running);
    m_revSec  ->setEnabled(!running);
    m_totalSec->setEnabled(!running);
    if (!status.isEmpty()){ m_status->setText(status); }
}

}  // namespace vrmc
