#include "JointControlPanel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace vrmc {

JointControlPanel::JointControlPanel(QWidget* parent) : QWidget(parent)
{
    m_label        = new QLabel(tr("(no slave selected)"), this);

    m_bringup      = new QPushButton(tr("Bringup"),    this);
    m_enable       = new QPushButton(tr("Enable"),     this);
    m_disable      = new QPushButton(tr("Disable"),    this);
    m_faultReset   = new QPushButton(tr("Fault reset"),this);

    m_modeCombo    = new QComboBox(this);
    m_modeCombo->addItem(tr("NONE"),     int(Mode::None));
    m_modeCombo->addItem(tr("TORQUE"),   int(Mode::Torque));
    m_modeCombo->addItem(tr("VELOCITY"), int(Mode::Velocity));
    m_modeCombo->addItem(tr("POSITION"), int(Mode::Position));

    m_targetCombo  = new QComboBox(this);
    m_targetCombo->addItem(tr("Position (rad)"), int(TargetKind::Position));
    m_targetCombo->addItem(tr("Velocity (rad/s)"), int(TargetKind::Velocity));
    m_targetCombo->addItem(tr("Torque (Nm)"),    int(TargetKind::Torque));

    m_valueSpin = new QDoubleSpinBox(this);
    m_valueSpin->setRange(-1000.0, 1000.0);
    m_valueSpin->setDecimals(4);
    m_valueSpin->setSingleStep(0.01);

    m_sendBtn = new QPushButton(tr("Send"), this);

    auto* powerRow = new QHBoxLayout;
    powerRow->addWidget(m_bringup);
    powerRow->addWidget(m_enable);
    powerRow->addWidget(m_disable);
    powerRow->addWidget(m_faultReset);
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

    auto* tgtBox = new QGroupBox(tr("Setpoint"), this);
    {
        auto* l = new QGridLayout(tgtBox);
        l->addWidget(m_targetCombo, 0, 0);
        l->addWidget(m_valueSpin,   0, 1);
        l->addWidget(m_sendBtn,     0, 2);
        connect(m_sendBtn, &QPushButton::clicked, this, &JointControlPanel::emitTarget);
    }

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(powerRow);
    root->addWidget(modeBox);
    root->addWidget(tgtBox);
    root->addStretch();

    connect(m_bringup,    &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit bringupRequested(m_idx, 2000);
    });
    connect(m_enable,     &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit enableRequested(m_idx);
    });
    connect(m_disable,    &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit disableRequested(m_idx);
    });
    connect(m_faultReset, &QPushButton::clicked, this, [this]{
        if (m_idx >= 0) emit faultResetRequested(m_idx);
    });

    setActiveSlave(-1);
}

void JointControlPanel::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    if (idx < 0){
        m_label->setText(tr("(select a slave above)"));
    } else {
        m_label->setText(tr("Slave %1  —  %2").arg(idx).arg(name));
    }
    const bool enabled = (idx >= 0);
    for (QWidget* w : { static_cast<QWidget*>(m_bringup),
                        static_cast<QWidget*>(m_enable),
                        static_cast<QWidget*>(m_disable),
                        static_cast<QWidget*>(m_faultReset),
                        static_cast<QWidget*>(m_modeCombo),
                        static_cast<QWidget*>(m_targetCombo),
                        static_cast<QWidget*>(m_valueSpin),
                        static_cast<QWidget*>(m_sendBtn) }){
        w->setEnabled(enabled);
    }
}

void JointControlPanel::emitTarget()
{
    if (m_idx < 0){ return; }
    emit targetRequested(m_idx,
        static_cast<TargetKind>(m_targetCombo->currentData().toInt()),
        static_cast<float>(m_valueSpin->value()));
}

}  // namespace vrmc
