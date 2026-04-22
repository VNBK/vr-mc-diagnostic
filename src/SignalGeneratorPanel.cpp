#include "SignalGeneratorPanel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace vrmc {

SignalGeneratorPanel::SignalGeneratorPanel(QWidget* parent) : QWidget(parent)
{
    m_label = new QLabel(tr("(no slave selected)"), this);

    m_shape = new QComboBox(this);
    m_shape->addItem(tr("Constant"), int(GenCfg::Constant));
    m_shape->addItem(tr("Step"),     int(GenCfg::Step));
    m_shape->addItem(tr("Ramp"),     int(GenCfg::Ramp));
    m_shape->addItem(tr("Sine"),     int(GenCfg::Sine));
    m_shape->addItem(tr("Chirp"),    int(GenCfg::Chirp));
    m_shape->setCurrentIndex(int(GenCfg::Step));

    m_target = new QComboBox(this);
    m_target->addItem(tr("Position (rad)"),    int(TargetKind::Position));
    m_target->addItem(tr("Velocity (rad/s)"),  int(TargetKind::Velocity));
    m_target->addItem(tr("Torque (Nm)"),       int(TargetKind::Torque));

    auto makeSpin = [this](double init, double step, int decimals){
        auto* sb = new QDoubleSpinBox(this);
        sb->setRange(-1e6, 1e6);
        sb->setDecimals(decimals);
        sb->setSingleStep(step);
        sb->setValue(init);
        return sb;
    };

    m_amp      = makeSpin(1.0, 0.1,  4);
    m_offset   = makeSpin(0.0, 0.1,  4);
    m_freq     = makeSpin(1.0, 0.1,  3);   m_freq->setSuffix(tr(" Hz"));
    m_freqEnd  = makeSpin(5.0, 0.1,  3);   m_freqEnd->setSuffix(tr(" Hz"));
    m_duration = makeSpin(5.0, 0.5,  2);   m_duration->setSuffix(tr(" s"));

    m_armBtn  = new QPushButton(tr("Arm / Run"), this);
    m_stopBtn = new QPushButton(tr("Stop"),      this);
    m_stopBtn->setEnabled(false);

    auto* form = new QFormLayout;
    form->addRow(tr("Waveform"),        m_shape);
    form->addRow(tr("Target"),          m_target);
    form->addRow(tr("Amplitude"),       m_amp);
    form->addRow(tr("Offset"),          m_offset);
    form->addRow(tr("Frequency"),       m_freq);
    form->addRow(tr("Frequency end"),   m_freqEnd);
    form->addRow(tr("Duration (0=∞)"),  m_duration);

    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(m_armBtn);
    btnRow->addWidget(m_stopBtn);
    btnRow->addStretch();

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(form);
    root->addLayout(btnRow);
    root->addStretch();

    connect(m_shape, &QComboBox::currentIndexChanged,
            this,    &SignalGeneratorPanel::applyShape);

    connect(m_armBtn, &QPushButton::clicked, this, [this]{
        if (m_idx < 0 || m_running){ return; }
        emit startRequested(m_idx, build());
    });
    connect(m_stopBtn, &QPushButton::clicked, this, [this]{
        if (m_idx < 0){ return; }
        emit stopRequested(m_idx);
    });

    applyShape();
    setActiveSlave(-1);
}

void SignalGeneratorPanel::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    if (idx < 0){
        m_label->setText(tr("(select a slave above)"));
    } else {
        m_label->setText(tr("Slave %1 — %2").arg(idx).arg(name));
    }
    const bool en = (idx >= 0);
    for (QWidget* w : { static_cast<QWidget*>(m_shape),
                        static_cast<QWidget*>(m_target),
                        static_cast<QWidget*>(m_amp),
                        static_cast<QWidget*>(m_offset),
                        static_cast<QWidget*>(m_duration),
                        static_cast<QWidget*>(m_armBtn) }){
        w->setEnabled(en && !m_running);
    }
    m_stopBtn->setEnabled(en && m_running);
    applyShape();
}

void SignalGeneratorPanel::applyShape()
{
    const int shape = m_shape->currentData().toInt();
    const bool freqRelevant    = (shape == GenCfg::Sine || shape == GenCfg::Chirp);
    const bool freqEndRelevant = (shape == GenCfg::Chirp);
    m_freq->setEnabled(freqRelevant && m_idx >= 0 && !m_running);
    m_freqEnd->setEnabled(freqEndRelevant && m_idx >= 0 && !m_running);
}

GenCfg SignalGeneratorPanel::build() const
{
    GenCfg c;
    c.shape        = static_cast<GenCfg::Shape>(m_shape->currentData().toInt());
    c.target       = static_cast<TargetKind>(m_target->currentData().toInt());
    c.amplitude    = static_cast<float>(m_amp->value());
    c.offset       = static_cast<float>(m_offset->value());
    c.frequency    = static_cast<float>(m_freq->value());
    c.frequencyEnd = static_cast<float>(m_freqEnd->value());
    c.durationSec  = static_cast<float>(m_duration->value());
    c.rateHz       = 50;
    return c;
}

void SignalGeneratorPanel::onGeneratorStarted(int idx)
{
    if (idx != m_idx){ return; }
    m_running = true;
    m_armBtn->setEnabled(false);
    m_stopBtn->setEnabled(true);
    for (QWidget* w : { static_cast<QWidget*>(m_shape),
                        static_cast<QWidget*>(m_target),
                        static_cast<QWidget*>(m_amp),
                        static_cast<QWidget*>(m_offset),
                        static_cast<QWidget*>(m_freq),
                        static_cast<QWidget*>(m_freqEnd),
                        static_cast<QWidget*>(m_duration) }){
        w->setEnabled(false);
    }
}

void SignalGeneratorPanel::onGeneratorStopped(int idx)
{
    if (idx != m_idx && m_running == false){ return; }
    m_running = false;
    setActiveSlave(m_idx);   /* re-enable widgets as before */
}

}  // namespace vrmc
