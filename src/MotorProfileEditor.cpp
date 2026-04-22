#include "MotorProfileEditor.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>
#include <QVBoxLayout>

namespace vrmc {

MotorProfileEditor::MotorProfileEditor(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Motor profile"));
    setMinimumWidth(440);

    /* Helpers ---------------------------------------------------------- */
    auto makeDouble = [this](double init, double step, int decimals,
                             double lo, double hi){
        auto* sb = new QDoubleSpinBox(this);
        sb->setRange(lo, hi);
        sb->setDecimals(decimals);
        sb->setSingleStep(step);
        sb->setValue(init);
        return sb;
    };
    auto makeInt = [this](int init, int lo, int hi, const QString& suffix = {}){
        auto* sb = new QSpinBox(this);
        sb->setRange(lo, hi);
        sb->setValue(init);
        if (!suffix.isEmpty()){ sb->setSuffix(suffix); }
        return sb;
    };

    /* Topology --------------------------------------------------------- */
    m_type = new QComboBox(this);
    m_type->addItem(tr("PMSM (sinusoidal)"),     int(MotorType::Pmsm));
    m_type->addItem(tr("BLDC (trapezoidal)"),    int(MotorType::Bldc));

    m_polePair = makeInt(4, 1, 64);

    auto* topoBox  = new QGroupBox(tr("Topology"), this);
    auto* topoForm = new QFormLayout(topoBox);
    topoForm->addRow(tr("Type"),       m_type);
    topoForm->addRow(tr("Pole pairs"), m_polePair);

    /* Electrical ------------------------------------------------------- */
    m_rs      = makeDouble(0.5,    0.01,  4, 0.0, 1000.0);
    m_lsd     = makeDouble(0.0015, 1e-5,  6, 0.0, 1.0);
    m_lsq     = makeDouble(0.0015, 1e-5,  6, 0.0, 1.0);
    m_flux    = makeDouble(0.05,   0.005, 5, 0.0, 10.0);
    m_inertia = makeDouble(5.0e-5, 1e-6,  7, 0.0, 1.0);

    m_rs->setSuffix(tr(" Ω"));
    m_lsd->setSuffix(tr(" H"));
    m_lsq->setSuffix(tr(" H"));
    m_flux->setSuffix(tr(" Wb"));
    m_inertia->setSuffix(tr(" kg·m²"));

    auto* elecBox  = new QGroupBox(tr("Electrical / mechanical"), this);
    auto* elecForm = new QFormLayout(elecBox);
    elecForm->addRow(tr("Rs (phase resistance)"),  m_rs);
    elecForm->addRow(tr("Ls_d (d-axis inductance)"), m_lsd);
    elecForm->addRow(tr("Ls_q (q-axis inductance)"), m_lsq);
    elecForm->addRow(tr("Rated PM flux λ_m"),      m_flux);
    elecForm->addRow(tr("Rotor inertia J"),        m_inertia);

    /* Speed + voltage + current --------------------------------------- */
    m_ratedSpd = makeInt(1000, 0, 100000, tr(" rad/s"));

    m_ratedVol = makeInt(24, 0, 1000, tr(" V"));
    m_minVol   = makeInt(12, 0, 1000, tr(" V"));
    m_maxVol   = makeInt(36, 0, 1000, tr(" V"));

    m_ratedCur  = makeInt(2, 0, 1000, tr(" A"));
    m_maxCur    = makeInt(6, 0, 1000, tr(" A"));
    m_stallCur  = makeInt(8, 0, 1000, tr(" A"));
    m_stallTime = makeInt(1, 0, 600,  tr(" s"));

    auto* speedBox  = new QGroupBox(tr("Speed / voltage / current"), this);
    auto* speedForm = new QFormLayout(speedBox);
    speedForm->addRow(tr("Rated speed"),     m_ratedSpd);
    speedForm->addRow(tr("Rated voltage"),   m_ratedVol);
    speedForm->addRow(tr("Min voltage"),     m_minVol);
    speedForm->addRow(tr("Max voltage"),     m_maxVol);
    speedForm->addRow(tr("Rated current"),   m_ratedCur);
    speedForm->addRow(tr("Max current"),     m_maxCur);
    speedForm->addRow(tr("Stall current"),   m_stallCur);
    speedForm->addRow(tr("Stall time"),      m_stallTime);

    /* Buttons ---------------------------------------------------------- */
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    /* Root ------------------------------------------------------------- */
    auto* root = new QVBoxLayout(this);
    root->addWidget(topoBox);
    root->addWidget(elecBox);
    root->addWidget(speedBox);
    root->addWidget(buttons);
}

void MotorProfileEditor::setParams(const MotorParams& mp)
{
    const int idx = m_type->findData(int(mp.type));
    m_type->setCurrentIndex(idx >= 0 ? idx : 0);
    m_polePair->setValue(static_cast<int>(mp.pole_pair));

    m_rs->setValue(mp.rs);
    m_lsd->setValue(mp.ls_d);
    m_lsq->setValue(mp.ls_q);
    m_flux->setValue(mp.rated_flux);
    m_inertia->setValue(mp.inertia);

    m_ratedSpd->setValue(mp.rated_speed);
    m_ratedVol->setValue(mp.rated_vol);
    m_minVol->setValue(mp.min_vol);
    m_maxVol->setValue(mp.max_vol);
    m_ratedCur->setValue(mp.rated_cur);
    m_maxCur->setValue(mp.max_cur);
    m_stallCur->setValue(mp.stall_cur);
    m_stallTime->setValue(mp.stall_time_cur);
}

MotorParams MotorProfileEditor::params() const
{
    MotorParams mp;
    mp.type           = static_cast<MotorType>(m_type->currentData().toInt());
    mp.pole_pair      = static_cast<uint32_t>(m_polePair->value());
    mp.rs             = static_cast<float>(m_rs->value());
    mp.ls_d           = static_cast<float>(m_lsd->value());
    mp.ls_q           = static_cast<float>(m_lsq->value());
    mp.rated_flux     = static_cast<float>(m_flux->value());
    mp.inertia        = static_cast<float>(m_inertia->value());
    mp.rated_speed    = m_ratedSpd->value();
    mp.rated_vol      = m_ratedVol->value();
    mp.min_vol        = m_minVol->value();
    mp.max_vol        = m_maxVol->value();
    mp.rated_cur      = m_ratedCur->value();
    mp.max_cur        = m_maxCur->value();
    mp.stall_cur      = m_stallCur->value();
    mp.stall_time_cur = m_stallTime->value();
    return mp;
}

}  // namespace vrmc
