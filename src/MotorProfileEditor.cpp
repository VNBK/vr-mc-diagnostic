#include "MotorProfileEditor.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
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

    /* Ratings --------------------------------------------------------- */
    /* Torque constant Kt (0x2070:9). 0 -> the drive derives 1.5*pole*flux
     * (PMSM); set explicitly for BLDC/trapezoidal motors. */
    m_torqueConst = makeDouble(0.0, 0.001, 5, 0.0, 1000.0);
    m_torqueConst->setSuffix(tr(" Nm/A"));
    m_ratedTrq = makeDouble(0.5, 0.01, 4, 0.0, 100000.0);
    m_ratedTrq->setSuffix(tr(" Nm"));
    /* Rated torque is DERIVED (= Kt * rated current) and read-only -- it
     * mirrors the drive's read-only 0x6076. */
    m_ratedTrq->setReadOnly(true);
    m_ratedTrq->setButtonSymbols(QAbstractSpinBox::NoButtons);
    m_ratedSpd = makeInt(1000, 0, 100000, tr(" rad/s"));
    m_ratedVol = makeInt(24, 0, 1000, tr(" V"));
    m_ratedCur = makeDouble(2.0, 0.01, 3, 0.0, 1000.0);   /* float A (sub-amp) */
    m_ratedCur->setSuffix(tr(" A"));

    /* Keep the derived rated torque live as its inputs change. */
    connect(m_polePair, &QSpinBox::valueChanged,
            this, &MotorProfileEditor::recomputeRatedTorque);
    connect(m_ratedCur, &QDoubleSpinBox::valueChanged,
            this, &MotorProfileEditor::recomputeRatedTorque);
    connect(m_flux, &QDoubleSpinBox::valueChanged,
            this, &MotorProfileEditor::recomputeRatedTorque);
    connect(m_torqueConst, &QDoubleSpinBox::valueChanged,
            this, &MotorProfileEditor::recomputeRatedTorque);

    auto* ratingBox  = new QGroupBox(tr("Ratings"), this);
    auto* ratingForm = new QFormLayout(ratingBox);
    ratingForm->addRow(tr("Torque constant Kt (0 = derive)"), m_torqueConst);
    ratingForm->addRow(tr("Rated torque (derived = Kt·Irated)"), m_ratedTrq);
    ratingForm->addRow(tr("Rated speed"),           m_ratedSpd);
    ratingForm->addRow(tr("Rated voltage"),         m_ratedVol);
    ratingForm->addRow(tr("Rated current"),         m_ratedCur);

    /* Encoder --------------------------------------------------------- */
    /* Incremental-encoder quadrature counts/rev (= 4 × lines), drive
     * object 0x2070:11 -- only present on the encoder variant (3FL_2). */
    m_cpr = makeInt(4000, 1, 4000000, tr(" cnt/rev"));

    auto* encBox  = new QGroupBox(tr("Encoder"), this);
    auto* encForm = new QFormLayout(encBox);
    encForm->addRow(tr("CPR (= 4 × lines, 0x2070:11)"), m_cpr);

    /* Buttons ---------------------------------------------------------- *
     * Read   = re-fetch 0x2070 + 0x6075 from the slave; updates the form.
     * Save   = accept dialog (caller handles SDO write + 0x1010:01 flash
     *          commit -- this dialog is data-only). Renamed from "OK" to
     *          make the storage semantics match the Drive Config dialog. */
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    if (auto* saveBtn = buttons->button(QDialogButtonBox::Save)){
        saveBtn->setText(tr("Save"));
        saveBtn->setToolTip(
            tr("Write profile to drive (0x2070 + 0x6075) AND commit to flash\n"
               "(0x1010:01 = \"save\"). Survives reboot."));
    }
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    m_readBtn = new QPushButton(tr("Read from drive"), this);
    m_readBtn->setToolTip(
        tr("Re-fetch 0x2070 + 0x6075 from the selected slave and refresh\n"
           "this form. Disabled until a slave is selected."));
    m_readBtn->setEnabled(false);
    connect(m_readBtn, &QPushButton::clicked,
            this,      &MotorProfileEditor::onReadClicked);
    /* Place Read on the LEFT side of the button box so it sits next to
     * the form fields, separate from the Save/Cancel pair on the right. */
    buttons->addButton(m_readBtn, QDialogButtonBox::ResetRole);

    /* Root ------------------------------------------------------------- */
    auto* root = new QVBoxLayout(this);
    root->addWidget(topoBox);
    root->addWidget(elecBox);
    root->addWidget(ratingBox);
    root->addWidget(encBox);
    root->addWidget(buttons);
}

void MotorProfileEditor::setSlaveContext(int slaveIdx)
{
    m_slaveIdx = slaveIdx;
    if (m_readBtn) m_readBtn->setEnabled(slaveIdx >= 0);
}

void MotorProfileEditor::onReadClicked()
{
    if (m_slaveIdx < 0) return;
    emit readRequested(m_slaveIdx);
}

void MotorProfileEditor::setParams(const MotorParams& mp)
{
    /* Cache so non-widget fields (encoder resolution) survive an edit. */
    m_cached = mp;

    const int idx = m_type->findData(int(mp.type));
    m_type->setCurrentIndex(idx >= 0 ? idx : 0);
    m_polePair->setValue(static_cast<int>(mp.pole_pair));

    m_rs->setValue(mp.rs);
    m_lsd->setValue(mp.ls_d);
    m_lsq->setValue(mp.ls_q);
    m_flux->setValue(mp.rated_flux);
    m_inertia->setValue(mp.inertia);

    m_torqueConst->setValue(mp.torque_constant);
    m_ratedSpd->setValue(mp.rated_speed);
    m_ratedVol->setValue(mp.rated_vol);
    m_ratedCur->setValue(mp.rated_cur);

    m_cpr->setValue(static_cast<int>(mp.cpr));

    /* Derived; ignore any stored value and recompute from the inputs. */
    recomputeRatedTorque();
}

void MotorProfileEditor::recomputeRatedTorque()
{
    /* Explicit Kt (0x2070:9) wins; 0 -> derive 1.5*pole*flux (PMSM). Mirrors
     * the drive's motor_calc_Kt. rated torque = Kt * rated current. */
    double kt = m_torqueConst->value();
    if (kt <= 0.0){
        kt = 1.5 * double(m_polePair->value()) * m_flux->value();
    }
    m_ratedTrq->setValue(kt * double(m_ratedCur->value()));
}

MotorParams MotorProfileEditor::params() const
{
    /* Start from the cached blob so fields without a widget (encoder
     * resolution) pass through unchanged. */
    MotorParams mp    = m_cached;
    mp.type           = static_cast<MotorType>(m_type->currentData().toInt());
    mp.pole_pair      = static_cast<uint32_t>(m_polePair->value());
    mp.rs             = static_cast<float>(m_rs->value());
    mp.ls_d           = static_cast<float>(m_lsd->value());
    mp.ls_q           = static_cast<float>(m_lsq->value());
    mp.rated_flux     = static_cast<float>(m_flux->value());
    mp.inertia        = static_cast<float>(m_inertia->value());
    /* Derived (= Kt * rated current); store the shown value for the JSON
     * cache, but it is never written to the drive (0x6076 is read-only). */
    mp.rated_torque   = static_cast<float>(m_ratedTrq->value());
    mp.torque_constant = static_cast<float>(m_torqueConst->value());
    mp.rated_speed    = m_ratedSpd->value();
    mp.rated_vol      = m_ratedVol->value();
    mp.rated_cur      = static_cast<float>(m_ratedCur->value());
    mp.cpr            = static_cast<uint32_t>(m_cpr->value());
    return mp;
}

}  // namespace vrmc
