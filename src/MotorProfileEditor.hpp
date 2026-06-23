/**
 * @file   MotorProfileEditor.hpp
 * @brief  Modal form-layout editor for @ref MotorParams.
 *
 * Bound directly to the JSON load/save path: MainWindow seeds the dialog
 * with the cached MotorParams, the user edits, OK pushes the new values
 * back. From there a Save Profile writes them out, or a future "Push to
 * drive" hook can write them via SDO.
 */

#pragma once

#include "MotorProfile.hpp"
#include <QDialog>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QSpinBox;

namespace vrmc {

class MotorProfileEditor : public QDialog
{
    Q_OBJECT
public:
    explicit MotorProfileEditor(QWidget* parent = nullptr);

    void setParams(const MotorParams& mp);
    MotorParams params() const;

    /** Wire which slave the editor is editing. Enables the Read button
     *  and gets emitted back on readRequested. <0 = no slave selected
     *  (Read disabled; Save still writes per the caller's policy). */
    void setSlaveContext(int slaveIdx);

signals:
    /** User clicked "Read from drive". MainWindow forwards to the worker
     *  and pushes the result back via @ref setParams. */
    void readRequested(int slaveIdx);

private slots:
    void onReadClicked();

private:
    /* Rated torque is DERIVED (= Kt * rated current, Kt = 1.5*pole*flux) and
     * shown read-only; recomputed live as pole/flux/rated-current change. */
    void recomputeRatedTorque();

    /* Topology + pole pairs. */
    QComboBox*       m_type      = nullptr;
    QSpinBox*        m_polePair  = nullptr;

    /* Electrical. */
    QDoubleSpinBox*  m_rs         = nullptr;
    QDoubleSpinBox*  m_lsd        = nullptr;
    QDoubleSpinBox*  m_lsq        = nullptr;
    QDoubleSpinBox*  m_flux       = nullptr;
    QDoubleSpinBox*  m_inertia    = nullptr;

    /* Ratings. */
    QDoubleSpinBox*  m_torqueConst = nullptr;  /**< Kt Nm/A (0x2070:9); 0=derive */
    QDoubleSpinBox*  m_ratedTrq   = nullptr;   /**< Nm  (0x6076)            */
    QSpinBox*        m_ratedSpd   = nullptr;
    QSpinBox*        m_ratedVol   = nullptr;
    QDoubleSpinBox*  m_ratedCur   = nullptr;   /**< A (float; sub-amp ok)    */

    /* Encoder. */
    QSpinBox*        m_cpr        = nullptr;    /**< CPR = 4·lines (0x2070:11) */

    /* Loaded params cached so non-widget fields (encoder resolution, now
     * a Manufacturer/drive object) survive a round-trip through the form. */
    MotorParams      m_cached;

    /* Slave context for Read button. */
    int              m_slaveIdx = -1;
    QPushButton*     m_readBtn  = nullptr;
};

}  // namespace vrmc
