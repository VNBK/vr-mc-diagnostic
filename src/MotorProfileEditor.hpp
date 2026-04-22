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
class QSpinBox;

namespace vrmc {

class MotorProfileEditor : public QDialog
{
    Q_OBJECT
public:
    explicit MotorProfileEditor(QWidget* parent = nullptr);

    void setParams(const MotorParams& mp);
    MotorParams params() const;

private:
    /* Topology + pole pairs. */
    QComboBox*       m_type      = nullptr;
    QSpinBox*        m_polePair  = nullptr;

    /* Electrical. */
    QDoubleSpinBox*  m_rs        = nullptr;
    QDoubleSpinBox*  m_lsd       = nullptr;
    QDoubleSpinBox*  m_lsq       = nullptr;
    QDoubleSpinBox*  m_flux      = nullptr;
    QDoubleSpinBox*  m_inertia   = nullptr;

    /* Speed. */
    QSpinBox*        m_ratedSpd  = nullptr;

    /* Voltage limits. */
    QSpinBox*        m_ratedVol  = nullptr;
    QSpinBox*        m_minVol    = nullptr;
    QSpinBox*        m_maxVol    = nullptr;

    /* Current limits. */
    QSpinBox*        m_ratedCur  = nullptr;
    QSpinBox*        m_maxCur    = nullptr;
    QSpinBox*        m_stallCur  = nullptr;
    QSpinBox*        m_stallTime = nullptr;
};

}  // namespace vrmc
