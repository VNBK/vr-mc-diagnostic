/**
 * @file   JointControlPanel.hpp
 * @brief  Per-slave control panel: mode + setpoint + enable/disable.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace vrmc {

class JointControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit JointControlPanel(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

signals:
    /* Routed by MainWindow to MasterWorker via QueuedConnection. */
    void bringupRequested  (int idx, uint32_t timeoutMs);
    void enableRequested   (int idx);
    void disableRequested  (int idx);
    void faultResetRequested(int idx);
    void modeRequested     (int idx, vrmc::Mode mode);
    void targetRequested   (int idx, vrmc::TargetKind which, float value);

private slots:
    void emitTarget();

private:
    int                m_idx = -1;
    QLabel*            m_label        = nullptr;
    QPushButton*       m_bringup      = nullptr;
    QPushButton*       m_enable       = nullptr;
    QPushButton*       m_disable      = nullptr;
    QPushButton*       m_faultReset   = nullptr;
    QComboBox*         m_modeCombo    = nullptr;
    QComboBox*         m_targetCombo  = nullptr;
    QDoubleSpinBox*    m_valueSpin    = nullptr;
    QPushButton*       m_sendBtn      = nullptr;
};

}  // namespace vrmc
