/**
 * @file   SignalGeneratorPanel.hpp
 * @brief  UI for the MasterWorker signal generator.
 *
 * Builds a @ref vrmc::GenCfg from spinboxes + waveform picker and
 * emits Arm / Stop. Auto-disables when the worker reports the
 * generator has stopped.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace vrmc {

class SignalGeneratorPanel : public QWidget
{
    Q_OBJECT
public:
    explicit SignalGeneratorPanel(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

signals:
    void startRequested(int idx, vrmc::GenCfg cfg);
    void stopRequested (int idx);

public slots:
    /** Reset UI state when the worker reports the generator is idle. */
    void onGeneratorStarted(int idx);
    void onGeneratorStopped(int idx);

private:
    GenCfg build() const;
    void   applyShape();    /**< enable/disable rows per-shape          */

    int              m_idx        = -1;
    bool             m_running    = false;

    QLabel*          m_label      = nullptr;
    QComboBox*       m_shape      = nullptr;
    QComboBox*       m_target     = nullptr;
    QDoubleSpinBox*  m_amp        = nullptr;
    QDoubleSpinBox*  m_offset     = nullptr;
    QDoubleSpinBox*  m_freq       = nullptr;
    QDoubleSpinBox*  m_freqEnd    = nullptr;
    QDoubleSpinBox*  m_duration   = nullptr;
    QPushButton*     m_armBtn     = nullptr;
    QPushButton*     m_stopBtn    = nullptr;
};

}  // namespace vrmc
