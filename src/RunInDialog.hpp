/**
 * @file   RunInDialog.hpp
 * @brief  Simple motor run-in ("roda") dialog: spin the selected drive
 *         forward for a time, then reverse for a time, repeating until a total
 *         run time elapses (0 = until Stop). Modeless; the actual motion is
 *         driven by MasterWorker::startRunIn / stopRunIn over CiA-402 velocity
 *         mode. No board-side changes.
 */
#pragma once

#include <QDialog>

class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QPushButton;
class QLabel;

namespace vrmc {

class RunInDialog : public QDialog
{
    Q_OBJECT
public:
    explicit RunInDialog(QWidget* parent = nullptr);

signals:
    /** User pressed Start. Speed in rpm; forward/reverse/total in seconds
     *  (total 0 = run until Stop). */
    void startRequested(double speedRpm, int fwdSec, int revSec, int totalSec);
    void stopRequested();

public slots:
    /** Reflect the worker's run state: toggle Start/Stop, lock inputs while
     *  running, and show @p status. */
    void setRunning(bool running, const QString& status);

private slots:
    void onButton();
    /** Toggle the speed spinbox between rpm and rad/s, converting the
     *  current value. The Start signal still carries rpm regardless. */
    void onUnitToggled(bool radS);

private:
    QCheckBox*      m_unitRadS  = nullptr;   /**< checked = rad/s, else rpm */
    QDoubleSpinBox* m_speed    = nullptr;   /**< rpm or rad/s (see m_unitRadS) */
    QSpinBox*       m_fwdSec    = nullptr;
    QSpinBox*       m_revSec    = nullptr;
    QSpinBox*       m_totalSec  = nullptr;   /**< 0 = until Stop */
    QPushButton*    m_button    = nullptr;
    QLabel*         m_status    = nullptr;
    bool            m_running   = false;
};

}  // namespace vrmc
