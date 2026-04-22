/**
 * @file   DriveConfigDialog.hpp
 * @brief  Tabbed editor for runtime-configurable drive parameters.
 *
 * Homing, motion profile, protection limits, and encoder scaling — the
 * CiA 402 OD entries that typically get tuned once per commissioning and
 * then stuck in flash. The dialog fills from @ref DriveConfig, edits in
 * place, and hands the result back to MainWindow which forwards it to
 * the worker for batch SDO I/O.
 */

#pragma once

#include "MasterWorker.hpp"

#include <QDialog>

class QComboBox;
class QDialogButtonBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace vrmc {

class DriveConfigDialog : public QDialog
{
    Q_OBJECT
public:
    explicit DriveConfigDialog(QWidget* parent = nullptr);

    void setSlaveContext(int idx, const QString& name);
    void setConfig(const vrmc::DriveConfig& cfg);
    vrmc::DriveConfig config() const;

signals:
    /** User pressed Read. MainWindow should call worker::readDriveConfig. */
    void readRequested  (int idx);
    /** User pressed Apply / OK. Forward to worker::writeDriveConfig. */
    void applyRequested (int idx, vrmc::DriveConfig cfg);
    /** Commissioning one-shots wired to the worker of the same names. */
    void startHomingRequested (int idx);
    void zeroEncoderRequested (int idx);
    void zeroTorqueRequested  (int idx);

public slots:
    /** Called by MainWindow after the worker responds. */
    void onReadResult  (int idx, const vrmc::DriveConfig& cfg,
                        bool ok, const QString& msg);
    void onWriteResult (int idx, bool ok, const QString& msg);
    void onCalibrationDone(int idx, const QString& what, qint64 raw,
                           bool ok, const QString& msg);
    /** Feed periodic snapshots in so the homing banner reflects the
     *  drive's live statusword bits (12 = attained, 13 = error). */
    void onSnapshots   (const QVector<vrmc::SlaveSnapshot>& snaps);

private slots:
    void onReadClicked ();
    void onApplyClicked();
    void onStartHomingClicked();
    void onZeroEncoderClicked();
    void onZeroTorqueClicked ();

private:
    void buildHomingTab  (QWidget* host);
    void buildMotionTab  (QWidget* host);
    void buildProtectTab (QWidget* host);
    void buildEncoderTab (QWidget* host);

    int     m_slaveIdx = -1;
    QString m_slaveName;
    QLabel* m_header = nullptr;

    /* Homing tab. */
    QComboBox* m_homingMethod   = nullptr;
    QSpinBox*  m_homeOffset     = nullptr;
    QSpinBox*  m_homingFast     = nullptr;
    QSpinBox*  m_homingSlow     = nullptr;
    QSpinBox*  m_homingAccel    = nullptr;

    /* Motion profile. */
    QSpinBox*  m_profileVel     = nullptr;
    QSpinBox*  m_profileAccel   = nullptr;
    QSpinBox*  m_profileDecel   = nullptr;
    QSpinBox*  m_quickstopDecel = nullptr;

    /* Protection. */
    QSpinBox*  m_followingError = nullptr;
    QSpinBox*  m_followingMs    = nullptr;
    QSpinBox*  m_posMin         = nullptr;
    QSpinBox*  m_posMax         = nullptr;
    QSpinBox*  m_maxSpeed       = nullptr;
    QSpinBox*  m_maxTorque      = nullptr;

    /* Encoder. */
    QSpinBox*  m_encRes         = nullptr;
    QSpinBox*  m_gearNum        = nullptr;
    QSpinBox*  m_gearDen        = nullptr;
    QSpinBox*  m_feedNum        = nullptr;
    QSpinBox*  m_feedDen        = nullptr;

    QPushButton*      m_readBtn  = nullptr;
    QPushButton*      m_applyBtn = nullptr;
    QDialogButtonBox* m_buttons  = nullptr;

    /* Homing-tab action widgets. */
    QPushButton* m_startHomingBtn = nullptr;
    QPushButton* m_zeroEncBtn     = nullptr;
    QLabel*      m_homingState    = nullptr;

    /* Protection-tab action widgets. */
    QPushButton* m_zeroTorqueBtn  = nullptr;
    QLabel*      m_torqueState    = nullptr;
};

}  // namespace vrmc
