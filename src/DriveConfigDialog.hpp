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
class QDoubleSpinBox;
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

protected:
    /** Refit the layout on every show so the dialog grows to whatever
     *  the (potentially edited) tab contents now want. */
    void showEvent(QShowEvent* e) override;

private slots:
    void onReadClicked ();
    void onApplyClicked();
    void onStartHomingClicked();
    void onZeroEncoderClicked();
    void onZeroTorqueClicked ();

signals:
    /** Operator pressed Read on the Custom SDO tab. byteLen is from the
     *  type combo (1/2/4 etc). Routed by MainWindow to MasterWorker. */
    void customSdoReadRequested (int idx, uint16_t odIdx, uint8_t sub, int byteLen);
    /** Operator pressed Write on the Custom SDO tab. bytes is the LE-
     *  encoded payload, sized to the type combo's width. */
    void customSdoWriteRequested(int idx, uint16_t odIdx, uint8_t sub, QByteArray bytes);

public slots:
    /** MainWindow forwards MasterWorker::customSdoDone here so the
     *  result label paints success / failure. */
    void onCustomSdoDone(int idx, bool isWrite, uint16_t odIdx, uint8_t sub,
                         bool ok, const QString& valueDecoded,
                         const QString& message);

private:
    void buildHomingTab       (QWidget* host);
    void buildMotionTab       (QWidget* host);
    void buildProtectTab      (QWidget* host);
    void buildEncoderTab      (QWidget* host);
    void buildManufacturerTab (QWidget* host);
    void buildCustomTab       (QWidget* host);

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
    QSpinBox*  m_ratedTorque    = nullptr;   /**< 0x6076 (mNm)            */
    QLabel*    m_currentActual  = nullptr;   /**< 0x6078 read-only (‰)    */

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

    /* Custom-SDO tab widgets. */
    QSpinBox*    m_customIdx      = nullptr;   /**< 0x0000..0xFFFF        */
    QSpinBox*    m_customSub      = nullptr;   /**< 0x00..0xFF            */
    QComboBox*   m_customType     = nullptr;   /**< u8 / u16 / u32 / i...  */
    QSpinBox*    m_customValue    = nullptr;   /**< write payload          */
    QPushButton* m_customReadBtn  = nullptr;
    QPushButton* m_customWriteBtn = nullptr;
    QLabel*      m_customResult   = nullptr;   /**< last R/W outcome line  */

    /* Manufacturer-range parameters (0x20xx). Parallel layout to the
     * DriveConfig struct's matching fields; setConfig/config round-
     * trips through them. Some drives may not expose all of these. */
    QSpinBox*       m_mfNodeId       = nullptr;   /**< 0x2000:00 (u8) */
    QDoubleSpinBox* m_mfCurKp        = nullptr;   /**< 0x2010:00      */
    QDoubleSpinBox* m_mfCurKi        = nullptr;   /**< 0x2011:00      */
    QDoubleSpinBox* m_mfVelKp        = nullptr;   /**< 0x2020:00      */
    QDoubleSpinBox* m_mfVelKi        = nullptr;   /**< 0x2021:00      */
    QDoubleSpinBox* m_mfPosKp        = nullptr;   /**< 0x2030:00      */
    QDoubleSpinBox* m_mfPosKi        = nullptr;   /**< 0x2031:00      */
    QSpinBox*       m_mfCurOffA      = nullptr;   /**< 0x2040:01 (i16)*/
    QSpinBox*       m_mfCurOffB      = nullptr;   /**< 0x2040:02      */
    QSpinBox*       m_mfCurOffC      = nullptr;   /**< 0x2040:03      */
    QSpinBox*       m_mfCurGainA     = nullptr;   /**< 0x2041:01 (u16)*/
    QSpinBox*       m_mfCurGainB     = nullptr;   /**< 0x2041:02      */
    QSpinBox*       m_mfCurGainC     = nullptr;   /**< 0x2041:03      */
    /* Fault thresholds — 0x2050:01..0A. */
    QSpinBox*       m_mfFltOverCur     = nullptr; /**< :01 mA           */
    QSpinBox*       m_mfFltOverLoad    = nullptr; /**< :02 % rated      */
    QSpinBox*       m_mfFltOverLoadMs  = nullptr; /**< :03 ms           */
    QSpinBox*       m_mfFltLossPhase   = nullptr; /**< :04 mA           */
    QSpinBox*       m_mfFltLossPhaseMs = nullptr; /**< :05 ms           */
    QSpinBox*       m_mfFltUnbalance   = nullptr; /**< :06 mA           */
    QSpinBox*       m_mfFltStallMs     = nullptr; /**< :07 ms           */
    QSpinBox*       m_mfFltOverVolt    = nullptr; /**< :08 mV           */
    QSpinBox*       m_mfFltUnderVolt   = nullptr; /**< :09 mV           */
    QSpinBox*       m_mfFltOverTemp    = nullptr; /**< :0A °C × 10      */
};

}  // namespace vrmc
