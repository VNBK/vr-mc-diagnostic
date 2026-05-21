/**
 * @file   DriveConfigDialog.hpp
 * @brief  Tabbed editor for runtime-configurable drive parameters.
 *
 * Homing, motion profile, and protection limits — the CiA 402 OD entries
 * that typically get tuned once per commissioning and then stuck in
 * flash. The dialog fills from @ref DriveConfig, edits in place, and
 * hands the result back to MainWindow which forwards it to the worker
 * for batch SDO I/O. (Loop gains live on the Control Panel's Gains tab
 * via the standard 0x60F6/0x60F9/0x60FB objects.)
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
class QTabWidget;

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
    void buildManufacturerTab (QWidget* host);
    void buildCustomTab       (QWidget* host);
    /** Walk every tab page and set its minimum size to the largest
     *  sizeHint across all tabs. Forces QTabWidget::sizeHint to
     *  reflect the widest tab's footprint, so adjustSize() can pick a
     *  dialog size that fits every tab (not just the currently-shown
     *  one). Called from ctor + showEvent. */
    void fitTabsToLargest();

    int          m_slaveIdx = -1;
    QString      m_slaveName;
    QLabel*      m_header = nullptr;
    QTabWidget*  m_tabs   = nullptr;

    /* Homing tab. Values shown in SI; converted to CiA wire units on R/W. */
    QComboBox*      m_homingMethod   = nullptr;
    QDoubleSpinBox* m_homeOffset     = nullptr;   /**< 0x607C  rad           */
    QDoubleSpinBox* m_homingFast     = nullptr;   /**< 0x6099:1 rad/s        */
    QDoubleSpinBox* m_homingSlow     = nullptr;   /**< 0x6099:2 rad/s        */
    QDoubleSpinBox* m_homingAccel    = nullptr;   /**< 0x609A  rad/s²        */

    /* Motion profile. */
    QDoubleSpinBox* m_profileVel     = nullptr;   /**< 0x6081  rad/s         */
    QDoubleSpinBox* m_profileAccel   = nullptr;   /**< 0x6083  rad/s²        */
    QDoubleSpinBox* m_profileDecel   = nullptr;   /**< 0x6084  rad/s²        */
    QDoubleSpinBox* m_quickstopDecel = nullptr;   /**< 0x6085  rad/s²        */

    /* Protection. */
    QDoubleSpinBox* m_followingError = nullptr;   /**< 0x6065  rad           */
    QSpinBox*       m_followingMs    = nullptr;   /**< 0x6066  ms (time)     */
    QDoubleSpinBox* m_posMin         = nullptr;   /**< 0x607D:1 rad          */
    QDoubleSpinBox* m_posMax         = nullptr;   /**< 0x607D:2 rad          */
    QDoubleSpinBox* m_maxSpeed       = nullptr;   /**< 0x6080  rad/s         */
    QDoubleSpinBox* m_maxTorque      = nullptr;   /**< 0x6072  Nm            */
    QDoubleSpinBox* m_ratedTorque    = nullptr;   /**< 0x6076  Nm            */
    QDoubleSpinBox* m_ratedCurrent   = nullptr;   /**< 0x6075  A             */
    QSpinBox*       m_encInc         = nullptr;   /**< 0x608F:1 increments   */
    QSpinBox*       m_encRevs        = nullptr;   /**< 0x608F:2 motor revs   */
    QSpinBox*       m_stallCurrent   = nullptr;   /**< 0x2050:1 mA           */
    QSpinBox*       m_stallTime      = nullptr;   /**< 0x2050:2 ms           */

    /* Counts/rev (0x608F) cached from the last read; drives SI scaling.
     * Falls back to 16384 until the drive reports it. */
    double          m_countsPerRev   = 16384.0;

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
    QSpinBox*       m_mfNodeId       = nullptr;   /**< 0x2000:01 (u8)      */
    QDoubleSpinBox* m_mfCurOffA      = nullptr;   /**< 0x2040:01 (RW)      */
    QDoubleSpinBox* m_mfCurOffB      = nullptr;   /**< 0x2040:02 (RW)      */
    QDoubleSpinBox* m_mfCurOffC      = nullptr;   /**< 0x2040:03 (RW)      */
    QDoubleSpinBox* m_mfCurGainA     = nullptr;   /**< 0x2041:01 (RW)      */
    QDoubleSpinBox* m_mfCurGainB     = nullptr;   /**< 0x2041:02 (RW)      */
    QDoubleSpinBox* m_mfCurGainC     = nullptr;   /**< 0x2041:03 (RW)      */
    QDoubleSpinBox* m_mfHallOffset   = nullptr;   /**< 0x2060 rad          */
};

}  // namespace vrmc
