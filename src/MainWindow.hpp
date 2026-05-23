/**
 * @file   MainWindow.hpp
 * @brief  Shell window wiring the MasterWorker thread, slave table, and
 *         per-slave widgets together.
 */

#pragma once

#include "MasterWorker.hpp"
#include "MotorProfile.hpp"

#include <QMainWindow>
#include <QThread>

class QAction;
class QDockWidget;
class QFile;
class QLabel;
class QProcess;
class QSplitter;
class QTableView;
class QTextStream;

QT_FORWARD_DECLARE_CLASS(QTabWidget)

namespace vrmc {

class ConnectionDialog;
class DriveConfigDialog;
class GainEditor;
class JointControlPanel;
class LogDock;
class MotorProfileView;
class MotorView;
class PdoMappingView;
class SignalGeneratorPanel;
class SlavePickerBar;
class SlaveTableModel;
class TelemetryWidget;
class TuningPane;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /** Skip the dialog and emit @c connectRequested directly. */
    void connectWithConfig(const vrmc::CanConfig& cfg)
    {
        m_lastConfig = cfg;
        emit connectRequested(cfg);
    }

    /** Screenshot helpers for the docs pipeline — reveal detachable
     *  panes, switch tabs, open dialogs. All no-ops if the target isn't
     *  available (e.g. no slave selected yet for drive-config). */
    void showDocPanel(const QString& name);
    void setLeftTab  (int idx);

signals:
    void connectRequested (const vrmc::CanConfig& cfg);
    void disconnectRequested();

private slots:
    void onConnect();
    void onDisconnect();
    void onConnected(int count);
    void onDisconnected();
    void onError(const QString& msg);
    void onInfo (const QString& msg);
    void onSnapshots(QVector<vrmc::SlaveSnapshot> snaps);
    void onSelectionChanged();

    /* File menu. */
    void onOpenProfile();
    void onSaveProfile();
    void onSaveProfileAs();
    void onEditMotorProfile();
    /** Result of MasterWorker::readMotorProfile — refresh the cached profile
     *  + view from the slave, then open the editor if a read-to-edit is pending. */
    void onMotorProfileRead(int idx, vrmc::MotorParams mp, bool ok, QString message);

    /* Drive menu. */
    void onSaveConfigToFlash();
    void onLoadConfigFromFlash();
    void onFactoryReset();
    void onUploadFirmware();
    void onConfigureDrive();
    void onDeviceInfo();
    void onDeviceInfoResult(int idx, vrmc::DeviceInfo info, bool ok,
                            QString message);

    /* Data menu. */
    void onToggleRecording(bool on);
    void onExportTelemetry();
    void onClearCharts();

    /** Append the current snapshot batch to the active recording file
     *  (no-op when not recording). Called from onSnapshots. */
    void recordSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);

    /* Help menu. */
    void onAbout();
    void onDocumentation();
    /** Prompt for slave count, spawn N cia402_drive_sim processes
     *  (one per node ID starting at the picker's first_id default),
     *  then auto-connect. The spawned processes are tracked so
     *  Stop demo (and app shutdown) can terminate them cleanly. */
    void onStartDemo();
    /** Kill every demo process and disconnect. No-op if none running. */
    void onStopDemo();
    /** Locate the cia402_drive_sim binary by walking up from the
     *  diagnostic's own dir; returns an empty string if not found. */
    QString findSimBinary() const;

private:
    void buildUi();
    void buildMenus();
    void buildToolbar();
    void wireWorker();

    /** Push the active motor profile's wire scaling (counts/rev from
     *  0x608F ratio + rated torque) to the worker so control-panel
     *  targets and telemetry convert per-motor. */
    void pushScalingToWorker();

    /** Open the modal motor-profile editor on the cached params + write the
     *  result to the selected slave. Split from onEditMotorProfile so the
     *  latter can read-from-slave first (see onMotorProfileRead). */
    void openMotorProfileEditor();

    void notImplemented(const QString& feature);

    QThread           m_workerThread;
    MasterWorker*     m_worker    = nullptr;

    /* Telemetry recording state. m_recordFile is non-null while a
     * recording is in progress; closed on toggle-off / disconnect. */
    QFile*            m_recordFile   = nullptr;
    QTextStream*      m_recordStream = nullptr;
    qint64            m_recordRowCount = 0;
    bool              m_recordHeaderWritten = false;

    /* Demo state — every cia402_drive_sim subprocess we spawn from
     * Help → Start demo lands here so onStopDemo / the destructor
     * can terminate them cleanly. Empty means no demo running. */
    QVector<QProcess*> m_demoProcs;
    QAction*          m_demoStartAct = nullptr;
    QAction*          m_demoStopAct  = nullptr;

    SlaveTableModel*      m_model       = nullptr;
    QTableView*           m_table       = nullptr;
    QWidget*              m_tableHost   = nullptr;   /**< collapsible wrap around m_table */
    SlavePickerBar*       m_picker      = nullptr;
    MotorView*            m_motorView   = nullptr;
    QTabWidget*           m_leftTabs    = nullptr;
    MotorProfileView*     m_profileView = nullptr;
    PdoMappingView*       m_pdoMap      = nullptr;
    JointControlPanel*    m_control     = nullptr;
    GainEditor*           m_gains       = nullptr;
    SignalGeneratorPanel* m_gen         = nullptr;
    TuningPane*           m_tuning      = nullptr;
    TelemetryWidget*      m_telemetry   = nullptr;
    LogDock*              m_log         = nullptr;

    /* Detachable panes promoted off the tab bar. Each is hidden by
     * default and toggled from the toolbar / View menu. Motor
     * profile is no longer a dock — it lives inline in the top
     * split's right pane, and its toolbar / View-menu toggle shows
     * or hides that pane (m_profileFrame). */
    QDockWidget*          m_tuningDock   = nullptr;
    QDockWidget*          m_pdoMapDock   = nullptr;
    QWidget*              m_profileFrame = nullptr;
    QAction*              m_profileAct   = nullptr;

    /* File menu actions. */
    QAction*          m_openProfileAct    = nullptr;
    QAction*          m_saveProfileAct    = nullptr;
    QAction*          m_saveProfileAsAct  = nullptr;
    QAction*          m_editProfileAct    = nullptr;
    QAction*          m_exitAct           = nullptr;

    /* Connection. */
    QAction*          m_connectAct        = nullptr;
    QAction*          m_disconnectAct     = nullptr;
    QAction*          m_reconnectAct      = nullptr;

    /* Drive. */
    QAction*          m_saveToFlashAct    = nullptr;
    QAction*          m_loadFromFlashAct  = nullptr;
    QAction*          m_factoryResetAct   = nullptr;
    QAction*          m_uploadFirmwareAct = nullptr;
    QAction*          m_configureDriveAct = nullptr;
    QAction*          m_deviceInfoAct     = nullptr;

    /* Modeless drive-config dialog, created on demand so it can sit
     * next to the main window while the operator tweaks values. */
    DriveConfigDialog* m_driveCfgDlg = nullptr;

    /* Data. */
    QAction*          m_recordAct         = nullptr;
    QAction*          m_exportTelemetry   = nullptr;
    QAction*          m_clearChartsAct    = nullptr;

    /* Help. */
    QAction*          m_aboutAct          = nullptr;
    QAction*          m_docsAct           = nullptr;

    /* E-stop. */
    QAction*          m_estopAct          = nullptr;

    /* Misc. */
    QLabel*                          m_statusLabel       = nullptr;
    QString                          m_currentProfilePath;
    vrmc::CanConfig                  m_lastConfig;
    vrmc::MotorParams                m_motorParams;       /**< loaded / edited */
    bool                             m_profileEditPending = false; /**< read-then-open */
    QVector<vrmc::SlaveSnapshot>     m_latestSnaps;       /**< most recent refresh */
};

}  // namespace vrmc
