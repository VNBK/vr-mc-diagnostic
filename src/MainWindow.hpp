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
class QLabel;
class QSplitter;
class QTableView;

QT_FORWARD_DECLARE_CLASS(QTabWidget)

namespace vrmc {

class ConnectionDialog;
class DriveConfigDialog;
class GainEditor;
class JointControlPanel;
class LogDock;
class MotorProfileView;
class PdoMappingView;
class SignalGeneratorPanel;
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

    /* Drive menu. */
    void onSaveConfigToFlash();
    void onLoadConfigFromFlash();
    void onFactoryReset();
    void onUploadFirmware();
    void onConfigureDrive();

    /* Data menu. */
    void onToggleRecording(bool on);
    void onExportTelemetry();
    void onClearCharts();

    /* Help menu. */
    void onAbout();
    void onDocumentation();

private:
    void buildUi();
    void buildMenus();
    void buildToolbar();
    void wireWorker();

    void notImplemented(const QString& feature);

    QThread           m_workerThread;
    MasterWorker*     m_worker    = nullptr;

    SlaveTableModel*      m_model       = nullptr;
    QTableView*           m_table       = nullptr;
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
     * default and toggled from the toolbar / View menu. */
    QDockWidget*          m_profileDock = nullptr;
    QDockWidget*          m_tuningDock  = nullptr;
    QDockWidget*          m_pdoMapDock  = nullptr;

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
    QVector<vrmc::SlaveSnapshot>     m_latestSnaps;       /**< most recent refresh */
};

}  // namespace vrmc
