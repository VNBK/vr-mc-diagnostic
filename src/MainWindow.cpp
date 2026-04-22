#include "MainWindow.hpp"

#include "ConnectionDialog.hpp"
#include "DriveConfigDialog.hpp"

#include <QComboBox>
#include "GainEditor.hpp"
#include "FirmwareUpgradeDialog.hpp"
#include "JointControlPanel.hpp"
#include "LogDock.hpp"
#include "MotorProfile.hpp"
#include "MotorProfileEditor.hpp"
#include "MotorProfileView.hpp"
#include "PdoMappingView.hpp"
#include "SignalGeneratorPanel.hpp"
#include "SlaveTableModel.hpp"
#include "TelemetryWidget.hpp"
#include "TuningPane.hpp"

#include <QAction>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDockWidget>
#include <QFileDialog>
#include <QHeaderView>
#include <QIcon>
#include <QKeySequence>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QShortcut>
#include <QSizePolicy>
#include <QSplitter>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QTableView>
#include <QTimer>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>

namespace vrmc {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle(tr("VR Motor Control Diagnostic Tool"));
    setWindowIcon(QIcon(QStringLiteral(":/brand/vinrobotic.png")));
    resize(1200, 800);

    /* Worker lives on its own thread. Construct it there so QTimer
     * fires on the worker's event loop, not the UI's. */
    m_worker = new MasterWorker;
    m_worker->moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_workerThread.start();

    buildUi();
    wireWorker();
}

MainWindow::~MainWindow()
{
    m_workerThread.quit();
    m_workerThread.wait();
}

void MainWindow::buildUi()
{
    /* --- Central: slave table above, per-slave control/telemetry below. */
    m_model = new SlaveTableModel(this);
    m_table = new QTableView;
    m_table->setModel(m_model);
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);

    m_profileView = new MotorProfileView;
    m_pdoMap      = new PdoMappingView;
    m_control     = new JointControlPanel;
    m_gains       = new GainEditor;
    m_gen         = new SignalGeneratorPanel;
    m_tuning      = new TuningPane;
    m_telemetry   = new TelemetryWidget;

    /* Core left-side tabs: the panels that belong next to the slave
     * table for hands-on tuning. Profile / Tuning / PDO map live on
     * their own dock widgets so the operator can float or dock them
     * beside the table. */
    m_leftTabs = new QTabWidget;
    m_leftTabs->addTab(m_control, tr("Control"));
    m_leftTabs->addTab(m_gains,   tr("Gains"));
    m_leftTabs->addTab(m_gen,     tr("Signal gen"));

    /* Edit button on the profile view → reuse the editor menu handler. */
    connect(m_profileView, &MotorProfileView::editRequested,
            this,          &MainWindow::onEditMotorProfile);
    m_profileView->setParams(m_motorParams);

    /* Detachable panes. Created hidden + floating so the main window
     * comes up uncluttered; toolbar actions bring them out. */
    auto makeDock = [this](const QString& title, const QString& obj,
                           QWidget* w, const QSize& sz) {
        auto* dock = new QDockWidget(title, this);
        dock->setObjectName(obj);
        dock->setAllowedAreas(Qt::AllDockWidgetAreas);
        dock->setWidget(w);
        addDockWidget(Qt::RightDockWidgetArea, dock);
        dock->setFloating(true);
        dock->resize(sz);
        dock->hide();
        return dock;
    };
    m_profileDock = makeDock(tr("Motor profile"), QStringLiteral("ProfileDock"),
                             m_profileView, QSize(420, 520));
    m_tuningDock  = makeDock(tr("Tuning"),        QStringLiteral("TuningDock"),
                             m_tuning,      QSize(720, 520));
    m_pdoMapDock  = makeDock(tr("PDO map"),       QStringLiteral("PdoMapDock"),
                             m_pdoMap,      QSize(520, 520));

    auto* bottomSplit = new QSplitter(Qt::Horizontal);
    bottomSplit->addWidget(m_leftTabs);
    bottomSplit->addWidget(m_telemetry);
    bottomSplit->setStretchFactor(0, 0);
    bottomSplit->setStretchFactor(1, 1);

    auto* splitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(m_table);
    splitter->addWidget(bottomSplit);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 2);

    setCentralWidget(splitter);

    /* --- Log dock. */
    m_log = new LogDock(this);
    addDockWidget(Qt::BottomDockWidgetArea, m_log);

    buildMenus();
    buildToolbar();

    m_statusLabel = new QLabel(tr("disconnected"));
    statusBar()->addPermanentWidget(m_statusLabel);

    /* Selection → per-slave panels. */
    connect(m_table->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::onSelectionChanged);
}

/* ==================================================================== *
 *  Menu + toolbar
 * ==================================================================== */

static QIcon makeRecordIcon()
{
    QPixmap pm(32, 32);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor("#c03030"));
    p.setPen(QPen(QColor("#501010"), 2));
    p.drawEllipse(4, 4, 24, 24);
    return QIcon(pm);
}

static QIcon makeFirmwareIcon()
{
    QPixmap pm(32, 32);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    p.setBrush(QColor("#3b7ea1"));
    p.setPen(QPen(QColor("#1a4b66"), 2));
    p.drawRoundedRect(6, 8, 20, 16, 2, 2);   /* chip body */
    p.setPen(QPen(QColor("#1a4b66"), 1));
    for (int i = 0; i < 4; ++i){
        const int y = 10 + i * 4;
        p.drawLine(2, y, 6, y);               /* left pins */
        p.drawLine(26, y, 30, y);             /* right pins */
    }
    return QIcon(pm);
}

/* Stylised window-pane icon, recoloured per toolbar entry so the user
 * can tell Profile / Tuning / PDO map apart at a glance. */
static QIcon makeWindowIcon(const QColor& tint)
{
    QPixmap pm(32, 32);
    pm.fill(Qt::transparent);
    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    QColor fill = tint; fill.setAlpha(200);
    QColor edge = tint.darker(160);
    p.setBrush(fill);
    p.setPen(QPen(edge, 2));
    p.drawRoundedRect(3, 5, 26, 22, 2, 2);
    p.setBrush(edge);
    p.drawRect(3, 5, 26, 5);                  /* title bar */
    p.setPen(QPen(edge.lighter(140), 1));
    for (int i = 0; i < 3; ++i){
        const int y = 13 + i * 4;
        p.drawLine(6, y, 26, y);              /* content lines */
    }
    return QIcon(pm);
}

void MainWindow::buildMenus()
{
    auto* style = this->style();

    /* --- File --- */
    auto* fileMenu = menuBar()->addMenu(tr("&File"));

    m_openProfileAct = new QAction(
        style->standardIcon(QStyle::SP_DialogOpenButton), tr("&Open Profile…"), this);
    m_openProfileAct->setShortcut(QKeySequence::Open);
    m_openProfileAct->setStatusTip(tr("Load a motor-profile JSON file"));
    connect(m_openProfileAct, &QAction::triggered, this, &MainWindow::onOpenProfile);

    m_saveProfileAct = new QAction(
        style->standardIcon(QStyle::SP_DialogSaveButton), tr("&Save Profile"), this);
    m_saveProfileAct->setShortcut(QKeySequence::Save);
    connect(m_saveProfileAct, &QAction::triggered, this, &MainWindow::onSaveProfile);

    m_saveProfileAsAct = new QAction(tr("Save Profile &As…"), this);
    m_saveProfileAsAct->setShortcut(QKeySequence::SaveAs);
    connect(m_saveProfileAsAct, &QAction::triggered, this, &MainWindow::onSaveProfileAs);

    m_editProfileAct = new QAction(tr("&Edit Motor Profile…"), this);
    m_editProfileAct->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
    connect(m_editProfileAct, &QAction::triggered, this, &MainWindow::onEditMotorProfile);

    m_exitAct = new QAction(
        style->standardIcon(QStyle::SP_DialogCloseButton), tr("E&xit"), this);
    m_exitAct->setShortcut(QKeySequence::Quit);
    connect(m_exitAct, &QAction::triggered, this, &QMainWindow::close);

    fileMenu->addAction(m_openProfileAct);
    fileMenu->addAction(m_saveProfileAct);
    fileMenu->addAction(m_saveProfileAsAct);
    fileMenu->addSeparator();
    fileMenu->addAction(m_editProfileAct);
    fileMenu->addSeparator();
    fileMenu->addAction(m_exitAct);

    /* --- Connection --- */
    auto* connMenu = menuBar()->addMenu(tr("&Connection"));
    m_connectAct    = new QAction(
        style->standardIcon(QStyle::SP_ComputerIcon), tr("&Connect…"), this);
    m_connectAct->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_K));
    connect(m_connectAct, &QAction::triggered, this, &MainWindow::onConnect);

    m_disconnectAct = new QAction(
        style->standardIcon(QStyle::SP_DialogCancelButton), tr("&Disconnect"), this);
    connect(m_disconnectAct, &QAction::triggered, this, &MainWindow::onDisconnect);
    m_disconnectAct->setEnabled(false);

    m_reconnectAct = new QAction(
        style->standardIcon(QStyle::SP_BrowserReload), tr("&Reconnect"), this);
    m_reconnectAct->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    connect(m_reconnectAct, &QAction::triggered, this, [this]{
        const CanConfig cached = m_lastConfig;
        onDisconnect();
        QTimer::singleShot(200, this, [this, cached]{
            emit connectRequested(cached);
        });
    });
    m_reconnectAct->setEnabled(false);

    connMenu->addAction(m_connectAct);
    connMenu->addAction(m_disconnectAct);
    connMenu->addAction(m_reconnectAct);

    /* --- Drive --- */
    auto* driveMenu = menuBar()->addMenu(tr("&Drive"));
    m_saveToFlashAct   = new QAction(tr("Save Config &to Flash"), this);
    m_loadFromFlashAct = new QAction(tr("&Load Config from Flash"), this);
    m_factoryResetAct  = new QAction(tr("&Factory Reset…"), this);
    m_uploadFirmwareAct = new QAction(
        makeFirmwareIcon(), tr("&Upload Firmware…"), this);
    m_configureDriveAct = new QAction(
        style->standardIcon(QStyle::SP_FileDialogDetailedView),
        tr("&Configure Drive…"), this);
    m_configureDriveAct->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C));
    connect(m_saveToFlashAct,    &QAction::triggered, this, &MainWindow::onSaveConfigToFlash);
    connect(m_loadFromFlashAct,  &QAction::triggered, this, &MainWindow::onLoadConfigFromFlash);
    connect(m_factoryResetAct,   &QAction::triggered, this, &MainWindow::onFactoryReset);
    connect(m_uploadFirmwareAct, &QAction::triggered, this, &MainWindow::onUploadFirmware);
    connect(m_configureDriveAct, &QAction::triggered, this, &MainWindow::onConfigureDrive);
    driveMenu->addAction(m_configureDriveAct);
    driveMenu->addSeparator();
    driveMenu->addAction(m_saveToFlashAct);
    driveMenu->addAction(m_loadFromFlashAct);
    driveMenu->addSeparator();
    driveMenu->addAction(m_factoryResetAct);
    driveMenu->addSeparator();
    driveMenu->addAction(m_uploadFirmwareAct);

    /* --- Data --- */
    auto* dataMenu = menuBar()->addMenu(tr("D&ata"));
    m_recordAct = new QAction(makeRecordIcon(), tr("&Record telemetry"), this);
    m_recordAct->setCheckable(true);
    m_recordAct->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_R));
    connect(m_recordAct, &QAction::toggled, this, &MainWindow::onToggleRecording);

    m_exportTelemetry = new QAction(
        style->standardIcon(QStyle::SP_ArrowDown), tr("&Export telemetry CSV…"), this);
    connect(m_exportTelemetry, &QAction::triggered, this, &MainWindow::onExportTelemetry);

    m_clearChartsAct = new QAction(
        style->standardIcon(QStyle::SP_TrashIcon), tr("&Clear charts"), this);
    connect(m_clearChartsAct, &QAction::triggered, this, &MainWindow::onClearCharts);

    dataMenu->addAction(m_recordAct);
    dataMenu->addAction(m_exportTelemetry);
    dataMenu->addSeparator();
    dataMenu->addAction(m_clearChartsAct);

    /* --- View --- */
    auto* viewMenu = menuBar()->addMenu(tr("&View"));
    viewMenu->addAction(m_profileDock->toggleViewAction());
    viewMenu->addAction(m_tuningDock->toggleViewAction());
    viewMenu->addAction(m_pdoMapDock->toggleViewAction());
    viewMenu->addSeparator();
    viewMenu->addAction(m_log->toggleViewAction());

    /* --- Help --- */
    auto* helpMenu = menuBar()->addMenu(tr("&Help"));
    m_docsAct  = new QAction(tr("&Documentation…"),  this);
    m_aboutAct = new QAction(
        style->standardIcon(QStyle::SP_DialogHelpButton), tr("&About"), this);
    m_docsAct->setShortcut(QKeySequence::HelpContents);
    connect(m_docsAct,  &QAction::triggered, this, &MainWindow::onDocumentation);
    connect(m_aboutAct, &QAction::triggered, this, &MainWindow::onAbout);
    helpMenu->addAction(m_docsAct);
    helpMenu->addSeparator();
    helpMenu->addAction(m_aboutAct);
}

void MainWindow::buildToolbar()
{
    auto* toolbar = addToolBar(tr("Main"));
    toolbar->setObjectName(QStringLiteral("MainToolbar"));
    toolbar->setMovable(false);
    toolbar->setIconSize(QSize(28, 28));
    toolbar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    /* File group. */
    toolbar->addAction(m_openProfileAct);
    toolbar->addAction(m_saveProfileAct);
    toolbar->addSeparator();

    /* Connection group. */
    toolbar->addAction(m_connectAct);
    toolbar->addAction(m_disconnectAct);
    toolbar->addAction(m_reconnectAct);
    toolbar->addSeparator();

    /* View group: toggle detachable panes. Qt's toggleViewAction keeps
     * the checkbox state in sync with the dock's visibility. */
    auto* profileAct = m_profileDock->toggleViewAction();
    auto* tuningAct  = m_tuningDock->toggleViewAction();
    auto* pdoMapAct  = m_pdoMapDock->toggleViewAction();
    profileAct->setText(tr("Profile"));
    tuningAct->setText(tr("Tuning"));
    pdoMapAct->setText(tr("PDO map"));
    profileAct->setIcon(makeWindowIcon(QColor("#3b9a6d")));
    tuningAct->setIcon(makeWindowIcon(QColor("#b28c32")));
    pdoMapAct->setIcon(makeWindowIcon(QColor("#3b7ea1")));
    toolbar->addAction(profileAct);
    toolbar->addAction(tuningAct);
    toolbar->addAction(pdoMapAct);
    toolbar->addSeparator();

    /* Drive group. */
    toolbar->addAction(m_configureDriveAct);
    toolbar->addAction(m_uploadFirmwareAct);
    toolbar->addSeparator();

    /* Data group. */
    toolbar->addAction(m_recordAct);
    toolbar->addAction(m_clearChartsAct);
    toolbar->addSeparator();

    /* Help. */
    toolbar->addAction(m_aboutAct);

    /* Push the E-STOP to the far right so it dominates the toolbar. */
    auto* spacer = new QWidget(toolbar);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    toolbar->addWidget(spacer);

    auto* estopBtn = new QPushButton(tr("E-STOP"), toolbar);
    estopBtn->setToolTip(tr("Emergency stop — shortcut: Space"));
    estopBtn->setMinimumHeight(64);
    estopBtn->setMinimumWidth(180);
    estopBtn->setCursor(Qt::PointingHandCursor);
    estopBtn->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #d02020;"
        "  color: white;"
        "  font-weight: 900;"
        "  font-size: 22px;"
        "  letter-spacing: 3px;"
        "  border: 3px solid #600000;"
        "  border-radius: 8px;"
        "  padding: 4px 18px;"
        "}"
        "QPushButton:hover  { background-color: #ee3030; }"
        "QPushButton:pressed{ background-color: #800000; }"
        "QPushButton:disabled{ background-color:#5a2020; color:#ccc;"
        "                       border-color:#3a0000; }"));
    connect(estopBtn, &QPushButton::clicked, this, [this]{
        QMetaObject::invokeMethod(m_worker, "disableAll", Qt::QueuedConnection);
    });
    toolbar->addWidget(estopBtn);

    /* Space shortcut — application-wide so it fires even when a control
     * has focus. */
    auto* estopShortcut = new QShortcut(QKeySequence(Qt::Key_Space), this);
    estopShortcut->setContext(Qt::ApplicationShortcut);
    connect(estopShortcut, &QShortcut::activated, estopBtn, &QPushButton::click);

    m_estopAct = new QAction(tr("E-STOP"), this);
    connect(m_estopAct, &QAction::triggered, estopBtn, &QPushButton::click);
}

void MainWindow::wireWorker()
{
    /* UI-to-worker (queued, always, regardless of sender). */
    connect(this,     &MainWindow::connectRequested,
            m_worker, &MasterWorker::connectCan,    Qt::QueuedConnection);
    connect(this,     &MainWindow::disconnectRequested,
            m_worker, &MasterWorker::disconnect_,   Qt::QueuedConnection);

    /* Panel -> worker. */
    connect(m_control, &JointControlPanel::bringupRequested,
            m_worker,  &MasterWorker::bringupOne,   Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::enableRequested,
            m_worker,  &MasterWorker::enableOne,    Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::disableRequested,
            m_worker,  &MasterWorker::disableOne,   Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::faultResetRequested,
            m_worker,  &MasterWorker::faultReset,   Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::modeRequested,
            m_worker,  &MasterWorker::setMode,      Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::targetRequested,
            m_worker,  &MasterWorker::setTarget,    Qt::QueuedConnection);

    /* GainEditor -> worker + worker -> GainEditor. */
    connect(m_gains,  &GainEditor::readGainRequested,
            m_worker, &MasterWorker::readGain,   Qt::QueuedConnection);
    connect(m_gains,  &GainEditor::writeGainRequested,
            m_worker, &MasterWorker::writeGain,  Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::gainRead,
            m_gains,  &GainEditor::onGainRead);

    /* SignalGenerator -> worker + worker -> SignalGenerator. */
    connect(m_gen,    &SignalGeneratorPanel::startRequested,
            m_worker, &MasterWorker::startGenerator, Qt::QueuedConnection);
    connect(m_gen,    &SignalGeneratorPanel::stopRequested,
            m_worker, &MasterWorker::stopGenerator,  Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::generatorStarted,
            m_gen,    &SignalGeneratorPanel::onGeneratorStarted);
    connect(m_worker, &MasterWorker::generatorStopped,
            m_gen,    &SignalGeneratorPanel::onGeneratorStopped);

    /* Tuning (Step + Bode) -> worker + snapshots fan-out. */
    connect(m_tuning->step(), &StepResponseView::startGenRequested,
            m_worker,         &MasterWorker::startGenerator, Qt::QueuedConnection);
    connect(m_tuning->step(), &StepResponseView::stopGenRequested,
            m_worker,         &MasterWorker::stopGenerator,  Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::generatorStarted,
            m_tuning->step(), &StepResponseView::onGeneratorStarted);
    connect(m_worker, &MasterWorker::generatorStopped,
            m_tuning->step(), &StepResponseView::onGeneratorStopped);

    connect(m_tuning->bode(), &BodeView::startGenRequested,
            m_worker,         &MasterWorker::startGenerator, Qt::QueuedConnection);
    connect(m_tuning->bode(), &BodeView::stopGenRequested,
            m_worker,         &MasterWorker::stopGenerator,  Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::generatorStarted,
            m_tuning->bode(), &BodeView::onGeneratorStarted);
    connect(m_worker, &MasterWorker::generatorStopped,
            m_tuning->bode(), &BodeView::onGeneratorStopped);

    /* PDO mapping edit → SDO push. */
    connect(m_pdoMap, &PdoMappingView::applyRequested,
            m_worker, &MasterWorker::applyPdoMapping);

    /* Worker -> UI. */
    connect(m_worker, &MasterWorker::connected,    this, &MainWindow::onConnected);
    connect(m_worker, &MasterWorker::disconnected, this, &MainWindow::onDisconnected);
    connect(m_worker, &MasterWorker::error,        this, &MainWindow::onError);
    connect(m_worker, &MasterWorker::info,         this, &MainWindow::onInfo);
    connect(m_worker, &MasterWorker::snapshots,    this, &MainWindow::onSnapshots);
}

void MainWindow::onConnect()
{
    ConnectionDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted){ return; }
    m_lastConfig = dlg.config();
    emit connectRequested(m_lastConfig);
}

void MainWindow::onDisconnect()
{
    emit disconnectRequested();
}

void MainWindow::onConnected(int count)
{
    m_connectAct->setEnabled(false);
    m_disconnectAct->setEnabled(true);
    m_reconnectAct->setEnabled(true);
    m_statusLabel->setText(tr("connected: %1 slave(s)").arg(count));
}

void MainWindow::onDisconnected()
{
    m_connectAct->setEnabled(true);
    m_disconnectAct->setEnabled(false);
    m_reconnectAct->setEnabled(false);
    m_statusLabel->setText(tr("disconnected"));
    m_control->setActiveSlave(-1);
    m_gains->setActiveSlave(-1);
    m_gen->setActiveSlave(-1);
    m_tuning->setActiveSlave(-1);
    m_telemetry->clear();
}

void MainWindow::onError(const QString& msg)
{
    m_log->appendError(msg);
}

void MainWindow::onInfo(const QString& msg)
{
    m_log->appendInfo(msg);
}

void MainWindow::onSnapshots(QVector<vrmc::SlaveSnapshot> snaps)
{
    m_latestSnaps = snaps;
    m_model->update(snaps);
    m_telemetry->push(snaps);
    m_tuning->step()->onSnapshots(snaps);
    m_tuning->bode()->onSnapshots(snaps);
    m_pdoMap->onSnapshots(snaps);
    if (m_driveCfgDlg && m_driveCfgDlg->isVisible()){
        m_driveCfgDlg->onSnapshots(snaps);
    }
}

void MainWindow::onSelectionChanged()
{
    const auto rows = m_table->selectionModel()->selectedRows();
    if (rows.isEmpty()){
        m_control->setActiveSlave(-1);
        m_gains->setActiveSlave(-1);
        m_gen->setActiveSlave(-1);
        m_telemetry->setActiveSlave(-1);
        m_pdoMap->setActiveSlave(-1);
        return;
    }
    const int row = rows.first().row();
    const auto idxCell  = m_model->index(row, SlaveTableModel::ColIdx);
    const auto nameCell = m_model->index(row, SlaveTableModel::ColName);
    const int idx = idxCell.data().toInt();
    const QString nm = nameCell.data().toString();
    m_control->setActiveSlave(idx, nm);
    m_gains->setActiveSlave(idx, nm);
    m_gen->setActiveSlave(idx, nm);
    m_tuning->setActiveSlave(idx, nm);
    m_telemetry->setActiveSlave(idx);
    m_pdoMap->setActiveSlave(idx, nm);
}

/* ==================================================================== *
 *  File menu — profile load / save
 * ==================================================================== */

void MainWindow::notImplemented(const QString& feature)
{
    m_log->appendInfo(tr("%1 — not yet implemented").arg(feature));
    QMessageBox::information(this, feature,
        tr("%1 is planned for a future release.\n\n"
           "The action is wired into the UI so the hook is ready; the "
           "handler currently logs and returns.").arg(feature));
}

void MainWindow::onOpenProfile()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open motor profile"), m_currentProfilePath,
        tr("Motor profile (*.json);;All files (*)"));
    if (path.isEmpty()){ return; }

    MotorProfile mp;
    QString err;
    if (!profile::load(path, &mp, &err)){
        m_log->appendError(tr("load profile failed: %1").arg(err));
        QMessageBox::warning(this, tr("Open motor profile"), err);
        return;
    }
    m_currentProfilePath = path;
    m_motorParams = mp.motor;
    m_lastConfig  = mp.connection;
    m_profileView->setParams(m_motorParams);
    m_log->appendInfo(tr("loaded profile %1 (schema v%2): "
                         "type=%3 pole_pair=%4 Rs=%5 Ls_d=%6 Ls_q=%7 "
                         "λ=%8 J=%9")
                          .arg(path).arg(mp.schemaVersion)
                          .arg(profile::motorTypeToString(mp.motor.type))
                          .arg(mp.motor.pole_pair)
                          .arg(mp.motor.rs, 0, 'g', 4)
                          .arg(mp.motor.ls_d, 0, 'g', 4)
                          .arg(mp.motor.ls_q, 0, 'g', 4)
                          .arg(mp.motor.rated_flux, 0, 'g', 4)
                          .arg(mp.motor.inertia,    0, 'g', 4));

    /* If we're already connected, drop the current link so the new
     * config takes effect cleanly. Then auto-connect with the loaded
     * settings. */
    if (m_disconnectAct->isEnabled()){
        emit disconnectRequested();
    }
    QTimer::singleShot(150, this, [this]{
        emit connectRequested(m_lastConfig);
    });
}

void MainWindow::onSaveProfile()
{
    if (m_currentProfilePath.isEmpty()){
        onSaveProfileAs();
        return;
    }
    MotorProfile mp;
    mp.motor      = m_motorParams;
    mp.connection = m_lastConfig;
    QString err;
    if (!profile::save(m_currentProfilePath, mp, &err)){
        m_log->appendError(tr("save profile failed: %1").arg(err));
        QMessageBox::warning(this, tr("Save motor profile"), err);
        return;
    }
    m_log->appendInfo(tr("saved profile %1").arg(m_currentProfilePath));
}

void MainWindow::onSaveProfileAs()
{
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save motor profile as"),
        m_currentProfilePath.isEmpty()
            ? QStringLiteral("vr_mc_profile.json")
            : m_currentProfilePath,
        tr("Motor profile (*.json);;All files (*)"));
    if (path.isEmpty()){ return; }
    m_currentProfilePath = path;
    onSaveProfile();
}

void MainWindow::onEditMotorProfile()
{
    MotorProfileEditor dlg(this);
    dlg.setParams(m_motorParams);
    if (dlg.exec() != QDialog::Accepted){ return; }
    m_motorParams = dlg.params();
    m_profileView->setParams(m_motorParams);
    m_log->appendInfo(tr("motor profile updated: type=%1 pole_pair=%2 "
                         "Rs=%3Ω Ls_d=%4H Ls_q=%5H λ=%6Wb J=%7kg·m²")
                          .arg(profile::motorTypeToString(m_motorParams.type))
                          .arg(m_motorParams.pole_pair)
                          .arg(m_motorParams.rs, 0, 'g', 4)
                          .arg(m_motorParams.ls_d, 0, 'g', 4)
                          .arg(m_motorParams.ls_q, 0, 'g', 4)
                          .arg(m_motorParams.rated_flux, 0, 'g', 4)
                          .arg(m_motorParams.inertia,    0, 'g', 4));
}

/* ==================================================================== *
 *  Drive menu
 * ==================================================================== */

void MainWindow::onSaveConfigToFlash()   { notImplemented(tr("Save Config to Flash")); }
void MainWindow::onLoadConfigFromFlash() { notImplemented(tr("Load Config from Flash")); }

void MainWindow::onFactoryReset()
{
    const auto btn = QMessageBox::warning(
        this, tr("Factory reset"),
        tr("This will restore the drive to factory defaults. Continue?"),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if (btn != QMessageBox::Ok){ return; }
    notImplemented(tr("Factory reset"));
}

void MainWindow::onUploadFirmware()
{
    FirmwareUpgradeDialog dlg(this);
    dlg.setSlaves(m_latestSnaps);
    dlg.exec();
}

void MainWindow::showDocPanel(const QString& name)
{
    /* Docks are normally floating so the main window stays uncluttered.
     * For the docs pipeline we re-dock them so QWidget::grab() on the
     * main window captures everything in one shot (floating docks are
     * top-level and wouldn't appear in the grab). */
    auto dockInline = [this](QDockWidget* d, Qt::DockWidgetArea where){
        d->setFloating(false);
        addDockWidget(where, d);
        d->show();
        d->raise();
    };
    if (name == QLatin1String("pdo-map")){
        dockInline(m_pdoMapDock, Qt::RightDockWidgetArea);
    } else if (name == QLatin1String("tuning")){
        dockInline(m_tuningDock, Qt::RightDockWidgetArea);
    } else if (name == QLatin1String("profile")){
        dockInline(m_profileDock, Qt::RightDockWidgetArea);
    } else if (name == QLatin1String("select-first")){
        if (m_table && m_model && m_model->rowCount() > 0){
            m_table->selectRow(0);
        }
    } else if (name.startsWith(QLatin1String("connect"))){
        /* Non-modal invocation is awkward; open the standard modal
         * ConnectionDialog, then let the screenshot pipeline grab it.
         * Support "connect:feetech" / "connect:dynamixel" etc. so the
         * docs pipeline can capture each transport's field set. */
        auto* dlg = new ConnectionDialog(this);
        dlg->setAttribute(Qt::WA_DeleteOnClose);
        dlg->setModal(false);
        dlg->show();
        const int colon = name.indexOf(':');
        if (colon > 0){
            const QString want = name.mid(colon + 1);
            QComboBox* box = dlg->findChild<QComboBox*>();
            if (box){
                for (int i = 0; i < box->count(); ++i){
                    if (box->itemText(i).toLower().contains(want.toLower())){
                        box->setCurrentIndex(i);
                        break;
                    }
                }
            }
        }
    } else if (name == QLatin1String("drive-config")){
        /* If no slave is selected but the table has rows, select the
         * first so the dialog isn't empty. */
        if (m_table->selectionModel()
            && m_table->selectionModel()->selectedRows().isEmpty()
            && m_model->rowCount() > 0){
            m_table->selectRow(0);
        }
        onConfigureDrive();
    }
}

void MainWindow::setLeftTab(int idx)
{
    if (m_leftTabs && idx >= 0 && idx < m_leftTabs->count()){
        m_leftTabs->setCurrentIndex(idx);
    }
}

void MainWindow::onConfigureDrive()
{
    /* Resolve the slave from the current table selection. If nothing's
     * selected, nudge the user toward picking one — the dialog needs a
     * target for read/write. */
    int idx = -1;
    QString name;
    const auto rows = m_table->selectionModel()
                          ? m_table->selectionModel()->selectedRows()
                          : QModelIndexList{};
    if (!rows.isEmpty()){
        const int row = rows.first().row();
        idx  = m_model->index(row, SlaveTableModel::ColIdx).data().toInt();
        name = m_model->index(row, SlaveTableModel::ColName).data().toString();
    }
    if (idx < 0){
        QMessageBox::information(this, tr("Configure drive"),
            tr("Select a slave in the table first."));
        return;
    }

    if (!m_driveCfgDlg){
        m_driveCfgDlg = new DriveConfigDialog(this);
        connect(m_driveCfgDlg, &DriveConfigDialog::readRequested,
                m_worker,      &MasterWorker::readDriveConfig,
                Qt::QueuedConnection);
        connect(m_driveCfgDlg, &DriveConfigDialog::applyRequested,
                m_worker,      &MasterWorker::writeDriveConfig,
                Qt::QueuedConnection);
        connect(m_driveCfgDlg, &DriveConfigDialog::startHomingRequested,
                m_worker,      &MasterWorker::startHoming,
                Qt::QueuedConnection);
        connect(m_driveCfgDlg, &DriveConfigDialog::zeroEncoderRequested,
                m_worker,      &MasterWorker::zeroEncoderHere,
                Qt::QueuedConnection);
        connect(m_driveCfgDlg, &DriveConfigDialog::zeroTorqueRequested,
                m_worker,      &MasterWorker::zeroTorqueHere,
                Qt::QueuedConnection);
        connect(m_worker,      &MasterWorker::driveConfigRead,
                m_driveCfgDlg, &DriveConfigDialog::onReadResult);
        connect(m_worker,      &MasterWorker::driveConfigWritten,
                m_driveCfgDlg, &DriveConfigDialog::onWriteResult);
        connect(m_worker,      &MasterWorker::calibrationDone,
                m_driveCfgDlg, &DriveConfigDialog::onCalibrationDone);
    }
    m_driveCfgDlg->setSlaveContext(idx, name);
    m_driveCfgDlg->show();
    m_driveCfgDlg->raise();
    m_driveCfgDlg->activateWindow();
    /* Kick off a read so the form reflects what's on the drive. */
    emit m_driveCfgDlg->readRequested(idx);
}

/* ==================================================================== *
 *  Data menu
 * ==================================================================== */

void MainWindow::onToggleRecording(bool on)
{
    if (on){
        m_log->appendInfo(tr("recording ON (stub)"));
    } else {
        m_log->appendInfo(tr("recording OFF"));
    }
    /* TODO: append snapshots to a CSV file while recording. */
}

void MainWindow::onExportTelemetry()
{
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export telemetry CSV"),
        QStringLiteral("telemetry_%1.csv").arg(stamp),
        tr("CSV (*.csv);;All files (*)"));
    if (path.isEmpty()){ return; }
    m_log->appendInfo(tr("export telemetry: %1").arg(path));
    notImplemented(tr("Export telemetry CSV"));
}

void MainWindow::onClearCharts()
{
    m_telemetry->clear();
    m_log->appendInfo(tr("charts cleared"));
}

/* ==================================================================== *
 *  Help menu
 * ==================================================================== */

void MainWindow::onAbout()
{
    QMessageBox::about(this, tr("About VR Motor Control Diagnostic Tool"),
        tr("<h3>VR Motor Control Diagnostic Tool</h3>"
           "<p>Qt6 diagnostic & tuning tool for the vr-mc-sdk "
           "motor-control stack.</p>"
           "<p>Transport: CiA 402 over UDP-multicast CAN "
           "(<code>hal_can_udp</code>). Master: "
           "<code>master_mgr</code> + <code>motor_drive_interface</code>.</p>"
           "<p>Built against Qt %1.</p>")
            .arg(QT_VERSION_STR));
}

void MainWindow::onDocumentation()
{
    const QString readme = QStringLiteral(
        "%1/../README.md").arg(QCoreApplication::applicationDirPath());
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(readme))){
        m_log->appendInfo(tr("documentation: %1 (open manually)").arg(readme));
    }
}

}  // namespace vrmc
