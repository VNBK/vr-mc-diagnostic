#include "MainWindow.hpp"

#include <algorithm>
#include <memory>

#include "ConnectionDialog.hpp"
#include "DriveConfigDialog.hpp"

#include <QComboBox>
#include "GainEditor.hpp"
#include "FirmwareUpgradeDialog.hpp"
#include "JointControlPanel.hpp"
#include "MotorView.hpp"
#include "SlavePickerBar.hpp"
#include "LogDock.hpp"
#include "MotorProfile.hpp"
#include "MotorProfileEditor.hpp"
#include "MotorProfileView.hpp"
#include "PdoMappingView.hpp"
#include "SignalGeneratorPanel.hpp"
#include "SlaveTableModel.hpp"
#include "TelemetryWidget.hpp"
#include "TuningPane.hpp"
#include "MotorProfile.hpp"

#include <QAction>
#include <QApplication>
#include <QClipboard>
#include <QCoreApplication>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QDockWidget>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHeaderView>
#include <QInputDialog>
#include <QProcess>
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
#include <QStandardPaths>
#include <QStatusBar>
#include <QStyle>
#include <QTabWidget>
#include <QTableView>
#include <QTextStream>
#include <QTimer>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>
#include <QScreen>

namespace vrmc {

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle(tr("VR Motor Control Diagnostic Tool"));
    setWindowIcon(QIcon(QStringLiteral(":/brand/vinrobotic.png")));

    /* Worker lives on its own thread. Construct it there so QTimer
     * fires on the worker's event loop, not the UI's. */
    m_worker = new MasterWorker;
    m_worker->moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_workerThread.start();

    buildUi();
    wireWorker();

    /* Size the window so every child widget fits without clipping.
     * The earlier hard 1200×800 default was too tight once the slave
     * table grew to 11 columns and the per-slave panels expanded.
     * adjustSize() asks the layout what it actually needs; we then
     * floor it at 1500×900 so a sparse first-launch (no slaves yet)
     * still gives the operator room to work, and use the layout's
     * minimum as the window minimum so the user can't shrink below
     * a usable size. */
    adjustSize();
    const QSize hinted = sizeHint();
    const int   w      = std::max(1500, hinted.width());
    const int   h      = std::max(900,  hinted.height());
    resize(w, h);
    setMinimumSize(std::min(1200, hinted.width()),
                   std::min(750,  hinted.height()));
}

MainWindow::~MainWindow()
{
    /* Kill any demo subprocesses we spawned so they don't outlive the
     * GUI as orphan listeners on the multicast group. */
    for (auto* p : m_demoProcs){
        if (p && p->state() != QProcess::NotRunning){
            p->kill();
            p->waitForFinished(200);
        }
    }
    m_workerThread.quit();
    m_workerThread.wait();
}

void MainWindow::buildUi()
{
    /* --- Central: slave table above, per-slave control/telemetry below. */
    m_model = new SlaveTableModel(this);
    m_table = new QTableView;
    m_table->setModel(m_model);
    /* Fit every column to its content so all 11 columns (idx / id /
     * name / state / online / pos / vel / trq / I / T / err) are
     * visible without horizontal scroll. setStretchLastSection used to
     * push the last column to fill remaining width, but that left wide
     * numeric columns (pos / vel) truncated on small screens. The
     * fit-to-contents mode resizes on every model update; a small CPU
     * cost for a much better default layout. The "name" column is
     * still Interactive so the operator can drag it wider for long
     * names. */
    auto* hh = m_table->horizontalHeader();
    hh->setSectionResizeMode(QHeaderView::ResizeToContents);
    hh->setSectionResizeMode(SlaveTableModel::ColName, QHeaderView::Interactive);
    hh->setStretchLastSection(false);
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
    pushProfileToTuning();

    /* Detachable panes. Created hidden + floating so the main window
     * comes up uncluttered; toolbar actions bring them out. The
     * motor profile no longer gets its own dock because it lives
     * inline in the top 3-pane split (see below). */
    auto makeDock = [this](const QString& title, const QString& obj,
                           QWidget* w, const QSize& sz) {
        auto* dock = new QDockWidget(title, this);
        dock->setObjectName(obj);
        dock->setAllowedAreas(Qt::AllDockWidgetAreas);
        dock->setWidget(w);
        addDockWidget(Qt::RightDockWidgetArea, dock);
        dock->setFloating(true);
        dock->resize(sz);

        if (QScreen *screen = QGuiApplication::primaryScreen()) {
            QRect geo = screen->geometry();
            dock->move((geo.width() - sz.width()) / 2, (geo.height() - sz.height()) / 2);
        }

        dock->hide();
        return dock;
    };
    m_tuningDock  = makeDock(tr("Tuning"),        QStringLiteral("TuningDock"),
                             m_tuning,      QSize(720, 520));
    m_pdoMapDock  = makeDock(tr("PDO map"),       QStringLiteral("PdoMapDock"),
                             m_pdoMap,      QSize(520, 520));

    /* Left-column width shared between the top-row slave table and the
     * bottom-row control panel — so their left edges align by default.
     * QSplitter is interactive, so user drag will diverge them later;
     * this just sets the initial geometry. */
    constexpr int kLeftColumnPx = 420;

    /* Wrap each major pane in a QFrame so visual boundaries match the
     * functional grouping. Helper avoids hand-repeating the same
     * StyledPanel/Sunken/1-px line config. */
    auto wrapFrame = [](QWidget* inner) -> QFrame* {
        auto* f = new QFrame;
        f->setFrameShape(QFrame::StyledPanel);
        f->setFrameShadow(QFrame::Sunken);
        f->setLineWidth(1);
        auto* l = new QVBoxLayout(f);
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(0);
        l->addWidget(inner);
        return f;
    };

    /* Bottom row: Control-tabs left, telemetry right. Left column
     * width tracks the top row's slave-table column so the right edge
     * of the table and the right edge of the control panel always
     * align (sync wired below via splitterMoved). */
    auto* bottomSplit = new QSplitter(Qt::Horizontal);
    bottomSplit->addWidget(wrapFrame(m_leftTabs));
    bottomSplit->addWidget(m_telemetry);
    bottomSplit->setStretchFactor(0, 0);
    bottomSplit->setStretchFactor(1, 1);
    bottomSplit->setSizes({kLeftColumnPx, 800});

    /* Top row: 3-pane horizontal split.
     *   left   = slave list
     *   centre = MotorView dial
     *   right  = motor profile view (narrow; togglable from toolbar) */
    m_motorView    = new MotorView;
    m_profileFrame = wrapFrame(m_profileView);
    auto* topSplit = new QSplitter(Qt::Horizontal);
    topSplit->addWidget(wrapFrame(m_table));
    topSplit->addWidget(wrapFrame(m_motorView));
    topSplit->addWidget(m_profileFrame);
    topSplit->setStretchFactor(0, 0);   /* table width follows kLeftColumnPx */
    topSplit->setStretchFactor(1, 1);
    topSplit->setStretchFactor(2, 0);   /* profile stays narrow on resize */
    topSplit->setSizes({kLeftColumnPx, 360, 300});

    /* Sync the first-handle position between the two splitters so the
     * right edge of slave table (topSplit's first pane) and the right
     * edge of control panel (bottomSplit's first pane) align. Two-way:
     * dragging either splitter's first handle moves the other. setSizes
     * doesn't re-fire splitterMoved, so no recursion guard needed. */
    connect(topSplit, &QSplitter::splitterMoved, this,
            [topSplit, bottomSplit](int /*pos*/, int idx){
        if (idx != 1){ return; }   /* only the table|motorView handle */
        const auto tops = topSplit->sizes();
        auto bots = bottomSplit->sizes();
        if (tops.size() < 3 || bots.size() < 2){ return; }
        const int total = bots[0] + bots[1];
        bots[0] = tops[0];
        bots[1] = std::max(0, total - tops[0]);
        bottomSplit->setSizes(bots);
    });
    connect(bottomSplit, &QSplitter::splitterMoved, this,
            [topSplit, bottomSplit](int /*pos*/, int idx){
        if (idx != 1){ return; }
        const auto bots = bottomSplit->sizes();
        auto tops = topSplit->sizes();
        if (tops.size() < 3 || bots.size() < 2){ return; }
        const int delta = bots[0] - tops[0];
        if (delta == 0){ return; }
        tops[0] = bots[0];
        /* Absorb the delta into the middle pane (MotorView). Clamp so
         * it can't go below a sensible minimum. */
        tops[1] = std::max(120, tops[1] - delta);
        topSplit->setSizes(tops);
    });

    /* Defer initial alignment to next event loop iteration. setSizes
     * called pre-show is clamped to whatever min-size hints the widgets
     * report at construction time — by the time the window paints, the
     * real widget hints are usually larger than what we asked for, so
     * the right edges end up misaligned. Re-applying after show with
     * the realised geometry locks both first panes to the max of the
     * two natural widths so they line up. */
    QTimer::singleShot(0, this, [topSplit, bottomSplit]{
        if (topSplit->count() < 3 || bottomSplit->count() < 2){ return; }
        const int wantLeft = std::max(
            topSplit   ->widget(0)->sizeHint().width(),
            bottomSplit->widget(0)->sizeHint().width());
        auto tops = topSplit->sizes();
        auto bots = bottomSplit->sizes();
        const int topTotal = tops[0] + tops[1] + tops[2];
        const int botTotal = bots[0] + bots[1];
        const int profileW = std::max(260, tops[2]);   /* fit profile table */
        tops[0] = wantLeft;
        tops[2] = profileW;
        tops[1] = std::max(120, topTotal - wantLeft - profileW);
        bots[0] = wantLeft;
        bots[1] = std::max(200, botTotal - wantLeft);
        topSplit   ->setSizes(tops);
        bottomSplit->setSizes(bots);
    });

    /* Picker bar above everything — slim row with combo + state strip.
     * Expand-grid button hidden: the table is now a permanent member
     * of the top split, so there's nothing to expand/collapse. */
    m_picker = new SlavePickerBar;
    m_picker->setExpandButtonVisible(false);

    /* m_tableHost was the collapsible host for the (formerly hidden)
     * table; now the table lives directly in the top-row left pane.
     * Keep the pointer null so onDisconnected etc. skip cleanly. */
    m_tableHost = nullptr;

    auto* splitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(topSplit);
    splitter->addWidget(bottomSplit);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

    auto* central = new QWidget;
    auto* centralLay = new QVBoxLayout(central);
    centralLay->setContentsMargins(0, 0, 0, 0);
    centralLay->setSpacing(0);
    centralLay->addWidget(m_picker);
    centralLay->addWidget(splitter, 1);
    setCentralWidget(central);

    /* Picker wiring — fold its combo into the existing table selection
     * model so every per-slave panel reacts uniformly regardless of
     * whether the operator picked via the picker combobox or by
     * clicking a row in the (now always-visible) table. */
    m_picker->bindModel(m_model, m_table->selectionModel());
    connect(m_picker, &SlavePickerBar::slaveSelected, this, [this](int row){
        if (m_table && row >= 0){ m_table->selectRow(row); }
    });
    /* Expand-grid button no longer toggles anything (the table is
     * always visible inline); the picker bar hides it via its
     * setExpandButtonVisible(false) call below. */

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

    /* JSON Open/Save Profile removed: this tool supports multiple motors
     * per session, and the motor name-plate is persisted per-slave on the
     * drive's own flash (motor_drive_params blob). Use the Motor Profile
     * editor to edit + flash the selected slave's profile directly. */

    m_editProfileAct = new QAction(tr("&Edit Motor Profile…"), this);
    m_editProfileAct->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
    connect(m_editProfileAct, &QAction::triggered, this, &MainWindow::onEditMotorProfile);

    m_exitAct = new QAction(
        style->standardIcon(QStyle::SP_DialogCloseButton), tr("E&xit"), this);
    m_exitAct->setShortcut(QKeySequence::Quit);
    connect(m_exitAct, &QAction::triggered, this, &QMainWindow::close);

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

    /* --- Drive ---
     *
     * Save / Load Config to/from Flash removed: the canonical per-blob
     * Save (0x1010:01/04/05) lives inside the Drive Config dialog
     * (Save button + Storage tab) and the Motor Profile editor's Save
     * button. "Load from Flash" never had a CiA-301 backing -- each
     * dialog's "Read from drive" button covers the UI-refresh use case.
     *
     * Factory Reset stays as a destructive shortcut (renamed to be
     * specific about what it wipes), and writes 0x1011:01 = "load". */
    auto* driveMenu = menuBar()->addMenu(tr("&Drive"));
    m_factoryResetAct  = new QAction(tr("&Restore Default Parameters…"), this);
    m_factoryResetAct->setToolTip(
        tr("Erase both parameter blobs on the selected drive (0x1011:01). "
           "Factory defaults take effect after the next power-cycle / NMT "
           "reset; the live motor keeps its current gains."));
    m_uploadFirmwareAct = new QAction(
        makeFirmwareIcon(), tr("&Upload Firmware…"), this);
    m_configureDriveAct = new QAction(
        style->standardIcon(QStyle::SP_FileDialogDetailedView),
        tr("&Configure Drive…"), this);
    m_configureDriveAct->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C));
    m_deviceInfoAct = new QAction(
        style->standardIcon(QStyle::SP_MessageBoxInformation),
        tr("Device &Info"), this);
    connect(m_factoryResetAct,   &QAction::triggered, this, &MainWindow::onFactoryReset);
    connect(m_uploadFirmwareAct, &QAction::triggered, this, &MainWindow::onUploadFirmware);
    connect(m_configureDriveAct, &QAction::triggered, this, &MainWindow::onConfigureDrive);
    connect(m_deviceInfoAct,     &QAction::triggered, this, &MainWindow::onDeviceInfo);
    driveMenu->addAction(m_configureDriveAct);
    driveMenu->addAction(m_deviceInfoAct);
    driveMenu->addSeparator();
    driveMenu->addAction(m_factoryResetAct);
    driveMenu->addSeparator();
    driveMenu->addAction(m_uploadFirmwareAct);

    /* --- Data --- */
    auto* dataMenu = menuBar()->addMenu(tr("D&ata"));
    m_recordAct = new QAction(makeRecordIcon(), tr("&Record"), this);
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
    /* Profile is inline but the operator still wants a quick toggle
     * to reclaim the screen width when they're not editing it. */
    m_profileAct = new QAction(tr("Motor &Profile"), this);
    m_profileAct->setCheckable(true);
    m_profileAct->setChecked(true);
    connect(m_profileAct, &QAction::toggled, this, [this](bool on){
        if (m_profileFrame){ m_profileFrame->setVisible(on); }
    });
    viewMenu->addAction(m_profileAct);
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

    /* Demo — spawn N cia402_drive_sim processes from inside the
     * diagnostic so the operator doesn't have to drop to a terminal
     * to bring up a fake bus. Stop counterpart kills the children. */
    m_demoStartAct = new QAction(tr("Start &demo…"), this);
    m_demoStopAct  = new QAction(tr("Stop demo"),    this);
    m_demoStopAct->setEnabled(false);
    m_fwDemoAct    = new QAction(tr("Firmware-upgrade demo…"), this);
    connect(m_demoStartAct, &QAction::triggered, this, &MainWindow::onStartDemo);
    connect(m_demoStopAct,  &QAction::triggered, this, &MainWindow::onStopDemo);
    connect(m_fwDemoAct,    &QAction::triggered, this, &MainWindow::onStartFwDemo);

    helpMenu->addAction(m_docsAct);
    helpMenu->addSeparator();
    helpMenu->addAction(m_demoStartAct);
    helpMenu->addAction(m_fwDemoAct);
    helpMenu->addAction(m_demoStopAct);
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

    /* Connection group. */
    toolbar->addAction(m_connectAct);
    toolbar->addAction(m_disconnectAct);
    toolbar->addAction(m_reconnectAct);
    toolbar->addSeparator();

    /* View group: toggle detachable panes. Qt's toggleViewAction keeps
     * the checkbox state in sync with the dock's visibility. Motor
     * profile is inline; its action toggles the visibility of the
     * inline frame on the top-right of the central widget. */
    if (m_profileAct){
        m_profileAct->setText(tr("Profile"));
        m_profileAct->setIcon(makeWindowIcon(QColor("#3b9a6d")));
        toolbar->addAction(m_profileAct);
    }
    auto* tuningAct  = m_tuningDock->toggleViewAction();
    auto* pdoMapAct  = m_pdoMapDock->toggleViewAction();
    tuningAct->setText(tr("Tuning"));
    pdoMapAct->setText(tr("PDO map"));
    tuningAct->setIcon(makeWindowIcon(QColor("#b28c32")));
    pdoMapAct->setIcon(makeWindowIcon(QColor("#3b7ea1")));
    toolbar->addAction(tuningAct);
    toolbar->addAction(pdoMapAct);
    toolbar->addSeparator();
    toolbar->addAction(m_deviceInfoAct);

    /* Tint each toggle-view button's background to match its icon when
     * checked, so the operator can see at-a-glance which panes are
     * currently open. Without this, all three buttons look identical
     * regardless of state — Qt's default checked indicator is too
     * subtle on most desktop themes. Object-name lookup is the cheapest
     * way to address individual QToolButtons via stylesheet. */
    if (auto* btn = toolbar->widgetForAction(m_profileAct)){
        btn->setObjectName(QStringLiteral("toggleProfileBtn"));
    }
    if (auto* btn = toolbar->widgetForAction(tuningAct)){
        btn->setObjectName(QStringLiteral("toggleTuningBtn"));
    }
    if (auto* btn = toolbar->widgetForAction(pdoMapAct)){
        btn->setObjectName(QStringLiteral("togglePdoMapBtn"));
    }
    toolbar->setStyleSheet(QStringLiteral(
        "QToolButton#toggleProfileBtn:checked,"
        "QToolButton#toggleTuningBtn:checked,"
        "QToolButton#togglePdoMapBtn:checked { "
        "  background: white; color: black; "
        "  border: 1px solid #888; border-radius: 4px; }"
    ));
    toolbar->addSeparator();

    /* Drive group. Firmware-upgrade lives in the Drive menu only — the
     * toolbar slot was removed to keep the destructive flash dialog
     * from being one accidental click away. */
    toolbar->addAction(m_configureDriveAct);
    toolbar->addSeparator();

    /* Data group. */
    toolbar->addAction(m_recordAct);
    toolbar->addAction(m_clearChartsAct);
    toolbar->addSeparator();

    /* Help → About lives in the Help menu only — the toolbar slot was
     * removed because operators kept clicking it by accident while
     * reaching for the adjacent record button. */

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
    connect(m_control, &JointControlPanel::quickStopRequested,
            m_worker,  &MasterWorker::quickStopOne, Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::faultResetRequested,
            m_worker,  &MasterWorker::faultReset,   Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::brakeRequested,
            m_worker,  &MasterWorker::setBrake,     Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::modeRequested,
            m_worker,  &MasterWorker::setMode,      Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::targetRequested,
            m_worker,  &MasterWorker::setTarget,    Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::walkControlwordRequested,
            m_worker,  &MasterWorker::walkControlwordOne,
            Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::vfStartRequested,
            m_worker,  &MasterWorker::startVfOpenLoop, Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::vfSetpointChanged,
            m_worker,  &MasterWorker::setVfSetpoint,   Qt::QueuedConnection);
    connect(m_control, &JointControlPanel::vfStopRequested,
            m_worker,  &MasterWorker::stopVfOpenLoop,  Qt::QueuedConnection);
    /* Push the master's cached controlword back to the panel so the
     * readout strip can mirror what's being streamed for the selected
     * slave. */
    connect(m_worker, &MasterWorker::controlwordCached,
            m_control, &JointControlPanel::onControlwordCached);

    /* GainEditor -> worker + worker -> GainEditor. */
    connect(m_gains,  &GainEditor::readGainRequested,
            m_worker, &MasterWorker::readGain,   Qt::QueuedConnection);
    connect(m_gains,  &GainEditor::writeGainRequested,
            m_worker, &MasterWorker::writeGain,  Qt::QueuedConnection);
    connect(m_gains,  &GainEditor::tuneGainRequested,
            m_worker, &MasterWorker::tuneGain,   Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::gainRead,
            m_gains,  &GainEditor::onGainRead);
    connect(m_worker, &MasterWorker::gainTuned,
            m_gains,  &GainEditor::onGainTuned);

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

    /* Auto-tune tab: tune + step capture via OD 0x2080. The tab listens
     * to the same MasterWorker::gainTuned signal as the Gains tab, so
     * either trigger updates both views' result fields. The readGain
     * connection is what makes the Result Kp/Ki field repopulate when the
     * dock is re-opened or the loop combo is changed. */
    connect(m_tuning->autoTune(), &AutoTuneView::tuneRequested,
            m_worker,             &MasterWorker::tuneGain,    Qt::QueuedConnection);
    connect(m_tuning->autoTune(), &AutoTuneView::captureStepRequested,
            m_worker,             &MasterWorker::captureStep, Qt::QueuedConnection);
    connect(m_tuning->autoTune(), &AutoTuneView::readGainRequested,
            m_worker,             &MasterWorker::readGain,    Qt::QueuedConnection);
    connect(m_worker, &MasterWorker::gainTuned,
            m_tuning->autoTune(), &AutoTuneView::onGainTuned);
    connect(m_worker, &MasterWorker::stepCaptured,
            m_tuning->autoTune(), &AutoTuneView::onStepCaptured);
    connect(m_worker, &MasterWorker::gainRead,
            m_tuning->autoTune(), &AutoTuneView::onGainRead);

    /* PDO mapping edit → SDO push. */
    connect(m_pdoMap, &PdoMappingView::applyRequested,
            m_worker, &MasterWorker::applyPdoMapping);

    /* Worker -> UI. */
    connect(m_worker, &MasterWorker::connected,    this, &MainWindow::onConnected);
    connect(m_worker, &MasterWorker::disconnected, this, &MainWindow::onDisconnected);
    connect(m_worker, &MasterWorker::error,        this, &MainWindow::onError);
    connect(m_worker, &MasterWorker::info,         this, &MainWindow::onInfo);
    connect(m_worker, &MasterWorker::snapshots,    this, &MainWindow::onSnapshots);
    connect(m_worker, &MasterWorker::deviceInfoRead, this, &MainWindow::onDeviceInfoResult);
    connect(m_worker, &MasterWorker::motorProfileRead, this, &MainWindow::onMotorProfileRead);
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
    /* Re-assert the active profile's scaling on the fresh worker link. */
    pushScalingToWorker();
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
    if (m_motorView){ m_motorView->setActiveSlave(-1); }

    /* Clear the slave table + selection so the next connect starts
     * from a clean state. Without this, the model keeps the previous
     * session's rows; on reconnect with the same slave count
     * SlaveTableModel::update takes the dataChanged fast path (no
     * resetModel), the lingering selection points at the same row,
     * selectionChanged never re-fires, and setActiveSlave(idx) never
     * runs — leaving every per-slave panel greyed out. */
    if (m_table && m_table->selectionModel()){
        m_table->selectionModel()->clearSelection();
    }
    m_model->update({});

    /* Close any in-progress CSV recording so it doesn't sit open with
     * stale state across a reconnect. Pop the toolbar button too. */
    if (m_recordAct && m_recordAct->isChecked()){
        m_recordAct->setChecked(false);   /* triggers onToggleRecording(false) */
    }

    /* If a demo was running, terminate its sim processes so the Help
     * menu actions reflect reality (Start re-enabled, Stop greyed).
     * The operator clicked Disconnect — they don't expect orphan sim
     * processes to keep running silently on the multicast group.
     * onStopDemo is idempotent (no-ops when m_demoProcs is empty) so
     * the more usual "operator clicked Stop demo" path which calls
     * disconnect_ → onDisconnected → here doesn't loop. */
    if (!m_demoProcs.isEmpty()){
        onStopDemo();
    }
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
    m_control->onSnapshots(snaps);
    if (m_motorView){ m_motorView->onSnapshots(snaps); }
    if (m_picker)   { m_picker   ->onSnapshots(snaps); }
    m_tuning->step()->onSnapshots(snaps);
    m_tuning->bode()->onSnapshots(snaps);
    m_tuning->autoTune()->onSnapshots(snaps);
    m_pdoMap->onSnapshots(snaps);
    if (m_driveCfgDlg && m_driveCfgDlg->isVisible()){
        m_driveCfgDlg->onSnapshots(snaps);
    }
    recordSnapshots(snaps);

    /* Default-select the first slave the moment any data lands and no
     * row is yet selected. Without this, the per-slave panels (Control,
     * Gains, Telemetry, PDO map) sit empty after a fresh Connect — the
     * operator always wants the first device immediately. Same logic
     * the --auto-connect path runs at line 763. */
    if (m_table && m_model && m_model->rowCount() > 0
        && m_table->selectionModel()
        && m_table->selectionModel()->selectedRows().isEmpty()){
        m_table->selectRow(0);
    }

    /* Force a one-shot column re-fit on every snapshot. The
     * setSectionResizeMode(ResizeToContents) below normally resizes on
     * model resets, but a snapshot of the same row count goes through
     * the dataChanged fast path which doesn't always re-measure column
     * widths. resizeColumnsToContents() is cheap for ~10 rows and
     * guarantees every column hugs its data. */
    if (m_table){ m_table->resizeColumnsToContents(); }
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
        if (m_motorView){ m_motorView->setActiveSlave(-1); }
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
    if (m_motorView){ m_motorView->setActiveSlave(idx, nm); }

    /* Auto-reload the motor profile from the freshly-selected drive so
     * reconnect (which clears + re-makes the selection) and per-row
     * picks both refresh m_motorParams from the live board instead of
     * showing the previous session's values. The Edit button path
     * (onMotorProfileEditRequested) still uses the same readMotorProfile
     * round-trip — the response handler is shared. */
    if (m_worker && idx >= 0){
        QMetaObject::invokeMethod(m_worker, "readMotorProfile",
                                  Qt::QueuedConnection, Q_ARG(int, idx));
    }
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

int MainWindow::selectedSlaveIdx() const
{
    if (!m_table || !m_table->selectionModel()) return -1;
    const auto rows = m_table->selectionModel()->selectedRows();
    if (rows.isEmpty()) return -1;
    const int row = rows.first().row();
    return m_model->index(row, SlaveTableModel::ColIdx).data().toInt();
}

void MainWindow::sendStorageCommand(uint16_t odIdx, uint8_t sub, uint32_t magic,
                                    const QString& busyMsg,
                                    const QString& okMsg)
{
    const int idx = selectedSlaveIdx();
    if (idx < 0){
        QMessageBox::information(this, busyMsg,
            tr("Select a slave in the table first."));
        return;
    }

    /* Encode the magic value as 4 LE bytes (matches the wire-side
     * uint32 the board parses in on_store_params_written /
     * on_restore_params_written). */
    QByteArray payload(4, '\0');
    payload[0] = static_cast<char>( magic        & 0xFFu);
    payload[1] = static_cast<char>((magic >> 8 ) & 0xFFu);
    payload[2] = static_cast<char>((magic >> 16) & 0xFFu);
    payload[3] = static_cast<char>((magic >> 24) & 0xFFu);

    statusBar()->showMessage(busyMsg);
    QApplication::setOverrideCursor(Qt::WaitCursor);

    /* One-shot listener filtered by (idx, odIdx, sub). The worker emits
     * customSdoDone once per customSdoWrite; we disconnect immediately
     * so a subsequent storage write doesn't trigger the wrong UI. */
    auto holder = std::make_shared<QMetaObject::Connection>();
    *holder = connect(m_worker, &MasterWorker::customSdoDone, this,
        [this, holder, idx, odIdx, sub, okMsg]
        (int doneIdx, bool isWrite, uint16_t doneOd, uint8_t doneSub,
         bool ok, QString /*valueDecoded*/, QString message){
            if (doneIdx != idx || !isWrite || doneOd != odIdx || doneSub != sub) return;
            QObject::disconnect(*holder);
            QApplication::restoreOverrideCursor();
            if (ok){
                statusBar()->showMessage(okMsg, 4000);
                m_log->appendInfo(okMsg);
            } else {
                statusBar()->clearMessage();
                const QString err = message.isEmpty() ? tr("unknown error") : message;
                m_log->appendError(tr("Storage SDO 0x%1:%2 failed: %3")
                                   .arg(odIdx, 4, 16, QChar('0'))
                                   .arg(sub)
                                   .arg(err));
                QMessageBox::warning(this, tr("Storage command failed"), err);
            }
        });

    QMetaObject::invokeMethod(m_worker, "customSdoWrite", Qt::QueuedConnection,
                              Q_ARG(int, idx),
                              Q_ARG(uint16_t, odIdx),
                              Q_ARG(uint8_t, sub),
                              Q_ARG(QByteArray, payload));
}

void MainWindow::onEditMotorProfile()
{
    /* Load the profile from the selected slave first, then open the editor
     * on the fresh values. With no slave selected, the editor opens with
     * whatever was last cached -- the user can still pick a slave and use
     * the in-dialog "Read from drive" button to refresh. */
    const int sel = selectedSlaveIdx();
    if (sel >= 0 && m_worker){
        m_profileEditPending = true;
        m_log->appendInfo(tr("reading motor profile from slave %1…").arg(sel));
        QMetaObject::invokeMethod(m_worker, "readMotorProfile",
                                  Qt::QueuedConnection, Q_ARG(int, sel));
        return;
    }
    openMotorProfileEditor();
}

void MainWindow::onMotorProfileRead(int idx, vrmc::MotorParams mp,
                                    bool ok, QString message)
{
    Q_UNUSED(idx);
    if (ok){
        /* 0x2070 now carries the full name-plate (incl. Kt :9, rated speed
         * :10, CPR :11) + rated current via 0x6075. The 0x608F encoder ratio
         * isn't read here, so keep its cached value. */
        mp.enc_increments = m_motorParams.enc_increments;
        mp.enc_motor_revs = m_motorParams.enc_motor_revs;
        m_motorParams = mp;
        m_profileView->setParams(m_motorParams);
        /* Keep the worker's CPR / Kt scaling in lock-step with whatever
         * we just read off the drive -- signal-generator amplitude conversion
         * and torque/velocity unit display both consume this. Without it
         * the auto-read on reconnect would leave the worker on the
         * previous session's scaling. */
        pushScalingToWorker();
        pushProfileToTuning();
        m_log->appendInfo(message);
    } else {
        m_log->appendError(message);
    }
    if (m_profileEditPending){
        m_profileEditPending = false;
        openMotorProfileEditor();
    }
}

void MainWindow::openMotorProfileEditor()
{
    MotorProfileEditor dlg(this);
    dlg.setParams(m_motorParams);

    const int sel = selectedSlaveIdx();
    dlg.setSlaveContext(sel);

    /* Wire the Read button: editor → worker → editor.setParams.
     * The connections are scoped to this dialog exec(); QDialog::exec()
     * runs a nested event loop, so signal/slot dispatch works normally. */
    auto rcRead = connect(&dlg, &MotorProfileEditor::readRequested, this,
        [this](int slaveIdx){
            QMetaObject::invokeMethod(m_worker, "readMotorProfile",
                                      Qt::QueuedConnection,
                                      Q_ARG(int, slaveIdx));
        });
    auto rcResult = connect(m_worker, &MasterWorker::motorProfileRead, &dlg,
        [&dlg](int /*idx*/, vrmc::MotorParams mp, bool ok, QString /*msg*/){
            if (ok) dlg.setParams(mp);
        });

    const int rc = dlg.exec();
    disconnect(rcRead);
    disconnect(rcResult);
    if (rc != QDialog::Accepted){ return; }

    m_motorParams = dlg.params();
    m_profileView->setParams(m_motorParams);
    pushProfileToTuning();
    m_log->appendInfo(tr("motor profile updated: type=%1 pole_pair=%2 "
                         "Rs=%3Ω Ls_d=%4H Ls_q=%5H λ=%6Wb J=%7kg·m²")
                          .arg(profile::motorTypeToString(m_motorParams.type))
                          .arg(m_motorParams.pole_pair)
                          .arg(m_motorParams.rs, 0, 'g', 4)
                          .arg(m_motorParams.ls_d, 0, 'g', 4)
                          .arg(m_motorParams.ls_q, 0, 'g', 4)
                          .arg(m_motorParams.rated_flux, 0, 'g', 4)
                          .arg(m_motorParams.inertia,    0, 'g', 4));
    pushScalingToWorker();

    if (sel < 0){
        m_log->appendInfo(tr("no slave selected — profile cached locally only "
                             "(not written to a drive)"));
        return;
    }

    /* Save = write 0x2070 + 0x6075 (live OD) AND fire 0x1010:01 = "save"
     * so the new profile lands in flash. Same magic value the Drive
     * Config dialog's Save button uses; serialized through the worker's
     * mutex so the storage SDO can't race the profile write. */
    QMetaObject::invokeMethod(m_worker, "writeMotorProfile",
                              Qt::QueuedConnection,
                              Q_ARG(int, sel),
                              Q_ARG(vrmc::MotorParams, m_motorParams));

    QByteArray payload(4, '\0');
    constexpr uint32_t kMagicSave = 0x65766173u;
    payload[0] = char( kMagicSave        & 0xFFu);
    payload[1] = char((kMagicSave >> 8 ) & 0xFFu);
    payload[2] = char((kMagicSave >> 16) & 0xFFu);
    payload[3] = char((kMagicSave >> 24) & 0xFFu);
    QMetaObject::invokeMethod(m_worker, "customSdoWrite",
                              Qt::QueuedConnection,
                              Q_ARG(int, sel),
                              Q_ARG(uint16_t, 0x1010),
                              Q_ARG(uint8_t,  0x01),
                              Q_ARG(QByteArray, payload));
    m_log->appendInfo(tr("profile written to slave %1 + 0x1010:01 = save").arg(sel));
}

void MainWindow::pushProfileToTuning()
{
    /* Both surfaces back-compute BW = f(Kp, profile) on every gain read /
     * tune callback; they cache the params locally so the recompute
     * doesn't race a pending SDO round-trip. */
    if (m_gains){ m_gains->setMotorParams(m_motorParams); }
    if (m_tuning && m_tuning->autoTune()){
        m_tuning->autoTune()->setMotorParams(m_motorParams);
    }
}

void MainWindow::pushScalingToWorker()
{
    if (!m_worker){ return; }
    const double cpr = (m_motorParams.enc_motor_revs != 0u)
        ? double(m_motorParams.enc_increments) / double(m_motorParams.enc_motor_revs)
        : 16384.0;
    /* Rated torque is DERIVED (= Kt * rated current). Kt formula depends on
     * MOTOR TYPE:
     *   PMSM → analytic 1.5·pole·flux   (rotor-flux PMSM model).
     *   BLDC → name-plate torque_constant (Nm/A) directly.
     * Picking the wrong formula makes the 0x6071 target_torque (per-mille of
     * rated_torque) round-trip wildly mis-scaled (e.g., BLDC profiles have
     * rated_flux = 0 → 1.5·pole·flux = 0 → divide-by-zero / huge scaling). */
    double kt = 0.0;
    if (m_motorParams.type == MotorType::Pmsm){
        kt = 1.5 * double(m_motorParams.pole_pair)
                 * double(m_motorParams.rated_flux);
    } else {
        /* BLDC */
        kt = double(m_motorParams.torque_constant);
    }
    const double ratedNm = kt * double(m_motorParams.rated_cur);
    m_motorParams.rated_torque = static_cast<float>(ratedNm);
    QMetaObject::invokeMethod(m_worker, "setScaling", Qt::QueuedConnection,
                              Q_ARG(double, cpr),
                              Q_ARG(double, ratedNm),
                              Q_ARG(double, double(m_motorParams.rated_cur)));
}

/* ==================================================================== *
 *  Drive menu
 * ==================================================================== */

/* Drive ▸ Restore Default Parameters…
 *
 * Sole storage shortcut left on the Drive menu (Save / Load Config to
 * Flash removed -- those duplicated the per-dialog Save button and the
 * Storage tab inside Configure Drive). This is the destructive path,
 * kept on the menu because the granular Restore buttons in the Storage
 * tab take 3 clicks to reach and a factory reset deserves an explicit
 * top-level shortcut.
 *
 * Magic-guarded SDO write to 0x1011:01 with "load" (0x64616F6C). The
 * board ERASES both blobs synchronously; defaults DO NOT apply live --
 * the user must power-cycle / NMT-reset for the boot init path to seed
 * the factory defaults back into RAM. The confirmation + the post-
 * success message say so. */
void MainWindow::onFactoryReset()
{
    if (selectedSlaveIdx() < 0){
        QMessageBox::information(this, tr("Restore Default Parameters"),
            tr("Select a slave in the table first."));
        return;
    }
    const auto btn = QMessageBox::warning(
        this, tr("Restore Default Parameters"),
        tr("This will erase BOTH parameter blobs on the selected drive:\n"
           "  • app_params (node id, calibrations, gear ratio, encoder offset)\n"
           "  • motor_drive_params (PI gains, limits, motor profile, current cal)\n\n"
           "The live motor keeps its current gains. Defaults take effect on "
           "the next power-on or NMT reset. Continue?"),
        QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
    if (btn != QMessageBox::Ok){ return; }

    sendStorageCommand(/*odIdx*/0x1011, /*sub*/0x01, /*magic*/0x64616F6Cu,
                       tr("Erasing parameter blobs…"),
                       tr("Parameter blobs erased — power-cycle the drive to "
                          "load factory defaults."));
}

void MainWindow::onUploadFirmware()
{
    FirmwareUpgradeDialog dlg(this);
    dlg.setSlaves(m_latestSnaps);

    /* Drive the real CANopen-FD upgrade through the worker. Connections
     * are scoped to this modal dialog — destroyed with it on close. */
    connect(&dlg, &FirmwareUpgradeDialog::startRequested,
            m_worker, &MasterWorker::startFirmwareUpgrade);
    connect(&dlg, &FirmwareUpgradeDialog::cancelRequested,
            m_worker, &MasterWorker::cancelFirmwareUpgrade);

    /* Bootloader target: Start -> open the Connect dialog (pick UDP sim /
     * ZLG hardware), connect OFFLINE to the boot node, then flash it as
     * slave idx 0. */
    connect(&dlg, &FirmwareUpgradeDialog::bootloaderRequested, &dlg,
            [this, &dlg](const QString& path){
        ConnectionDialog cdlg(this);
        cdlg.presetForBootloader(1);
        if (cdlg.exec() != QDialog::Accepted){
            dlg.onFinished(0, false, tr("cancelled — no connection opened"));
            return;
        }
        CanConfig cfg     = cdlg.config();
        cfg.count         = 1;
        cfg.allow_offline = true;   /* boot node sends no PDO heartbeat */
        connectWithConfig(cfg);

        /* The allow_offline connect settles in <200 ms; give it margin then
         * flash. startFirmwareUpgrade emits upgradeFinished("not connected")
         * if the transport failed to open (e.g. no ZLG adapter), so the
         * dialog gets a clean result with no extra error wiring. */
        QTimer::singleShot(800, &dlg, [this, path]{   /* &dlg: auto-cancel if closed */
            QMetaObject::invokeMethod(m_worker, [w = m_worker, path]{
                w->startFirmwareUpgrade(0, path);
            }, Qt::QueuedConnection);
        });
    });

    connect(m_worker, &MasterWorker::upgradeProgress,
            &dlg, &FirmwareUpgradeDialog::onProgress);
    connect(m_worker, &MasterWorker::upgradeFinished,
            &dlg, &FirmwareUpgradeDialog::onFinished);

    dlg.exec();
    /* If the operator closes mid-flight, make sure the worker stops. Queue
     * it onto the worker thread — the boot timer/objects are thread-affine. */
    QMetaObject::invokeMethod(m_worker, [w = m_worker]{
        w->cancelFirmwareUpgrade();
    }, Qt::QueuedConnection);
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
        /* No-op: motor profile is permanently inline now. The CLI
         * panel-name is kept for back-compat with --show-panel calls
         * that still pass "profile". */
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
        /* Custom-SDO tab plumbing — operator types an OD entry on the
         * dialog's "Custom SDO" tab and the worker pokes it via the
         * existing CanBackend SDO helpers. */
        connect(m_driveCfgDlg, &DriveConfigDialog::customSdoReadRequested,
                m_worker,      &MasterWorker::customSdoRead,
                Qt::QueuedConnection);
        connect(m_driveCfgDlg, &DriveConfigDialog::customSdoWriteRequested,
                m_worker,      &MasterWorker::customSdoWrite,
                Qt::QueuedConnection);
        /* Storage tab buttons (0x1010/0x1011) share the customSdoWrite
         * wire shape — same slot, same response signal. The dialog
         * routes the result back via the customSdoDone fan-out below. */
        connect(m_driveCfgDlg, &DriveConfigDialog::storageCommandRequested,
                m_worker,      &MasterWorker::customSdoWrite,
                Qt::QueuedConnection);
        connect(m_worker,      &MasterWorker::customSdoDone,
                m_driveCfgDlg, &DriveConfigDialog::onCustomSdoDone);
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

void MainWindow::onDeviceInfo()
{
    int idx = -1;
    const auto rows = m_table->selectionModel()
                          ? m_table->selectionModel()->selectedRows()
                          : QModelIndexList{};
    if (!rows.isEmpty()){
        idx = m_model->index(rows.first().row(), SlaveTableModel::ColIdx)
                  .data().toInt();
    }
    if (idx < 0){
        QMessageBox::information(this, tr("Device info"),
            tr("Select a slave in the table first."));
        return;
    }
    QMetaObject::invokeMethod(m_worker, "readDeviceInfo",
                              Qt::QueuedConnection, Q_ARG(int, idx));
}

void MainWindow::onDeviceInfoResult(int idx, vrmc::DeviceInfo info, bool ok,
                                    QString message)
{
    if (!ok){
        QMessageBox::warning(this, tr("Device info"),
            tr("Slave %1: no identity objects readable.\n%2")
                .arg(idx).arg(message));
        return;
    }
    const QString text =
        tr("<b>Slave %1 — Identity</b><br><br>"
           "Device type (0x1000): 0x%2<br>"
           "Device name (0x1008): %3<br>"
           "HW version (0x1009): %4<br>"
           "SW version (0x100A): %5<br><br>"
           "Vendor ID (0x1018:1): 0x%6<br>"
           "Product code (0x1018:2): 0x%7<br>"
           "Revision (0x1018:3): 0x%8<br>"
           "Serial (0x1018:4): 0x%9")
            .arg(idx)
            .arg(info.device_type,  8, 16, QChar('0'))
            .arg(info.device_name.isEmpty() ? tr("—") : info.device_name)
            .arg(info.hw_version.isEmpty()  ? tr("—") : info.hw_version)
            .arg(info.sw_version.isEmpty()  ? tr("—") : info.sw_version)
            .arg(info.vendor_id,    8, 16, QChar('0'))
            .arg(info.product_code, 8, 16, QChar('0'))
            .arg(info.revision,     8, 16, QChar('0'))
            .arg(info.serial,       8, 16, QChar('0'));

    QMessageBox box(this);
    box.setWindowTitle(tr("Device info"));
    box.setTextFormat(Qt::RichText);
    box.setText(text);
    if (!message.isEmpty()){
        box.setInformativeText(message);
    }
    box.setIcon(QMessageBox::Information);
    box.exec();
}

/* ==================================================================== *
 *  Data menu
 * ==================================================================== */

void MainWindow::onToggleRecording(bool on)
{
    if (on){
        /* Prompt for a destination path on every record-start. Default
         * filename uses a timestamp so successive recordings don't
         * silently overwrite each other. */
        const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss");
        const QString defaultName = QStringLiteral("vrmc_record_%1.csv").arg(stamp);
        const QString path = QFileDialog::getSaveFileName(
            this, tr("Record telemetry to CSV"),
            defaultName,
            tr("CSV (*.csv);;All files (*)"));
        if (path.isEmpty()){
            /* User cancelled — pop the toolbar action back out. */
            m_recordAct->blockSignals(true);
            m_recordAct->setChecked(false);
            m_recordAct->blockSignals(false);
            return;
        }
        m_recordFile = new QFile(path);
        if (!m_recordFile->open(QIODevice::WriteOnly | QIODevice::Truncate
                                | QIODevice::Text)){
            const QString err = m_recordFile->errorString();
            delete m_recordFile;  m_recordFile = nullptr;
            m_log->appendError(tr("record: open failed (%1): %2").arg(path).arg(err));
            m_recordAct->blockSignals(true);
            m_recordAct->setChecked(false);
            m_recordAct->blockSignals(false);
            return;
        }
        m_recordStream = new QTextStream(m_recordFile);
        m_recordRowCount      = 0;
        m_recordHeaderWritten = false;
        m_log->appendInfo(tr("recording started → %1").arg(path));
    } else {
        if (m_recordStream){ m_recordStream->flush(); }
        if (m_recordFile){
            m_recordFile->close();
            m_log->appendInfo(tr("recording stopped (%1 rows written → %2)")
                                  .arg(m_recordRowCount)
                                  .arg(m_recordFile->fileName()));
        }
        delete m_recordStream;  m_recordStream = nullptr;
        delete m_recordFile;    m_recordFile   = nullptr;
    }
}

void MainWindow::recordSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps)
{
    if (!m_recordStream){ return; }
    if (!m_recordHeaderWritten){
        /* One header line for all slaves. ts_ms is the wallclock when
         * the snapshot batch landed in MainWindow; per-slave columns
         * follow. */
        *m_recordStream << "ts_ms,slave_idx,slave_id,name,state,statusword,"
                           "error_code,pos_rad,vel_rad_s,torque_nm,"
                           "current_A,temperature_C,tracking_error\n";
        m_recordHeaderWritten = true;
    }
    const qint64 ts = QDateTime::currentMSecsSinceEpoch();
    for (const auto& s : snaps){
        *m_recordStream << ts << ','
                        << s.idx << ',' << s.id << ','
                        << '"' << s.name << '"' << ','
                        << s.state << ','
                        << s.statusword << ','
                        << s.errorCode << ','
                        << s.position << ','
                        << s.velocity << ','
                        << s.torque << ','
                        << s.current << ','
                        << s.temperature << ','
                        << s.trackingError << '\n';
        ++m_recordRowCount;
    }
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

QString MainWindow::findSimBinary() const
{
    /* Prefer the diagnostic's own in-tree simulator (`vrmc_sim`) — it
     * lives next to the diagnostic binary in the same CMake build
     * tree, so the demo doesn't depend on the SDK's compiled
     * `cia402_drive_sim`. The SDK binary is still accepted as a fall-
     * back so existing development checkouts keep working. */
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        /* in-tree sim, sibling to the diagnostic binary */
        appDir + "/vrmc_sim",
        appDir + "/tools/vrmc_sim",
        /* legacy: SDK's compiled sim */
        appDir + "/../../vr-mc-sdk/build/app/cia402_drive_sim",
        appDir + "/../vr-mc-sdk/build/app/cia402_drive_sim",
        appDir + "/cia402_drive_sim",
    };
    for (const QString& c : candidates){
        const QFileInfo fi(QDir::cleanPath(c));
        if (fi.exists() && fi.isExecutable()){
            return fi.canonicalFilePath();
        }
    }
    /* Last resort: PATH lookup. Try our own name first, then the
     * SDK's. */
    for (const QString& name : { QStringLiteral("vrmc_sim"),
                                 QStringLiteral("cia402_drive_sim") }){
        const QString fromPath = QStandardPaths::findExecutable(name);
        if (!fromPath.isEmpty()){ return fromPath; }
    }
    return QString();
}

void MainWindow::onStartDemo()
{
    if (!m_demoProcs.isEmpty()){
        QMessageBox::information(this, tr("Demo"),
            tr("A demo is already running. Stop it first via Help → "
               "Stop demo."));
        return;
    }
    const QString binary = findSimBinary();
    if (binary.isEmpty()){
        QMessageBox::warning(this, tr("Demo"),
            tr("vrmc_sim not found.\n\n"
               "The simulator is built as part of the diagnostic — "
               "rebuild with `cmake --build vr-mc-diagnostic/build` "
               "and try again."));
        return;
    }

    bool ok = false;
    const int count = QInputDialog::getInt(
        this, tr("Start demo"),
        tr("How many simulated slaves? Each runs as a separate "
           "cia402_drive_sim process on UDP multicast "
           "239.192.0.42:23400 with sequential node IDs starting "
           "at 5."),
        /*default*/ 3,
        /*min*/     1,
        /*max*/     32,
        /*step*/    1,
        &ok);
    if (!ok){ return; }

    /* Spawn one process per slave. Each gets its own --node id so
     * their statusword/PDO traffic doesn't collide on the bus. */
    constexpr int kFirstId = 5;
    for (int i = 0; i < count; ++i){
        auto* p = new QProcess(this);
        p->setProgram(binary);
        p->setArguments({QStringLiteral("--udp"),
                         QStringLiteral("--node"),
                         QString::number(kFirstId + i)});
        /* Tee stderr/stdout straight to the parent so failures land in
         * the calling terminal; the diagnostic's LogDock would be more
         * polished but stderr is cheap and good enough for a demo. */
        p->setProcessChannelMode(QProcess::ForwardedChannels);
        p->start();
        if (!p->waitForStarted(1500)){
            const QString err = p->errorString();
            /* Tear down everything we did spawn and abort. */
            for (auto* q : m_demoProcs){ q->kill(); q->deleteLater(); }
            m_demoProcs.clear();
            p->deleteLater();
            QMessageBox::warning(this, tr("Demo"),
                tr("Failed to start cia402_drive_sim for node %1: %2")
                    .arg(kFirstId + i).arg(err));
            return;
        }
        m_demoProcs.push_back(p);
        m_log->appendInfo(tr("demo: spawned cia402_drive_sim --node %1 (pid %2)")
                              .arg(kFirstId + i).arg(p->processId()));
    }

    m_demoStartAct->setEnabled(false);
    m_demoStopAct ->setEnabled(true);

    /* Give the sims a moment to bind their UDP sockets + start
     * emitting the heartbeat TPDO, then auto-connect. The probe in
     * CanBackend::open waits up to 250 ms for that first heartbeat
     * — bake in a little extra here so the connect doesn't false-
     * negative on slower machines. */
    QTimer::singleShot(400, this, [this, count]{
        CanConfig cfg;
        cfg.kind     = CanKind::Udp;
        cfg.count    = static_cast<uint8_t>(count);
        cfg.first_id = 5;
        connectWithConfig(cfg);
    });
}

void MainWindow::onStopDemo()
{
    if (m_demoProcs.isEmpty()){ return; }
    /* Drop the bus first so the diagnostic doesn't yell about
     * disappearing telemetry while the sims tear down. */
    QMetaObject::invokeMethod(m_worker, "disconnect_", Qt::QueuedConnection);
    for (auto* p : m_demoProcs){
        if (p->state() != QProcess::NotRunning){
            p->terminate();
            if (!p->waitForFinished(500)){ p->kill(); p->waitForFinished(200); }
        }
        p->deleteLater();
    }
    m_log->appendInfo(tr("demo: stopped %1 sim process(es)").arg(m_demoProcs.size()));
    m_demoProcs.clear();
    m_demoStartAct->setEnabled(true);
    m_demoStopAct ->setEnabled(false);
}

QString MainWindow::findBootSimBinary() const
{
    /* Same search strategy as findSimBinary(), for the bootloader-slave
     * counterpart built alongside the diagnostic (tools/vrmc_boot_sim.c). */
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        appDir + "/vrmc_boot_sim",
        appDir + "/tools/vrmc_boot_sim",
    };
    for (const QString& c : candidates){
        const QFileInfo fi(QDir::cleanPath(c));
        if (fi.exists() && fi.isExecutable()){ return fi.canonicalFilePath(); }
    }
    const QString fromPath = QStandardPaths::findExecutable(
        QStringLiteral("vrmc_boot_sim"));
    return fromPath;
}

void MainWindow::onStartFwDemo()
{
    if (!m_demoProcs.isEmpty()){
        QMessageBox::information(this, tr("Demo"),
            tr("A demo is already running. Stop it first via Help → "
               "Stop demo."));
        return;
    }
    const QString binary = findBootSimBinary();
    if (binary.isEmpty()){
        QMessageBox::warning(this, tr("Firmware-upgrade demo"),
            tr("vrmc_boot_sim not found.\n\n"
               "It is built as part of the diagnostic — rebuild with "
               "`cmake --build vr-mc-diagnostic/build` and try again."));
        return;
    }

    /* Work dir for the simulated 'flash' (app.bin etc. land here). */
    const QString workDir = QDir(QDir::tempPath()).filePath(
        QStringLiteral("vrmc_boot_demo"));
    QDir().mkpath(workDir);

    auto* p = new QProcess(this);
    p->setProgram(binary);
    p->setArguments({QStringLiteral("--work"), workDir});
    p->setProcessChannelMode(QProcess::ForwardedChannels);
    p->start();
    if (!p->waitForStarted(1500)){
        const QString err = p->errorString();
        p->deleteLater();
        QMessageBox::warning(this, tr("Firmware-upgrade demo"),
            tr("Failed to start vrmc_boot_sim: %1").arg(err));
        return;
    }
    m_demoProcs.push_back(p);
    m_demoStartAct->setEnabled(false);
    m_demoStopAct ->setEnabled(true);
    m_log->appendInfo(tr("fw-demo: vrmc_boot_sim (node %1) pid %2, flash → %3")
                          .arg(0x0A).arg(p->processId()).arg(workDir));

    /* Connect to the boot node (id 10 = BOOT_COFD_NODE_ID_DEFAULT). It is
     * an SDO server only, so allow_offline lets the connect register it
     * without waiting for a PDO heartbeat.
     *
     * Open the upgrade dialog only AFTER the worker reports 'connected'
     * (the slave is registered) plus a short settle for the first snapshot
     * to populate the slave list — otherwise the target dropdown opens
     * empty / without the node id. One-shot connection + a safety fallback
     * in case 'connected' never arrives. */
    QTimer::singleShot(400, this, [this]{
        auto opened = std::make_shared<bool>(false);
        auto conn   = std::make_shared<QMetaObject::Connection>();
        auto openDlg = [this, opened]{
            if (*opened){ return; }
            *opened = true;
            m_log->appendInfo(tr("fw-demo: pick a firmware image and Start — "
                                 "it streams over UDP to the boot sim."));
            onUploadFirmware();
        };
        *conn = connect(m_worker, &MasterWorker::connected, this,
                        [this, conn, openDlg](int){
            QObject::disconnect(*conn);
            /* let one UI refresh emit a snapshot carrying the node id */
            QTimer::singleShot(250, this, openDlg);
        });
        QTimer::singleShot(3000, this, openDlg);   /* fallback */

        CanConfig cfg;
        cfg.kind          = CanKind::Udp;
        cfg.count         = 1;
        cfg.first_id      = 0x0A;
        cfg.allow_offline = true;
        connectWithConfig(cfg);
    });
}

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
    /* Search a few well-known locations because the binary can live in
     * several layouts:
     *   - source tree   : build/vr_mc_diagnostic  →  ../docs/, ../README.md
     *   - AppImage      : usr/bin/vr_mc_diagnostic →  ../share/vr_mc_diagnostic/docs/
     *   - system install: /usr/bin/vr_mc_diagnostic → same as AppImage
     * Prefer rendered HTML over the raw markdown so the browser-side
     * formatting is preserved; fall back to README.md at the repo
     * root for dev builds where docs haven't been rendered yet. */
    const QString appDir = QCoreApplication::applicationDirPath();
    const QStringList candidates = {
        appDir + "/../share/vr_mc_diagnostic/docs/user_guide.html",
        appDir + "/../share/vr_mc_diagnostic/docs/user_guide.pdf",
        appDir + "/../share/vr_mc_diagnostic/docs/user_guide.md",
        appDir + "/../docs/user_guide.html",
        appDir + "/../docs/user_guide.pdf",
        appDir + "/../docs/user_guide.md",
        appDir + "/../README.md",
    };
    QStringList tried;
    for (const QString& c : candidates){
        const QString clean = QDir::cleanPath(c);
        tried << clean;
        if (QFile::exists(clean)){
            const QUrl url = QUrl::fromLocalFile(clean);
            if (QDesktopServices::openUrl(url)){
                m_log->appendInfo(tr("documentation: opened %1").arg(clean));
            } else {
                /* xdg-open / system handler refused — copy the path
                 * to the clipboard so the operator can paste it. */
                QApplication::clipboard()->setText(clean);
                m_log->appendInfo(tr("documentation: %1 (clipboard — "
                                     "open it manually)").arg(clean));
            }
            return;
        }
    }
    m_log->appendError(tr("documentation: not found. Tried:\n  %1")
                           .arg(tried.join(QStringLiteral("\n  "))));
}

}  // namespace vrmc
