#include "MainWindow.hpp"
#include "MasterWorker.hpp"

#include <QApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <algorithm>
#include <QFont>
#include <QIcon>
#include <QPainter>
#include <QPixmap>
#include <QSplashScreen>
#include <QTimer>

/* Build a 640x360 splash canvas: the VinRobotic mark, app title, and a
 * status line paintSplash() can rewrite while the backend comes up. The
 * PNG is loaded from the Qt resource at :/brand/vinrobotic.png (see
 * resources/vrmc.qrc). */
static QPixmap makeSplashPixmap()
{
    QPixmap canvas(640, 360);
    canvas.fill(QColor("#0f1a2a"));

    QPainter p(&canvas);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::SmoothPixmapTransform);

    /* Tinted bar across the top for some branding weight. */
    p.fillRect(0, 0, canvas.width(), 6, QColor("#3b9a6d"));

    QPixmap logo(QStringLiteral(":/brand/vinrobotic.png"));
    if (!logo.isNull()){
        const QPixmap scaled = logo.scaled(
            260, 160, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        const int x = (canvas.width()  - scaled.width())  / 2;
        const int y = 48;
        p.drawPixmap(x, y, scaled);
    }

    p.setPen(QColor("#e8e8e8"));
    QFont f = p.font();
    f.setPointSize(20);
    f.setWeight(QFont::DemiBold);
    p.setFont(f);
    p.drawText(QRect(0, 220, canvas.width(), 32),
               Qt::AlignHCenter | Qt::AlignVCenter,
               QStringLiteral("VR Motor Control Diagnostic Tool"));

    p.setPen(QColor("#9fb4c6"));
    f.setPointSize(10);
    f.setWeight(QFont::Normal);
    p.setFont(f);
    p.drawText(QRect(0, 254, canvas.width(), 20),
               Qt::AlignHCenter | Qt::AlignVCenter,
               QStringLiteral("VinRobotic"));

    return canvas;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setApplicationName(QStringLiteral("VR Motor Control Diagnostic Tool"));
    /* App-wide icon picked up by the window manager / taskbar and used as
     * the default for every top-level window that doesn't override it. */
    app.setWindowIcon(QIcon(QStringLiteral(":/brand/vinrobotic.png")));

    /* Metatypes used across thread boundaries. */
    qRegisterMetaType<vrmc::CanConfig>("vrmc::CanConfig");
    qRegisterMetaType<vrmc::Mode>("vrmc::Mode");
    qRegisterMetaType<vrmc::TargetKind>("vrmc::TargetKind");
    qRegisterMetaType<vrmc::SlaveSnapshot>("vrmc::SlaveSnapshot");
    qRegisterMetaType<QVector<vrmc::SlaveSnapshot>>("QVector<vrmc::SlaveSnapshot>");
    qRegisterMetaType<vrmc::Loop>("vrmc::Loop");
    qRegisterMetaType<vrmc::GenCfg>("vrmc::GenCfg");
    qRegisterMetaType<vrmc::CanBackend::PdoMapEntry>(
        "vrmc::CanBackend::PdoMapEntry");
    qRegisterMetaType<QVector<vrmc::CanBackend::PdoMapEntry>>(
        "QVector<vrmc::CanBackend::PdoMapEntry>");
    qRegisterMetaType<vrmc::DriveConfig>("vrmc::DriveConfig");
    qRegisterMetaType<vrmc::DeviceInfo>("vrmc::DeviceInfo");
    qRegisterMetaType<vrmc::MotorParams>("vrmc::MotorParams");

    QCommandLineParser cli;
    cli.setApplicationDescription(QStringLiteral("VR Motor Control Diagnostic Tool"));
    cli.addHelpOption();
    QCommandLineOption autoConn(QStringList{"a", "auto-connect"},
        QStringLiteral("Connect with default config as soon as the window is up."));
    QCommandLineOption quitAfter(QStringList{"quit-after"},
        QStringLiteral("Quit after <ms> milliseconds (for headless smoke tests)."),
        QStringLiteral("ms"));
    QCommandLineOption shot(QStringList{"screenshot"},
        QStringLiteral("Grab the main window to <path> after <ms> ms and quit."),
        QStringLiteral("path[:ms]"));
    QCommandLineOption showPanel(QStringList{"show-panel"},
        QStringLiteral("Open a panel before screenshotting: "
                       "'pdo-map', 'tuning', 'profile', 'drive-config'."),
        QStringLiteral("name"));
    QCommandLineOption showTab(QStringList{"show-tab"},
        QStringLiteral("Select the given left-tab by index (0=Control, "
                       "1=Gains, 2=Signal gen)."),
        QStringLiteral("idx"));
    QCommandLineOption onlyDialog(QStringList{"only-dialog"},
        QStringLiteral("If set, screenshot captures only the topmost dialog "
                       "(modal or modeless), not the main window."));
    cli.addOption(autoConn);
    cli.addOption(quitAfter);
    cli.addOption(shot);
    cli.addOption(showPanel);
    cli.addOption(showTab);
    cli.addOption(onlyDialog);
    cli.process(app);

    /* Splash up immediately so the user sees branding while Qt/widgets
     * get wired. We hide it once the main window is on screen, with a
     * small floor so it isn't a subliminal flash on fast machines. */
    QSplashScreen splash(makeSplashPixmap());
    splash.setWindowFlag(Qt::WindowStaysOnTopHint);
    splash.showMessage(QStringLiteral("Initializing…"),
                       Qt::AlignBottom | Qt::AlignHCenter,
                       QColor("#cfd8e3"));
    splash.show();
    app.processEvents();

    QElapsedTimer splashTimer;
    splashTimer.start();

    vrmc::MainWindow w;     /* construct but do NOT show yet — splash
                             * owns the screen until its timeout fires */
    splash.showMessage(QStringLiteral("Ready."),
                       Qt::AlignBottom | Qt::AlignHCenter,
                       QColor("#cfd8e3"));
    app.processEvents();

    /* Splash dwells for a minimum kMinSplashMs so it isn't a subliminal
     * flash on machines where construction was fast. Only after the
     * dwell elapses do we close() the splash AND show() the main window
     * — the operator wanted "MainWindow only appears once the splash
     * has gone", not "MainWindow comes up underneath and gets uncovered
     * later". Subtract whatever time MainWindow construction already
     * consumed so the timeout isn't doubled on slow boots. */
    constexpr qint64 kMinSplashMs = 1200;
    const qint64 elapsed = splashTimer.elapsed();
    const int    remain  = int(std::max<qint64>(0, kMinSplashMs - elapsed));
    QTimer::singleShot(remain, &splash, [&splash, &w]{
        splash.close();
        w.show();
        w.raise();
        w.activateWindow();
    });

    if (cli.isSet(autoConn)){
        QTimer::singleShot(200, &w, [&w]{
            vrmc::CanConfig cfg;   /* defaults match the motor_drive_cia402 sim */
            w.connectWithConfig(cfg);
        });
    }
    if (cli.isSet(quitAfter)){
        QTimer::singleShot(cli.value(quitAfter).toInt(),
                           &app, &QApplication::quit);
    }
    if (cli.isSet(showTab)){
        const int tab = cli.value(showTab).toInt();
        QTimer::singleShot(400, &w, [tab, &w]{ w.setLeftTab(tab); });
    }
    if (cli.isSet(showPanel)){
        const QString name = cli.value(showPanel);
        QTimer::singleShot(900, &w, [name, &w]{ w.showDocPanel(name); });
    }
    if (cli.isSet(autoConn)){
        /* After auto-connecting, select the first slave row so per-slave
         * panels (Control, Telemetry, PDO map) receive data. Without
         * this the chart would stay empty even as snapshots flow. */
        QTimer::singleShot(600, &w, [&w]{ w.setLeftTab(0); w.showDocPanel("select-first"); });
    }
    if (cli.isSet(shot)){
        /* Format: path[:delayMs]. Grabs the main window via QWidget::grab,
         * which works even under the offscreen QPA plugin. */
        const QString spec  = cli.value(shot);
        const int     colon = spec.lastIndexOf(':');
        QString path = spec;
        int     delay = 1500;
        if (colon > 0 && spec.mid(colon + 1).toInt() > 0){
            path  = spec.left(colon);
            delay = spec.mid(colon + 1).toInt();
        }
        const bool dialogOnly = cli.isSet(onlyDialog);
        QTimer::singleShot(delay, &w, [path, &w, &app, dialogOnly]{
            if (dialogOnly){
                /* Find the topmost non-main-window, non-splash widget
                 * and grab just that one. Useful for clean dialog shots. */
                for (QWidget* tl : QApplication::topLevelWidgets()){
                    if (tl == &w || !tl->isVisible() || tl->size().isEmpty()){
                        continue;
                    }
                    if (tl->inherits("QSplashScreen")){ continue; }
                    if (tl->inherits("QMessageBox")){ continue; }
                    QPixmap pm = tl->grab();
                    if (!pm.save(path, "PNG")){
                        qWarning("screenshot save failed: %s",
                                 qPrintable(path));
                    }
                    app.quit();
                    return;
                }
                qWarning("only-dialog: no dialog visible");
                app.quit();
                return;
            }
            /* Composite every visible top-level widget onto a single
             * canvas so modal dialogs / floating docks land in the
             * frame, not just the main window. Offscreen QPA reports
             * (0,0) for most widgets, so we fall back to stacking
             * anything that doesn't have a useful geometry next to
             * the main window. */
            QPixmap main = w.grab();
            QList<QPixmap> extra;
            for (QWidget* tl : QApplication::topLevelWidgets()){
                if (tl == &w || !tl->isVisible() || tl->size().isEmpty()){
                    continue;
                }
                extra.append(tl->grab());
            }
            int extraW = 0, extraH = 0;
            for (const QPixmap& e : extra){
                extraW = std::max(extraW, e.width());
                extraH += e.height() + 12;
            }
            QPixmap canvas(main.width() + (extraW ? extraW + 16 : 0),
                           std::max(main.height(), extraH));
            canvas.fill(Qt::white);
            QPainter p(&canvas);
            p.drawPixmap(0, 0, main);
            int y = 0;
            for (const QPixmap& e : extra){
                p.drawPixmap(main.width() + 16, y, e);
                y += e.height() + 12;
            }
            p.end();
            if (!canvas.save(path, "PNG")){
                qWarning("screenshot save failed: %s", qPrintable(path));
            }
            app.quit();
        });
    }

    return app.exec();
}
