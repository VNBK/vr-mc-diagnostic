#include "FirmwareUpgradeDialog.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace vrmc {

FirmwareUpgradeDialog::FirmwareUpgradeDialog(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(tr("Firmware upgrade"));
    setMinimumWidth(540);
    setMinimumHeight(440);

    /* --- target + image picker --- */
    m_target = new QComboBox(this);
    m_path   = new QLineEdit(this);
    m_path->setPlaceholderText(tr("(no image selected)"));
    m_path->setReadOnly(true);
    m_browse = new QPushButton(tr("Browse…"), this);
    connect(m_browse, &QPushButton::clicked, this, &FirmwareUpgradeDialog::onBrowse);

    auto* pickRow = new QHBoxLayout;
    pickRow->addWidget(m_path, 1);
    pickRow->addWidget(m_browse);

    auto* pickBox = new QGroupBox(tr("Image"), this);
    auto* pickForm = new QFormLayout(pickBox);
    pickForm->addRow(tr("Target slave"), m_target);
    pickForm->addRow(tr("Firmware file"), pickRow);

    /* --- progress + status --- */
    m_bar = new QProgressBar(this);
    m_bar->setRange(0, 100);
    m_bar->setValue(0);

    m_status = new QLabel(tr("idle — pick an image and click Start"), this);
    m_status->setWordWrap(true);

    /* --- log area --- */
    m_log = new QPlainTextEdit(this);
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);
    m_log->setStyleSheet(QStringLiteral(
        "QPlainTextEdit{background:#111;color:#ddd;font-family:monospace;}"));

    /* --- buttons --- */
    m_start  = new QPushButton(tr("Start"),  this);
    m_cancel = new QPushButton(tr("Cancel"), this);
    m_cancel->setEnabled(false);
    auto* btnRow = new QHBoxLayout;
    btnRow->addStretch();
    btnRow->addWidget(m_cancel);
    btnRow->addWidget(m_start);

    connect(m_start,  &QPushButton::clicked, this, &FirmwareUpgradeDialog::onStart);
    connect(m_cancel, &QPushButton::clicked, this, &FirmwareUpgradeDialog::onCancel);

    /* --- layout --- */
    auto* root = new QVBoxLayout(this);
    root->addWidget(pickBox);
    root->addWidget(m_bar);
    root->addWidget(m_status);
    root->addWidget(m_log, 1);
    root->addLayout(btnRow);

    /* --- timer for the simulated upload (V1: no real protocol). --- */
    m_tick = new QTimer(this);
    m_tick->setInterval(100);   /* 100 ms steps */
    connect(m_tick, &QTimer::timeout, this, &FirmwareUpgradeDialog::onTick);

    appendLog(tr("Firmware upgrade pane ready. (V1: simulated upload — "
                 "no protocol wired yet)"));
}

void FirmwareUpgradeDialog::setSlaves(const QVector<SlaveSnapshot>& snaps)
{
    m_target->clear();
    if (snaps.isEmpty()){
        m_target->addItem(tr("(no slaves — connect first)"));
        m_target->setEnabled(false);
        m_start->setEnabled(false);
        return;
    }
    m_target->setEnabled(true);
    m_start->setEnabled(true);
    for (const auto& s : snaps){
        const QString label = QStringLiteral("idx %1 — id %2 — %3")
                                  .arg(s.idx).arg(s.id).arg(s.name);
        m_target->addItem(label, s.idx);
    }
}

void FirmwareUpgradeDialog::onBrowse()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Select firmware image"), {},
        tr("Firmware (*.bin *.hex *.elf *.mot);;All files (*)"));
    if (path.isEmpty()){ return; }
    m_path->setText(path);
    QFileInfo fi(path);
    appendLog(tr("selected: %1 (%2 bytes)").arg(path).arg(fi.size()));
}

void FirmwareUpgradeDialog::onStart()
{
    if (m_running){ return; }
    if (m_path->text().isEmpty()){
        QMessageBox::information(this, windowTitle(),
            tr("Pick a firmware image first."));
        return;
    }
    if (m_target->count() == 0 || !m_target->isEnabled()){
        QMessageBox::information(this, windowTitle(),
            tr("Connect to a drive first so we have a target slave."));
        return;
    }
    m_running  = true;
    m_progress = 0;
    m_bar->setValue(0);
    m_start->setEnabled(false);
    m_cancel->setEnabled(true);
    m_browse->setEnabled(false);
    m_target->setEnabled(false);
    m_status->setText(tr("uploading to %1 …").arg(m_target->currentText()));
    appendLog(tr("=== upload started: %1 → %2")
                  .arg(QFileInfo(m_path->text()).fileName())
                  .arg(m_target->currentText()));
    m_tick->start();
}

void FirmwareUpgradeDialog::onCancel()
{
    if (!m_running){
        reject();
        return;
    }
    m_tick->stop();
    appendLog(tr("=== cancelled at %1%%").arg(m_progress));
    m_status->setText(tr("cancelled"));
    resetIdle();
}

void FirmwareUpgradeDialog::onTick()
{
    /* Simulated upload: 0 → 100 over ~5 s, with a few canned step messages. */
    m_progress += 2;
    if (m_progress == 10){
        appendLog(tr("erase block 0..."));
    } else if (m_progress == 30){
        appendLog(tr("transfer (%1 / %2 KB)").arg(m_progress).arg(100));
    } else if (m_progress == 60){
        appendLog(tr("verify checksum..."));
    } else if (m_progress == 90){
        appendLog(tr("commit + reboot..."));
    }
    if (m_progress >= 100){
        m_progress = 100;
        m_bar->setValue(100);
        m_tick->stop();
        appendLog(tr("=== upload complete (simulated)"));
        m_status->setText(tr("done"));
        QMessageBox::information(this, windowTitle(),
            tr("Upload simulation complete.\n\nPlug in real CiA 1F50 / "
               "vendor FW-update support to make this hit the wire."));
        resetIdle();
        return;
    }
    m_bar->setValue(m_progress);
}

void FirmwareUpgradeDialog::resetIdle()
{
    m_running = false;
    m_start->setEnabled(true);
    m_cancel->setEnabled(false);
    m_browse->setEnabled(true);
    m_target->setEnabled(m_target->count() > 0);
}

void FirmwareUpgradeDialog::appendLog(const QString& line)
{
    const QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    m_log->appendPlainText(QStringLiteral("[%1] %2").arg(ts, line));
}

}  // namespace vrmc
