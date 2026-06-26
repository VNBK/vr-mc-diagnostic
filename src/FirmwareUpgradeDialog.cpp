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

    appendLog(tr("Firmware upgrade ready. Pick an image and click Start; the "
                 "tool resets the board into its bootloader and streams over "
                 "CAN-FD."));
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
    const QVariant data = m_target->currentData();
    if (!data.isValid()){
        QMessageBox::information(this, windowTitle(),
            tr("Select a valid target slave."));
        return;
    }
    const int ret = QMessageBox::warning(this, windowTitle(),
        tr("This resets slave %1 into its bootloader and overwrites its "
           "application flash. Do not power off during the upgrade.\n\nProceed?")
            .arg(m_target->currentText()),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (ret != QMessageBox::Yes){ return; }

    m_activeIdx = data.toInt();
    m_running   = true;
    m_bar->setValue(0);
    m_start->setEnabled(false);
    m_cancel->setEnabled(true);
    m_browse->setEnabled(false);
    m_target->setEnabled(false);
    m_status->setText(tr("starting upgrade to %1 …").arg(m_target->currentText()));
    appendLog(tr("=== upgrade started: %1 → %2")
                  .arg(QFileInfo(m_path->text()).fileName())
                  .arg(m_target->currentText()));
    emit startRequested(m_activeIdx, m_path->text());
}

void FirmwareUpgradeDialog::onCancel()
{
    if (!m_running){
        reject();
        return;
    }
    appendLog(tr("=== cancel requested"));
    m_status->setText(tr("cancelling…"));
    m_cancel->setEnabled(false);
    emit cancelRequested();
}

void FirmwareUpgradeDialog::onProgress(int idx, int pct, QString stage)
{
    if (!m_running || idx != m_activeIdx){ return; }
    m_bar->setValue(pct);
    m_status->setText(tr("%1% — %2").arg(pct).arg(stage));
    appendLog(tr("[%1%] %2").arg(pct, 3).arg(stage));
}

void FirmwareUpgradeDialog::onFinished(int idx, bool ok, QString message)
{
    if (idx != m_activeIdx){ return; }
    if (ok){
        m_bar->setValue(100);
        m_status->setText(tr("done — %1").arg(message));
        appendLog(tr("=== %1").arg(message));
    } else {
        m_status->setText(tr("failed — %1").arg(message));
        appendLog(tr("=== FAILED: %1").arg(message));
    }
    resetIdle();
    if (ok){ QMessageBox::information(this, windowTitle(), message); }
    else   { QMessageBox::warning   (this, windowTitle(), message); }
}

void FirmwareUpgradeDialog::resetIdle()
{
    m_running   = false;
    m_activeIdx = -1;
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
