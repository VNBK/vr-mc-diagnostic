/**
 * @file   FirmwareUpgradeDialog.hpp
 * @brief  Modal screen that walks a firmware upload to a selected slave.
 *
 * Drives a real CANopen-FD firmware upgrade through MasterWorker
 * (vr_bootmaster → the device's 0x3003 boot OD). The dialog only owns the
 * UI: @ref startRequested / @ref cancelRequested go out to the worker, and
 * @ref onProgress / @ref onFinished come back to render progress.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QDialog>

class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QLabel;

namespace vrmc {

class FirmwareUpgradeDialog : public QDialog
{
    Q_OBJECT
public:
    explicit FirmwareUpgradeDialog(QWidget* parent = nullptr);

    /** Populate the slave dropdown from the live snapshot list. */
    void setSlaves(const QVector<vrmc::SlaveSnapshot>& snaps);

signals:
    /** Operator hit Start: stream @p path to @p slaveIdx. */
    void startRequested(int slaveIdx, QString path);
    /** Operator hit Cancel during an in-flight upgrade. */
    void cancelRequested();

public slots:
    /** Wired to @c MasterWorker::upgradeProgress. */
    void onProgress(int idx, int pct, QString stage);
    /** Wired to @c MasterWorker::upgradeFinished. */
    void onFinished(int idx, bool ok, QString message);

private slots:
    void onBrowse();
    void onStart();
    void onCancel();

private:
    void appendLog(const QString& line);
    void resetIdle();

    /* --- UI --- */
    QComboBox*       m_target  = nullptr;
    QLineEdit*       m_path    = nullptr;
    QPushButton*     m_browse  = nullptr;
    QProgressBar*    m_bar     = nullptr;
    QLabel*          m_status  = nullptr;
    QPushButton*     m_start   = nullptr;
    QPushButton*     m_cancel  = nullptr;
    QPlainTextEdit*  m_log     = nullptr;

    /* --- state --- */
    bool             m_running    = false;
    int              m_activeIdx  = -1;
};

}  // namespace vrmc
