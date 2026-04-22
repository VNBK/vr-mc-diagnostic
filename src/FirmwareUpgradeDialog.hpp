/**
 * @file   FirmwareUpgradeDialog.hpp
 * @brief  Modal screen that walks a firmware upload to a selected slave.
 *
 * V1 simulates the upload over a few seconds with a QTimer so the UX is
 * fully exercised without the actual CiA 1F50 / vendor-specific transfer
 * protocol. Plumb a real worker slot into @c performUpload() once the
 * SDK exposes the FW-update primitives.
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
class QTimer;

namespace vrmc {

class FirmwareUpgradeDialog : public QDialog
{
    Q_OBJECT
public:
    explicit FirmwareUpgradeDialog(QWidget* parent = nullptr);

    /** Populate the slave dropdown from the live snapshot list. */
    void setSlaves(const QVector<vrmc::SlaveSnapshot>& snaps);

private slots:
    void onBrowse();
    void onStart();
    void onCancel();
    void onTick();

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
    QTimer*          m_tick    = nullptr;
    int              m_progress = 0;
    bool             m_running  = false;
};

}  // namespace vrmc
