/**
 * @file    FaultDiagDialog.hpp
 * @brief   ROADMAP #1 — Fault / EMCY diagnostics.
 *
 * Reads the standard CiA-301 + CiA-402 fault surface via SDO and
 * renders it in operator-friendly form:
 *
 *   - **0x1001** Error Register — 8 bits (generic / current / voltage /
 *     temperature / communication / device-specific / reserved /
 *     manufacturer-specific). Rendered as a row of LED-style
 *     indicators; active bits light red.
 *   - **0x603F** Current Error Code — u16, decoded via
 *     @ref JointControlPanel::decodeErrorCode. Same code the PDO
 *     snapshot already surfaces, but shown here alongside history +
 *     bit register for context.
 *   - **0x1003** Predefined Error Field — CiA-301 fault history.
 *     Sub-0 is the count (u8); sub-1..N are u32 codes (LSB = 0x603F
 *     code, MSB = manufacturer-specific info). Rendered as a table
 *     with decoded name.
 *   - **Clear history** button — writes 0x1003:00 = 0 per CiA-301.
 *
 * Data flow: dialog is a passive consumer of @ref MasterWorker's
 * existing @c customSdoRead / @c customSdoWrite slots + the
 * @c customSdoDone fan-out signal. Filtering by (odIdx, sub) lets
 * multiple dialogs share the same fan-out without cross-talk.
 *
 * Not yet implemented (follow-up): live EMCY frame capture. That
 * requires a backend-level RX callback that the current CanBackend
 * doesn't expose; the roadmap flags it as a separate deliverable.
 * The dialog surfaces a placeholder note so the operator knows the
 * gap exists.
 */
#pragma once

#include <QDialog>
#include <QSet>
#include <QVector>


class QLabel;
class QPushButton;
class QTableWidget;

namespace vrmc {

class FaultDiagDialog : public QDialog
{
    Q_OBJECT
public:
    explicit FaultDiagDialog(QWidget* parent = nullptr);

    /** @brief Bind to a specific slave. Refresh + Clear buttons target
     *  this slave; the header label reflects the context. */
    void setSlaveContext(int slaveIdx, const QString& name);

signals:
    /** @brief Consumed by MainWindow → MasterWorker::customSdoRead. */
    void sdoReadRequested (int slaveIdx, quint16 odIdx, quint8 sub, int byteLen);
    /** @brief Consumed by MainWindow → MasterWorker::customSdoWrite. */
    void sdoWriteRequested(int slaveIdx, quint16 odIdx, quint8 sub,
                            QByteArray bytes);

public slots:
    /** @brief Wired from MasterWorker::customSdoDone. Filters by
     *  (odIdx, sub) against @c m_pending so we only react to the reads
     *  we asked for. */
    void onCustomSdoDone(int slaveIdx, bool isWrite,
                          quint16 odIdx, quint8 sub,
                          bool ok, QString valueDecoded, QString message);

    /** @brief Fire off the read sequence:
     *    1. 0x1001:00 (u8)  — error register bitmask.
     *    2. 0x603F:00 (u16) — current error code.
     *    3. 0x1003:00 (u8)  — fault-history count; on completion, we
     *                        follow up with reads for each sub-index.
     *  Idempotent: buttons disable while a refresh is in flight. */
    void refresh();

    /** @brief Write 0x1003:00 = 0 (u8) to reset the fault history per
     *  CiA-301. Refreshes on completion. */
    void clearHistory();

private:
    /** @brief Rebuild the 8-bit register display from the latest
     *  0x1001 read. */
    void renderErrorRegister(quint8 bits);
    /** @brief Update the current-error-code label from the latest
     *  0x603F read. */
    void renderCurrentErrorCode(quint16 code);
    /** @brief Append a row to the history table for one 0x1003 entry. */
    void renderHistoryRow(int slot, quint32 rawCode);
    /** @brief Enable / disable action buttons based on refresh state. */
    void updateBusyState();

    /** @brief Parse @p hex into a u32. Empty / malformed → 0. */
    static quint32 parseHex(const QString& hex);

    int      m_slaveIdx = -1;
    QString  m_slaveName;

    /* Pending-read tracking. Key = (odIdx << 8) | sub. */
    QSet<quint32>  m_pending;
    int            m_historyLeft = 0;

    /* Widgets. */
    QLabel*       m_headerLabel  = nullptr;
    QPushButton*  m_refreshBtn   = nullptr;
    QPushButton*  m_clearBtn     = nullptr;

    QVector<QLabel*>  m_regBits;
    QLabel*           m_regHex       = nullptr;

    QLabel*       m_curCodeLabel = nullptr;

    QTableWidget* m_histTable    = nullptr;

    QLabel*       m_emcyNote     = nullptr;
};

}  // namespace vrmc
