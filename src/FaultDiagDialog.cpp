#include "FaultDiagDialog.hpp"
#include "JointControlPanel.hpp"    /* decodeErrorCode */

#include <QDialogButtonBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>


namespace vrmc {

namespace {

/* CiA-301 §7.5.5 — Error Register bit names, ordered from LSB (bit 0)
 * to MSB (bit 7). Each shows up as its own indicator in the top row. */
const char* kRegBitNames[8] = {
    "generic",
    "current",
    "voltage",
    "temperature",
    "communication",
    "device profile",
    "reserved",
    "manufacturer",
};

QLabel* makeBitLed(QWidget* parent, const char* label)
{
    auto* w = new QLabel(parent);
    w->setAlignment(Qt::AlignCenter);
    w->setMinimumWidth(90);
    w->setText(QStringLiteral("<b>%1</b>").arg(QString::fromLatin1(label)));
    /* Start in the "clear" state (grey). renderErrorRegister repaints
     * each frame based on the read bitmask. */
    w->setStyleSheet("QLabel { background: #eeeeee; color: #888; "
                     "border: 1px solid #ccc; border-radius: 3px; "
                     "padding: 6px 8px; }");
    return w;
}

}  // namespace


FaultDiagDialog::FaultDiagDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Fault diagnostics"));
    setModal(false);
    resize(720, 560);

    auto* root = new QVBoxLayout(this);

    m_headerLabel = new QLabel(tr("<i>No slave selected.</i>"), this);
    m_headerLabel->setStyleSheet("QLabel { padding: 6px; "
                                  "background: #f4f4f4; "
                                  "border: 1px solid #ddd; }");
    root->addWidget(m_headerLabel);

    auto* btnRow = new QHBoxLayout();
    m_refreshBtn = new QPushButton(tr("Refresh"), this);
    m_refreshBtn->setToolTip(tr("Read 0x1001, 0x603F, and the 0x1003 "
        "history from the selected slave via SDO."));
    m_clearBtn   = new QPushButton(tr("Clear history"), this);
    m_clearBtn->setToolTip(tr("Write 0x1003:00 = 0 to erase the drive's "
        "predefined error field (CiA-301 §7.5.6)."));
    connect(m_refreshBtn, &QPushButton::clicked, this, &FaultDiagDialog::refresh);
    connect(m_clearBtn,   &QPushButton::clicked, this, &FaultDiagDialog::clearHistory);
    btnRow->addWidget(m_refreshBtn);
    btnRow->addWidget(m_clearBtn);
    btnRow->addStretch(1);
    root->addLayout(btnRow);

    /* ---- Error Register (0x1001) ------------------------------ */
    {
        auto* box = new QGroupBox(tr("Error Register (0x1001)"), this);
        auto* v = new QVBoxLayout(box);
        auto* row = new QHBoxLayout();
        for (int i = 0; i < 8; ++i){
            auto* led = makeBitLed(box, kRegBitNames[i]);
            m_regBits.append(led);
            row->addWidget(led);
        }
        v->addLayout(row);
        m_regHex = new QLabel(tr("Raw: —"), box);
        m_regHex->setStyleSheet("QLabel { color: #555; padding: 2px 4px; }");
        v->addWidget(m_regHex);
        root->addWidget(box);
    }

    /* ---- Current Error Code (0x603F) -------------------------- */
    {
        auto* box = new QGroupBox(tr("Current Error Code (0x603F)"), this);
        auto* v = new QVBoxLayout(box);
        m_curCodeLabel = new QLabel(tr("—"), box);
        m_curCodeLabel->setStyleSheet("QLabel { font-family: monospace; "
                                       "font-size: 11pt; padding: 4px; }");
        v->addWidget(m_curCodeLabel);
        root->addWidget(box);
    }

    /* ---- Fault History (0x1003) ------------------------------- */
    {
        auto* box = new QGroupBox(tr("Predefined Error Field (0x1003) — "
                                      "most-recent first"), this);
        auto* v = new QVBoxLayout(box);
        m_histTable = new QTableWidget(0, 4, box);
        m_histTable->setHorizontalHeaderLabels(
            {tr("#"), tr("Raw"), tr("CiA-402 code"), tr("Vendor info")});
        m_histTable->verticalHeader()->setVisible(false);
        m_histTable->horizontalHeader()->setSectionResizeMode(
            0, QHeaderView::ResizeToContents);
        m_histTable->horizontalHeader()->setSectionResizeMode(
            1, QHeaderView::ResizeToContents);
        m_histTable->horizontalHeader()->setSectionResizeMode(
            2, QHeaderView::Stretch);
        m_histTable->horizontalHeader()->setSectionResizeMode(
            3, QHeaderView::ResizeToContents);
        m_histTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_histTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_histTable->setAlternatingRowColors(true);
        v->addWidget(m_histTable);
        root->addWidget(box, /*stretch=*/1);
    }

    /* ---- EMCY note ------------------------------------------- */
    m_emcyNote = new QLabel(tr(
        "<i>Note: live EMCY frame capture is a follow-up item — needs a "
        "backend-level RX callback that the current CanBackend doesn't "
        "expose. Track the fault history table above for post-hoc "
        "analysis.</i>"), this);
    m_emcyNote->setWordWrap(true);
    m_emcyNote->setStyleSheet("QLabel { color: #5a3d00; "
                                "background: #fff8e1; "
                                "border: 1px solid #f9a825; "
                                "padding: 6px; }");
    root->addWidget(m_emcyNote);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);

    updateBusyState();
}


void FaultDiagDialog::setSlaveContext(int slaveIdx, const QString& name)
{
    m_slaveIdx  = slaveIdx;
    m_slaveName = name;
    m_headerLabel->setText(tr("Slave <b>#%1</b> — %2")
                              .arg(slaveIdx).arg(name.isEmpty() ? tr("?") : name));
    /* Fresh slave context → wipe stale reads. */
    m_pending.clear();
    m_historyLeft = 0;
    for (auto* led : m_regBits){
        led->setStyleSheet("QLabel { background: #eeeeee; color: #888; "
                           "border: 1px solid #ccc; border-radius: 3px; "
                           "padding: 6px 8px; }");
    }
    m_regHex->setText(tr("Raw: —"));
    m_curCodeLabel->setText(tr("—"));
    m_histTable->setRowCount(0);
    updateBusyState();
}


void FaultDiagDialog::updateBusyState()
{
    const bool busy    = !m_pending.isEmpty() || m_historyLeft > 0;
    const bool haveIdx = (m_slaveIdx >= 0);
    m_refreshBtn->setEnabled(haveIdx && !busy);
    m_clearBtn  ->setEnabled(haveIdx && !busy);
}


void FaultDiagDialog::refresh()
{
    if (m_slaveIdx < 0){ return; }
    m_pending.clear();
    m_historyLeft = 0;
    m_histTable->setRowCount(0);
    /* Queue three initial reads. Follow-up 0x1003 sub-1..N reads are
     * fired once we know the count. */
    m_pending.insert((quint32(0x1001) << 8) | 0x00);
    m_pending.insert((quint32(0x603F) << 8) | 0x00);
    m_pending.insert((quint32(0x1003) << 8) | 0x00);
    updateBusyState();
    emit sdoReadRequested(m_slaveIdx, 0x1001, 0x00, 1);
    emit sdoReadRequested(m_slaveIdx, 0x603F, 0x00, 2);
    emit sdoReadRequested(m_slaveIdx, 0x1003, 0x00, 1);
}


void FaultDiagDialog::clearHistory()
{
    if (m_slaveIdx < 0){ return; }
    /* Track the write so onCustomSdoDone knows to trigger a refresh. */
    m_pending.insert((quint32(0x1003) << 8) | 0x00 | 0x00010000u);
    /* Note: writes and reads to the same (odIdx, sub) are distinguished
     * by the customSdoDone `isWrite` flag; we still track via m_pending
     * to gate the busy state. */
    QByteArray payload(1, char(0));    /* u8 = 0 per CiA-301 §7.5.6 */
    updateBusyState();
    emit sdoWriteRequested(m_slaveIdx, 0x1003, 0x00, payload);
}


quint32 FaultDiagDialog::parseHex(const QString& hex)
{
    if (hex.isEmpty()){ return 0; }
    QString s = hex;
    if (s.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){
        s = s.mid(2);
    }
    bool ok = false;
    const quint32 v = s.toUInt(&ok, 16);
    return ok ? v : 0;
}


void FaultDiagDialog::renderErrorRegister(quint8 bits)
{
    for (int i = 0; i < 8 && i < m_regBits.size(); ++i){
        const bool on = (bits & (1u << i)) != 0;
        if (on){
            m_regBits[i]->setStyleSheet(
                "QLabel { background: #ffe5e5; color: #b71c1c; "
                "border: 1px solid #b71c1c; border-radius: 3px; "
                "padding: 6px 8px; font-weight: bold; }");
        } else {
            m_regBits[i]->setStyleSheet(
                "QLabel { background: #eeeeee; color: #888; "
                "border: 1px solid #ccc; border-radius: 3px; "
                "padding: 6px 8px; }");
        }
    }
    m_regHex->setText(tr("Raw: 0x%1  (%2)")
                          .arg(bits, 2, 16, QChar('0'))
                          .arg(bits == 0 ? tr("no faults latched")
                                          : tr("%n fault bit(s) set", nullptr,
                                                __builtin_popcount(bits))));
}


void FaultDiagDialog::renderCurrentErrorCode(quint16 code)
{
    if (code == 0){
        m_curCodeLabel->setText(tr("<b>0x0000</b>  no active error"));
        m_curCodeLabel->setStyleSheet("QLabel { font-family: monospace; "
            "font-size: 11pt; padding: 4px; color: #2e7d32; }");
    } else {
        const QString decoded = JointControlPanel::decodeErrorCode(code);
        m_curCodeLabel->setText(QStringLiteral("<b>%1</b>")
                                     .arg(decoded.isEmpty()
                                             ? QStringLiteral("0x%1")
                                                   .arg(code, 4, 16, QChar('0'))
                                             : decoded));
        m_curCodeLabel->setStyleSheet("QLabel { font-family: monospace; "
            "font-size: 11pt; padding: 4px; color: #b71c1c; }");
    }
}


void FaultDiagDialog::renderHistoryRow(int slot, quint32 rawCode)
{
    const int row = m_histTable->rowCount();
    m_histTable->insertRow(row);
    /* 0x1003 semantics: sub-1 is the MOST RECENT error. Show the sub
     * index in column 0 so the operator sees the CiA-301 slot numbering
     * directly rather than a runtime row index. */
    m_histTable->setItem(row, 0, new QTableWidgetItem(QString::number(slot)));
    m_histTable->setItem(row, 1,
        new QTableWidgetItem(QStringLiteral("0x%1")
            .arg(rawCode, 8, 16, QChar('0'))));
    const quint16 cia   = quint16(rawCode & 0xFFFFu);
    const quint16 vendor = quint16((rawCode >> 16) & 0xFFFFu);
    m_histTable->setItem(row, 2,
        new QTableWidgetItem(cia == 0 ? tr("—")
                                       : JointControlPanel::decodeErrorCode(cia)));
    m_histTable->setItem(row, 3,
        new QTableWidgetItem(QStringLiteral("0x%1")
            .arg(vendor, 4, 16, QChar('0'))));
}


void FaultDiagDialog::onCustomSdoDone(int slaveIdx, bool isWrite,
                                       quint16 odIdx, quint8 sub,
                                       bool ok, QString valueDecoded,
                                       QString message)
{
    if (slaveIdx != m_slaveIdx){ return; }
    const quint32 key       = (quint32(odIdx) << 8) | sub;
    const quint32 writeKey  = key | 0x00010000u;

    if (isWrite){
        /* Clear-history write: on success, trigger a refresh so the
         * empty history is visible; on failure, just log via header. */
        if (m_pending.contains(writeKey)){
            m_pending.remove(writeKey);
            updateBusyState();
            if (ok){
                refresh();
            } else {
                m_headerLabel->setText(m_headerLabel->text() +
                    tr("  <span style='color:#c62828;'>[clear failed: "
                       "%1]</span>").arg(message));
            }
        }
        return;
    }

    /* Read path — dispatch on which OD entry replied. Sub-index reads
     * for 0x1003 arrive with sub in 1..N and aren't in m_pending; they
     * decrement m_historyLeft instead. */
    if (m_pending.contains(key)){
        m_pending.remove(key);
        const quint32 raw = ok ? parseHex(valueDecoded) : 0;
        if (odIdx == 0x1001 && sub == 0x00){
            renderErrorRegister(ok ? quint8(raw & 0xFFu) : 0);
        } else if (odIdx == 0x603F && sub == 0x00){
            renderCurrentErrorCode(ok ? quint16(raw & 0xFFFFu) : 0);
        } else if (odIdx == 0x1003 && sub == 0x00){
            /* Count landed — issue a read per sub-index. Clamp to a
             * sanity ceiling (CiA-301 allows up to 254). */
            const int count = ok ? int(raw & 0xFFu) : 0;
            const int capped = qMin(count, 32);
            m_historyLeft = capped;
            for (int i = 1; i <= capped; ++i){
                emit sdoReadRequested(m_slaveIdx, 0x1003, quint8(i), 4);
            }
        }
    } else if (odIdx == 0x1003 && sub > 0){
        if (ok){ renderHistoryRow(sub, parseHex(valueDecoded)); }
        if (m_historyLeft > 0){ --m_historyLeft; }
    }
    updateBusyState();
}

}  // namespace vrmc
