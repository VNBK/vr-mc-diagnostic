#include "PdoMappingView.hpp"

#include "PdoMappingEditor.hpp"

#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include <cmath>

namespace vrmc {

namespace {

/* Catalog of CiA 402 objects the view knows how to decode. When a
 * mapped entry matches one of these indices the Value column shows the
 * decoded wire value; otherwise it stays as "—". */
struct KnownEntry {
    uint16_t    idx;
    const char* name;
};

static const KnownEntry kKnown[] = {
    { 0x6040, "Controlword"          },
    { 0x6041, "Statusword"           },
    { 0x603F, "Error code"           },
    { 0x6060, "Modes of operation"   },
    { 0x6061, "Modes of op display"  },
    { 0x607A, "Target position"      },
    { 0x6064, "Position actual"      },
    { 0x60FF, "Target velocity"      },
    { 0x606C, "Velocity actual"      },
    { 0x6071, "Target torque"        },
    { 0x6077, "Torque actual"        },
    { 0x6078, "Current actual"       },
    { 0x6079, "DC link voltage"      },
};

QString nameForIndex(uint16_t idx)
{
    for (const auto& k : kKnown){
        if (k.idx == idx){ return QString::fromUtf8(k.name); }
    }
    return QStringLiteral("(unknown)");
}

/* Wire conversions must match MasterWorker's CiA 402 scaling. */
constexpr double kCountsPerRev  = 16384.0;
constexpr double kRadPerCount   = (2.0 * M_PI) / kCountsPerRev;
constexpr double kRatedTorqueNm = 0.5;

int tableBodyHeight(int rows)
{
    return 28 * std::max(1, rows) + 28;
}

void setCell(QTableWidget* t, int row, int col,
             const QString& text, bool rightAlign = false)
{
    auto* it = t->item(row, col);
    if (!it){
        it = new QTableWidgetItem(text);
        t->setItem(row, col, it);
    } else {
        it->setText(text);
    }
    if (rightAlign){
        it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    }
}

int totalBytes(const QVector<CanBackend::PdoMapEntry>& entries)
{
    int n = 0;
    for (const auto& e : entries){ n += e.bits / 8; }
    return n;
}

}  // namespace


PdoMappingView::PdoMappingView(QWidget* parent) : QWidget(parent)
{
    /* CiA 402 defaults (matches reconfigure_default_pdos in the sim). */
    m_tpdoEntries = {
        { 0x6041, 0, 16 },
        { 0x6064, 0, 32 },
        { 0x606C, 0, 32 },
        { 0x6077, 0, 16 },
        { 0x603F, 0, 16 },
    };
    m_rpdoEntries = {
        { 0x6040, 0, 16 },
        { 0x607A, 0, 32 },
        { 0x60FF, 0, 32 },
        { 0x6071, 0, 16 },
    };

    auto* tpdoBox = new QGroupBox(tr("TPDO1 — slave → master"), this);
    m_tpdoHeader  = new QLabel(tpdoBox);
    m_tpdoHeader->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_tpdo        = makeTable(m_tpdoEntries.size());
    auto* tpdoEditBtn = new QPushButton(tr("Edit…"), tpdoBox);
    connect(tpdoEditBtn, &QPushButton::clicked,
            this,        &PdoMappingView::onEditTpdo);
    {
        auto* l = new QVBoxLayout(tpdoBox);
        l->addWidget(m_tpdoHeader);
        l->addWidget(m_tpdo);
        auto* row = new QHBoxLayout;
        row->addStretch(1);
        row->addWidget(tpdoEditBtn);
        l->addLayout(row);
    }

    auto* rpdoBox = new QGroupBox(tr("RPDO1 — master → slave"), this);
    m_rpdoHeader  = new QLabel(rpdoBox);
    m_rpdoHeader->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_rpdo        = makeTable(m_rpdoEntries.size());
    auto* rpdoEditBtn = new QPushButton(tr("Edit…"), rpdoBox);
    connect(rpdoEditBtn, &QPushButton::clicked,
            this,        &PdoMappingView::onEditRpdo);
    {
        auto* l = new QVBoxLayout(rpdoBox);
        l->addWidget(m_rpdoHeader);
        l->addWidget(m_rpdo);
        auto* row = new QHBoxLayout;
        row->addStretch(1);
        row->addWidget(rpdoEditBtn);
        l->addLayout(row);
    }

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(6, 6, 6, 6);
    root->addWidget(tpdoBox);
    root->addWidget(rpdoBox);
    root->addStretch(1);

    buildTpdoTable();
    buildRpdoTable();
    refreshHeaders(nullptr);
    refreshValues(nullptr);
}

QTableWidget* PdoMappingView::makeTable(int rows)
{
    auto* t = new QTableWidget(rows, 6, this);
    t->setHorizontalHeaderLabels(
        { tr("Ofs"), tr("Index"), tr("Sub"), tr("Bits"),
          tr("Name"), tr("Value") });
    t->verticalHeader()->setVisible(false);
    t->setEditTriggers(QAbstractItemView::NoEditTriggers);
    t->setSelectionMode(QAbstractItemView::NoSelection);
    t->setFocusPolicy(Qt::NoFocus);
    t->setAlternatingRowColors(true);
    t->horizontalHeader()->setStretchLastSection(true);
    t->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    t->horizontalHeader()->setSectionResizeMode(4, QHeaderView::ResizeToContents);
    t->setFixedHeight(tableBodyHeight(rows));
    return t;
}

static void fillRows(QTableWidget* t,
                     const QVector<CanBackend::PdoMapEntry>& entries)
{
    t->setRowCount(entries.size());
    int offset = 0;
    for (int i = 0; i < entries.size(); ++i){
        const auto& e = entries[i];
        setCell(t, i, 0, QString::number(offset),                /*right*/ true);
        setCell(t, i, 1, QStringLiteral("0x%1")
                             .arg(e.idx, 4, 16, QChar('0'))
                             .toUpper().replace("0X", "0x"));
        setCell(t, i, 2, QString::number(e.sub));
        setCell(t, i, 3, QString::number(e.bits),                /*right*/ true);
        setCell(t, i, 4, nameForIndex(e.idx));
        setCell(t, i, 5, QStringLiteral("—"),                    /*right*/ true);
        offset += e.bits / 8;
    }
    t->setFixedHeight(tableBodyHeight(entries.size()));
}

void PdoMappingView::buildTpdoTable() { fillRows(m_tpdo, m_tpdoEntries); }
void PdoMappingView::buildRpdoTable() { fillRows(m_rpdo, m_rpdoEntries); }

void PdoMappingView::onEditTpdo()
{
    if (m_activeIdx < 0){ return; }
    PdoMappingEditor dlg(/*isTpdo=*/true, this);
    dlg.setEntries(m_tpdoEntries);
    if (dlg.exec() == QDialog::Accepted){
        m_tpdoEntries = dlg.entries();
        buildTpdoTable();
        emit applyRequested(m_activeIdx, /*isTpdo=*/true, m_tpdoEntries);
    }
}

void PdoMappingView::onEditRpdo()
{
    if (m_activeIdx < 0){ return; }
    PdoMappingEditor dlg(/*isTpdo=*/false, this);
    dlg.setEntries(m_rpdoEntries);
    if (dlg.exec() == QDialog::Accepted){
        m_rpdoEntries = dlg.entries();
        buildRpdoTable();
        emit applyRequested(m_activeIdx, /*isTpdo=*/false, m_rpdoEntries);
    }
}

void PdoMappingView::setActiveSlave(int idx, const QString& name)
{
    m_activeIdx  = idx;
    m_activeName = name;
    refreshHeaders(nullptr);
    refreshValues(nullptr);
}

void PdoMappingView::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    const SlaveSnapshot* sel = nullptr;
    if (m_activeIdx >= 0){
        for (const auto& s : snaps){
            if (s.idx == m_activeIdx){ sel = &s; break; }
        }
    }
    refreshHeaders(sel);
    refreshValues(sel);
}

void PdoMappingView::refreshHeaders(const SlaveSnapshot* s)
{
    const int tpdoBytes = totalBytes(m_tpdoEntries);
    const int rpdoBytes = totalBytes(m_rpdoEntries);
    auto tpdoConf = QStringLiteral("COB-ID %1 · trans 254 (async-mfr) · %2 B");
    auto rpdoConf = QStringLiteral("COB-ID %1 · trans 254 (async-mfr) · %2 B");
    QString tpdoCob = QStringLiteral("0x180+id");
    QString rpdoCob = QStringLiteral("0x200+id");
    QString live;
    if (s){
        tpdoCob = QStringLiteral("0x%1")
                      .arg(0x180u + s->id, 3, 16, QChar('0'));
        rpdoCob = QStringLiteral("0x%1")
                      .arg(0x200u + s->id, 3, 16, QChar('0'));
        live = s->pdoFresh
                 ? QStringLiteral("  |  rx=%1 · fresh").arg(qulonglong(s->pdoRxCount))
                 : QStringLiteral("  |  no PDO yet");
    }
    m_tpdoHeader->setText(tpdoConf.arg(tpdoCob).arg(tpdoBytes) + live);
    m_rpdoHeader->setText(rpdoConf.arg(rpdoCob).arg(rpdoBytes));
}

static QString decodeTpdoValue(uint16_t idx, const SlaveSnapshot* s)
{
    if (!s || !s->pdoFresh){ return QStringLiteral("—"); }
    auto rawPos = [](float rad) {
        return int32_t(std::round(rad / kRadPerCount));
    };
    auto rawVel = [](float rps) {
        return int32_t(std::round(rps / kRadPerCount));
    };
    auto rawTrq = [](float nm) {
        return int16_t(std::round(nm / kRatedTorqueNm * 1000.0));
    };
    switch (idx){
    case 0x6041: return QStringLiteral("0x%1")
                            .arg(s->statusword, 4, 16, QChar('0'))
                            .toUpper().replace("0X", "0x");
    case 0x603F: return QStringLiteral("0x%1")
                            .arg(s->errorCode, 4, 16, QChar('0'))
                            .toUpper().replace("0X", "0x");
    case 0x6064: return QString::number(rawPos(s->position));
    case 0x606C: return QString::number(rawVel(s->velocity));
    case 0x6077: return QString::number(rawTrq(s->torque));
    case 0x6078: return QString::number(double(s->current), 'f', 3);
    default:     return QStringLiteral("—");
    }
}

static QString decodeRpdoValue(uint16_t idx, const SlaveSnapshot* s)
{
    if (!s){ return QStringLiteral("—"); }
    auto rawPos = [](float rad) {
        return int32_t(std::round(rad / kRadPerCount));
    };
    auto rawVel = [](float rps) {
        return int32_t(std::round(rps / kRadPerCount));
    };
    auto rawTrq = [](float nm) {
        return int16_t(std::round(nm / kRatedTorqueNm * 1000.0));
    };
    switch (idx){
    case 0x607A: return QString::number(rawPos(s->cmdPosition));
    case 0x60FF: return QString::number(rawVel(s->cmdVelocity));
    case 0x6071: return QString::number(rawTrq(s->cmdTorque));
    default:     return QStringLiteral("—");
    }
}

void PdoMappingView::refreshValues(const SlaveSnapshot* s)
{
    for (int i = 0; i < m_tpdoEntries.size(); ++i){
        setCell(m_tpdo, i, 5,
                decodeTpdoValue(m_tpdoEntries[i].idx, s), /*right*/ true);
    }
    for (int i = 0; i < m_rpdoEntries.size(); ++i){
        setCell(m_rpdo, i, 5,
                decodeRpdoValue(m_rpdoEntries[i].idx, s), /*right*/ true);
    }
}

}  // namespace vrmc
