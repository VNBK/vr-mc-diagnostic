#include "PdoMappingEditor.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace vrmc {

namespace {

/* Standard CiA 402 object catalog, filtered to PDO-mappable entries.
 * Each tuple is (idx, sub, bits, display name, direction hint). The
 * "direction" just drives the default visibility when the editor opens,
 * but all entries are legal in either PDO; the catalog doesn't enforce
 * it. */
struct CatalogEntry {
    uint16_t idx;
    uint8_t  sub;
    uint8_t  bits;
    const char* name;
    bool     defaultTpdo;   /* true → TPDO-flavoured, false → RPDO */
};

static const CatalogEntry kCatalog[] = {
    /* RPDO-flavoured (master writes → slave consumes). */
    { 0x6040, 0, 16, "Controlword",       false },
    { 0x6060, 0,  8, "Modes of operation",false },
    { 0x607A, 0, 32, "Target position",   false },
    { 0x60FF, 0, 32, "Target velocity",   false },
    { 0x6071, 0, 16, "Target torque",     false },

    /* TPDO-flavoured (slave produces → master consumes). */
    { 0x6041, 0, 16, "Statusword",        true  },
    { 0x6061, 0,  8, "Modes of op display",true },
    { 0x6064, 0, 32, "Position actual",   true  },
    { 0x606C, 0, 32, "Velocity actual",   true  },
    { 0x6077, 0, 16, "Torque actual",     true  },
    { 0x6078, 0, 16, "Current actual",    true  },
    { 0x6079, 0, 32, "DC link voltage",   true  },
    { 0x603F, 0, 16, "Error code",        true  },
};

constexpr int kCatalogSize = int(sizeof(kCatalog) / sizeof(kCatalog[0]));

}  // namespace


PdoMappingEditor::PdoMappingEditor(bool isTpdo, QWidget* parent)
    : QDialog(parent), m_isTpdo(isTpdo)
{
    setWindowTitle(isTpdo ? tr("Edit TPDO1 mapping (slave → master)")
                          : tr("Edit RPDO1 mapping (master → slave)"));
    resize(560, 420);

    auto* intro = new QLabel(
        tr("Each row maps one object-dictionary entry into the PDO in the "
           "listed order. Bit-length must be a multiple of 8, and the sum "
           "of bytes must fit a CAN-FD frame (64 B)."), this);
    intro->setWordWrap(true);

    m_table = new QTableWidget(0, 4, this);
    m_table->setHorizontalHeaderLabels(
        { tr("Index"), tr("Sub"), tr("Bits"), tr("Name") });
    m_table->verticalHeader()->setVisible(false);
    m_table->setEditTriggers(QAbstractItemView::DoubleClicked |
                             QAbstractItemView::SelectedClicked);
    m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_table->setSelectionMode(QAbstractItemView::SingleSelection);
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

    m_catalog   = new QComboBox(this);
    m_addBtn    = new QPushButton(tr("Add"),      this);
    m_removeBtn = new QPushButton(tr("Remove"),   this);
    m_upBtn     = new QPushButton(tr("Up"),       this);
    m_downBtn   = new QPushButton(tr("Down"),     this);
    populateCatalog();

    connect(m_addBtn,    &QPushButton::clicked, this, &PdoMappingEditor::onAddFromCatalog);
    connect(m_removeBtn, &QPushButton::clicked, this, &PdoMappingEditor::onRemoveSelected);
    connect(m_upBtn,     &QPushButton::clicked, this, &PdoMappingEditor::onMoveUp);
    connect(m_downBtn,   &QPushButton::clicked, this, &PdoMappingEditor::onMoveDown);

    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(new QLabel(tr("Catalog:"), this));
    btnRow->addWidget(m_catalog, 1);
    btnRow->addWidget(m_addBtn);
    btnRow->addWidget(m_removeBtn);
    btnRow->addWidget(m_upBtn);
    btnRow->addWidget(m_downBtn);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* root = new QVBoxLayout(this);
    root->addWidget(intro);
    root->addWidget(m_table, 1);
    root->addLayout(btnRow);
    root->addWidget(buttons);
}

void PdoMappingEditor::populateCatalog()
{
    /* List the direction-matching entries first for convenience, but
     * include all catalog items so nothing is hidden. */
    m_catalog->clear();
    int catalogIdx = 0;
    for (int pass = 0; pass < 2; ++pass){
        const bool wantTpdoFirst = (pass == 0) == m_isTpdo;
        for (int i = 0; i < kCatalogSize; ++i){
            const bool isTpdoEntry = kCatalog[i].defaultTpdo;
            if (isTpdoEntry != wantTpdoFirst){ continue; }
            const QString label = QStringLiteral("0x%1 · %2  (%3 bit)")
                .arg(kCatalog[i].idx, 4, 16, QChar('0')).toUpper()
                .replace("0X", "0x")
                + QStringLiteral("  ")
                + QString::fromUtf8(kCatalog[i].name)
                + QStringLiteral(" [%1 bit]").arg(kCatalog[i].bits);
            (void)label;
            const QString display = QStringLiteral("0x%1/%2  %3  [%4 bit]")
                .arg(kCatalog[i].idx, 4, 16, QChar('0'))
                .arg(kCatalog[i].sub)
                .arg(QString::fromUtf8(kCatalog[i].name))
                .arg(kCatalog[i].bits);
            m_catalog->addItem(display, QVariant(i));
            ++catalogIdx;
        }
    }
}

uint16_t PdoMappingEditor::catalogRowIdx(int catalogIdx) const
{
    const int i = m_catalog->itemData(catalogIdx).toInt();
    return (i >= 0 && i < kCatalogSize) ? kCatalog[i].idx : 0;
}

uint8_t PdoMappingEditor::catalogRowSub(int catalogIdx) const
{
    const int i = m_catalog->itemData(catalogIdx).toInt();
    return (i >= 0 && i < kCatalogSize) ? kCatalog[i].sub : 0;
}

int PdoMappingEditor::catalogRowBits(int catalogIdx) const
{
    const int i = m_catalog->itemData(catalogIdx).toInt();
    return (i >= 0 && i < kCatalogSize) ? kCatalog[i].bits : 0;
}

QString PdoMappingEditor::catalogRowName(int catalogIdx) const
{
    const int i = m_catalog->itemData(catalogIdx).toInt();
    return (i >= 0 && i < kCatalogSize)
               ? QString::fromUtf8(kCatalog[i].name)
               : QString();
}

void PdoMappingEditor::addRow(uint16_t idx, uint8_t sub, uint8_t bits,
                              const QString& name)
{
    const int row = m_table->rowCount();
    m_table->insertRow(row);
    auto put = [&](int col, const QString& text, bool editable = true){
        auto* it = new QTableWidgetItem(text);
        if (!editable){
            it->setFlags(it->flags() & ~Qt::ItemIsEditable);
        }
        m_table->setItem(row, col, it);
    };
    put(0, QStringLiteral("0x%1").arg(idx, 4, 16, QChar('0')));
    put(1, QString::number(sub));
    put(2, QString::number(bits));
    put(3, name, /*editable=*/false);
}

void PdoMappingEditor::onAddFromCatalog()
{
    const int sel = m_catalog->currentIndex();
    if (sel < 0){ return; }
    addRow(catalogRowIdx(sel), catalogRowSub(sel),
           uint8_t(catalogRowBits(sel)), catalogRowName(sel));
    m_table->selectRow(m_table->rowCount() - 1);
}

void PdoMappingEditor::onRemoveSelected()
{
    const int r = m_table->currentRow();
    if (r < 0){ return; }
    m_table->removeRow(r);
}

void PdoMappingEditor::onMoveUp()
{
    const int r = m_table->currentRow();
    if (r <= 0){ return; }
    /* Swap r-1 <-> r by taking items and putting them back. */
    for (int c = 0; c < m_table->columnCount(); ++c){
        auto* a = m_table->takeItem(r - 1, c);
        auto* b = m_table->takeItem(r,     c);
        m_table->setItem(r - 1, c, b);
        m_table->setItem(r,     c, a);
    }
    m_table->selectRow(r - 1);
}

void PdoMappingEditor::onMoveDown()
{
    const int r = m_table->currentRow();
    if (r < 0 || r + 1 >= m_table->rowCount()){ return; }
    for (int c = 0; c < m_table->columnCount(); ++c){
        auto* a = m_table->takeItem(r,     c);
        auto* b = m_table->takeItem(r + 1, c);
        m_table->setItem(r,     c, b);
        m_table->setItem(r + 1, c, a);
    }
    m_table->selectRow(r + 1);
}

void PdoMappingEditor::setEntries(const QVector<CanBackend::PdoMapEntry>& entries)
{
    m_table->setRowCount(0);
    for (const auto& e : entries){
        /* Try to resolve the human-readable name from the catalog. */
        QString name;
        for (int i = 0; i < kCatalogSize; ++i){
            if (kCatalog[i].idx == e.idx && kCatalog[i].sub == e.sub){
                name = QString::fromUtf8(kCatalog[i].name);
                break;
            }
        }
        addRow(e.idx, e.sub, e.bits, name);
    }
}

QVector<CanBackend::PdoMapEntry> PdoMappingEditor::entries() const
{
    QVector<CanBackend::PdoMapEntry> out;
    out.reserve(m_table->rowCount());
    for (int r = 0; r < m_table->rowCount(); ++r){
        auto* it0 = m_table->item(r, 0);
        auto* it1 = m_table->item(r, 1);
        auto* it2 = m_table->item(r, 2);
        if (!it0 || !it1 || !it2){ continue; }
        bool ok0 = false, ok1 = false, ok2 = false;
        /* Accept "0x6041" or "6041" for the index; sub + bits decimal. */
        QString sIdx = it0->text().trimmed();
        if (sIdx.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){
            sIdx = sIdx.mid(2);
        }
        const uint16_t idx  = uint16_t(sIdx.toUInt(&ok0, 16));
        const uint8_t  sub  = uint8_t (it1->text().toUInt(&ok1, 10));
        const uint8_t  bits = uint8_t (it2->text().toUInt(&ok2, 10));
        if (!ok0 || !ok1 || !ok2){ continue; }
        out.push_back({ idx, sub, bits });
    }
    return out;
}

}  // namespace vrmc
