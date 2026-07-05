#include "ObjectDictionaryDialog.hpp"

#include <QComboBox>
#include <QDateTime>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QStandardPaths>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>


namespace vrmc {

namespace {

/* Compact-encoding key for (index, sub). Same shape as FaultDiagDialog
 * uses so both dialogs speak the same key format when tracking pending
 * reads. */
inline quint32 keyFor(quint16 idx, quint8 sub){
    return (quint32(idx) << 8) | sub;
}

/* CiA-301 category label the tree uses to group entries. Keys pin the
 * ordering the tree renders. */
constexpr const char* kCatComm    = "Communication (0x1xxx)";
constexpr const char* kCatDrive   = "Drive Profile (0x6xxx)";
constexpr const char* kCatMfr     = "Manufacturer (0x2xxx)";

}  // namespace


ObjectDictionaryDialog::ObjectDictionaryDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Object Dictionary browser"));
    setModal(false);
    resize(1000, 700);

    auto* root = new QVBoxLayout(this);

    m_headerLabel = new QLabel(tr("<i>No slave selected.</i>"), this);
    m_headerLabel->setStyleSheet("QLabel { padding: 6px; "
                                  "background: #f4f4f4; "
                                  "border: 1px solid #ddd; }");
    root->addWidget(m_headerLabel);

    /* --- Split top: tree | details --------------------------------- */
    auto* topSplit = new QSplitter(Qt::Horizontal, this);

    m_tree = new QTreeWidget(topSplit);
    m_tree->setHeaderLabels({tr("Object"), tr("Type"), tr("Access")});
    m_tree->setColumnWidth(0, 320);
    m_tree->setColumnWidth(1, 60);
    m_tree->setAlternatingRowColors(true);
    connect(m_tree, &QTreeWidget::itemSelectionChanged,
            this, &ObjectDictionaryDialog::onTreeSelectionChanged);

    auto* detailsBox = new QWidget(topSplit);
    auto* detailsLay = new QVBoxLayout(detailsBox);
    m_detailsLabel = new QLabel(tr("Select an entry on the left."), detailsBox);
    m_detailsLabel->setWordWrap(true);
    m_detailsLabel->setStyleSheet("QLabel { padding: 8px; "
                                    "background: #fafafa; "
                                    "border: 1px solid #ddd; }");
    detailsLay->addWidget(m_detailsLabel);

    m_valueLabel = new QLabel(tr("Value: —"), detailsBox);
    m_valueLabel->setStyleSheet("QLabel { font-family: monospace; "
                                  "font-size: 11pt; padding: 8px; }");
    detailsLay->addWidget(m_valueLabel);

    auto* btnRow = new QHBoxLayout();
    m_readBtn     = new QPushButton(tr("Read"), detailsBox);
    m_readAllBtn  = new QPushButton(tr("Read all"), detailsBox);
    m_addWatchBtn = new QPushButton(tr("Add to watch"), detailsBox);
    m_readAllBtn->setToolTip(tr("Read every entry in the catalog. Rate-"
        "limited to one SDO in flight at a time."));
    connect(m_readBtn,     &QPushButton::clicked, this, &ObjectDictionaryDialog::onReadSelected);
    connect(m_readAllBtn,  &QPushButton::clicked, this, &ObjectDictionaryDialog::onReadAll);
    connect(m_addWatchBtn, &QPushButton::clicked, this, &ObjectDictionaryDialog::onAddToWatch);
    btnRow->addWidget(m_readBtn);
    btnRow->addWidget(m_readAllBtn);
    btnRow->addWidget(m_addWatchBtn);
    btnRow->addStretch(1);
    detailsLay->addLayout(btnRow);

    /* --- Write row (visible only for RW entries) ---------------- */
    m_writeRow  = new QWidget(detailsBox);
    auto* wLine = new QHBoxLayout(m_writeRow);
    wLine->setContentsMargins(0, 8, 0, 0);
    wLine->addWidget(new QLabel(tr("Write:"), m_writeRow));
    m_writeInput = new QLineEdit(m_writeRow);
    m_writeInput->setPlaceholderText(tr("decimal or 0x-hex"));
    m_writeInput->setToolTip(tr("Value to write. Signed types accept "
        "negative decimals; hex values are unsigned and get sign-"
        "extended to the entry's declared width."));
    m_writeBtn   = new QPushButton(tr("Write"), m_writeRow);
    connect(m_writeInput, &QLineEdit::returnPressed,
            this, &ObjectDictionaryDialog::onWriteSelected);
    connect(m_writeBtn,   &QPushButton::clicked,
            this, &ObjectDictionaryDialog::onWriteSelected);
    wLine->addWidget(m_writeInput, /*stretch=*/1);
    wLine->addWidget(m_writeBtn);
    detailsLay->addWidget(m_writeRow);
    m_writeHint = new QLabel(detailsBox);
    m_writeHint->setStyleSheet("QLabel { color: #888; padding: 2px 4px; }");
    m_writeHint->setWordWrap(true);
    m_writeHint->setVisible(false);
    detailsLay->addWidget(m_writeHint);
    m_writeRow->setVisible(false);   /* toggled by selection change */

    /* --- Import / Export ---------------------------------------- */
    auto* xmlRow = new QHBoxLayout();
    m_exportBtn = new QPushButton(tr("Export XML…"), detailsBox);
    m_importBtn = new QPushButton(tr("Import XML…"), detailsBox);
    m_exportBtn->setToolTip(tr("Dump the current catalog + last-read "
        "values to an ESI-style XML file for archiving / offline diff."));
    m_importBtn->setToolTip(tr("Load a catalog + values from an XML "
        "file. Replaces the in-memory catalog and rehydrates the "
        "last-read cache."));
    connect(m_exportBtn, &QPushButton::clicked, this, &ObjectDictionaryDialog::onExportXml);
    connect(m_importBtn, &QPushButton::clicked, this, &ObjectDictionaryDialog::onImportXml);
    xmlRow->addWidget(m_exportBtn);
    xmlRow->addWidget(m_importBtn);
    xmlRow->addStretch(1);
    detailsLay->addLayout(xmlRow);
    detailsLay->addStretch(1);

    topSplit->addWidget(m_tree);
    topSplit->addWidget(detailsBox);
    topSplit->setStretchFactor(0, 3);
    topSplit->setStretchFactor(1, 2);
    root->addWidget(topSplit, /*stretch=*/3);

    /* --- Watch list bottom ---------------------------------------- */
    {
        auto* box  = new QWidget(this);
        auto* wLay = new QVBoxLayout(box);
        auto* row  = new QHBoxLayout();
        row->addWidget(new QLabel(tr("Watch — poll rate:"), box));
        m_watchRate = new QComboBox(box);
        m_watchRate->addItem(tr("1 Hz"),  1000);
        m_watchRate->addItem(tr("2 Hz"),   500);
        m_watchRate->addItem(tr("5 Hz"),   200);
        m_watchRate->addItem(tr("10 Hz"),  100);
        row->addWidget(m_watchRate);
        connect(m_watchRate, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this](int){
                    if (m_watchTimer){
                        m_watchTimer->setInterval(
                            m_watchRate->currentData().toInt());
                    }
                });
        m_watchEnableBtn = new QPushButton(tr("Enable"), box);
        m_watchEnableBtn->setCheckable(true);
        connect(m_watchEnableBtn, &QPushButton::toggled,
                this, &ObjectDictionaryDialog::onWatchEnableToggled);
        m_watchRemoveBtn = new QPushButton(tr("Remove selected"), box);
        connect(m_watchRemoveBtn, &QPushButton::clicked,
                this, &ObjectDictionaryDialog::onRemoveFromWatch);
        m_watchClearBtn  = new QPushButton(tr("Clear"), box);
        connect(m_watchClearBtn, &QPushButton::clicked,
                this, &ObjectDictionaryDialog::onClearWatch);
        row->addWidget(m_watchEnableBtn);
        row->addWidget(m_watchRemoveBtn);
        row->addWidget(m_watchClearBtn);
        row->addStretch(1);
        wLay->addLayout(row);

        m_watchTable = new QTableWidget(0, 4, box);
        m_watchTable->setHorizontalHeaderLabels(
            {tr("Object"), tr("Name"), tr("Value"), tr("Age (ms)")});
        m_watchTable->verticalHeader()->setVisible(false);
        m_watchTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        m_watchTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_watchTable->horizontalHeader()->setSectionResizeMode(
            0, QHeaderView::ResizeToContents);
        m_watchTable->horizontalHeader()->setSectionResizeMode(
            1, QHeaderView::Stretch);
        m_watchTable->horizontalHeader()->setSectionResizeMode(
            2, QHeaderView::ResizeToContents);
        m_watchTable->horizontalHeader()->setSectionResizeMode(
            3, QHeaderView::ResizeToContents);
        wLay->addWidget(m_watchTable);
        root->addWidget(box, /*stretch=*/2);
    }

    m_watchTimer = new QTimer(this);
    m_watchTimer->setInterval(m_watchRate->currentData().toInt());
    connect(m_watchTimer, &QTimer::timeout,
            this, &ObjectDictionaryDialog::onWatchTick);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);

    buildCatalog();
    buildTree();
    onTreeSelectionChanged();
    /* First-launch: drop a template XML next to the diagnostic's other
     * config data so operators know where the format is documented +
     * have a hand-editable starting point. Idempotent — subsequent
     * launches leave whatever's on disk alone (operators may have
     * customised it). */
    writeTemplateIfMissing();
}


QString ObjectDictionaryDialog::templatePath()
{
    const QString dir = QStandardPaths::writableLocation(
        QStandardPaths::AppConfigLocation);
    return dir + QStringLiteral("/od_template.xml");
}


void ObjectDictionaryDialog::writeTemplateIfMissing() const
{
    const QString path = templatePath();
    if (QFileInfo::exists(path)){ return; }
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){ return; }
    QXmlStreamWriter xml(&f);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeComment(QStringLiteral(
        " od_template.xml — reference / starter file for the "
        "ObjectDictionary browser's Import feature. "
        "Attribute schema:\n"
        "     index=\"0xNNNN\"  sub=\"0xNN\"\n"
        "     name=\"…\"          type=\"u8|u16|u32|i8|i16|i32\"\n"
        "     byteLen=\"1|2|4\"   access=\"RO|RW\"\n"
        "     unit=\"…\"          category=\"…\"\n"
        "     value=\"0x…\"       (optional; seeds the last-read cache)\n"
        "Edit / add entries then load via ODDialog ▸ Import XML. "));
    xml.writeStartElement(QStringLiteral("ObjectDictionary"));
    xml.writeAttribute(QStringLiteral("schemaVersion"), QStringLiteral("1"));
    xml.writeAttribute(QStringLiteral("exportedBy"),
                        QStringLiteral("vr_mc_diagnostic (template)"));
    xml.writeStartElement(QStringLiteral("Slave"));
    xml.writeAttribute(QStringLiteral("index"), QStringLiteral("-1"));
    xml.writeAttribute(QStringLiteral("name"),  QStringLiteral("(template)"));
    xml.writeEndElement();
    xml.writeTextElement(QStringLiteral("Timestamp"),
        QDateTime::currentDateTime().toString(Qt::ISODate));
    xml.writeStartElement(QStringLiteral("Objects"));
    for (const auto& e : m_catalog){
        xml.writeStartElement(QStringLiteral("Object"));
        xml.writeAttribute(QStringLiteral("index"),
            QStringLiteral("0x%1").arg(e.index, 4, 16, QChar('0')));
        xml.writeAttribute(QStringLiteral("sub"),
            QStringLiteral("0x%1").arg(e.sub, 2, 16, QChar('0')));
        xml.writeAttribute(QStringLiteral("name"),    e.name);
        xml.writeAttribute(QStringLiteral("type"),    e.type);
        xml.writeAttribute(QStringLiteral("byteLen"), QString::number(e.byteLen));
        xml.writeAttribute(QStringLiteral("access"),  e.readOnly ? "RO" : "RW");
        if (!e.unit.isEmpty()){
            xml.writeAttribute(QStringLiteral("unit"), e.unit);
        }
        xml.writeAttribute(QStringLiteral("category"), e.category);
        /* Template intentionally omits value= — operators seed it
         * themselves when saving a golden snapshot. */
        xml.writeEndElement();
    }
    xml.writeEndElement();   /* Objects */
    xml.writeEndElement();   /* ObjectDictionary */
    xml.writeEndDocument();
    f.close();
}


void ObjectDictionaryDialog::setSlaveContext(int slaveIdx, const QString& name)
{
    m_slaveIdx  = slaveIdx;
    m_slaveName = name;
    m_headerLabel->setText(tr("Slave <b>#%1</b> — %2")
                              .arg(slaveIdx)
                              .arg(name.isEmpty() ? tr("?") : name));
    /* Slave change → wipe stale values so we don't attribute Slave-A's
     * last read to Slave-B's row. */
    m_latestHex.clear();
    m_latestTsMs.clear();
    /* Repaint watch column so the old values disappear. */
    for (int r = 0; r < m_watchTable->rowCount(); ++r){
        if (auto* it = m_watchTable->item(r, 2)){ it->setText(QStringLiteral("—")); }
        if (auto* it = m_watchTable->item(r, 3)){ it->setText(QStringLiteral("—")); }
    }
    onTreeSelectionChanged();
}


/* ============================================================
 * Catalog build
 * ============================================================
 *
 * Curated subset of the most-referenced entries in the CiA-301
 * comms area, the CiA-402 drive profile, and the manufacturer's
 * 0x20xx range that the diagnostic already knows about (matches
 * DriveConfigDialog's manufacturer tab). Extending is a one-line
 * push per row.
 */
void ObjectDictionaryDialog::buildCatalog()
{
    m_catalog.clear();
    auto add = [this](quint16 idx, quint8 sub, const char* name,
                       const char* type, int len, const char* unit,
                       bool ro, const char* cat){
        Entry e;
        e.index    = idx;
        e.sub      = sub;
        e.name     = QString::fromLatin1(name);
        e.type     = QString::fromLatin1(type);
        e.byteLen  = len;
        e.unit     = QString::fromLatin1(unit);
        e.readOnly = ro;
        e.category = QString::fromLatin1(cat);
        m_catalog.push_back(e);
    };

    /* --- Communication (CiA-301) --------------------------------- */
    add(0x1000, 0, "Device Type",                    "u32", 4, "", true,  kCatComm);
    add(0x1001, 0, "Error Register",                 "u8",  1, "", true,  kCatComm);
    add(0x1003, 0, "Predefined Error Field (count)", "u8",  1, "", true,  kCatComm);
    add(0x1008, 0, "Manufacturer Device Name",       "str", 4, "", true,  kCatComm);
    add(0x1009, 0, "Manufacturer Hardware Version",  "str", 4, "", true,  kCatComm);
    add(0x100A, 0, "Manufacturer Software Version",  "str", 4, "", true,  kCatComm);
    add(0x1010, 1, "Store Parameters (all)",         "u32", 4, "", false, kCatComm);
    add(0x1011, 1, "Restore Default Parameters",     "u32", 4, "", false, kCatComm);
    add(0x1018, 1, "Vendor ID",                      "u32", 4, "", true,  kCatComm);
    add(0x1018, 2, "Product code",                   "u32", 4, "", true,  kCatComm);
    add(0x1018, 3, "Revision number",                "u32", 4, "", true,  kCatComm);
    add(0x1018, 4, "Serial number",                  "u32", 4, "", true,  kCatComm);

    /* --- CiA-402 Drive Profile ------------------------------------ */
    add(0x603F, 0, "Error Code",                     "u16", 2, "",     true,  kCatDrive);
    add(0x6040, 0, "Controlword",                    "u16", 2, "",     false, kCatDrive);
    add(0x6041, 0, "Statusword",                     "u16", 2, "",     true,  kCatDrive);
    add(0x6060, 0, "Modes of Operation",             "i8",  1, "",     false, kCatDrive);
    add(0x6061, 0, "Modes of Operation Display",     "i8",  1, "",     true,  kCatDrive);
    add(0x6062, 0, "Position Demand Value",          "i32", 4, "inc",  true,  kCatDrive);
    add(0x6064, 0, "Position Actual Value",          "i32", 4, "inc",  true,  kCatDrive);
    add(0x606C, 0, "Velocity Actual Value",          "i32", 4, "inc/s",true,  kCatDrive);
    add(0x6072, 0, "Max Torque",                     "u16", 2, "‰",    false, kCatDrive);
    add(0x6073, 0, "Max Current",                    "u16", 2, "‰",    false, kCatDrive);
    add(0x6077, 0, "Torque Actual Value",            "i16", 2, "‰",    true,  kCatDrive);
    add(0x607A, 0, "Target Position",                "i32", 4, "inc",  false, kCatDrive);
    add(0x607C, 0, "Home Offset",                    "i32", 4, "inc",  false, kCatDrive);
    add(0x6080, 0, "Max Motor Speed",                "u32", 4, "rpm",  false, kCatDrive);
    add(0x6081, 0, "Profile Velocity",               "u32", 4, "inc/s",false, kCatDrive);
    add(0x6083, 0, "Profile Acceleration",           "u32", 4, "inc/s²",false, kCatDrive);
    add(0x6098, 0, "Homing Method",                  "i8",  1, "",     false, kCatDrive);
    add(0x6099, 1, "Homing Speed to switch",         "u32", 4, "inc/s",false, kCatDrive);
    add(0x6099, 2, "Homing Speed to zero",           "u32", 4, "inc/s",false, kCatDrive);
    add(0x609A, 0, "Homing Acceleration",            "u32", 4, "inc/s²",false, kCatDrive);
    add(0x60F6, 1, "Current-loop Kp",                "u32", 4, "",     false, kCatDrive);
    add(0x60F6, 2, "Current-loop Ki",                "u32", 4, "",     false, kCatDrive);

    /* --- Manufacturer (0x20xx) — matches DriveConfigDialog ------- */
    add(0x2000, 1, "Node ID",                        "u8",  1, "",     true,  kCatMfr);
    add(0x2030, 1, "Open-loop V/f trigger",          "u8",  1, "",     false, kCatMfr);
    add(0x2031, 1, "Calibration control trigger",    "u8",  1, "",     false, kCatMfr);
    add(0x2031, 2, "Calibration control status",     "u8",  1, "",     true,  kCatMfr);
    add(0x2040, 1, "Current offset A",               "i16", 2, "raw",  false, kCatMfr);
    add(0x2040, 2, "Current offset B",               "i16", 2, "raw",  false, kCatMfr);
    add(0x2040, 3, "Current offset C",               "i16", 2, "raw",  false, kCatMfr);
    add(0x2041, 1, "Current gain A",                 "u16", 2, "raw",  false, kCatMfr);
    add(0x2041, 2, "Current gain B",                 "u16", 2, "raw",  false, kCatMfr);
    add(0x2041, 3, "Current gain C",                 "u16", 2, "raw",  false, kCatMfr);
    add(0x2060, 0, "Hall offset (rad × 1e-6)",       "i32", 4, "µrad", false, kCatMfr);
    add(0x2070, 1, "Motor profile: pole pairs",      "u8",  1, "",     false, kCatMfr);
    add(0x2070, 2, "Motor profile: rated current",   "u16", 2, "mA",   false, kCatMfr);
    add(0x2070, 3, "Motor profile: rated speed",     "u16", 2, "rpm",  false, kCatMfr);
    add(0x2080, 1, "Auto-tune loop",                 "u8",  1, "",     false, kCatMfr);
    add(0x2080, 2, "Auto-tune bandwidth",            "u16", 2, "Hz",   false, kCatMfr);
    add(0x2080, 3, "Auto-tune trigger",              "u8",  1, "",     false, kCatMfr);
    add(0x2080, 4, "Auto-tune status",               "u8",  1, "",     true,  kCatMfr);

    /* Fast-lookup index. */
    m_catalogByKey.clear();
    for (int i = 0; i < m_catalog.size(); ++i){
        m_catalogByKey.insert(keyFor(m_catalog[i].index, m_catalog[i].sub), i);
    }
}


void ObjectDictionaryDialog::buildTree()
{
    m_tree->clear();
    QHash<QString, QTreeWidgetItem*> parents;
    /* Ensure the categories always render in a stable, non-alphabetic
     * order: Communication → Drive Profile → Manufacturer. */
    for (const char* cat : {kCatComm, kCatDrive, kCatMfr}){
        auto* it = new QTreeWidgetItem(m_tree,
            {QString::fromLatin1(cat)});
        QFont f = it->font(0); f.setBold(true); it->setFont(0, f);
        parents.insert(QString::fromLatin1(cat), it);
    }
    for (int i = 0; i < m_catalog.size(); ++i){
        const auto& e = m_catalog[i];
        auto* parent = parents.value(e.category, nullptr);
        if (!parent){ continue; }
        auto* leaf = new QTreeWidgetItem(parent, {
            QStringLiteral("0x%1:%2  %3")
                .arg(e.index, 4, 16, QChar('0'))
                .arg(e.sub,   2, 16, QChar('0'))
                .arg(e.name),
            e.type,
            e.readOnly ? tr("RO") : tr("RW"),
        });
        /* Store the catalog index in Qt::UserRole so selectionChanged
         * can find the Entry without a linear scan. */
        leaf->setData(0, Qt::UserRole, i);
    }
    for (auto* p : parents){ p->setExpanded(true); }
}


void ObjectDictionaryDialog::onTreeSelectionChanged()
{
    auto items = m_tree->selectedItems();
    const bool haveSel = !items.isEmpty()
                          && items.first()->data(0, Qt::UserRole).isValid();
    m_readBtn    ->setEnabled(haveSel && m_slaveIdx >= 0);
    m_addWatchBtn->setEnabled(haveSel);
    m_readAllBtn ->setEnabled(m_slaveIdx >= 0);
    if (!haveSel){
        m_detailsLabel->setText(tr("Select an entry on the left."));
        m_valueLabel  ->setText(tr("Value: —"));
        if (m_writeRow ){ m_writeRow ->setVisible(false); }
        if (m_writeHint){ m_writeHint->setVisible(false); }
        return;
    }
    const int catIdx = items.first()->data(0, Qt::UserRole).toInt();
    if (catIdx < 0 || catIdx >= m_catalog.size()){ return; }
    const auto& e = m_catalog[catIdx];
    renderDetails(e);
    /* Write UI toggles with RW-ness of the selected entry. Also update
     * a per-type hint so the operator sees the accepted range before
     * hitting Enter — cheaper than an error dialog on bad input. */
    const bool showWrite = !e.readOnly;
    if (m_writeRow){ m_writeRow->setVisible(showWrite); }
    if (m_writeBtn){ m_writeBtn->setEnabled(showWrite && m_slaveIdx >= 0); }
    if (m_writeHint){
        if (showWrite){
            QString hint;
            if      (e.type == QLatin1String("u8"))  hint = tr("range 0..255");
            else if (e.type == QLatin1String("i8"))  hint = tr("range -128..127");
            else if (e.type == QLatin1String("u16")) hint = tr("range 0..65535");
            else if (e.type == QLatin1String("i16")) hint = tr("range -32768..32767");
            else if (e.type == QLatin1String("u32")) hint = tr("range 0..4294967295");
            else if (e.type == QLatin1String("i32")) hint = tr("range -2147483648..2147483647");
            else                                     hint = e.type;
            m_writeHint->setText(tr("Type <b>%1</b> — %2. "
                "Accepts decimal (%3) or hex (0x%4).")
                    .arg(e.type, hint)
                    .arg(e.type.startsWith('i') ? "-123" : "123")
                    .arg("1F"));
            m_writeHint->setVisible(true);
        } else {
            m_writeHint->setVisible(false);
        }
    }
}


void ObjectDictionaryDialog::renderDetails(const Entry& e)
{
    m_detailsLabel->setText(tr(
        "<b>0x%1:%2</b> — %3<br/>"
        "Type: <code>%4</code> &middot; Size: %5 byte(s) &middot; "
        "Access: %6%7")
        .arg(e.index, 4, 16, QChar('0'))
        .arg(e.sub,   2, 16, QChar('0'))
        .arg(e.name.toHtmlEscaped(), e.type)
        .arg(e.byteLen)
        .arg(e.readOnly ? tr("RO") : tr("RW"))
        .arg(e.unit.isEmpty() ? QString()
                              : tr(" &middot; Unit: %1").arg(e.unit)));
    const auto it = m_latestHex.constFind(keyFor(e.index, e.sub));
    if (it == m_latestHex.constEnd()){
        m_valueLabel->setText(tr("Value: —  (not read yet)"));
    } else {
        m_valueLabel->setText(tr("Value: <b>%1</b>  (raw: %2)")
                                  .arg(formatValue(e, *it), *it));
    }
}


QString ObjectDictionaryDialog::formatValue(const Entry& e,
                                              const QString& hex) const
{
    if (hex.isEmpty()){ return QStringLiteral("—"); }
    QString s = hex;
    if (s.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){ s = s.mid(2); }
    bool ok = false;
    const quint64 raw = s.toULongLong(&ok, 16);
    if (!ok){ return hex; }
    /* Sign-extend for signed types by narrowing to the concrete width
     * then casting through the matching signed type. */
    if (e.type == QLatin1String("i8")){
        return QString::number(qint8(raw & 0xFFu));
    }
    if (e.type == QLatin1String("i16")){
        return QString::number(qint16(raw & 0xFFFFu));
    }
    if (e.type == QLatin1String("i32")){
        return QString::number(qint32(raw & 0xFFFFFFFFu));
    }
    if (e.type == QLatin1String("u8"))  { return QString::number(quint8(raw & 0xFFu));    }
    if (e.type == QLatin1String("u16")) { return QString::number(quint16(raw & 0xFFFFu)); }
    if (e.type == QLatin1String("u32")) { return QString::number(quint32(raw)); }
    return hex;
}


const ObjectDictionaryDialog::Entry* ObjectDictionaryDialog::findEntry(
    quint16 index, quint8 sub) const
{
    const auto it = m_catalogByKey.constFind(keyFor(index, sub));
    if (it == m_catalogByKey.constEnd()){ return nullptr; }
    return &m_catalog[*it];
}


void ObjectDictionaryDialog::onReadSelected()
{
    if (m_slaveIdx < 0){ return; }
    auto items = m_tree->selectedItems();
    if (items.isEmpty()){ return; }
    const int catIdx = items.first()->data(0, Qt::UserRole).toInt();
    if (catIdx < 0 || catIdx >= m_catalog.size()){ return; }
    const auto& e = m_catalog[catIdx];
    emit sdoReadRequested(m_slaveIdx, e.index, e.sub, e.byteLen);
}


void ObjectDictionaryDialog::onReadAll()
{
    if (m_slaveIdx < 0){ return; }
    m_readAllQueue.clear();
    for (int i = 0; i < m_catalog.size(); ++i){
        m_readAllQueue.append(i);
    }
    if (m_readAllQueue.isEmpty()){ return; }
    m_readAllCursor = 0;
    const auto& e = m_catalog[m_readAllQueue.first()];
    m_readAllBtn->setEnabled(false);
    m_readAllBtn->setText(tr("Reading… 1 / %1").arg(m_readAllQueue.size()));
    emit sdoReadRequested(m_slaveIdx, e.index, e.sub, e.byteLen);
}


QByteArray ObjectDictionaryDialog::packValue(const Entry& e,
                                               const QString& text,
                                               QString* outError) const
{
    const QString raw = text.trimmed();
    if (raw.isEmpty()){
        if (outError){ *outError = tr("empty value"); }
        return {};
    }
    /* Two accepted forms: 0x-hex (always unsigned; sign-extended to
     * the entry's width) and signed decimal (only meaningful for
     * signed types; still accepted for unsigned as a convenience). */
    bool ok = false;
    quint64 raw64 = 0;
    if (raw.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){
        raw64 = raw.mid(2).toULongLong(&ok, 16);
    } else if (e.type.startsWith('i')){
        /* Signed decimal → cast into two's-complement quint64 so the
         * LE packing loop below sees the right byte pattern. */
        const qint64 s = raw.toLongLong(&ok, 10);
        raw64 = quint64(s);
    } else {
        raw64 = raw.toULongLong(&ok, 10);
    }
    if (!ok){
        if (outError){ *outError = tr("could not parse '%1'").arg(text); }
        return {};
    }
    /* Range-check for unsigned types after parse. Signed types skip
     * the check — the LE truncation below is well-defined mod 2^N. */
    quint64 cap = 0;
    if      (e.byteLen == 1) cap = 0xFFull;
    else if (e.byteLen == 2) cap = 0xFFFFull;
    else if (e.byteLen == 4) cap = 0xFFFFFFFFull;
    else                     cap = ~0ull;
    if (!e.type.startsWith('i') && raw64 > cap){
        if (outError){
            *outError = tr("value 0x%1 exceeds %2-bit range")
                            .arg(raw64, 0, 16).arg(e.byteLen * 8);
        }
        return {};
    }
    QByteArray b(e.byteLen, char(0));
    for (int i = 0; i < e.byteLen; ++i){
        b[i] = char((raw64 >> (8 * i)) & 0xFFull);
    }
    return b;
}


void ObjectDictionaryDialog::onWriteSelected()
{
    if (m_slaveIdx < 0){ return; }
    auto items = m_tree->selectedItems();
    if (items.isEmpty()){ return; }
    const int catIdx = items.first()->data(0, Qt::UserRole).toInt();
    if (catIdx < 0 || catIdx >= m_catalog.size()){ return; }
    const auto& e = m_catalog[catIdx];
    if (e.readOnly){ return; }
    QString err;
    const QByteArray bytes = packValue(e, m_writeInput->text(), &err);
    if (bytes.isEmpty()){
        QMessageBox::warning(this, tr("Write value"),
            tr("Invalid input:\n%1").arg(err));
        return;
    }
    emit sdoWriteRequested(m_slaveIdx, e.index, e.sub, bytes);
    /* Optimistic read-back so the value column updates when the write
     * lands. The next customSdoDone repaints the details pane. */
    emit sdoReadRequested(m_slaveIdx, e.index, e.sub, e.byteLen);
}


void ObjectDictionaryDialog::onAddToWatch()
{
    auto items = m_tree->selectedItems();
    if (items.isEmpty()){ return; }
    const int catIdx = items.first()->data(0, Qt::UserRole).toInt();
    if (catIdx < 0 || catIdx >= m_catalog.size()){ return; }
    const auto& e = m_catalog[catIdx];
    const quint32 key = keyFor(e.index, e.sub);
    if (m_watchKeys.contains(key)){ return; }
    m_watchKeys.append(key);
    const int row = m_watchTable->rowCount();
    m_watchTable->insertRow(row);
    m_watchTable->setItem(row, 0, new QTableWidgetItem(
        QStringLiteral("0x%1:%2")
            .arg(e.index, 4, 16, QChar('0'))
            .arg(e.sub,   2, 16, QChar('0'))));
    m_watchTable->setItem(row, 1, new QTableWidgetItem(e.name));
    m_watchTable->setItem(row, 2, new QTableWidgetItem(QStringLiteral("—")));
    m_watchTable->setItem(row, 3, new QTableWidgetItem(QStringLiteral("—")));
}


void ObjectDictionaryDialog::onRemoveFromWatch()
{
    auto rows = m_watchTable->selectionModel()
                    ? m_watchTable->selectionModel()->selectedRows()
                    : QModelIndexList{};
    if (rows.isEmpty()){ return; }
    /* Remove bottom-up so index arithmetic stays valid. */
    QList<int> rowIdxs;
    for (const auto& mi : rows){ rowIdxs.append(mi.row()); }
    std::sort(rowIdxs.begin(), rowIdxs.end(), std::greater<int>());
    for (int r : rowIdxs){
        if (r >= 0 && r < m_watchKeys.size()){
            m_watchKeys.remove(r);
            m_watchTable->removeRow(r);
        }
    }
    if (m_watchCursor >= m_watchKeys.size()){ m_watchCursor = 0; }
}


void ObjectDictionaryDialog::onClearWatch()
{
    m_watchKeys.clear();
    m_watchTable->setRowCount(0);
    m_watchCursor = 0;
}


void ObjectDictionaryDialog::rebuildCatalogIndex()
{
    m_catalogByKey.clear();
    for (int i = 0; i < m_catalog.size(); ++i){
        m_catalogByKey.insert(keyFor(m_catalog[i].index, m_catalog[i].sub), i);
    }
}


void ObjectDictionaryDialog::onExportXml()
{
    /* Suggest a filename in the config folder alongside od_template.xml
     * so exports collect in one predictable place; operators who want
     * a different destination just navigate away from it in the file
     * dialog. */
    const QString cfgDir = QFileInfo(templatePath()).absolutePath();
    const QString suggested = cfgDir + QStringLiteral("/od_dump_%1.xml")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QDir().mkpath(cfgDir);
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export Object Dictionary"), suggested,
        tr("XML (*.xml);;All files (*)"));
    if (path.isEmpty()){ return; }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        QMessageBox::warning(this, tr("Export XML"),
            tr("Cannot write %1").arg(path));
        return;
    }
    QXmlStreamWriter xml(&f);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    /* Attribute-driven layout inspired by EtherCAT ESI's <Object>
     * shape but far simpler — just what we need for archiving +
     * offline diff. Round-trippable via onImportXml. */
    xml.writeStartElement(QStringLiteral("ObjectDictionary"));
    xml.writeAttribute(QStringLiteral("schemaVersion"), QStringLiteral("1"));
    xml.writeAttribute(QStringLiteral("exportedBy"),
                        QStringLiteral("vr_mc_diagnostic"));

    xml.writeStartElement(QStringLiteral("Slave"));
    xml.writeAttribute(QStringLiteral("index"), QString::number(m_slaveIdx));
    xml.writeAttribute(QStringLiteral("name"),  m_slaveName);
    xml.writeEndElement();

    xml.writeTextElement(QStringLiteral("Timestamp"),
        QDateTime::currentDateTime().toString(Qt::ISODate));

    xml.writeStartElement(QStringLiteral("Objects"));
    for (const auto& e : m_catalog){
        xml.writeStartElement(QStringLiteral("Object"));
        xml.writeAttribute(QStringLiteral("index"),
            QStringLiteral("0x%1").arg(e.index, 4, 16, QChar('0')));
        xml.writeAttribute(QStringLiteral("sub"),
            QStringLiteral("0x%1").arg(e.sub, 2, 16, QChar('0')));
        xml.writeAttribute(QStringLiteral("name"),     e.name);
        xml.writeAttribute(QStringLiteral("type"),     e.type);
        xml.writeAttribute(QStringLiteral("byteLen"),  QString::number(e.byteLen));
        xml.writeAttribute(QStringLiteral("access"),   e.readOnly ? "RO" : "RW");
        if (!e.unit.isEmpty()){
            xml.writeAttribute(QStringLiteral("unit"), e.unit);
        }
        xml.writeAttribute(QStringLiteral("category"), e.category);
        const auto it = m_latestHex.constFind(keyFor(e.index, e.sub));
        if (it != m_latestHex.constEnd()){
            xml.writeAttribute(QStringLiteral("value"), *it);
        }
        xml.writeEndElement();
    }
    xml.writeEndElement();   /* Objects */
    xml.writeEndElement();   /* ObjectDictionary */
    xml.writeEndDocument();
    f.close();
    m_headerLabel->setText(m_headerLabel->text() +
        tr("  <span style='color:#2e7d32;'>[exported to %1]</span>")
            .arg(QFileInfo(path).fileName()));
}


void ObjectDictionaryDialog::onImportXml()
{
    const QString cfgDir = QFileInfo(templatePath()).absolutePath();
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Import Object Dictionary"), cfgDir,
        tr("XML (*.xml);;All files (*)"));
    if (path.isEmpty()){ return; }
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)){
        QMessageBox::warning(this, tr("Import XML"),
            tr("Cannot read %1").arg(path));
        return;
    }
    QXmlStreamReader xml(&f);
    QVector<Entry> loaded;
    QHash<quint32, QString> loadedValues;
    QString parseErr;

    auto attrHex16 = [](const QStringView& v) -> quint16 {
        QString s = v.toString();
        if (s.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){
            s = s.mid(2);
        }
        return quint16(s.toUInt(nullptr, 16));
    };
    auto attrHex8 = [](const QStringView& v) -> quint8 {
        QString s = v.toString();
        if (s.startsWith(QLatin1String("0x"), Qt::CaseInsensitive)){
            s = s.mid(2);
        }
        return quint8(s.toUInt(nullptr, 16));
    };

    while (!xml.atEnd() && !xml.hasError()){
        xml.readNext();
        if (xml.tokenType() != QXmlStreamReader::StartElement){ continue; }
        if (xml.name() != QLatin1String("Object")){ continue; }
        const auto attrs = xml.attributes();
        if (!attrs.hasAttribute(QLatin1String("index"))
            || !attrs.hasAttribute(QLatin1String("sub"))){
            continue;
        }
        Entry e;
        e.index    = attrHex16(attrs.value(QLatin1String("index")));
        e.sub      = attrHex8 (attrs.value(QLatin1String("sub")));
        e.name     = attrs.value(QLatin1String("name")).toString();
        e.type     = attrs.value(QLatin1String("type")).toString();
        /* byteLen may be missing in older exports; derive from type
         * as a fallback so the import doesn't drop otherwise-valid
         * entries. */
        if (attrs.hasAttribute(QLatin1String("byteLen"))){
            e.byteLen = attrs.value(QLatin1String("byteLen")).toInt();
        } else if (e.type == QLatin1String("u8") || e.type == QLatin1String("i8")){
            e.byteLen = 1;
        } else if (e.type == QLatin1String("u16") || e.type == QLatin1String("i16")){
            e.byteLen = 2;
        } else {
            e.byteLen = 4;
        }
        e.unit     = attrs.value(QLatin1String("unit")).toString();
        e.readOnly = (attrs.value(QLatin1String("access")).toString() == "RO");
        e.category = attrs.value(QLatin1String("category")).toString();
        if (e.category.isEmpty()){
            /* Fall back to category-by-index range so imports of
             * hand-authored XML that skip the category still tree
             * properly. */
            if      (e.index < 0x2000) e.category = QString::fromLatin1(kCatComm);
            else if (e.index < 0x6000) e.category = QString::fromLatin1(kCatMfr);
            else                        e.category = QString::fromLatin1(kCatDrive);
        }
        loaded.append(e);
        if (attrs.hasAttribute(QLatin1String("value"))){
            loadedValues.insert(keyFor(e.index, e.sub),
                                 attrs.value(QLatin1String("value")).toString());
        }
    }
    if (xml.hasError()){
        parseErr = xml.errorString();
    }
    f.close();

    if (loaded.isEmpty()){
        QMessageBox::warning(this, tr("Import XML"),
            tr("No <Object> entries found in %1%2")
                .arg(QFileInfo(path).fileName())
                .arg(parseErr.isEmpty() ? QString()
                                          : tr(" (parse error: %1)").arg(parseErr)));
        return;
    }

    /* Commit — replace catalog + values, rebuild tree, wipe stale
     * watch entries whose (index, sub) no longer exists in the new
     * catalog. Selection lost intentionally; new tree is a different
     * catalog. */
    m_catalog     = std::move(loaded);
    m_latestHex   = std::move(loadedValues);
    m_latestTsMs.clear();
    rebuildCatalogIndex();
    buildTree();
    QVector<quint32> validWatch;
    for (quint32 k : m_watchKeys){
        if (m_catalogByKey.contains(k)){ validWatch.append(k); }
    }
    m_watchKeys = std::move(validWatch);
    /* Rebuild the watch table rows to match. */
    m_watchTable->setRowCount(0);
    for (quint32 k : m_watchKeys){
        const quint16 idx = quint16((k >> 8) & 0xFFFFu);
        const quint8  sub = quint8(k & 0xFFu);
        const Entry* e = findEntry(idx, sub);
        if (!e){ continue; }
        const int row = m_watchTable->rowCount();
        m_watchTable->insertRow(row);
        m_watchTable->setItem(row, 0, new QTableWidgetItem(
            QStringLiteral("0x%1:%2").arg(idx, 4, 16, QChar('0'))
                                       .arg(sub, 2, 16, QChar('0'))));
        m_watchTable->setItem(row, 1, new QTableWidgetItem(e->name));
        const auto vit = m_latestHex.constFind(k);
        m_watchTable->setItem(row, 2, new QTableWidgetItem(
            vit == m_latestHex.constEnd()
                ? QStringLiteral("—") : formatValue(*e, *vit)));
        m_watchTable->setItem(row, 3, new QTableWidgetItem(QStringLiteral("—")));
    }
    onTreeSelectionChanged();
    m_headerLabel->setText(m_headerLabel->text() +
        tr("  <span style='color:#2e7d32;'>[imported %1 entries]</span>")
            .arg(m_catalog.size()));
}


void ObjectDictionaryDialog::onWatchEnableToggled(bool on)
{
    m_watchOn = on;
    m_watchEnableBtn->setText(on ? tr("Disable") : tr("Enable"));
    if (on && m_slaveIdx >= 0){
        m_watchTimer->start();
    } else {
        m_watchTimer->stop();
    }
}


void ObjectDictionaryDialog::onWatchTick()
{
    if (!m_watchOn || m_watchKeys.isEmpty() || m_slaveIdx < 0){ return; }
    /* Round-robin — one SDO per tick, so the poll rate is really
     * "cycle rate ÷ N entries". Keeps SDO traffic bounded even with
     * a large watch list; operators who need bursts can bump the
     * rate combobox. */
    if (m_watchCursor >= m_watchKeys.size()){ m_watchCursor = 0; }
    const quint32 key = m_watchKeys[m_watchCursor];
    const quint16 idx = quint16((key >> 8) & 0xFFFFu);
    const quint8  sub = quint8(key & 0xFFu);
    m_watchCursor = (m_watchCursor + 1) % m_watchKeys.size();
    const Entry* e = findEntry(idx, sub);
    if (!e){ return; }
    emit sdoReadRequested(m_slaveIdx, e->index, e->sub, e->byteLen);
}


void ObjectDictionaryDialog::onCustomSdoDone(int slaveIdx, bool isWrite,
                                               quint16 odIdx, quint8 sub,
                                               bool ok, QString valueDecoded,
                                               QString message)
{
    Q_UNUSED(message);
    if (isWrite){ return; }
    if (slaveIdx != m_slaveIdx){ return; }
    const Entry* e = findEntry(odIdx, sub);
    if (!e){ return; }   /* not in catalog — some other dialog's read */
    const quint32 key = keyFor(odIdx, sub);
    if (ok){
        m_latestHex.insert(key, valueDecoded);
        m_latestTsMs.insert(key, QDateTime::currentMSecsSinceEpoch());
    }

    /* Update details pane if this entry is selected. */
    auto items = m_tree->selectedItems();
    if (!items.isEmpty()){
        const int catIdx = items.first()->data(0, Qt::UserRole).toInt();
        if (catIdx >= 0 && catIdx < m_catalog.size()
            && m_catalog[catIdx].index == odIdx
            && m_catalog[catIdx].sub   == sub){
            renderDetails(m_catalog[catIdx]);
        }
    }

    /* Update watch row if the entry is watched. */
    const int watchRow = m_watchKeys.indexOf(key);
    if (watchRow >= 0){
        m_watchTable->setItem(watchRow, 2, new QTableWidgetItem(
            ok ? formatValue(*e, valueDecoded) : tr("(err)")));
        m_watchTable->setItem(watchRow, 3, new QTableWidgetItem(
            QStringLiteral("0")));
    }

    /* Advance Read-All batch. */
    if (m_readAllCursor >= 0 && m_readAllCursor < m_readAllQueue.size()){
        const int expectedCatIdx = m_readAllQueue[m_readAllCursor];
        if (expectedCatIdx >= 0 && expectedCatIdx < m_catalog.size()
            && m_catalog[expectedCatIdx].index == odIdx
            && m_catalog[expectedCatIdx].sub   == sub){
            ++m_readAllCursor;
            if (m_readAllCursor >= m_readAllQueue.size()){
                m_readAllBtn->setEnabled(true);
                m_readAllBtn->setText(tr("Read all"));
                m_readAllCursor = -1;
                m_readAllQueue.clear();
            } else {
                const auto& ne = m_catalog[m_readAllQueue[m_readAllCursor]];
                m_readAllBtn->setText(tr("Reading… %1 / %2")
                                          .arg(m_readAllCursor + 1)
                                          .arg(m_readAllQueue.size()));
                emit sdoReadRequested(m_slaveIdx, ne.index, ne.sub, ne.byteLen);
            }
        }
    }
}

}  // namespace vrmc
