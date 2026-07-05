/**
 * @file    ObjectDictionaryDialog.hpp
 * @brief   ROADMAP #3 — Object-Dictionary browser.
 *
 * Named OD tree with type/unit metadata, on-demand reads, and a live
 * watch list. Replaces the "type the index" flow that DriveConfigDialog
 * ▸ Custom-SDO tab exposed for one-off pokes.
 *
 * UI layout:
 *   ┌ Slave header ────────────────────────────────────────────────┐
 *   ├ Tree of OD entries ─┬── Details pane (name / type / value) ──┤
 *   │  Communication      │  0x6041:00 Statusword (u16, RO)        │
 *   │    0x1001 …         │  Value: 0x2437                          │
 *   │    0x1003 …         │  [Read]  [Add to watch]                 │
 *   │  Drive profile      │                                          │
 *   │    0x6040 …         │                                          │
 *   │  Manufacturer       │                                          │
 *   │    0x2070 …         │                                          │
 *   ├ Watch list (bottom) ────────────────────────────────────────────┤
 *   │  Poll rate: [1 Hz ▼]  [Enable] [Clear]                          │
 *   │  index  name         value     age                              │
 *   └─────────────────────────────────────────────────────────────────┘
 *
 * Write support — RW entries render an inline value input + Write
 * button in the details pane. Input accepts decimal or 0x-prefixed hex;
 * type-aware sign extension and LE byte packing happen inside the
 * dialog so the operator writes in the entry's declared unit rather
 * than raw hex bytes.
 *
 * Persistence — the catalog + last-read values round-trip via a
 * compact XML format (attribute-driven; inspired by EtherCAT ESI but
 * far simpler). Export dumps the current state; Import replaces the
 * catalog and rehydrates the "last read" cache so a saved run can be
 * re-opened days later without re-issuing SDOs.
 */
#pragma once

#include <QDialog>
#include <QHash>
#include <QSet>
#include <QVector>


class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTimer;
class QTreeWidget;
class QTreeWidgetItem;
class QWidget;

namespace vrmc {

class ObjectDictionaryDialog : public QDialog
{
    Q_OBJECT
public:
    /** @brief One row of the OD catalog. name/unit are cosmetic;
     *  (index, sub, byteLen) is the SDO wire address. */
    struct Entry {
        quint16 index;
        quint8  sub;
        QString name;
        QString type;       /**< "u8" / "u16" / "u32" / "i16" / "i32" */
        int     byteLen;    /**< 1 / 2 / 4 */
        QString unit;       /**< empty when dimensionless / bitmask */
        bool    readOnly;
        QString category;   /**< tree parent header */
    };

    explicit ObjectDictionaryDialog(QWidget* parent = nullptr);

    void setSlaveContext(int slaveIdx, const QString& name);

signals:
    /** @brief Consumed by MainWindow → MasterWorker::customSdoRead. */
    void sdoReadRequested(int slaveIdx, quint16 odIdx, quint8 sub, int byteLen);
    /** @brief Consumed by MainWindow → MasterWorker::customSdoWrite.
     *  Fires only for RW entries when the operator clicks Write with
     *  a well-formed value. */
    void sdoWriteRequested(int slaveIdx, quint16 odIdx, quint8 sub,
                            QByteArray bytes);

public slots:
    /** @brief Fan-out from MasterWorker::customSdoDone. */
    void onCustomSdoDone(int slaveIdx, bool isWrite,
                          quint16 odIdx, quint8 sub,
                          bool ok, QString valueDecoded, QString message);

    /** @brief Issue a single read for the currently-selected entry. */
    void onReadSelected();
    /** @brief Write the value in @ref m_writeInput to the currently-
     *  selected entry. No-op for RO entries or malformed input. */
    void onWriteSelected();
    /** @brief Read every entry in the tree in sequence. Rate-limited
     *  by the customSdoDone round-trip. */
    void onReadAll();
    /** @brief Add the currently-selected entry to the watch list. */
    void onAddToWatch();
    /** @brief Remove the currently-selected watch row. */
    void onRemoveFromWatch();
    /** @brief Wipe the watch list. */
    void onClearWatch();
    /** @brief Serialise catalog + latest values to an XML file. */
    void onExportXml();
    /** @brief Load catalog + values from an XML file. Replaces the
     *  in-memory catalog + rehydrates the last-read cache. */
    void onImportXml();

private slots:
    void onTreeSelectionChanged();
    void onWatchTick();
    void onWatchEnableToggled(bool on);

private:
    void buildCatalog();
    void buildTree();
    void renderDetails(const Entry& e);
    /** @brief Format a raw hex string per the entry's type
     *  (u8 → decimal, u16 → 0x…, i16 → signed decimal, etc.). */
    QString formatValue(const Entry& e, const QString& hex) const;
    /** @brief Lookup an entry by (index, sub). Returns nullptr when
     *  the wire address isn't in the catalog. */
    const Entry* findEntry(quint16 index, quint8 sub) const;
    /** @brief Parse @p text (decimal or 0x-prefixed hex) into the
     *  packed LE bytes required by @p e's byteLen. Empty return =
     *  parse failure. Handles sign extension for signed types. */
    QByteArray packValue(const Entry& e, const QString& text,
                          QString* outError = nullptr) const;
    /** @brief Rebuild the fast-lookup index after catalog changes
     *  (Import). No-op if the catalog is empty. */
    void rebuildCatalogIndex();
    /** @brief Return @c ~/.config/<AppName>/od_template.xml so the
     *  Export/Import file dialogs can start there and the template
     *  auto-generator has a persistent home. */
    static QString templatePath();
    /** @brief First-launch helper: if @ref templatePath doesn't exist
     *  yet, write out the current in-memory catalog (no values) so
     *  operators have a hand-editable reference file alongside the
     *  rest of the diagnostic's config data. */
    void writeTemplateIfMissing() const;

    /* Slave binding. */
    int      m_slaveIdx = -1;
    QString  m_slaveName;

    /* Catalog + fast lookup. */
    QVector<Entry>                 m_catalog;
    QHash<quint32, int>            m_catalogByKey;   /**< key → catalog index */

    /* Widgets. */
    QLabel*        m_headerLabel   = nullptr;
    QTreeWidget*   m_tree          = nullptr;
    QLabel*        m_detailsLabel  = nullptr;
    QLabel*        m_valueLabel    = nullptr;
    QPushButton*   m_readBtn       = nullptr;
    QPushButton*   m_readAllBtn    = nullptr;
    QPushButton*   m_addWatchBtn   = nullptr;
    /* Write UI — shown only when the selected entry is RW. */
    QWidget*       m_writeRow      = nullptr;
    QLineEdit*     m_writeInput    = nullptr;
    QPushButton*   m_writeBtn      = nullptr;
    QLabel*        m_writeHint     = nullptr;
    /* Import / Export. */
    QPushButton*   m_exportBtn     = nullptr;
    QPushButton*   m_importBtn     = nullptr;

    /* Read-All batch state — indexes into m_catalog; kicked off in
     * onReadAll, drained one-per-response in onCustomSdoDone. */
    QVector<int>   m_readAllQueue;
    int            m_readAllCursor = -1;

    /* Latest read values per (index, sub) — feeds the detail pane
     * value + the watch list. Rehydrated on read completion. */
    QHash<quint32, QString>  m_latestHex;
    QHash<quint32, qint64>   m_latestTsMs;

    /* Watch list. */
    QTableWidget*  m_watchTable    = nullptr;
    QVector<quint32> m_watchKeys;
    QComboBox*     m_watchRate     = nullptr;
    QPushButton*   m_watchEnableBtn= nullptr;
    QPushButton*   m_watchClearBtn = nullptr;
    QPushButton*   m_watchRemoveBtn= nullptr;
    QTimer*        m_watchTimer    = nullptr;
    bool           m_watchOn       = false;
    /* Round-robin cursor so the poll timer only kicks one read per tick;
     * customSdoDone advances the cursor once the previous one lands. */
    int            m_watchCursor   = 0;
};

}  // namespace vrmc
