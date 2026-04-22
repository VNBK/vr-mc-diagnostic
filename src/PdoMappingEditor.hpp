/**
 * @file   PdoMappingEditor.hpp
 * @brief  Modal editor for one PDO's mapping (TPDO1 or RPDO1).
 *
 * The dialog presents a table of @c (idx, sub, bits, name) rows the user
 * can grow / shrink / reorder. A combo box pre-populated from the
 * standard CiA 402 object catalog lets the operator add an entry without
 * hand-typing hex indices. On accept the edited entries ride back to
 * @ref MainWindow which forwards them to the worker for the real SDO
 * push.
 */

#pragma once

#include "backends/CanBackend.hpp"

#include <QDialog>
#include <QVector>

class QComboBox;
class QPushButton;
class QTableWidget;

namespace vrmc {

class PdoMappingEditor : public QDialog
{
    Q_OBJECT
public:
    explicit PdoMappingEditor(bool isTpdo, QWidget* parent = nullptr);

    void setEntries(const QVector<vrmc::CanBackend::PdoMapEntry>& entries);
    QVector<vrmc::CanBackend::PdoMapEntry> entries() const;

    bool isTpdo() const { return m_isTpdo; }

private slots:
    void onAddFromCatalog();
    void onRemoveSelected();
    void onMoveUp();
    void onMoveDown();

private:
    void          addRow(uint16_t idx, uint8_t sub, uint8_t bits,
                         const QString& name);
    void          populateCatalog();
    int           catalogRowBits(int catalogIdx) const;
    uint16_t      catalogRowIdx (int catalogIdx) const;
    uint8_t       catalogRowSub (int catalogIdx) const;
    QString       catalogRowName(int catalogIdx) const;

    bool          m_isTpdo = true;
    QTableWidget* m_table  = nullptr;
    QComboBox*    m_catalog = nullptr;
    QPushButton*  m_addBtn  = nullptr;
    QPushButton*  m_removeBtn = nullptr;
    QPushButton*  m_upBtn   = nullptr;
    QPushButton*  m_downBtn = nullptr;
};

}  // namespace vrmc
