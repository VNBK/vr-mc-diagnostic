/**
 * @file   SlavePickerBar.hpp
 * @brief  Single-row slave selector + live inline telemetry strip.
 *
 * Replaces the multi-column QTableView at the top of MainWindow for
 * the common case where the operator only ever works with one slave
 * at a time. Layout:
 *
 *   [Slave: 5 — jnt_0 ▼]  ●OP_ENABLED  q +0.52  ω +0.04  τ +0.12  err 0x0000   [Expand grid ⤵]
 *
 * The combobox is bound to the existing @ref SlaveTableModel (column
 * = name) so it reflects every Connect / Disconnect automatically.
 * The strip alongside is repainted from the per-tick snapshot stream
 * for whatever slave the combobox currently points at. The
 * @em Expand-grid toggle button shows / hides the full table widget
 * elsewhere in the window (MainWindow owns the table; the picker
 * just emits the toggle).
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>

class QAbstractItemModel;
class QComboBox;
class QItemSelectionModel;
class QLabel;
class QPushButton;

namespace vrmc {

class SlavePickerBar : public QWidget
{
    Q_OBJECT
public:
    explicit SlavePickerBar(QWidget* parent = nullptr);

    /** Bind the picker's combo to a slave-list model (typically the
     *  same @ref SlaveTableModel that backs the table). Optional
     *  @p selectionModel is mirrored so picking a row in the table
     *  while it's expanded keeps the combo in sync. */
    void bindModel(QAbstractItemModel*       model,
                   QItemSelectionModel*      tableSelection);

    /** Show / hide the Expand-grid toggle. Useful when the slave
     *  table is permanently visible elsewhere (then the expand
     *  button has nothing to toggle). Default: visible. */
    void setExpandButtonVisible(bool on);

public slots:
    /** Push the per-tick snapshot stream so the inline strip stays
     *  live. Filters internally to the currently-selected slave. */
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);

signals:
    /** Operator changed the combobox. @p modelRow is the index into
     *  the bound model; MainWindow uses it to mirror selection back
     *  into the table widget. */
    void slaveSelected(int modelRow);
    /** Expand toggle flipped. @p on = show full table grid. */
    void expandToggled(bool on);

private:
    static QString stateName(uint16_t statusword);
    static int     stateCode(uint16_t statusword);

    void onComboChanged(int row);
    void onTableSelectionChanged();

    QAbstractItemModel*  m_model         = nullptr;
    QItemSelectionModel* m_tableSelModel = nullptr;
    int                  m_currentRow    = -1;
    int                  m_currentIdx    = -1;
    bool                 m_syncing       = false;

    QComboBox*   m_combo      = nullptr;
    QLabel*      m_stateLbl   = nullptr;
    QLabel*      m_posLbl     = nullptr;
    QLabel*      m_velLbl     = nullptr;
    QLabel*      m_trqLbl     = nullptr;
    QLabel*      m_errLbl     = nullptr;
    QPushButton* m_expandBtn  = nullptr;
};

}  // namespace vrmc
