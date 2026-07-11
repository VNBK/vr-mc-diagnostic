/**
 * @file    ScenarioEditorDialog.hpp
 * @brief   Modal editor for the Control-tab Scenarios store.
 *
 * Layout: three-pane splitter — scenario list (left) + name/description
 * fields (upper right) + steps table (lower right).
 *
 *   ┌─ Scenarios ──┐┌─ Name / description ────────────────────────┐
 *   │  wave_slow   ││ Name: [ position sweep ±π/4                ] │
 *   │  step_torque ││ Desc: [ Alternate ±π/4 rad position steps  ] │
 *   │  velocity_sq ││       [ holds each for 1.5 s               ] │
 *   │              │├─ Steps ────────────────────────────────────┤ │
 *   │ [+][ − ][📄] ││ mode      target       hold_ms             │ │
 *   │              ││ position   0.7854      1500                │ │
 *   │              ││ position  -0.7854      1500                │ │
 *   │              ││ [+ Add ] [ Remove ] [ ↑ ] [ ↓ ]            │ │
 *   └──────────────┘└──────────────────────────────────────────────┘
 *
 * Data flow: constructed with a @c QVector<Scenario> snapshot; on
 * Accept, @ref scenarios() returns the mutated vector. Zero-side-
 * effect design — no worker involvement, no disk writes. The caller
 * (JointControlPanel) persists the result to scenarios.json.
 */
#pragma once

#include "JointControlPanel.hpp"    /* Scenario / ScenarioStep types */

#include <QDialog>


class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QTextEdit;

namespace vrmc {

class ScenarioEditorDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ScenarioEditorDialog(
        const QVector<JointControlPanel::Scenario>& initial,
        QWidget* parent = nullptr);

    /** @brief The edited scenario set. Valid after @c exec() returns
     *  @c QDialog::Accepted. */
    QVector<JointControlPanel::Scenario> scenarios() const { return m_scenarios; }

private slots:
    void onNewScenario();
    void onDuplicateScenario();
    void onDeleteScenario();
    void onScenarioSelected(int row);
    void onNameEdited(const QString& text);
    void onDescriptionEdited();
    void onAddStep();
    void onRemoveStep();
    void onMoveStepUp();
    void onMoveStepDown();
    void onStepCellChanged(int row, int col);

private:
    /** @brief Push @c m_scenarios[row] into the right-hand editor. */
    void loadScenarioIntoEditor(int row);
    /** @brief Read the right-hand editor back into @c m_scenarios[row]. */
    void syncEditorToModel(int row);
    /** @brief Repopulate the left-pane list from @c m_scenarios. */
    void rebuildList();
    /** @brief Rebuild the steps table from the current scenario. */
    void rebuildStepsTable();
    /** @brief Insert one row into the steps table with the given step. */
    void appendStepRow(const JointControlPanel::ScenarioStep& st);

    QVector<JointControlPanel::Scenario>  m_scenarios;
    int                                    m_currentRow = -1;

    /* Widgets. */
    QListWidget*   m_list        = nullptr;
    QPushButton*   m_newBtn      = nullptr;
    QPushButton*   m_dupBtn      = nullptr;
    QPushButton*   m_delBtn      = nullptr;

    QLineEdit*     m_nameEdit    = nullptr;
    QTextEdit*     m_descEdit    = nullptr;
    QTableWidget*  m_stepsTable  = nullptr;
    QPushButton*   m_addStepBtn  = nullptr;
    QPushButton*   m_rmStepBtn   = nullptr;
    QPushButton*   m_upStepBtn   = nullptr;
    QPushButton*   m_dnStepBtn   = nullptr;

    /* Guard so programmatic table edits (rebuildStepsTable) don't
     * feed back into onStepCellChanged and re-emit signals. */
    bool           m_populatingTable = false;
};

}  // namespace vrmc
