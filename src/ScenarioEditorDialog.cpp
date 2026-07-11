#include "ScenarioEditorDialog.hpp"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextEdit>
#include <QVBoxLayout>


namespace vrmc {

namespace {

/* Compact string ↔ enum bridge. Uses the same wire strings the JSON
 * store uses so save/load round-trip cleanly. */
const char* modeToString(Mode m){
    switch (m){
        case Mode::Position: return "position";
        case Mode::Velocity: return "velocity";
        case Mode::Torque:   return "torque";
        default:                                 return "position";
    }
}

Mode stringToMode(const QString& s){
    if (s == QLatin1String("velocity")) return Mode::Velocity;
    if (s == QLatin1String("torque"))   return Mode::Torque;
    return Mode::Position;
}

TargetKind kindForMode(Mode m){
    switch (m){
        case Mode::Velocity: return TargetKind::Velocity;
        case Mode::Torque:   return TargetKind::Torque;
        default:                                 return TargetKind::Position;
    }
}

}  // namespace


ScenarioEditorDialog::ScenarioEditorDialog(
    const QVector<JointControlPanel::Scenario>& initial,
    QWidget* parent)
    : QDialog(parent), m_scenarios(initial)
{
    setWindowTitle(tr("Edit scenarios"));
    setModal(true);
    resize(920, 600);

    auto* root = new QVBoxLayout(this);
    auto* split = new QSplitter(Qt::Horizontal, this);

    /* ---- Left pane: scenario list + New/Dup/Delete ------------- */
    auto* leftBox = new QWidget(split);
    auto* leftLay = new QVBoxLayout(leftBox);
    leftLay->addWidget(new QLabel(tr("<b>Scenarios</b>"), leftBox));
    m_list = new QListWidget(leftBox);
    connect(m_list, &QListWidget::currentRowChanged,
            this, &ScenarioEditorDialog::onScenarioSelected);
    leftLay->addWidget(m_list, /*stretch=*/1);
    auto* leftBtnRow = new QHBoxLayout();
    m_newBtn = new QPushButton(tr("+"),   leftBox);
    m_dupBtn = new QPushButton(tr("Dup"), leftBox);
    m_delBtn = new QPushButton(tr("−"),   leftBox);
    m_newBtn->setToolTip(tr("New scenario (empty)."));
    m_dupBtn->setToolTip(tr("Duplicate the selected scenario."));
    m_delBtn->setToolTip(tr("Delete the selected scenario."));
    connect(m_newBtn, &QPushButton::clicked, this, &ScenarioEditorDialog::onNewScenario);
    connect(m_dupBtn, &QPushButton::clicked, this, &ScenarioEditorDialog::onDuplicateScenario);
    connect(m_delBtn, &QPushButton::clicked, this, &ScenarioEditorDialog::onDeleteScenario);
    leftBtnRow->addWidget(m_newBtn);
    leftBtnRow->addWidget(m_dupBtn);
    leftBtnRow->addWidget(m_delBtn);
    leftBtnRow->addStretch(1);
    leftLay->addLayout(leftBtnRow);
    split->addWidget(leftBox);

    /* ---- Right pane: name / description / steps ---------------- */
    auto* rightBox = new QWidget(split);
    auto* rightLay = new QVBoxLayout(rightBox);
    rightLay->addWidget(new QLabel(tr("Name"), rightBox));
    m_nameEdit = new QLineEdit(rightBox);
    connect(m_nameEdit, &QLineEdit::textEdited,
            this, &ScenarioEditorDialog::onNameEdited);
    rightLay->addWidget(m_nameEdit);
    rightLay->addWidget(new QLabel(tr("Description"), rightBox));
    m_descEdit = new QTextEdit(rightBox);
    m_descEdit->setMaximumHeight(80);
    connect(m_descEdit, &QTextEdit::textChanged,
            this, &ScenarioEditorDialog::onDescriptionEdited);
    rightLay->addWidget(m_descEdit);

    rightLay->addWidget(new QLabel(tr("<b>Steps</b>"), rightBox));
    m_stepsTable = new QTableWidget(0, 3, rightBox);
    m_stepsTable->setHorizontalHeaderLabels(
        {tr("Mode"), tr("Target"), tr("Hold (ms)")});
    m_stepsTable->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::ResizeToContents);
    m_stepsTable->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::Stretch);
    m_stepsTable->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::ResizeToContents);
    m_stepsTable->verticalHeader()->setVisible(false);
    m_stepsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    connect(m_stepsTable, &QTableWidget::cellChanged,
            this, &ScenarioEditorDialog::onStepCellChanged);
    rightLay->addWidget(m_stepsTable, /*stretch=*/1);

    auto* stepBtnRow = new QHBoxLayout();
    m_addStepBtn = new QPushButton(tr("+ Add step"), rightBox);
    m_rmStepBtn  = new QPushButton(tr("− Remove"),   rightBox);
    m_upStepBtn  = new QPushButton(tr("↑"),          rightBox);
    m_dnStepBtn  = new QPushButton(tr("↓"),          rightBox);
    connect(m_addStepBtn, &QPushButton::clicked, this, &ScenarioEditorDialog::onAddStep);
    connect(m_rmStepBtn,  &QPushButton::clicked, this, &ScenarioEditorDialog::onRemoveStep);
    connect(m_upStepBtn,  &QPushButton::clicked, this, &ScenarioEditorDialog::onMoveStepUp);
    connect(m_dnStepBtn,  &QPushButton::clicked, this, &ScenarioEditorDialog::onMoveStepDown);
    stepBtnRow->addWidget(m_addStepBtn);
    stepBtnRow->addWidget(m_rmStepBtn);
    stepBtnRow->addWidget(m_upStepBtn);
    stepBtnRow->addWidget(m_dnStepBtn);
    stepBtnRow->addStretch(1);
    rightLay->addLayout(stepBtnRow);
    split->addWidget(rightBox);

    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 3);
    root->addWidget(split, /*stretch=*/1);

    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    root->addWidget(buttons);

    rebuildList();
    if (!m_scenarios.isEmpty()){
        m_list->setCurrentRow(0);
    } else {
        loadScenarioIntoEditor(-1);
    }
}


void ScenarioEditorDialog::rebuildList()
{
    m_list->blockSignals(true);
    m_list->clear();
    for (const auto& s : m_scenarios){
        m_list->addItem(s.name.isEmpty() ? tr("(unnamed)") : s.name);
    }
    m_list->blockSignals(false);
}


void ScenarioEditorDialog::loadScenarioIntoEditor(int row)
{
    m_currentRow = row;
    const bool valid = (row >= 0 && row < m_scenarios.size());
    m_nameEdit ->setEnabled(valid);
    m_descEdit ->setEnabled(valid);
    m_stepsTable->setEnabled(valid);
    m_addStepBtn->setEnabled(valid);
    m_rmStepBtn ->setEnabled(valid);
    m_upStepBtn ->setEnabled(valid);
    m_dnStepBtn ->setEnabled(valid);
    m_dupBtn    ->setEnabled(valid);
    m_delBtn    ->setEnabled(valid);
    if (!valid){
        QSignalBlocker _n(m_nameEdit);
        QSignalBlocker _d(m_descEdit);
        m_nameEdit->clear();
        m_descEdit->clear();
        m_stepsTable->setRowCount(0);
        return;
    }
    const auto& s = m_scenarios[row];
    {
        QSignalBlocker _n(m_nameEdit);
        QSignalBlocker _d(m_descEdit);
        m_nameEdit->setText(s.name);
        m_descEdit->setPlainText(s.description);
    }
    rebuildStepsTable();
}


void ScenarioEditorDialog::rebuildStepsTable()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    m_populatingTable = true;
    m_stepsTable->setRowCount(0);
    for (const auto& st : m_scenarios[m_currentRow].steps){
        appendStepRow(st);
    }
    m_populatingTable = false;
}


void ScenarioEditorDialog::appendStepRow(const JointControlPanel::ScenarioStep& st)
{
    const int row = m_stepsTable->rowCount();
    m_stepsTable->insertRow(row);

    /* Mode column: combobox so the operator can't type an invalid mode.
     * Wired to sync back into m_scenarios on change. */
    auto* modeCombo = new QComboBox();
    modeCombo->addItem(tr("position"), QStringLiteral("position"));
    modeCombo->addItem(tr("velocity"), QStringLiteral("velocity"));
    modeCombo->addItem(tr("torque"),   QStringLiteral("torque"));
    const int modeIdx = modeCombo->findData(QString::fromLatin1(modeToString(st.mode)));
    modeCombo->setCurrentIndex(modeIdx >= 0 ? modeIdx : 0);
    m_stepsTable->setCellWidget(row, 0, modeCombo);
    connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this, row](int){
                if (m_populatingTable) { return; }
                if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
                if (row >= m_scenarios[m_currentRow].steps.size()){ return; }
                auto* cb = qobject_cast<QComboBox*>(m_stepsTable->cellWidget(row, 0));
                if (!cb){ return; }
                auto& st_ = m_scenarios[m_currentRow].steps[row];
                st_.mode = stringToMode(cb->currentData().toString());
                st_.kind = kindForMode(st_.mode);
            });

    /* Target: signed double with a range wide enough for the extremes
     * across all three modes (±2π rad, ±100 rad/s, ±20 N·m). */
    auto* tgtSpin = new QDoubleSpinBox();
    tgtSpin->setRange(-1000.0, 1000.0);
    tgtSpin->setDecimals(4);
    tgtSpin->setSingleStep(0.1);
    tgtSpin->setValue(st.target);
    m_stepsTable->setCellWidget(row, 1, tgtSpin);
    connect(tgtSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this, row](double v){
                if (m_populatingTable) { return; }
                if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
                if (row >= m_scenarios[m_currentRow].steps.size()){ return; }
                m_scenarios[m_currentRow].steps[row].target = v;
            });

    /* Hold ms: bounded to 5 min per step. Longer runs = multiple steps. */
    auto* holdSpin = new QSpinBox();
    holdSpin->setRange(0, 300000);
    holdSpin->setSingleStep(100);
    holdSpin->setSuffix(tr(" ms"));
    holdSpin->setValue(st.holdMs);
    m_stepsTable->setCellWidget(row, 2, holdSpin);
    connect(holdSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this, row](int v){
                if (m_populatingTable) { return; }
                if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
                if (row >= m_scenarios[m_currentRow].steps.size()){ return; }
                m_scenarios[m_currentRow].steps[row].holdMs = v;
            });
}


void ScenarioEditorDialog::syncEditorToModel(int row)
{
    if (row < 0 || row >= m_scenarios.size()){ return; }
    m_scenarios[row].name        = m_nameEdit->text();
    m_scenarios[row].description = m_descEdit->toPlainText();
}


void ScenarioEditorDialog::onScenarioSelected(int row)
{
    /* Persist the current form before switching so the operator doesn't
     * lose in-progress edits by clicking another item. */
    if (m_currentRow >= 0){ syncEditorToModel(m_currentRow); }
    loadScenarioIntoEditor(row);
}


void ScenarioEditorDialog::onNewScenario()
{
    JointControlPanel::Scenario s;
    s.name        = tr("New scenario");
    s.description = QString();
    m_scenarios.append(s);
    rebuildList();
    m_list->setCurrentRow(m_scenarios.size() - 1);
}


void ScenarioEditorDialog::onDuplicateScenario()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    syncEditorToModel(m_currentRow);
    JointControlPanel::Scenario copy = m_scenarios[m_currentRow];
    copy.name = tr("%1 (copy)").arg(copy.name);
    m_scenarios.insert(m_currentRow + 1, copy);
    rebuildList();
    m_list->setCurrentRow(m_currentRow + 1);
}


void ScenarioEditorDialog::onDeleteScenario()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    m_scenarios.remove(m_currentRow);
    /* Adjust the current-row cursor: prefer keeping the same index so
     * the operator's eye stays where they clicked, unless we ran off
     * the end. */
    const int nextRow = std::min(m_currentRow, int(m_scenarios.size()) - 1);
    m_currentRow = -1;   /* prevent onScenarioSelected from syncing stale */
    rebuildList();
    if (nextRow >= 0){
        m_list->setCurrentRow(nextRow);
    } else {
        loadScenarioIntoEditor(-1);
    }
}


void ScenarioEditorDialog::onNameEdited(const QString& text)
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    m_scenarios[m_currentRow].name = text;
    /* Update the list item's label live so the operator sees the rename
     * as they type. blockSignals to avoid re-entering onScenarioSelected. */
    m_list->blockSignals(true);
    m_list->item(m_currentRow)->setText(text.isEmpty() ? tr("(unnamed)") : text);
    m_list->blockSignals(false);
}


void ScenarioEditorDialog::onDescriptionEdited()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    m_scenarios[m_currentRow].description = m_descEdit->toPlainText();
}


void ScenarioEditorDialog::onAddStep()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    JointControlPanel::ScenarioStep st;
    st.mode   = Mode::Position;
    st.kind   = TargetKind::Position;
    st.target = 0.0;
    st.holdMs = 500;
    m_scenarios[m_currentRow].steps.append(st);
    appendStepRow(st);
    m_stepsTable->setCurrentCell(m_stepsTable->rowCount() - 1, 1);
}


void ScenarioEditorDialog::onRemoveStep()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    const int row = m_stepsTable->currentRow();
    if (row < 0 || row >= m_scenarios[m_currentRow].steps.size()){ return; }
    m_scenarios[m_currentRow].steps.remove(row);
    rebuildStepsTable();
    if (m_stepsTable->rowCount() > 0){
        m_stepsTable->setCurrentCell(std::min(row, m_stepsTable->rowCount() - 1), 0);
    }
}


void ScenarioEditorDialog::onMoveStepUp()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    const int row = m_stepsTable->currentRow();
    if (row <= 0){ return; }
    auto& steps = m_scenarios[m_currentRow].steps;
    std::swap(steps[row], steps[row - 1]);
    rebuildStepsTable();
    m_stepsTable->setCurrentCell(row - 1, 0);
}


void ScenarioEditorDialog::onMoveStepDown()
{
    if (m_currentRow < 0 || m_currentRow >= m_scenarios.size()){ return; }
    const int row = m_stepsTable->currentRow();
    auto& steps = m_scenarios[m_currentRow].steps;
    if (row < 0 || row + 1 >= steps.size()){ return; }
    std::swap(steps[row], steps[row + 1]);
    rebuildStepsTable();
    m_stepsTable->setCurrentCell(row + 1, 0);
}


void ScenarioEditorDialog::onStepCellChanged(int /*row*/, int /*col*/)
{
    /* All step cells are widgets (combobox / spinboxes), so cellChanged
     * only fires for text items — which we never insert. This hook is
     * kept for future extension without changing the connect. */
}

}  // namespace vrmc
