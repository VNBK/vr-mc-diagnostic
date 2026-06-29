/*
 * TestDescriptionDialog.cpp — see header for purpose.
 */
#include "TestDescriptionDialog.hpp"
#include "ParamsEditDialog.hpp"
#include "TestRunnerWindow.hpp"   /* for TestSpec + persist target */

#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>


TestDescriptionDialog::TestDescriptionDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Test description"));
    setModal(false);                       /* non-modal — runner stays interactive */
    setWindowFlags(Qt::Window);
    resize(560, 600);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(10, 10, 10, 10);

    /* Header rows — id / name / parent DB / duration / blocks. */
    auto* head = new QFormLayout();
    id_label_       = new QLabel(this);
    name_label_     = new QLabel(this);
    db_label_       = new QLabel(this);
    duration_label_ = new QLabel(this);
    samples_label_  = new QLabel(this);
    blocks_label_   = new QLabel(this);

    QFont bold = id_label_->font();
    bold.setBold(true);
    for (auto* l : {id_label_, name_label_}){ l->setFont(bold); }

    head->addRow(tr("ID:"),       id_label_);
    head->addRow(tr("Name:"),     name_label_);
    head->addRow(tr("Database:"), db_label_);
    head->addRow(tr("Duration:"), duration_label_);
    head->addRow(tr("Samples:"),  samples_label_);
    head->addRow(tr("Blocks:"),   blocks_label_);
    outer->addLayout(head);

    /* Description — multi-paragraph rationale. */
    outer->addWidget(new QLabel(tr("<b>Description:</b>"), this));
    description_view_ = new QPlainTextEdit(this);
    description_view_->setReadOnly(true);
    description_view_->setMaximumHeight(120);
    outer->addWidget(description_view_);

    /* Pre-conditions — environmental + state assumptions. */
    outer->addWidget(new QLabel(tr("<b>Pre-conditions:</b>"), this));
    pre_conditions_list_ = new QListWidget(this);
    pre_conditions_list_->setMaximumHeight(80);
    outer->addWidget(pre_conditions_list_);

    /* Procedure — step list. */
    outer->addWidget(new QLabel(tr("<b>Procedure:</b>"), this));
    procedure_list_ = new QListWidget(this);
    procedure_list_->setMaximumHeight(140);
    outer->addWidget(procedure_list_);

    /* Teardown — schema-v2 cleanup actions. Hidden when the spec
     * has no entries so a v1 test description stays compact. */
    teardown_label_ = new QLabel(tr("<b>Teardown:</b>"), this);
    outer->addWidget(teardown_label_);
    teardown_list_ = new QListWidget(this);
    teardown_list_->setMaximumHeight(80);
    outer->addWidget(teardown_list_);

    /* Metrics + pass criteria. */
    outer->addWidget(new QLabel(tr("<b>Metrics captured:</b>"), this));
    metrics_list_ = new QListWidget(this);
    metrics_list_->setMaximumHeight(80);
    outer->addWidget(metrics_list_);

    outer->addWidget(new QLabel(tr("<b>Pass criteria:</b>"), this));
    pass_criteria_view_ = new QLabel(this);
    pass_criteria_view_->setWordWrap(true);
    pass_criteria_view_->setStyleSheet(
        "QLabel { background: #ffffff; color: #1a1a1a; padding: 6px; "
        "border: 1px solid #888; border-radius: 4px; "
        "font-family: monospace; }");
    outer->addWidget(pass_criteria_view_);

    /* Parameters — the JSON-defined knobs the runner will read. Lives
     * right under pass criteria because operators tuning a threshold
     * naturally read criteria → "where do I change it?" → params. */
    outer->addWidget(new QLabel(tr("<b>Parameters (from tests.json):</b>"), this));
    params_table_ = new QTableWidget(0, 3, this);
    params_table_->setHorizontalHeaderLabels(
        {tr("Key"), tr("Value"), tr("Meaning")});
    params_table_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::ResizeToContents);
    params_table_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents);
    params_table_->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::Stretch);
    params_table_->verticalHeader()->setVisible(false);
    params_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    params_table_->setWordWrap(true);
    params_table_->setMaximumHeight(160);
    outer->addWidget(params_table_);

    /* Edit button — hidden until a persist target is wired by
     * setPersistTarget(). Lives on its own row so the form layout
     * doesn't squash it next to the table. */
    auto* edit_row = new QHBoxLayout();
    edit_row->addStretch(1);
    edit_params_btn_ = new QPushButton(tr("Edit Params…"), this);
    edit_params_btn_->setVisible(false);
    edit_row->addWidget(edit_params_btn_);
    outer->addLayout(edit_row);
    connect(edit_params_btn_, &QPushButton::clicked, this, [this]{
        if (!persist_target_ || current_test_id_.isEmpty()){ return; }
        ParamsEditDialog dlg(current_test_id_,
                             current_params_,
                             current_param_meanings_,
                             this);
        if (dlg.exec() != QDialog::Accepted){ return; }
        const QJsonObject edited = dlg.editedParams();
        QString err;
        if (!persist_target_->writeParams(current_test_id_, edited, &err)){
            QMessageBox::warning(this, tr("Save failed"),
                tr("Could not persist params for %1:\n%2")
                    .arg(current_test_id_, err));
            return;
        }
        emit paramsEdited(current_test_id_);
    });

    depends_on_label_ = new QLabel(this);
    depends_on_label_->setWordWrap(true);
    outer->addWidget(depends_on_label_);

    /* Footer Close button. */
    auto* close_btn = new QPushButton(tr("Close"), this);
    close_btn->setDefault(true);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::hide);
    outer->addWidget(close_btn);
}


void TestDescriptionDialog::show(const TestSpec& spec)
{
    id_label_      ->setText(spec.id);
    name_label_    ->setText(spec.name);
    db_label_      ->setText(spec.db_id);
    duration_label_->setText(tr("~%1 s").arg(spec.duration_estimate_s, 0, 'f', 1));
    samples_label_ ->setText(spec.num_samples > 0
                                ? QString::number(spec.num_samples)
                                : tr("(unspecified)"));
    blocks_label_  ->setText(spec.blocks_if_fails ? tr("YES — failure aborts run")
                                                   : tr("no"));

    description_view_->setPlainText(spec.description);

    pre_conditions_list_->clear();
    for (const QString& pc : spec.pre_conditions){
        pre_conditions_list_->addItem(QStringLiteral("• ") + pc);
    }
    if (spec.pre_conditions.isEmpty()){
        pre_conditions_list_->addItem(tr("(none specified)"));
    }

    procedure_list_->clear();
    int n = 1;
    for (const QString& step : spec.procedure){
        procedure_list_->addItem(QStringLiteral("%1. %2").arg(n++).arg(step));
    }

    teardown_list_->clear();
    for (const QString& step : spec.teardown){
        teardown_list_->addItem(QStringLiteral("• ") + step);
    }
    const bool has_teardown = !spec.teardown.isEmpty();
    teardown_label_->setVisible(has_teardown);
    teardown_list_ ->setVisible(has_teardown);

    metrics_list_->clear();
    /* v2 — prefer captures (with catalog meanings) when present;
     * fall back to v1 metrics/metric_meanings for older files. */
    if (!spec.captures.isEmpty() && acq_table_){
        for (const QString& cap : spec.captures){
            const auto it = acq_table_->constFind(cap);
            const QString meaning = (it != acq_table_->constEnd())
                ? it.value().meaning
                : tr("(not in acquisition_table)");
            metrics_list_->addItem(QStringLiteral("%1 — %2").arg(cap, meaning));
        }
    }
    if (metrics_list_->count() == 0){
        metrics_list_->addItem(tr("(no captures declared)"));
    }

    pass_criteria_view_->setText(spec.pass_criteria);

    /* Render spec.probe_params as a flat key=value table. JSON values
     * are stringified through QJsonValue::toVariant() so we get the
     * native type formatting (1.2e+02, true, ["a","b"], …) without
     * a custom formatter. */
    params_table_->setRowCount(0);
    if (spec.probe_params.isEmpty()){
        params_table_->insertRow(0);
        auto* k = new QTableWidgetItem(tr("(no probe_params declared)"));
        QFont f = k->font(); f.setItalic(true); k->setFont(f);
        params_table_->setItem(0, 0, k);
        params_table_->setItem(0, 1, new QTableWidgetItem());
        params_table_->setItem(0, 2, new QTableWidgetItem());
    } else {
        for (auto it = spec.probe_params.constBegin();
                  it != spec.probe_params.constEnd(); ++it){
            const int r = params_table_->rowCount();
            params_table_->insertRow(r);
            params_table_->setItem(r, 0, new QTableWidgetItem(it.key()));
            params_table_->setItem(r, 1, new QTableWidgetItem(
                it.value().toVariant().toString()));
            params_table_->setItem(r, 2, new QTableWidgetItem(QString{}));
        }
    }

    if (spec.depends_on.isEmpty()){
        depends_on_label_->setText(tr("Dependencies: none"));
    } else {
        depends_on_label_->setText(tr("Dependencies: %1")
            .arg(spec.depends_on.join(", ")));
    }

    /* Cache snapshot for the Edit Params button. */
    current_test_id_        = spec.id;
    /* Schema v2: the editable knobs live in probe_params (map shape).
     * The legacy spec.params + spec.param_meanings are unused after
     * we dropped the v1 readers. */
    current_params_         = spec.probe_params;
    current_param_meanings_ = {};

    QDialog::show();
    raise();
    activateWindow();
}


void TestDescriptionDialog::setPersistTarget(TestRunnerWindow* target)
{
    persist_target_ = target;
    if (edit_params_btn_){
        edit_params_btn_->setVisible(target != nullptr);
    }
}


void TestDescriptionDialog::setAcquisitionTable(
    const QHash<QString, AcquisitionEntry>* tbl)
{
    acq_table_ = tbl;
}
