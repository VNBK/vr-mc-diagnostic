/*
 * TestResultDialog.cpp — see header for purpose.
 */
#include "TestResultDialog.hpp"
#include "TestRunnerWindow.hpp"        /* for TestSpec */

#include <QFormLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>


TestResultDialog::TestResultDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Test result"));
    setModal(false);
    setWindowFlags(Qt::Window);
    resize(680, 720);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(10, 10, 10, 10);

    /* Header — id / name / DB / status / duration. */
    auto* head = new QFormLayout();
    id_label_       = new QLabel(this);
    name_label_     = new QLabel(this);
    db_label_       = new QLabel(this);
    status_label_   = new QLabel(this);
    duration_label_ = new QLabel(this);
    samples_label_  = new QLabel(this);
    QFont bold = id_label_->font();
    bold.setBold(true);
    id_label_      ->setFont(bold);
    name_label_    ->setFont(bold);
    status_label_  ->setFont(bold);
    head->addRow(tr("ID:"),       id_label_);
    head->addRow(tr("Name:"),     name_label_);
    head->addRow(tr("Database:"), db_label_);
    head->addRow(tr("Status:"),   status_label_);
    head->addRow(tr("Duration:"), duration_label_);
    head->addRow(tr("Samples:"),  samples_label_);
    outer->addLayout(head);

    /* Description. */
    outer->addWidget(new QLabel(tr("<b>Description:</b>"), this));
    description_view_ = new QPlainTextEdit(this);
    description_view_->setReadOnly(true);
    description_view_->setMaximumHeight(100);
    outer->addWidget(description_view_);

    /* Pre-conditions. */
    outer->addWidget(new QLabel(tr("<b>Pre-conditions:</b>"), this));
    pre_conditions_list_ = new QListWidget(this);
    pre_conditions_list_->setMaximumHeight(80);
    outer->addWidget(pre_conditions_list_);

    /* Procedure. */
    outer->addWidget(new QLabel(tr("<b>Procedure:</b>"), this));
    procedure_list_ = new QListWidget(this);
    procedure_list_->setMaximumHeight(120);
    outer->addWidget(procedure_list_);

    /* Expected (read-only, monospace, light bg). */
    outer->addWidget(new QLabel(tr("<b>Expected result (pass criteria):</b>"), this));
    expected_view_ = new QLabel(this);
    expected_view_->setWordWrap(true);
    expected_view_->setStyleSheet(
        "QLabel { background: #ffffff; color: #1a1a1a; padding: 6px; "
        "border: 1px solid #888; font-family: monospace; }");
    outer->addWidget(expected_view_);

    /* Actual (multi-line for richer detail). */
    outer->addWidget(new QLabel(tr("<b>Actual result:</b>"), this));
    actual_view_ = new QPlainTextEdit(this);
    actual_view_->setReadOnly(true);
    actual_view_->setStyleSheet(
        "QPlainTextEdit { background: #ffffff; color: #1a1a1a; "
        "border: 1px solid #888; font-family: monospace; }");
    actual_view_->setMaximumHeight(100);
    outer->addWidget(actual_view_);

    /* Parameters — JSON-defined limits the test ran against. Sits
     * above Metrics so the eye reads "what we asked for" → "what we
     * measured". */
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
    params_table_->setMaximumHeight(140);
    outer->addWidget(params_table_);

    /* Metrics — table with Name, Meaning, Value columns. */
    outer->addWidget(new QLabel(tr("<b>Metrics:</b>"), this));
    metrics_table_ = new QTableWidget(0, 3, this);
    metrics_table_->setHorizontalHeaderLabels(
        {tr("Metric"), tr("Meaning"), tr("Value")});
    metrics_table_->horizontalHeader()->setStretchLastSection(false);
    metrics_table_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::ResizeToContents);
    metrics_table_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::Stretch);
    metrics_table_->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::Interactive);
    metrics_table_->setColumnWidth(2, 180);
    metrics_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    metrics_table_->setMaximumHeight(160);
    outer->addWidget(metrics_table_);

    /* Close. */
    auto* close_btn = new QPushButton(tr("Close"), this);
    close_btn->setDefault(true);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::hide);
    outer->addWidget(close_btn);
}


void TestResultDialog::show(const TestSpec& spec,
                             bool passed,
                             double duration_s,
                             const QString& actual,
                             const QHash<QString, QVariant>& metrics)
{
    id_label_      ->setText(spec.id);
    name_label_    ->setText(spec.name);
    db_label_      ->setText(spec.db_id);
    status_label_  ->setText(passed ? tr("PASS") : tr("FAIL"));
    status_label_  ->setStyleSheet(
        passed ? "color: #2e7d32; font-weight: bold;"
                : "color: #c62828; font-weight: bold;");
    duration_label_->setText(tr("%1 s").arg(duration_s, 0, 'f', 3));
    samples_label_ ->setText(spec.num_samples > 0
                                ? QString::number(spec.num_samples)
                                : tr("(unspecified)"));

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

    expected_view_->setText(spec.pass_criteria);
    actual_view_  ->setPlainText(actual);

    /* Render spec.probe_params as key=value pairs. Empty table →
     * operator knows the test has no tunable knobs (e.g. the bus-
     * enum sanity check that runs purely off snapshot captures). */
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

    /* Metrics table: drive row order from spec.captures so the report
     * stays predictable, look up each name's meaning in the acquisition
     * table, and take the value from the runtime metrics map. Extra
     * runtime keys (e.g. wildcard-expanded probe metrics) get appended
     * below with "(undeclared)" as their meaning. */
    metrics_table_->setRowCount(0);
    QSet<QString> seen;
    for (const QString& cap : spec.captures){
        const int row = metrics_table_->rowCount();
        metrics_table_->insertRow(row);
        metrics_table_->setItem(row, 0, new QTableWidgetItem(cap));
        QString meaning;
        if (acq_table_){
            const auto it = acq_table_->constFind(cap);
            meaning = (it != acq_table_->constEnd())
                ? it.value().meaning
                : tr("(not in acquisition_table)");
        }
        metrics_table_->setItem(row, 1, new QTableWidgetItem(meaning));
        metrics_table_->setItem(row, 2, new QTableWidgetItem(
            metrics.value(cap).toString()));
        seen.insert(cap);
    }
    /* Append any runtime keys that weren't declared in captures. */
    for (auto it = metrics.constBegin(); it != metrics.constEnd(); ++it){
        if (seen.contains(it.key())){ continue; }
        const int row = metrics_table_->rowCount();
        metrics_table_->insertRow(row);
        metrics_table_->setItem(row, 0, new QTableWidgetItem(it.key()));
        metrics_table_->setItem(row, 1, new QTableWidgetItem(tr("(undeclared)")));
        metrics_table_->setItem(row, 2, new QTableWidgetItem(it.value().toString()));
    }

    QDialog::show();
    raise();
    activateWindow();
}


void TestResultDialog::setAcquisitionTable(
    const QHash<QString, AcquisitionEntry>* tbl)
{
    acq_table_ = tbl;
}
