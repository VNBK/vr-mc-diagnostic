/*
 * AddTestDialog.cpp — see header.
 */
#include "AddTestDialog.hpp"
#include "TestRunnerWindow.hpp"   /* for TestSpec */

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTabWidget>
#include <QVBoxLayout>


namespace {

/* Wrap @p widget in a small QWidget so we can put a +/- button row
 * underneath a QTableWidget and return one composite widget that the
 * tab layout can take. Keeps the dialog ctor readable. */
QWidget* tableWithRowButtons(QTableWidget* table,
                              QPushButton** out_add,
                              QPushButton** out_del,
                              QWidget* parent)
{
    auto* w = new QWidget(parent);
    auto* lay = new QVBoxLayout(w);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(table, 1);
    auto* row = new QHBoxLayout();
    row->addStretch(1);
    *out_add = new QPushButton(QObject::tr("+ Add row"), w);
    *out_del = new QPushButton(QObject::tr("− Remove selected"), w);
    row->addWidget(*out_add);
    row->addWidget(*out_del);
    lay->addLayout(row);
    return w;
}

}  // namespace


AddTestDialog::AddTestDialog(Mode mode,
                              const QStringList& db_ids,
                              QWidget* parent)
    : QDialog(parent), mode_(mode), db_ids_(db_ids)
{
    setWindowTitle(mode_ == Mode::Add ? tr("Add test case")
                                       : tr("Edit test case"));
    setModal(true);
    resize(720, 620);

    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(12, 12, 12, 12);

    auto* hint = new QLabel(
        mode_ == Mode::Add
            ? tr("Fill in every tab, then OK to create. params + "
                 "metrics use the new array-of-objects shape — leave "
                 "values blank to skip a row.")
            : tr("Edit any field across the tabs, then OK to save. "
                 "ID and parent DB are locked to keep run-queue "
                 "references stable."),
        this);
    hint->setWordWrap(true);
    outer->addWidget(hint);

    tabs_ = new QTabWidget(this);
    outer->addWidget(tabs_, 1);

    /* ===== Tab 1 — Basic =========================================== */
    auto* basic = new QWidget(this);
    auto* basic_form = new QFormLayout(basic);
    db_combo_ = new QComboBox(basic);
    db_combo_->addItems(db_ids_);
    basic_form->addRow(tr("Parent database:"), db_combo_);

    id_edit_ = new QLineEdit(basic);
    id_edit_->setPlaceholderText(tr("e.g. DB1-09"));
    basic_form->addRow(tr("Test ID:"), id_edit_);

    name_edit_ = new QLineEdit(basic);
    name_edit_->setPlaceholderText(tr("e.g. Brake-release latency"));
    basic_form->addRow(tr("Name:"), name_edit_);

    description_edit_ = new QPlainTextEdit(basic);
    description_edit_->setPlaceholderText(
        tr("Multi-paragraph rationale — what this test measures and why."));
    description_edit_->setMaximumHeight(120);
    basic_form->addRow(tr("Description:"), description_edit_);

    pass_criteria_edit_ = new QLineEdit(basic);
    pass_criteria_edit_->setPlaceholderText(
        tr("Boolean expression — e.g. trials_acked == trials_total AND max <= cap"));
    basic_form->addRow(tr("Pass criteria:"), pass_criteria_edit_);

    duration_edit_ = new QDoubleSpinBox(basic);
    duration_edit_->setRange(0.0, 3600.0);
    duration_edit_->setDecimals(2);
    duration_edit_->setSingleStep(0.5);
    basic_form->addRow(tr("Duration estimate (s):"), duration_edit_);

    num_samples_edit_ = new QSpinBox(basic);
    num_samples_edit_->setRange(0, 1000000);
    basic_form->addRow(tr("Num samples:"), num_samples_edit_);

    blocks_if_fails_ = new QCheckBox(tr("Failure aborts run"), basic);
    basic_form->addRow(tr("Blocks:"), blocks_if_fails_);

    tabs_->addTab(basic, tr("Basic"));

    /* ===== Tab 2 — Steps (pre-conditions + procedure) ============== */
    auto* steps = new QWidget(this);
    auto* steps_lay = new QVBoxLayout(steps);
    steps_lay->addWidget(new QLabel(tr("Pre-conditions (one per line):"), steps));
    pre_conditions_edit_ = new QPlainTextEdit(steps);
    pre_conditions_edit_->setPlaceholderText(
        tr("Hand powered and connected.\nOperator clear of moving parts."));
    steps_lay->addWidget(pre_conditions_edit_, 1);
    steps_lay->addWidget(new QLabel(tr("Procedure (one step per line):"), steps));
    procedure_edit_ = new QPlainTextEdit(steps);
    procedure_edit_->setPlaceholderText(
        tr("Open hand.\nVerify dof == expected_dof.\nVerify joint names non-empty."));
    steps_lay->addWidget(procedure_edit_, 1);
    tabs_->addTab(steps, tr("Steps"));

    /* ===== Tab 3 — Metrics ========================================= */
    metrics_table_ = new QTableWidget(0, 2, this);
    metrics_table_->setHorizontalHeaderLabels({tr("Name"), tr("Meaning")});
    metrics_table_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::ResizeToContents);
    metrics_table_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::Stretch);
    metrics_table_->verticalHeader()->setVisible(false);
    QPushButton *m_add = nullptr, *m_del = nullptr;
    QWidget* metrics_pane = tableWithRowButtons(metrics_table_,
                                                  &m_add, &m_del, this);
    connect(m_add, &QPushButton::clicked, this, [this]{
        appendMetricsRow(QString(), QString());
        metrics_table_->setCurrentCell(metrics_table_->rowCount() - 1, 0);
    });
    connect(m_del, &QPushButton::clicked, this, [this]{
        const int r = metrics_table_->currentRow();
        if (r >= 0){ metrics_table_->removeRow(r); }
    });
    tabs_->addTab(metrics_pane, tr("Metrics"));

    /* ===== Tab 4 — Params ========================================== */
    params_table_ = new QTableWidget(0, 3, this);
    params_table_->setHorizontalHeaderLabels(
        {tr("Name"), tr("Value (JSON)"), tr("Meaning")});
    params_table_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::ResizeToContents);
    params_table_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents);
    params_table_->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::Stretch);
    params_table_->verticalHeader()->setVisible(false);
    QPushButton *p_add = nullptr, *p_del = nullptr;
    QWidget* params_pane = tableWithRowButtons(params_table_,
                                                &p_add, &p_del, this);
    connect(p_add, &QPushButton::clicked, this, [this]{
        appendParamsRow(QString(), QJsonValue(QString()), QString());
        params_table_->setCurrentCell(params_table_->rowCount() - 1, 0);
    });
    connect(p_del, &QPushButton::clicked, this, [this]{
        const int r = params_table_->currentRow();
        if (r >= 0){ params_table_->removeRow(r); }
    });
    tabs_->addTab(params_pane, tr("Params"));

    /* ===== Tab 5 — Pass rules (Fork E1) =========================== */
    auto* rules_tab = new QWidget(this);
    auto* rules_lay = new QVBoxLayout(rules_tab);
    auto* combiner_row = new QHBoxLayout();
    combiner_row->addWidget(new QLabel(tr("Combiner:"), rules_tab));
    pass_combiner_combo_ = new QComboBox(rules_tab);
    pass_combiner_combo_->addItems({tr("AND"), tr("OR")});
    combiner_row->addWidget(pass_combiner_combo_);
    combiner_row->addStretch(1);
    rules_lay->addLayout(combiner_row);

    /* Schema-v2 pass_criteria editor: one row per `{check, error_code,
     * error_msg}` object. `check` is a single free-form expression
     * (the runner tokenises it into reducer:lhs OP rhs); error_code +
     * error_msg are surfaced on rule failure so the operator sees the
     * spec-author's diagnosis alongside the numeric mismatch. */
    pass_rules_table_ = new QTableWidget(0, 3, rules_tab);
    pass_rules_table_->setHorizontalHeaderLabels(
        {tr("Check"), tr("Error code"), tr("Error message")});
    pass_rules_table_->horizontalHeader()->setSectionResizeMode(
        0, QHeaderView::Stretch);
    pass_rules_table_->horizontalHeader()->setSectionResizeMode(
        1, QHeaderView::ResizeToContents);
    pass_rules_table_->horizontalHeader()->setSectionResizeMode(
        2, QHeaderView::Stretch);
    pass_rules_table_->verticalHeader()->setVisible(false);
    pass_rules_table_->setWordWrap(true);
    QPushButton *r_add = nullptr, *r_del = nullptr;
    QWidget* rules_pane = tableWithRowButtons(pass_rules_table_,
                                                &r_add, &r_del, this);
    connect(r_add, &QPushButton::clicked, this, [this]{
        appendPassRuleRow(QString(), QString(), QString());
        pass_rules_table_->setCurrentCell(
            pass_rules_table_->rowCount() - 1, 0);
    });
    connect(r_del, &QPushButton::clicked, this, [this]{
        const int r = pass_rules_table_->currentRow();
        if (r >= 0){ pass_rules_table_->removeRow(r); }
    });
    rules_lay->addWidget(rules_pane, 1);

    /* Inline syntax cheat-sheet for the `check:` expression. */
    auto* hint_label = new QLabel(rules_tab);
    hint_label->setWordWrap(true);
    hint_label->setText(tr(
        "<b>Check</b>: <code>&lt;lhs&gt; &lt;op&gt; &lt;rhs&gt;</code> "
        "where <b>op</b> is one of <code>==</code> <code>!=</code> "
        "<code>&lt;</code> <code>&lt;=</code> <code>&gt;</code> "
        "<code>&gt;=</code>.<br>"
        "<b>lhs</b>: bare metric name, or <code>&lt;reducer&gt;:name</code> "
        "for arrays — reducers: <code>max</code> <code>min</code> "
        "<code>mean</code> <code>p99</code> <code>sum</code> "
        "<code>stdev</code> <code>range</code> <code>cv</code> "
        "<code>len</code> <code>count_nonzero</code> "
        "<code>div:a/b</code>.<br>"
        "<b>rhs</b>: literal (<code>100</code>, <code>true</code>) or "
        "bare identifier (resolved against metrics / probe_params at "
        "run time).<br>"
        "<b>Error code</b> + <b>Error message</b> are optional; they "
        "tag the failure reason if the check trips."));
    hint_label->setStyleSheet("QLabel { color: #555; padding: 4px; "
                               "border-top: 1px solid #ddd; }");
    rules_lay->addWidget(hint_label);

    tabs_->addTab(rules_tab, tr("Pass criteria"));

    /* ===== Tab 6 — Captures (schema v2 — Fork P3-a) =============== */
    auto* cap_tab = new QWidget(this);
    auto* cap_lay = new QVBoxLayout(cap_tab);

    auto* cap_hint = new QLabel(tr(
        "<b>captures</b>: one acquisition-table entry per line "
        "(e.g. <code>state.dof</code>, <code>telemetry.frames</code>, "
        "<code>probe.round_trip_ms</code>). The runner groups by "
        "kind and runs each group. See Help ▸ Field reference for "
        "the available kinds."), cap_tab);
    cap_hint->setWordWrap(true);
    cap_lay->addWidget(cap_hint);
    captures_edit_ = new QPlainTextEdit(cap_tab);
    captures_edit_->setPlaceholderText(
        tr("state.dof\nstate.joint_names\nconfig.expected_dof"));
    cap_lay->addWidget(captures_edit_, 1);

    /* Browse Catalog… — non-modal viewer on the runner's
     * acquisition_table. Hidden until the runner wires the table
     * via setAcquisitionTable(). */
    auto* browse_row = new QHBoxLayout();
    browse_row->addStretch(1);
    browse_catalog_btn_ = new QPushButton(tr("Browse Catalog…"), cap_tab);
    browse_catalog_btn_->setToolTip(
        tr("Open a read-only viewer on the acquisition_table — every "
           "measurable quantity by name, kind, and meaning."));
    browse_catalog_btn_->setVisible(false);
    browse_row->addWidget(browse_catalog_btn_);
    cap_lay->addLayout(browse_row);
    connect(browse_catalog_btn_, &QPushButton::clicked, this, [this]{
        if (!acq_table_){ return; }
        QDialog dlg(this);
        dlg.setWindowTitle(tr("Acquisition table"));
        dlg.resize(820, 560);
        auto* lay = new QVBoxLayout(&dlg);
        auto* hint = new QLabel(
            tr("Catalog of every measurable quantity. Use these names "
               "in your <code>captures</code> list. Read-only here — "
               "edit by hand in tests.json."), &dlg);
        hint->setWordWrap(true);
        lay->addWidget(hint);
        auto* tbl = new QTableWidget(0, 6, &dlg);
        tbl->setHorizontalHeaderLabels(
            {tr("Name"), tr("Kind"), tr("Source / Stat / Probe"),
             tr("Output"), tr("Type"), tr("Meaning")});
        tbl->horizontalHeader()->setSectionResizeMode(
            0, QHeaderView::ResizeToContents);
        tbl->horizontalHeader()->setSectionResizeMode(
            1, QHeaderView::ResizeToContents);
        tbl->horizontalHeader()->setSectionResizeMode(
            2, QHeaderView::ResizeToContents);
        tbl->horizontalHeader()->setSectionResizeMode(
            3, QHeaderView::ResizeToContents);
        tbl->horizontalHeader()->setSectionResizeMode(
            4, QHeaderView::ResizeToContents);
        tbl->horizontalHeader()->setSectionResizeMode(
            5, QHeaderView::Stretch);
        tbl->verticalHeader()->setVisible(false);
        tbl->setEditTriggers(QAbstractItemView::NoEditTriggers);
        tbl->setSelectionBehavior(QAbstractItemView::SelectRows);
        tbl->setSortingEnabled(true);
        /* Sort A→Z by name out of the gate so the catalog reads
         * left-to-right regardless of JSON authoring order. */
        for (auto it = acq_table_->constBegin();
                  it != acq_table_->constEnd(); ++it){
            const auto& e = it.value();
            const QString sourceish = !e.source.isEmpty() ? e.source
                                       : !e.stat.isEmpty() ? e.stat
                                       : e.probe;
            const int r = tbl->rowCount();
            tbl->insertRow(r);
            tbl->setItem(r, 0, new QTableWidgetItem(it.key()));
            tbl->setItem(r, 1, new QTableWidgetItem(e.kind));
            tbl->setItem(r, 2, new QTableWidgetItem(sourceish));
            tbl->setItem(r, 3, new QTableWidgetItem(e.output));
            tbl->setItem(r, 4, new QTableWidgetItem(e.type_hint));
            tbl->setItem(r, 5, new QTableWidgetItem(e.meaning));
        }
        tbl->sortByColumn(0, Qt::AscendingOrder);
        lay->addWidget(tbl, 1);

        /* Double-click → append the name to the captures edit, so
         * the operator can browse and pick without retyping. */
        connect(tbl, &QTableWidget::cellDoubleClicked, &dlg,
                [this, tbl, &dlg](int row, int /*col*/){
            const auto* item = tbl->item(row, 0);
            if (!item){ return; }
            const QString name = item->text();
            QString cur = captures_edit_->toPlainText();
            if (!cur.isEmpty() && !cur.endsWith('\n')){ cur += '\n'; }
            cur += name;
            captures_edit_->setPlainText(cur);
            dlg.accept();
        });

        auto* close_btn = new QPushButton(tr("Close"), &dlg);
        close_btn->setDefault(true);
        connect(close_btn, &QPushButton::clicked, &dlg, &QDialog::accept);
        lay->addWidget(close_btn);
        dlg.exec();
    });

    auto* window_row = new QHBoxLayout();
    window_row->addWidget(new QLabel(tr("window_s:"), cap_tab));
    window_s_edit_ = new QDoubleSpinBox(cap_tab);
    window_s_edit_->setRange(0.0, 3600.0);
    window_s_edit_->setDecimals(2);
    window_s_edit_->setSingleStep(0.5);
    window_s_edit_->setSuffix(tr(" s"));
    window_row->addWidget(window_s_edit_);
    window_row->addWidget(new QLabel(
        tr("  (only used when captures include window_agg)"), cap_tab));
    window_row->addStretch(1);
    cap_lay->addLayout(window_row);

    cap_lay->addWidget(new QLabel(
        tr("<b>probe_params</b> (JSON object, only used when captures "
           "include probe entries):"), cap_tab));
    probe_params_edit_ = new QPlainTextEdit(cap_tab);
    probe_params_edit_->setPlaceholderText(
        tr("{\n  \"probe\": \"round_trip\",\n  \"joint_index\": 3,\n  \"trials\": 20\n}"));
    probe_params_edit_->setMaximumHeight(160);
    cap_lay->addWidget(probe_params_edit_);

    auto* pending_row = new QHBoxLayout();
    not_implemented_chk_ = new QCheckBox(tr("not_implemented"), cap_tab);
    pending_reason_edit_ = new QLineEdit(cap_tab);
    pending_reason_edit_->setPlaceholderText(
        tr("Reason shown in actual result when not_implemented is true."));
    pending_row->addWidget(not_implemented_chk_);
    pending_row->addWidget(pending_reason_edit_, 1);
    cap_lay->addLayout(pending_row);

    tabs_->addTab(cap_tab, tr("Captures"));

    /* Footer OK / Cancel. */
    auto* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]{
        if (id_edit_->text().trimmed().isEmpty()){
            tabs_->setCurrentIndex(0); id_edit_->setFocus(); return;
        }
        if (name_edit_->text().trimmed().isEmpty()){
            tabs_->setCurrentIndex(0); name_edit_->setFocus(); return;
        }
        accept();
    });
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);

    /* In Edit mode lock the structural fields. */
    if (mode_ == Mode::Edit){
        id_edit_->setReadOnly(true);
        id_edit_->setToolTip(tr("ID is locked in Edit mode."));
        db_combo_->setEnabled(false);
        db_combo_->setToolTip(tr("Parent DB is locked in Edit mode."));
    }
}


void AddTestDialog::loadFromSpec(const TestSpec& spec)
{
    if (db_combo_->findText(spec.db_id) >= 0){
        db_combo_->setCurrentText(spec.db_id);
    }
    id_edit_     ->setText(spec.id);
    name_edit_   ->setText(spec.name);
    description_edit_->setPlainText(spec.description);
    pass_criteria_edit_->setText(spec.pass_criteria);
    duration_edit_->setValue(spec.duration_estimate_s);
    num_samples_edit_->setValue(spec.num_samples);
    blocks_if_fails_->setChecked(spec.blocks_if_fails);

    pre_conditions_edit_->setPlainText(spec.pre_conditions.join('\n'));
    procedure_edit_     ->setPlainText(spec.procedure.join('\n'));

    /* Schema-v2 no longer carries spec.metrics / spec.params /
     * spec.window_s / spec.not_implemented / spec.pending_reason, so
     * we clear the orphan widgets here without trying to populate
     * them. The widgets stay in the layout for binary compatibility
     * with this iteration of the dialog; a future cleanup pass can
     * remove them outright. */
    metrics_table_->setRowCount(0);
    params_table_ ->setRowCount(0);

    captures_edit_->setPlainText(spec.captures.join('\n'));
    if (window_s_edit_){       window_s_edit_      ->setValue(0.0); }
    if (not_implemented_chk_){ not_implemented_chk_->setChecked(false); }
    if (pending_reason_edit_){ pending_reason_edit_->clear(); }
    if (!spec.probe_params.isEmpty()){
        probe_params_edit_->setPlainText(
            QString::fromUtf8(QJsonDocument(spec.probe_params)
                                .toJson(QJsonDocument::Indented)));
    }

    pass_combiner_combo_->setCurrentText(
        spec.pass_combiner.isEmpty() ? "AND" : spec.pass_combiner);
    pass_rules_table_->setRowCount(0);
    for (const auto& rule : spec.pass_rules){
        /* Reconstruct the `check:` expression. rule.lhs_raw kept the
         * original `<reducer>:name` form; the `$` on a context-ref RHS
         * is stripped on the way back into the cell because schema-v2
         * treats bare identifiers as context lookups automatically —
         * the user can re-add `$` if they want the legacy explicit
         * form. */
        QString rhs_text;
        switch (rule.rhs.type()){
            case QJsonValue::Bool:   rhs_text = rule.rhs.toBool() ? "true" : "false"; break;
            case QJsonValue::Double: rhs_text = QString::number(rule.rhs.toDouble(), 'g', 10); break;
            case QJsonValue::String: {
                const QString s = rule.rhs.toString();
                rhs_text = s.startsWith('$') ? s.mid(1) : s;
                break;
            }
            default:                 rhs_text.clear();
        }
        QString check;
        if (!rule.lhs_raw.isEmpty() && !rule.op.isEmpty()){
            check = QStringLiteral("%1 %2 %3")
                       .arg(rule.lhs_raw, rule.op, rhs_text);
        } else {
            check = rule.lhs_raw;   /* fallback when load couldn't tokenise */
        }
        appendPassRuleRow(check, rule.error_code, rule.error_msg);
    }
}


void AddTestDialog::setSuggestedId(const QString& id)
{
    if (mode_ == Mode::Add){ id_edit_->setText(id); }
}


void AddTestDialog::setSuggestedDb(const QString& db_id)
{
    if (mode_ == Mode::Edit){ return; }
    const int idx = db_combo_->findText(db_id);
    if (idx >= 0){ db_combo_->setCurrentIndex(idx); }
}


QString AddTestDialog::parentDbId() const
{
    return db_combo_->currentText();
}


void AddTestDialog::appendMetricsRow(const QString& name,
                                       const QString& meaning)
{
    const int r = metrics_table_->rowCount();
    metrics_table_->insertRow(r);
    metrics_table_->setItem(r, 0, new QTableWidgetItem(name));
    metrics_table_->setItem(r, 1, new QTableWidgetItem(meaning));
}


void AddTestDialog::appendPassRuleRow(const QString& check,
                                        const QString& error_code,
                                        const QString& error_msg)
{
    const int r = pass_rules_table_->rowCount();
    pass_rules_table_->insertRow(r);
    pass_rules_table_->setItem(r, 0, new QTableWidgetItem(check));
    pass_rules_table_->setItem(r, 1, new QTableWidgetItem(error_code));
    pass_rules_table_->setItem(r, 2, new QTableWidgetItem(error_msg));
}


void AddTestDialog::appendParamsRow(const QString& name,
                                      const QJsonValue& value,
                                      const QString& meaning)
{
    const int r = params_table_->rowCount();
    params_table_->insertRow(r);
    params_table_->setItem(r, 0, new QTableWidgetItem(name));
    /* Render value as canonical JSON so round-trip is type-stable
     * (booleans stay booleans, arrays stay arrays). */
    QString rendered;
    switch (value.type()){
        case QJsonValue::Bool:    rendered = value.toBool() ? "true" : "false"; break;
        case QJsonValue::Double:  rendered = QString::number(value.toDouble(), 'g', 10); break;
        case QJsonValue::String:  rendered = value.toString(); break;
        case QJsonValue::Array:
        case QJsonValue::Object:
            rendered = QJsonDocument(value.isArray()
                                      ? QJsonDocument(value.toArray())
                                      : QJsonDocument(value.toObject()))
                            .toJson(QJsonDocument::Compact);
            break;
        default: rendered = QString();
    }
    params_table_->setItem(r, 1, new QTableWidgetItem(rendered));
    params_table_->setItem(r, 2, new QTableWidgetItem(meaning));
}


void AddTestDialog::setAcquisitionTable(
    const QHash<QString, AcquisitionEntry>* tbl)
{
    acq_table_ = tbl;
    if (browse_catalog_btn_){
        browse_catalog_btn_->setVisible(tbl != nullptr);
    }
}


QJsonValue AddTestDialog::parseValueCell(const QString& text)
{
    const QString t = text.trimmed();
    if (t.isEmpty()){ return QJsonValue(QString()); }
    /* Try parsing as JSON by wrapping in an array so primitives parse
     * (raw JSON top-level must be array or object). */
    QJsonParseError err{};
    const auto doc = QJsonDocument::fromJson(
        ("[" + t + "]").toUtf8(), &err);
    if (err.error == QJsonParseError::NoError
        && doc.isArray() && doc.array().size() == 1){
        return doc.array().first();
    }
    return QJsonValue(text);
}


QJsonObject AddTestDialog::toTestJson() const
{
    QJsonObject t;
    t.insert("id",            id_edit_->text().trimmed());
    t.insert("name",          name_edit_->text().trimmed());
    t.insert("description",   description_edit_->toPlainText());

    auto split_lines = [](const QString& blob){
        QJsonArray arr;
        for (const QString& line : blob.split('\n', Qt::SkipEmptyParts)){
            const QString s = line.trimmed();
            if (!s.isEmpty()){ arr.append(s); }
        }
        return arr;
    };
    t.insert("pre_conditions", split_lines(pre_conditions_edit_->toPlainText()));
    t.insert("procedure",      split_lines(procedure_edit_     ->toPlainText()));

    /* (schema-v1 `metrics: [{name, meaning}]`, `params: [...]`, plain-
     * string `pass_criteria`, `window_s`, `not_implemented`, and
     * `pending_reason` saves dropped — schema v2 uses captures,
     * probe_params, and structured pass_criteria as their replacements.
     * The orphan UI controls left in this dialog are harmless until
     * they get retired in a follow-up cleanup pass.) */

    /* Pass-criteria — walk the editable table and emit the schema-v2
     * `pass_criteria: [{check, error_code, error_msg}]` shape. The
     * dialog now exposes the full check expression as a single column
     * so the operator types `max:probe.round_trip_ms <= max_round_trip_ms`
     * exactly as it lands in YAML; error_code + error_msg ride along
     * verbatim. Rows with an empty check are skipped. */
    QJsonArray criteria;
    for (int r = 0; r < pass_rules_table_->rowCount(); ++r){
        auto cellText = [&](int col){
            return pass_rules_table_->item(r, col)
                ? pass_rules_table_->item(r, col)->text().trimmed()
                : QString{};
        };
        const QString check      = cellText(0);
        const QString error_code = cellText(1);
        const QString error_msg  = cellText(2);
        if (check.isEmpty()){ continue; }
        QJsonObject o;
        o.insert("check",      check);
        o.insert("error_code", error_code);
        o.insert("error_msg",  error_msg);
        criteria.append(o);
    }
    t.insert("pass_criteria",  criteria);
    t.insert("pass_combiner",  pass_combiner_combo_->currentText());

    /* captures (acquisition-table names) + probe_params (map of knobs
     * passed to the probe). teardown comes from a future text field;
     * for now the dialog has no editor for it so we never write the
     * key — load preserves it round-trip via the in-memory list. */
    QJsonArray captures;
    for (const QString& line :
            captures_edit_->toPlainText().split('\n', Qt::SkipEmptyParts)){
        const QString s = line.trimmed();
        if (!s.isEmpty()){ captures.append(s); }
    }
    t.insert("captures", captures);
    {
        const QString pp_text = probe_params_edit_->toPlainText().trimmed();
        if (!pp_text.isEmpty()){
            QJsonParseError err{};
            const auto doc = QJsonDocument::fromJson(pp_text.toUtf8(), &err);
            if (err.error == QJsonParseError::NoError && doc.isObject()){
                t.insert("probe_params", doc.object());
            }
        }
    }

    t.insert("duration_estimate_s",duration_edit_->value());
    t.insert("num_samples",        num_samples_edit_->value());
    t.insert("blocks_if_fails",    blocks_if_fails_->isChecked());
    return t;
}
