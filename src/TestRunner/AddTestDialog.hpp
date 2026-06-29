/**
 * @file    AddTestDialog.hpp
 * @brief   Full-form editor for a single TestSpec — both creating a
 *          new test (Add mode) and editing an existing one (Edit
 *          mode). Lays out every persisted field across tabs:
 *
 *            • Basic        — parent DB, id, name, description,
 *                              pass criteria, duration, samples,
 *                              blocks-if-fails
 *            • Steps        — pre-conditions and procedure, one
 *                              item per line in QPlainTextEdit
 *            • Metrics      — editable table (Name | Meaning)
 *            • Params       — editable table (Name | Value | Meaning),
 *                              value column tries to JSON-parse so the
 *                              schema preserves bool / number / array
 *                              types correctly
 *
 *  Returns a complete test JSON object in the new array-of-objects
 *  shape via @ref toTestJson(); the runner can hand that to
 *  addTest() (insert) or replaceTest() (overwrite by id).
 */
#pragma once

#include <QDialog>
#include <QJsonObject>
#include <QStringList>

struct TestSpec;
struct AcquisitionEntry;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPlainTextEdit;
class QSpinBox;
class QTableWidget;
class QTabWidget;


class AddTestDialog : public QDialog
{
    Q_OBJECT
public:
    enum class Mode { Add, Edit };

    /** @param mode      Add or Edit — controls window title, default
     *                   focus, and id-locking
     *  @param db_ids    available parent-DB ids ("DB1", "DB2", …) */
    explicit AddTestDialog(Mode mode,
                            const QStringList& db_ids,
                            QWidget* parent = nullptr);

    /** @brief Pre-fill the dialog from an existing spec. In Edit mode
     *  the id and parent-DB combo are locked because relocating /
     *  renaming during operator use would break run-queue references. */
    void loadFromSpec(const TestSpec& spec);

    /** @brief Set a suggested id (Add mode). No-op in Edit mode. */
    void setSuggestedId(const QString& id);

    /** @brief Set the initial parent-DB selection (Add mode). */
    void setSuggestedDb(const QString& db_id);

    /** @brief Parent DB id the operator picked. */
    QString parentDbId() const;

    /** @brief Full test JSON in the new merged shape — every field
     *  the parser knows about. params + metrics are array-of-objects,
     *  pre_conditions + procedure are arrays of strings. Caller
     *  inserts (Add) or replaces by id (Edit). */
    QJsonObject toTestJson() const;

    /** @brief Wire the runner's catalog so the Browse Catalog…
     *  button in the Captures tab can show what's available. Pass
     *  nullptr to hide the button. */
    void setAcquisitionTable(const QHash<QString, AcquisitionEntry>* tbl);

private:
    /** @brief Build a row in the params table from a name / JSON
     *  value / meaning triple. JSON value is rendered as its native
     *  toString form (true / 0.05 / [0,1,2]). */
    void appendParamsRow(const QString& name,
                          const QJsonValue& value,
                          const QString& meaning);
    void appendMetricsRow(const QString& name, const QString& meaning);

    /** @brief Append a row to the pass-criteria table. Each row carries
     *  one schema-v2 `{check, error_code, error_msg}` object — saved
     *  verbatim as YAML on accept. @p check is the full free-form
     *  expression (e.g. "max:probe.round_trip_ms <= max_round_trip_ms");
     *  the runner re-tokenises it into reducer:lhs OP rhs at load. */
    void appendPassRuleRow(const QString& check,
                            const QString& error_code,
                            const QString& error_msg);

    /** @brief Parse the operator-entered value cell as JSON. Falls
     *  back to QString when the text isn't a valid JSON literal. */
    static QJsonValue parseValueCell(const QString& text);

    Mode             mode_;
    QStringList      db_ids_;
    QTabWidget*      tabs_                = nullptr;
    QComboBox*       db_combo_            = nullptr;
    QLineEdit*       id_edit_             = nullptr;
    QLineEdit*       name_edit_           = nullptr;
    QPlainTextEdit*  description_edit_    = nullptr;
    QPlainTextEdit*  pre_conditions_edit_ = nullptr;
    QPlainTextEdit*  procedure_edit_      = nullptr;
    QTableWidget*    metrics_table_       = nullptr;
    QTableWidget*    params_table_        = nullptr;
    /* Pass-rules tab (Fork E1). Each row: LHS expr | Op combobox |
     * RHS expr. Combiner combo controls AND vs OR at the table top. */
    QTableWidget*    pass_rules_table_    = nullptr;
    class QComboBox* pass_combiner_combo_ = nullptr;
    QLineEdit*       pass_criteria_edit_  = nullptr;
    /* v2 Captures tab — multiline list of catalog names + optional
     * window_s spinbox and probe_params editor. Empty captures +
     * not_implemented checked → short-circuit PASS. */
    class QPlainTextEdit* captures_edit_       = nullptr;
    class QDoubleSpinBox* window_s_edit_       = nullptr;
    class QPlainTextEdit* probe_params_edit_   = nullptr;
    class QCheckBox*      not_implemented_chk_ = nullptr;
    class QLineEdit*      pending_reason_edit_ = nullptr;
    QDoubleSpinBox*  duration_edit_       = nullptr;
    QSpinBox*        num_samples_edit_    = nullptr;
    QCheckBox*       blocks_if_fails_     = nullptr;
    /* Catalog reference — when non-null, the Browse Catalog… button
     * in the Captures tab opens a read-only viewer on the runner's
     * acquisition_table. */
    class QPushButton*                       browse_catalog_btn_ = nullptr;
    const QHash<QString, AcquisitionEntry>*  acq_table_          = nullptr;
};
