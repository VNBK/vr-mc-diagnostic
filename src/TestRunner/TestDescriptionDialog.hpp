/**
 * @file    TestDescriptionDialog.hpp
 * @brief   Non-modal popup that shows a single TestSpec's full metadata.
 *
 * Triggered by a click on any test row in the TestRunnerWindow's
 * suite tree. Stays open while the operator interacts with the rest
 * of the runner; calling @ref show again with a different spec
 * re-renders the popup in place instead of stacking a new window.
 */
#pragma once

#include <QDialog>
#include <QHash>
#include <QJsonObject>

struct TestSpec;
struct AcquisitionEntry;
class QLabel;
class QListWidget;
class QPlainTextEdit;


class TestRunnerWindow;            /* for the persist callback */


class TestDescriptionDialog : public QDialog
{
    Q_OBJECT
public:
    explicit TestDescriptionDialog(QWidget* parent = nullptr);

    /** @brief Re-render the popup for @p spec. Brings the window to
     *  the front if it's already open. */
    void show(const TestSpec& spec);

    /** @brief Wire the persist target so the Edit Params… button
     *  knows where to push edited values. Optional — if unset, the
     *  Edit button is hidden because there's nowhere to save to. */
    void setPersistTarget(TestRunnerWindow* target);

    /** @brief Wire the v2 acquisition-table reference. Used to look
     *  up each capture's meaning when rendering. */
    void setAcquisitionTable(const QHash<QString, AcquisitionEntry>* tbl);

signals:
    /** @brief Emitted after the user OKs the edit dialog AND the
     *  runner has written the new params back to tests.json. Caller
     *  can refresh derived UI. */
    void paramsEdited(const QString& test_id);

private:
    QLabel*         id_label_           = nullptr;
    QLabel*         name_label_         = nullptr;
    QLabel*         db_label_           = nullptr;
    QLabel*         duration_label_     = nullptr;
    QLabel*         samples_label_      = nullptr;
    QLabel*         blocks_label_       = nullptr;
    QPlainTextEdit* description_view_   = nullptr;
    QListWidget*    pre_conditions_list_= nullptr;
    QListWidget*    procedure_list_     = nullptr;
    /** @brief Cleanup actions surfaced from schema-v2 `teardown:` —
     *  documentation-only, lives between Procedure and Metrics. Hidden
     *  when the spec has no teardown so legacy files stay compact. */
    QLabel*         teardown_label_     = nullptr;
    QListWidget*    teardown_list_      = nullptr;
    QListWidget*    metrics_list_       = nullptr;
    QLabel*         pass_criteria_view_ = nullptr;
    QLabel*         depends_on_label_   = nullptr;
    /* Per-test knobs from spec.probe_params, rendered as a key=value
     * table so the operator sees the actual thresholds the test will
     * run against — they live in tests.yaml, no rebuild to tune. */
    class QTableWidget* params_table_   = nullptr;
    /* Button next to the params table — opens the type-aware editor.
     * Hidden when no persist target is wired. */
    class QPushButton*  edit_params_btn_ = nullptr;
    /* Cached snapshot for the Edit handler — last spec we rendered.
     * Keeping just the bits the editor needs avoids exposing TestSpec
     * by value through this header. */
    QString                 current_test_id_;
    QJsonObject             current_params_;
    QHash<QString, QString> current_param_meanings_;
    TestRunnerWindow*       persist_target_ = nullptr;
    /* Pointer to the runner-owned acquisition table — used to
     * resolve capture meanings in the metrics list. nullptr keeps
     * the legacy v1 metrics rendering path active. */
    const QHash<QString, AcquisitionEntry>* acq_table_ = nullptr;
};
