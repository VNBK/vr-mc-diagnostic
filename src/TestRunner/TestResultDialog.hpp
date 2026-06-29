/**
 * @file    TestResultDialog.hpp
 * @brief   Non-modal popup showing one test's FULL outcome — spec
 *          (description + pre-conditions + procedure) AND result
 *          (status, duration, actual values, metrics with meanings).
 *
 *  Opened by double-clicking a row in @ref TestRunnerWindow's results
 *  table. Stays open while the operator interacts with the runner;
 *  re-showing for a different row re-renders this same window
 *  instead of stacking. Distinct from @ref TestDescriptionDialog
 *  (spec-only, shown via single-click) so the two readers don't get
 *  conflated.
 */
#pragma once

#include <QDialog>
#include <QHash>

struct TestSpec;
struct AcquisitionEntry;
class TestRunnerWindow;          /* for ResultRow's parent type */
class QLabel;
class QListWidget;
class QPlainTextEdit;
class QTableWidget;


class TestResultDialog : public QDialog
{
    Q_OBJECT
public:
    explicit TestResultDialog(QWidget* parent = nullptr);

    /**
     * @brief Re-render the popup for one test row.
     * @param spec   the TestSpec the row came from (full metadata)
     * @param status PASS / FAIL / SKIPPED — drives the status label
     *               colour
     * @param duration_s   measured wall-clock duration
     * @param actual       Actual Result string from the runner
     * @param metrics      key/value pairs from the test body
     */
    void show(const TestSpec& spec,
              bool passed,
              double duration_s,
              const QString& actual,
              const QHash<QString, QVariant>& metrics);

    /** @brief Wire the v2 acquisition-table reference. Used to look
     *  up each capture's meaning when rendering the metrics table. */
    void setAcquisitionTable(const QHash<QString, AcquisitionEntry>* tbl);

private:
    QLabel*         id_label_              = nullptr;
    QLabel*         name_label_            = nullptr;
    QLabel*         db_label_              = nullptr;
    QLabel*         status_label_          = nullptr;
    QLabel*         duration_label_        = nullptr;
    QLabel*         samples_label_         = nullptr;
    QPlainTextEdit* description_view_      = nullptr;
    QListWidget*    pre_conditions_list_   = nullptr;
    QListWidget*    procedure_list_        = nullptr;
    QLabel*         expected_view_         = nullptr;
    QPlainTextEdit* actual_view_           = nullptr;
    QTableWidget*   metrics_table_         = nullptr;
    /* JSON-defined knobs the test ran against. Same widget as the
     * description popup so the operator can compare "thresholds" vs
     * "actual values" at a glance. */
    QTableWidget*   params_table_          = nullptr;
    const QHash<QString, AcquisitionEntry>* acq_table_ = nullptr;
};
