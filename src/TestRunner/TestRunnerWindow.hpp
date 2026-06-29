/**
 * @file    TestRunnerWindow.hpp
 * @brief   Standalone test-runner top-level window.
 *
 * Lives outside the main diagnostic — opened via Tools → Test Runner.
 * Loads a tests.yaml (DB1 + DB2 + ... + DB5), shows a tree of
 * databases / tests, and runs the selected set against the connected
 * hand. Per-test descriptions are surfaced as click-to-see popups.
 *
 * Threading: the runner orchestrates from the UI thread. Actual test
 * bodies run as queued slots on the existing HandWorker thread (so
 * they share the live SDK connection) and emit progress signals back
 * here. No new threads are spawned in this class — keeps signal
 * routing predictable.
 *
 * Test bodies themselves are NOT in this header — they'll be small
 * standalone functions registered with a factory map. This file is
 * the UI shell + orchestration only.
 */
#pragma once

#include <QWidget>
#include <QJsonObject>

#include <memory>

class HandWorker;
class QTreeWidget;
class QTreeWidgetItem;
class QPushButton;
class QCheckBox;
class QPlainTextEdit;
class QTextEdit;
class QLabel;
class QTableWidget;
class TestDescriptionDialog;


/**
 * @brief Metadata for a single test as loaded from tests.yaml.
 *
 * Mirrors the schema 1:1 — easier to pass-through and render than to
 * normalise prematurely. The runner only needs id + the execution
 * fields; everything else exists for the description popup and the
 * generated report.
 */
/**
 * @brief One entry of the schema-v2 acquisition table.
 *
 *  Tests reference these by name in their @c captures list. The
 *  runner dispatches on @c kind to decide where to pull the value
 *  from at run time:
 *      - "snapshot"   → pull from cached HandWorker state
 *      - "config"     → pull from file-header fields (expected_dof, ...)
 *      - "window_agg" → compute from a telemetry-window aggregator
 *      - "probe"      → harvest from a probe's batchDone metrics
 */
struct AcquisitionEntry
{
    QString kind;          /**< snapshot / config / window_agg / probe */
    QString source;        /**< snapshot / config: which state field */
    QString stat;          /**< window_agg: which aggregator stat */
    QString probe;         /**< probe: which probe class */
    QString output;        /**< probe: which key in the probe's metrics */
    QString type_hint;     /**< type label for the UI: "int", "double", "array<double>", … */
    QString meaning;       /**< human-readable description, shown in catalog UI */
};


struct TestSpec
{
    QString     id;                 /**< e.g. "DB1-03" */
    QString     name;               /**< human label */
    QString     description;        /**< multi-paragraph rationale */
    /** @brief Pre-conditions: gating checks the operator (or runner) must
     *  satisfy before the test body runs. Schema accepts either a list
     *  of strings (legacy) or a list of objects
     *  `{check, on_fail: skip|fail, msg}`; objects get stringified
     *  into a single human-readable line here so existing render
     *  code keeps working unchanged. */
    QStringList pre_conditions;
    QStringList procedure;          /**< step list */
    /** @brief Cleanup actions to run after the test body (e.g. "drive
     *  joint back to home"). Documentation-only today — surfaced in
     *  the description popup so the operator knows what cleanup ran. */
    QStringList teardown;
    /** @brief List of acquisition-table entries to materialise into
     *  the metrics object before pass_criteria evaluation. */
    QStringList captures;
    /** @brief Probe configuration object — the "probe" sub-key picks
     *  the probe class (round_trip / bandwidth_sweep / ...) and the
     *  remaining keys are passed to the probe's startBatch() verbatim.
     *  Bare-identifier $names in pass_criteria check expressions
     *  also fall through to this map. */
    QJsonObject probe_params;
    /** @brief Human-readable multi-line summary synthesized from the
     *  schema-v2 pass_criteria array (one line per `check:`). Shown
     *  as the "Expected Result" column in the suite tree + result
     *  popup. The structured rules themselves live in @ref pass_rules
     *  for the evaluator. */
    QString     pass_criteria;
    /** @brief Structured PASS/FAIL rules (Fork E1) — a flat list of
     *  comparisons against metrics + params evaluated after the test
     *  body completes. When non-empty, this list is AUTHORITATIVE
     *  over whatever boolean the test body computed; when empty, the
     *  body's hint is used (covers short-circuit cases like
     *  implemented:false). Combined with @c pass_combiner (default
     *  "AND"). */
    struct PassRule
    {
        enum class Reducer
        {
            None,           /**< plain metric / param scalar */
            Max, Min, Mean, P99, Sum,
            Stdev,          /**< sample stdev (N-1 denominator) */
            Range,          /**< max - min */
            Cv,             /**< coefficient of variation = stdev / mean */
            Len, CountNonzero,
            Div             /**< lhs_arg / lhs_arg2 */
        };
        QString    lhs_raw;     /**< canonical "max:round_trip_ms[]" — kept for the failure-reason printer */
        Reducer    lhs_reducer = Reducer::None;
        QString    lhs_arg;     /**< primary operand (metric / param name) */
        QString    lhs_arg2;    /**< Div secondary operand only */
        QString    op;          /**< "==", "!=", "<=", ">=", "<", ">" */
        QJsonValue rhs;         /**< literal value, OR "$name" for context lookup */
        /** @brief Optional metadata carried from schema-v2 pass_criteria
         *  objects: `{check, error_code, error_msg}`. error_code is a
         *  short symbol the report can group failures by; error_msg is
         *  the human-readable line shown when the rule fails. Empty for
         *  legacy pass_rules entries. */
        QString    error_code;
        QString    error_msg;
    };
    QList<PassRule> pass_rules;
    QString         pass_combiner = QStringLiteral("AND");   /**< "AND" | "OR" */
    double      duration_estimate_s = 0.0;
    /** @brief Sample / trial count the test performs. Distinct from
     *  duration: a 30 s test that captures 600 frames has
     *  duration_estimate_s = 30 and num_samples = 600. For repeatability
     *  tests this is "trials per pose". Defaults to 0 = unspecified. */
    int         num_samples = 0;
    bool        blocks_if_fails    = false;
    QStringList depends_on;         /**< other test ids that must PASS first */
    QString     db_id;              /**< parent database id */
};


/**
 * @brief Metadata for one test database (DB1, DB2, ...).
 */
struct TestDatabase
{
    QString          id;
    QString          name;
    QString          description;
    QList<TestSpec>  tests;
};


/**
 * @brief Standalone Qt window hosting the suite tree, run controls,
 *        live log, and results summary.
 *
 *  Non-modal — opened by MainWindow via Tools → Test Runner. Owns the
 *  parsed JSON tree, a reference to the live HandWorker (NOT a copy
 *  — runs against the operator's existing connection), and the
 *  in-progress run state.
 */
class TestRunnerWindow : public QWidget
{
    Q_OBJECT
public:
    /**
     * @param worker  Live HandWorker — must outlive this window. The
     *                runner enqueues test bodies to worker_'s thread
     *                via QMetaObject::invokeMethod, so the worker
     *                handles the SDK calls and we just observe.
     */
    explicit TestRunnerWindow(HandWorker* worker, QWidget* parent = nullptr);
    ~TestRunnerWindow() override;

    /** @brief Load a tests.yaml (replaces the in-memory tree). Auto-
     *  called once at construction with the bundled per-HandName
     *  file when the operator connects to a known hand. JSON files
     *  are still accepted because YAML is a JSON superset; new files
     *  are written as YAML. */
    bool loadConfig(const QString& path, QString* out_error = nullptr);

    /** @brief Persist a fresh @c params block for one test, both into
     *  the in-memory tree and back to the loaded tests.yaml file.
     *  Round-trips the on-disk JSON (read → mutate just this test's
     *  params → write) so any other fields stay untouched. Returns
     *  false + out_error when the file isn't writable, the test id
     *  isn't found, or parsing fails. Called from
     *  @ref TestDescriptionDialog when the operator OKs the editor. */
    bool writeParams(const QString& test_id,
                     const QJsonObject& new_params,
                     QString* out_error = nullptr);

    /** @brief Insert a brand-new test (TestSpec-shaped JSON) under
     *  @p db_id, both in-memory and persisted to tests.yaml. Refuses
     *  to overwrite an existing id. */
    bool addTest(const QString& db_id,
                  const QJsonObject& test_json,
                  QString* out_error = nullptr);

    /** @brief Remove a test by id from memory and from tests.yaml.
     *  Refuses when the id isn't found. */
    bool deleteTest(const QString& test_id,
                     QString* out_error = nullptr);

    /** @brief Patch metadata-only fields (name + description) for
     *  an existing test. params + metrics + everything else stays
     *  untouched on disk. */
    bool updateTestMeta(const QString& test_id,
                         const QJsonObject& meta,
                         QString* out_error = nullptr);

    /** @brief Replace every field of an existing test with the JSON
     *  produced by @ref AddTestDialog::toTestJson(). Used by the
     *  Edit-testcase flow once it's allowed to mutate any field. */
    bool replaceTest(const QString& test_id,
                      const QJsonObject& test_json,
                      QString* out_error = nullptr);

    /** @brief Append a brand-new TestDatabase block. Refuses to
     *  overwrite an existing id. */
    bool addDatabase(const QString& id,
                      const QString& name,
                      const QString& description,
                      QString* out_error = nullptr);

    /** @brief Edit metadata of an existing DB. If @p new_id differs
     *  from @p old_id, every test in this DB has its db_id rewritten
     *  too (so run-queue references stay valid). */
    bool updateDatabase(const QString& old_id,
                         const QString& new_id,
                         const QString& new_name,
                         const QString& new_description,
                         QString* out_error = nullptr);

    /** @brief Drop a DB and every test inside it. Requires caller
     *  confirmation — this method just performs the delete. */
    bool deleteDatabase(const QString& id,
                         QString* out_error = nullptr);

    /** @brief Write a fresh, empty-databases tests.yaml template at
     *  @p path and load it. Used by File ▸ New TestDatabase. */
    bool createNewDatabase(const QString& path,
                            int expected_dof,
                            QString* out_error = nullptr);

    /** @brief Delete the currently-loaded tests.yaml from disk and
     *  clear the in-memory tree. Used by File ▸ Delete TestDatabase. */
    bool deleteCurrentDatabase(QString* out_error = nullptr);

    /** @brief Merge tests from another tests.yaml into the current
     *  in-memory tree + on-disk file. Tests matching by id are
     *  skipped (logged); new ids are appended to their matching DB.
     *  DBs not present in the current file are appended as a whole.
     *  Caller-visible counts are reported via the out params. */
    bool importDatabase(const QString& path,
                         int* out_added       = nullptr,
                         int* out_skipped     = nullptr,
                         int* out_dbs_added   = nullptr,
                         QString* out_error   = nullptr);

    /** @brief Save the current in-memory tree to @p path verbatim.
     *  loaded_config_path_ stays unchanged — operator can pick this
     *  up via File ▸ Open if they want to switch. */
    bool exportDatabase(const QString& path,
                         QString* out_error = nullptr);

protected:
    /** @brief Resize hook — keeps the Results table's Expected and
     *  Actual columns equal-width and spanning whatever room is left
     *  after the fixed (ID / Status / Time / Name) columns. Lets the
     *  operator widen the window and have both prose columns grow
     *  together. */
    void resizeEvent(QResizeEvent* event) override;

public slots:
    /** @brief Build the run queue from currently-checked leaves and
     *  kick off the first test. The walk continues asynchronously via
     *  @ref stepNext between tests so the event loop pumps. Calling
     *  while a run is already in progress is a no-op. */
    void runAll();

    /** @brief Cancel mid-run. Aborts the current test cleanly, restores
     *  the hand to home, then stops. */
    void stop();

    /** @brief Toggle pause between tests (clean pause point — does
     *  NOT interrupt a test mid-execution). */
    void pause(bool on);

    /** @brief Re-arm: clear results, uncheck every leaf, re-enable
     *  the Run button. */
    void reset();

private slots:
    /** @brief Operator clicked a test row. Show its description popup. */
    void onTreeItemClicked(QTreeWidgetItem* item, int column);

    /** @brief Execute the test at @c run_cursor_ if any. On finish
     *  the run-cursor is advanced and stepNext is re-scheduled until
     *  the queue is drained. Separated from @ref runAll so the queue
     *  is built once at run start and not rebuilt between tests. */
    void stepNext();

    /** @brief Cache dof + joint names from the live worker so the
     *  tests can read them without crossing thread boundaries. */
    void onWorkerConnected(int dof, QStringList joint_names);

    /** @brief Cache the latest vendor error codes so DB2-01 (latched-
     *  error scan) can read them without crossing thread boundaries. */
    void onWorkerErrorCodes(QVector<int> codes);

    /** @brief Subscribed during DB1-02 / DB1-03 / DB1-04 / DB1-05
     *  windows — records frame timestamps, frame contents, NaN /
     *  inf counts, and per-joint sample counts. Each test reads what
     *  it needs from the captured window and resets between tests. */
    void onWorkerStateUpdated(QVector<double> q, QVector<double> tau);

    /** @brief Suite-edit handlers — wired to the toolbar buttons.
     *  Add prompts via @ref AddTestDialog and persists via @ref addTest;
     *  Delete prompts a confirmation and persists via @ref deleteTest.
     *  Both refresh the suite tree by re-loading the JSON. */
    void onAddTestClicked();
    void onDeleteTestClicked();

    /** @brief Right-click handler on the suite tree — builds the
     *  appropriate context menu based on whether the click landed
     *  on a DB header, a test leaf, or empty space. */
    void onSuiteContextMenu(const QPoint& pos);

    /** @brief Open the Help ▸ Field reference popup — a single-pane
     *  HTML cheatsheet of every TestSpec field plus the PASS/FAIL
     *  evaluation flow. */
    void onShowFieldHelp();

    /** @brief Status-bar colour helpers. Green for connected (with
     *  the dof N suffix), red for disconnected. Centralised so the
     *  styling stays consistent across construction + signal paths. */
    void setStatusConnected(int dof);
    void setStatusDisconnected();

    /** @brief Save the results table to a CSV the operator picks. */
    void onSaveCsv();

    /** @brief Render the results to a printable HTML report and ask
     *  the operator where to save the PDF. */
    void onGeneratePdf();

 private:
    /** @brief Find the spec for a test id in @c databases_, or return
     *  a default-constructed TestSpec when not found. */
    TestSpec findSpec(const QString& test_id) const;

    /** @brief Schema-v2 dispatcher — walks @c spec.captures, groups
     *  them by acquisition kind, runs each group, then calls
     *  @ref finishCurrent with the combined metrics object.
     *
     *  - All snapshot/config captures resolve synchronously.
     *  - Any window_agg captures trigger ONE telemetry-window of
     *    5 s (or @c spec.probe_params.window_s, capped at 5 s);
     *    every needed aggregate is computed from the same window.
     *  - Any probe captures invoke the probe named in
     *    @c spec.probe_params["probe"] (currently round_trip or
     *    bandwidth_sweep). Outputs are harvested under their
     *    catalog names; a `probe.<name>.*` wildcard capture lifts
     *    every emitted metric under its original probe-side name.
     *
     *  Empty captures → no acquisition, finishCurrent fires
     *  immediately with PASS. */
    void runTestBody(const TestSpec& spec);

    /** @brief Resolve a "snapshot" entry to its current value. */
    QJsonValue resolveSnapshot(const QString& source) const;

    /** @brief Resolve a "config" entry to its current value. */
    QJsonValue resolveConfig(const QString& source) const;


    /** @brief Wrap up the currently-running test: stop the timer,
     *  compute the duration, and call @ref onTestFinished. */
    void finishCurrent(bool passed,
                       const QString& summary,
                       const QJsonObject& metrics);

    /** @brief Render the result set as standalone HTML for the report. */
    QString resultsAsHtml() const;

    /** @brief Severity levels for the log panel — drive distinct
     *  colours and per-line prefixes. */
    enum class LogLevel { Info, Warn, Error };

    /** @brief Append a coloured + prefixed line to the log panel.
     *  Replaces every prior `log_panel_->appendPlainText(…)` call so
     *  the level prefix and colour stay in sync. */
    void appendLog(LogLevel level, const QString& message);

    /** @brief A test (run in HandWorker's thread) finished — update
     *  status, log line, results table; advance to the next test. */
    void onTestFinished(QString test_id,
                        bool    passed,
                        QString summary,
                        QJsonObject metrics);

private:
    /** @brief Build the suite tree from the parsed databases_. */
    void rebuildTree();

    HandWorker*                       worker_      = nullptr;
    QList<TestDatabase>               databases_;
    QJsonObject                       defaults_;       /**< from _defaults block */
    /** @brief v2 — name → AcquisitionEntry, loaded once from the file
     *  header. Dispatcher walks @c spec.captures and looks each name
     *  up here to decide where to pull the value from. */
    QHash<QString, AcquisitionEntry>  acquisition_table_;
    int                               schema_version_ = 1;
    /** @brief Top-level free-form `_meta` string from schema v2 — narrates
     *  the acquisition_table convention (prefix-based dispatch). Stored
     *  for display in the description popup; not used at runtime. */
    QString                           top_meta_;
    /** @brief Path of the tests.yaml that's currently loaded — used
     *  by @ref writeParams to round-trip edits back to disk. Empty
     *  before @ref loadConfig succeeds. */
    QString                           loaded_config_path_;

    /* --- UI widgets -------------------------------------------------- */
    QLabel*           status_label_     = nullptr;
    QPushButton*      run_btn_          = nullptr;
    QPushButton*      stop_btn_         = nullptr;
    QPushButton*      pause_btn_        = nullptr;
    QPushButton*      reset_btn_        = nullptr;
    QCheckBox*        continue_on_fail_ = nullptr;
    QTreeWidget*      suite_tree_       = nullptr;
    /* Rich-text live panel so we can bold the test name + use HTML
     * for layout. QPlainTextEdit doesn't honour QTextCharFormat
     * inline, hence the swap. */
    QTextEdit*        live_panel_       = nullptr;
    /* Rich-text log so info / warn / error get distinct colours.
     * QTextEdit (not QPlainTextEdit) — QPlainTextEdit doesn't honour
     * QTextCharFormat colours on appendPlainText. */
    QTextEdit*        log_panel_        = nullptr;
    QTableWidget*     results_table_    = nullptr;

    /* --- Run state --------------------------------------------------- */
    QStringList                       run_queue_;      /**< test ids, in order */
    int                               run_cursor_ = -1;
    bool                              run_active_ = false; /**< prevent runAll re-entry */
    bool                              paused_     = false;
    /** Per-test record kept for the results table + report exports
     *  (PDF / CSV). Indexed by test id; iteration order = run order.
     *  Carries the full spec snapshot at run time so the exporter
     *  doesn't have to re-resolve from databases_ (whose contents
     *  might have been reloaded by then). */
    struct ResultRow
    {
        QString test_id;
        QString test_name;
        QString db_id;
        QString description;
        QStringList pre_conditions;
        QStringList procedure;
        QString expected_result;    /**< pass_criteria */
        QString actual_result;      /**< formatted from metrics */
        bool    passed = false;
        double  duration_s = 0.0;
        QString summary;
        QJsonObject metrics;
    };
    QList<ResultRow>                  results_;
    qint64                            current_test_start_ms_ = 0;
    /** @brief Cached state from the worker's `connected` signal so
     *  the tests don't have to reach across thread boundaries to read
     *  dof / joint_names. Updated automatically. */
    int                               cached_dof_         = 0;
    QStringList                       cached_joint_names_;
    QVector<int>                      cached_error_codes_;
    /** @brief Cached URDF-derived limits, fed by
     *  HandWorker::urdfJointLimits + the synchronous accessors. Used
     *  by resolveSnapshot to back $urdf.joint_lo / $urdf.joint_hi. */
    QVector<double>                   cached_joint_lo_;
    QVector<double>                   cached_joint_hi_;
    /** State captured during a telemetry-window test. Reset at the
     *  start of each window; consumed by the test's finalizer. */
    struct TelemetryWindow
    {
        bool             active = false;
        qint64           start_ms = 0;
        int              frames = 0;
        int              malformed = 0;
        QVector<qint64>  ts_ms;             /**< inter-frame intervals */
        QVector<int>     samples_per_joint; /**< per-joint sample counter */
        /* Per-joint torque accumulators for DB2-03 — running sum and
         * sum-of-squares so mean/stdev are O(1) per frame. */
        QVector<double>  tau_sum_per_joint;
        QVector<double>  tau_sq_sum_per_joint;
        int              tau_samples = 0;
    };
    TelemetryWindow                   window_;
    /** @brief DB1-07 round-trip measurement lives in this probe,
     *  which inherits @ref WorkerThreadProbe and runs in HandWorker's
     *  thread for tight Qt-queue-free timing. Created in the runner's
     *  constructor; receives params via startBatch() and reports
     *  batchDone() back here. */
    class RoundTripProbe*             round_trip_probe_ = nullptr;
    /** @brief DB1-08 closed-loop bandwidth sweep. Same probe pattern
     *  as round_trip_probe_ — worker-thread state machine, batchDone
     *  back to the GUI for finishCurrent(). */
    class BandwidthSweepProbe*        bandwidth_probe_  = nullptr;
    /* DB2 family probes — same WorkerThreadProbe pattern. Hand-only
     * probes (GesturePlayback, TrajectoryFollow, RangeOfMotion) were
     * dropped — gestures + trajectories are Hand-stack concepts and
     * the URDF joint-limit plumbing RangeOfMotion needs doesn't
     * exist on the MC side. */
    class OperationModeProbe*         opmode_probe_     = nullptr;
    class SoftLimitProbe*             softlimit_probe_  = nullptr;
    class EstopProbe*                 estop_probe_      = nullptr;
    /** @brief expected_dof from the top of tests.yaml — populated by
     *  loadConfig so DB1-01 (bus enumeration) can compare. */
    int                               expected_dof_ = 0;
    /** @brief Output dir for PDF/CSV exports, sticky across saves. */
    QString                           default_export_dir_;
    /** @brief Where the file dialog opens by default — populated
     *  during construction from the bundled share dir so the operator
     *  doesn't have to navigate from $HOME on every Browse click. */
    QString                           default_config_dir_;

    /* Long-lived description popup — re-used (just refresh content)
     * so multiple clicks don't stack windows. */
    TestDescriptionDialog*            desc_popup_ = nullptr;
    /* Same model for the result detail popup. */
    class TestResultDialog*           result_popup_ = nullptr;
};
