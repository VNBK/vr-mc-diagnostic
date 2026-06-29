/*
 * TestRunnerWindow.cpp — see header for purpose.
 *
 * Phase-2 scope: JSON config → suite tree → description popup → run
 * loop with REAL test bodies for some IDs (currently DB1-01 Bus
 * enumeration). Unrecognised ids fall back to a Phase-1 simulated
 * PASS so the UI loop stays exercisable while the rest of the
 * bodies are written. Per-test duration, live procedure-step echo,
 * results table with five columns, plus CSV + PDF export are wired.
 */
#include "TestRunnerWindow.hpp"
#include "AddTestDialog.hpp"
/* Probe headers ship as on-disk reference templates only. They live
 * under TestRunner/probes/ and target HandWorker methods (setPosition /
 * setForce / stateUpdated) that MasterWorker doesn't expose. Compile
 * them back in once they're ported to MasterWorker; until then the
 * dispatcher reports `probe.<name>` captures as "not yet ported".
 *
 * Hand-only probes (Gesture, Trajectory, RangeOfMotion) were dropped
 * outright — gestures + trajectories are Hand-stack concepts and the
 * URDF joint-limit plumbing RangeOfMotionProbe relies on doesn't exist
 * on the MC side. The remaining probes (RoundTrip, BandwidthSweep,
 * Estop, SoftLimit, OperationMode) are generic actuator-level tests
 * that should port over with thin SDK-glue rewrites. */
#define VR_MC_TESTRUNNER_PROBES_ENABLED 0
#if VR_MC_TESTRUNNER_PROBES_ENABLED
#include "probes/BandwidthSweepProbe.hpp"
#include "probes/EstopProbe.hpp"
#include "probes/OperationModeProbe.hpp"
#include "probes/SoftLimitProbe.hpp"
#include "probes/RoundTripProbe.hpp"
#endif
#include "DatabaseEditDialog.hpp"
#include "TestDescriptionDialog.hpp"
#include "TestResultDialog.hpp"
#include "YamlBridge.hpp"
#include "AppConfig.hpp"
#include "HandWorkerCompat.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QInputDialog>
#include <QKeySequence>
#include <QMenu>
#include <QMenuBar>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGuiApplication>
#include <QScreen>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPrinter>
#include <QPushButton>
#include <QScrollBar>
#include <QSet>
#include <QSplitter>
#include <QStandardPaths>
#include <QTextBrowser>
#include <QTableWidget>
#include <QTextDocument>
#include <QTextEdit>
#include <QTimer>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

/* ament_index_cpp is a ROS2-only convenience for finding a package's
 * installed share/ tree. vr_mc_diagnostic doesn't depend on ROS2, so
 * we replace the call with a stub that throws std::runtime_error —
 * the existing try/catch around the call falls through to a Qt-native
 * default. */
#include <stdexcept>
#include <string>
namespace ament_index_cpp {
inline std::string get_package_share_directory(const std::string&)
{
    throw std::runtime_error("ament_index_cpp not available in MC build");
}
}  // namespace ament_index_cpp

#include <algorithm>
#include <cmath>
#include <cstring>


/* Column layout for the results table — single source so future
 * resizes / index lookups stay in sync. */
namespace {
constexpr int kColId        = 0;
constexpr int kColName      = 1;
constexpr int kColStatus    = 2;
constexpr int kColTime      = 3;
constexpr int kColExpected  = 4;
constexpr int kColActual    = 5;
constexpr int kColCount     = 6;


/* ============================================================== *
 *  Schema-v2 helpers — derive runtime-dispatch fields from a flat
 *  acquisition_table name (e.g. "telemetry.cycle_mean_us") and
 *  parse a free-form pass_criteria `check:` expression into the
 *  reducer:lhs op rhs shape PassRule already uses.
 * ============================================================== */

/** @brief Derive an AcquisitionEntry from a bare metric name.
 *
 *  The new schema's `acquisition_table` is a flat list of names; the
 *  legacy schema spelt out kind/source/stat/probe/output per entry.
 *  Convention recovered here so the existing dispatcher stays unchanged:
 *
 *    state.<x>              → snapshot      , source = "worker.<x>"
 *    config.<x>             → config        , source = "<x>"
 *    telemetry.<x>          → window_agg    , stat   = "<x>"
 *                             (special-case: "cycle_*_us" → "interval_*_us"
 *                              for parity with the InspireHandTestDB
 *                              telemetry-window aggregator names)
 *    probe.<pname>.<out>    → probe         , probe="<pname>", output="<out>"
 *    probe.<pname>.*        → probe         , probe="<pname>", output="*"
 *    urdf.<x>               → urdf          , source = "<x>"
 *    anything else          → empty entry (treated as unknown)
 */
AcquisitionEntry deriveAcquisitionEntry(const QString& name)
{
    AcquisitionEntry a;
    const int dot = name.indexOf('.');
    if (dot <= 0){ return a; }
    const QString head = name.left(dot);
    const QString tail = name.mid(dot + 1);
    if (head == "state"){
        a.kind   = QStringLiteral("snapshot");
        a.source = QStringLiteral("worker.") + tail;
    } else if (head == "config"){
        a.kind   = QStringLiteral("config");
        a.source = tail;
    } else if (head == "telemetry"){
        a.kind = QStringLiteral("window_agg");
        a.stat = tail.startsWith("cycle_")
                    ? QStringLiteral("interval_") + tail.mid(QStringLiteral("cycle_").size())
                    : tail;
    } else if (head == "urdf"){
        a.kind   = QStringLiteral("urdf");
        a.source = tail;
    } else if (head == "probe"){
        const int dot2 = tail.indexOf('.');
        if (dot2 > 0){
            a.kind   = QStringLiteral("probe");
            a.probe  = tail.left(dot2);
            a.output = tail.mid(dot2 + 1);
        }
    }
    return a;
}


/** @brief Parse a schema-v2 `check:` expression into the existing
 *  PassRule shape (reducer:lhs OP rhs).
 *
 *  Accepted operators (longest match wins): `<=`, `>=`, `==`, `!=`,
 *  `<`, `>`. LHS keeps any `reducer:` prefix unchanged so the existing
 *  tokeniser downstream still works. RHS is interpreted as:
 *    - a literal number  → QJsonValue::Double
 *    - "true" / "false"  → QJsonValue::Bool
 *    - a quoted "string" → QJsonValue::String
 *    - a bare identifier → "$identifier" (context lookup, mirroring
 *                          how the legacy parser flagged $name refs)
 *
 *  Returns true on success; on syntax failure the rule's op is empty
 *  and the caller can flag a parse warning. */
bool parseCheckExpr(const QString& raw_in,
                     QString*      out_lhs,
                     QString*      out_op,
                     QJsonValue*   out_rhs)
{
    const QString raw = raw_in.trimmed();
    static const char* const kOps[] = { "<=", ">=", "==", "!=", "<", ">" };
    int      op_pos  = -1;
    int      op_len  = 0;
    QString  op;
    for (const char* candidate : kOps){
        const int p = raw.indexOf(QLatin1String(candidate));
        if (p < 0){ continue; }
        if (op_pos < 0 || p < op_pos
            || (p == op_pos && (int)std::strlen(candidate) > op_len)){
            op_pos = p;
            op_len = (int)std::strlen(candidate);
            op     = QString::fromLatin1(candidate);
        }
    }
    if (op_pos < 0){ return false; }
    *out_lhs = raw.left(op_pos).trimmed();
    *out_op  = op;
    const QString rhs = raw.mid(op_pos + op_len).trimmed();
    if (rhs.isEmpty()){ return false; }
    if ((rhs.startsWith('"') && rhs.endsWith('"'))
     || (rhs.startsWith('\'') && rhs.endsWith('\''))){
        *out_rhs = rhs.mid(1, rhs.size() - 2);
        return true;
    }
    if (rhs == "true" || rhs == "True"){ *out_rhs = true;  return true; }
    if (rhs == "false"|| rhs == "False"){*out_rhs = false; return true; }
    bool num_ok = false;
    const double dv = rhs.toDouble(&num_ok);
    if (num_ok){
        *out_rhs = dv;
        return true;
    }
    /* Bare identifier — treat as a context lookup, matching the
     * "$name" convention the legacy parser uses. */
    *out_rhs = QStringLiteral("$") + rhs;
    return true;
}


/* ============================================================== *
 *  Rule evaluator — turns metrics + spec.probe_params + the
 *  spec.pass_rules list into a boolean + per-failed-rule reason.
 *  Free helpers in this anonymous namespace; no state.
 * ============================================================== */

/** @brief Resolve a name to its QJsonValue. Metrics shadow params on
 *  collision so the rule writer can rely on "the latest measured
 *  value" without having to namespace.
 *
 *  Tolerant to the @c [] array-decoration: probes emit array metrics
 *  under keys like "round_trip_ms[]" while rule authors usually
 *  write "round_trip_ms" (the LHS parser already strips the suffix
 *  for visual reasons). Lookup tries both forms in metrics so the
 *  rule resolves no matter which convention the author picked. */
QJsonValue lookupName(const QString& name,
                       const QJsonObject& metrics,
                       const QJsonObject& params,
                       const QJsonObject& probe_params = {})
{
    /* Try a name and its "[]"-array form against the metric/param/
     * probe_params maps in priority order. Helper so the prefix-strip
     * fallback below doesn't repeat itself. */
    auto tryName = [&](const QString& n) -> QJsonValue {
        if (metrics      .contains(n))                       { return metrics      .value(n); }
        if (metrics      .contains(n + QStringLiteral("[]"))){ return metrics      .value(n + QStringLiteral("[]")); }
        if (probe_params .contains(n))                       { return probe_params .value(n); }
        if (params       .contains(n))                       { return params       .value(n); }
        return QJsonValue::Undefined;
    };

    QJsonValue v = tryName(name);
    if (!v.isUndefined()){ return v; }

    /* Prefix-strip fallback for schema-v2 pass_criteria that reference
     * a metric via its acquisition-table path. E.g. a probe captured
     * via `probe.round_trip.*` emits flat names like `round_trip_ms`;
     * a `check:` expression that writes `probe.round_trip_ms` or
     * `probe.round_trip.round_trip_ms` should still resolve.
     *
     * Strategy: peel one leading "<word>." prefix at a time and retry,
     * up to two strips (covers both shorthand and fully-qualified
     * forms). Safe because metric names themselves never start with
     * "probe.", "state.", "telemetry.", "config." or "urdf." in
     * probe-emitted output. */
    QString cur = name;
    for (int peel = 0; peel < 2; ++peel){
        const int dot = cur.indexOf('.');
        if (dot <= 0){ break; }
        cur = cur.mid(dot + 1);
        v = tryName(cur);
        if (!v.isUndefined()){ return v; }
    }
    return QJsonValue::Undefined;
}


/** @brief Resolve a rule's RHS. "$name" means context lookup; plain
 *  values are literals returned as-is (numbers, bools, strings). */
QJsonValue resolveRhs(const QJsonValue& rhs,
                       const QJsonObject& metrics,
                       const QJsonObject& params,
                       const QJsonObject& probe_params = {})
{
    if (rhs.isString() && rhs.toString().startsWith('$')){
        return lookupName(rhs.toString().mid(1), metrics, params, probe_params);
    }
    return rhs;
}


/** @brief Apply a reducer to an array. Returns false when the array
 *  is empty for reducers that can't yield a value on empty (max /
 *  min / mean / p99 / sum). Len and CountNonzero always succeed.
 *  Div is handled by the caller — not really an array reducer. */
bool reduceArray(TestSpec::PassRule::Reducer r,
                  const QJsonArray& arr,
                  double* out)
{
    using R = TestSpec::PassRule::Reducer;
    if (r == R::Len){          *out = double(arr.size()); return true; }
    if (r == R::CountNonzero){
        int c = 0;
        for (const auto& v : arr){ if (v.toDouble() != 0.0){ ++c; } }
        *out = double(c);
        return true;
    }
    if (arr.isEmpty()){ return false; }
    QVector<double> v;
    v.reserve(arr.size());
    for (const auto& x : arr){ v.append(x.toDouble()); }
    switch (r){
        case R::Max: {
            double m = v.first();
            for (double x : v){ m = std::max(m, x); }
            *out = m;
            return true;
        }
        case R::Min: {
            double m = v.first();
            for (double x : v){ m = std::min(m, x); }
            *out = m;
            return true;
        }
        case R::Mean: {
            double s = 0.0;
            for (double x : v){ s += x; }
            *out = s / v.size();
            return true;
        }
        case R::P99: {
            std::sort(v.begin(), v.end());
            *out = v[std::min<int>(v.size() - 1, int(v.size() * 0.99))];
            return true;
        }
        case R::Sum: {
            double s = 0.0;
            for (double x : v){ s += x; }
            *out = s;
            return true;
        }
        case R::Stdev: {
            /* Sample stdev — N-1 denominator. Falls back to 0 when
             * the array has a single element (no spread to measure). */
            if (v.size() < 2){ *out = 0.0; return true; }
            double s = 0.0;
            for (double x : v){ s += x; }
            const double mean = s / v.size();
            double sq = 0.0;
            for (double x : v){ sq += (x - mean) * (x - mean); }
            *out = std::sqrt(sq / (v.size() - 1));
            return true;
        }
        case R::Range: {
            double lo = v.first(), hi = v.first();
            for (double x : v){ lo = std::min(lo, x); hi = std::max(hi, x); }
            *out = hi - lo;
            return true;
        }
        case R::Cv: {
            /* Coefficient of variation = stdev / mean. Zero-mean
             * arrays fail the rule (CV undefined). */
            if (v.size() < 2){ *out = 0.0; return true; }
            double s = 0.0;
            for (double x : v){ s += x; }
            const double mean = s / v.size();
            if (mean == 0.0){ return false; }
            double sq = 0.0;
            for (double x : v){ sq += (x - mean) * (x - mean); }
            const double stdev = std::sqrt(sq / (v.size() - 1));
            *out = stdev / mean;
            return true;
        }
        default: return false;
    }
}


/** @brief Compute the LHS scalar value for one rule. Returns false
 *  when the metric / param can't be found or the array is empty
 *  for a non-len reducer. */
bool evalLhs(const TestSpec::PassRule& rule,
              const QJsonObject& metrics,
              const QJsonObject& params,
              const QJsonObject& probe_params,
              double* out)
{
    using R = TestSpec::PassRule::Reducer;
    if (rule.lhs_reducer == R::Div){
        const auto a = lookupName(rule.lhs_arg,  metrics, params, probe_params);
        const auto b = lookupName(rule.lhs_arg2, metrics, params, probe_params);
        if (a.isUndefined() || b.isUndefined()){ return false; }
        const double bv = b.toDouble();
        if (bv == 0.0){ return false; }
        *out = a.toDouble() / bv;
        return true;
    }
    const auto v = lookupName(rule.lhs_arg, metrics, params, probe_params);
    if (v.isUndefined()){ return false; }
    if (rule.lhs_reducer == R::None){
        *out = v.toDouble();   /* bool → 0/1, int/double → numeric */
        return true;
    }
    return reduceArray(rule.lhs_reducer, v.toArray(), out);
}


bool compareOp(double lhs, const QString& op, double rhs)
{
    if (op == "==") { return lhs == rhs; }
    if (op == "!=") { return lhs != rhs; }
    if (op == "<=") { return lhs <= rhs; }
    if (op == ">=") { return lhs >= rhs; }
    if (op == "<")  { return lhs <  rhs; }
    if (op == ">")  { return lhs >  rhs; }
    return false;
}


/** @brief Per-rule outcome — used both for the combined boolean and
 *  for the failure-reason printer the runner appends to the summary. */
struct RuleOutcome
{
    bool    passed   = false;
    bool    evaluated= false;       /**< false on missing-metric / div0 */
    double  lhs      = 0.0;
    double  rhs      = 0.0;
    QString reason;                 /**< human-readable failure detail */
};


RuleOutcome runOneRule(const TestSpec::PassRule& rule,
                        const QJsonObject& metrics,
                        const QJsonObject& params,
                        const QJsonObject& probe_params)
{
    RuleOutcome r;
    if (!evalLhs(rule, metrics, params, probe_params, &r.lhs)){
        r.reason = QObject::tr("%1 — couldn't evaluate (missing metric "
                                "or div-by-zero)").arg(rule.lhs_raw);
        return r;
    }
    const auto rhs_resolved = resolveRhs(rule.rhs, metrics, params, probe_params);
    if (rhs_resolved.isUndefined()){
        r.reason = QObject::tr("%1: rhs '%2' not found")
                       .arg(rule.lhs_raw,
                             rule.rhs.isString() ? rule.rhs.toString()
                                                  : QObject::tr("<literal>"));
        return r;
    }
    r.evaluated = true;
    r.rhs       = rhs_resolved.toDouble();
    r.passed    = compareOp(r.lhs, rule.op, r.rhs);
    if (!r.passed){
        /* "max:round_trip_ms[] = 97.5 > $max_round_trip_ms (100)" — concise
         * enough for the Actual Result cell, precise enough for triage. */
        const QString rhs_text = rule.rhs.isString()
                                    ? QString("%1 (%2)")
                                          .arg(rule.rhs.toString())
                                          .arg(r.rhs, 0, 'g', 6)
                                    : QString::number(r.rhs, 'g', 6);
        r.reason = QString("%1 = %2 %3 %4")
                       .arg(rule.lhs_raw)
                       .arg(r.lhs, 0, 'g', 6)
                       .arg(rule.op)
                       .arg(rhs_text);
        /* Schema-v2 pass_criteria entries carry an optional error_code
         * + error_msg. Surface them in the failure reason so the
         * operator sees the spec-author's diagnosis text alongside
         * the numeric mismatch. */
        if (!rule.error_code.isEmpty() || !rule.error_msg.isEmpty()){
            QStringList tag;
            if (!rule.error_code.isEmpty()){ tag << QStringLiteral("[%1]").arg(rule.error_code); }
            if (!rule.error_msg .isEmpty()){ tag << rule.error_msg; }
            r.reason = QStringLiteral("%1 — %2")
                           .arg(r.reason, tag.join(QChar(' ')));
        }
    }
    return r;
}


/** @brief Walk a spec's rule list. Returns the combined boolean +
 *  a list of human-readable reasons for any rule that failed. */
bool evaluateRules(const TestSpec& spec,
                    const QJsonObject& metrics,
                    QStringList* out_failed_reasons)
{
    if (spec.pass_rules.isEmpty()){ return true; }
    const bool is_or = (spec.pass_combiner == "OR");
    bool combined = !is_or;
    /* Schema v2 dropped spec.params; probe_params is the only context
     * the evaluator pulls $-references from on the rule side. */
    static const QJsonObject kEmptyParams;
    for (const auto& rule : spec.pass_rules){
        const RuleOutcome r = runOneRule(rule, metrics, kEmptyParams,
                                           spec.probe_params);
        const bool ok = r.evaluated && r.passed;
        if (!ok && out_failed_reasons){
            out_failed_reasons->append(r.reason);
        }
        combined = is_or ? (combined || ok) : (combined && ok);
    }
    return combined;
}
}  // namespace


/* ============================================================== *
 *  ctor / dtor
 * ============================================================== */
TestRunnerWindow::TestRunnerWindow(HandWorker* worker, QWidget* parent)
    : QWidget(parent)
    , worker_(worker)
{
    setWindowTitle(tr("Test Runner"));
    setWindowFlags(Qt::Window);                      /* top-level, NOT a tab */
    /* Clamp to the available screen so the title bar never falls off
     * on 1366x768 panels (taskbar + chrome leaves ~720 px of usable
     * height — our desired 720 would overflow by exactly that amount). */
    {
        const QRect avail = QGuiApplication::primaryScreen()->availableGeometry();
        resize(std::min(1100, avail.width()  - 40),
                std::min(720,  avail.height() - 80));
    }

    /* Browse-dialog start dir — restore the last-picked folder from
     * QSettings so the operator doesn't re-navigate every launch.
     * Persistence key: test_runner/last_db_dir. Falls back to the
     * bundled share dir on first run / corrupted settings. Wrap the
     * ament lookup because it throws when AMENT_PREFIX_PATH isn't
     * indexed. */
    default_config_dir_ = AppConfig::instance()
        .value(QStringLiteral("test_runner/last_db_dir"), QString{})
        .toString();
    if (default_config_dir_.isEmpty() || !QDir(default_config_dir_).exists()){
        try {
            const QString share = QString::fromStdString(
                ament_index_cpp::get_package_share_directory(
                    std::string("vr_hand_diagnostic")));
            if (!share.isEmpty()){
                default_config_dir_ = share + "/config";
            }
        } catch (const std::exception&) {
            /* leave default_config_dir_ empty — Qt opens at $HOME */
        }
    }
    default_export_dir_ = QStandardPaths::writableLocation(
        QStandardPaths::DocumentsLocation);
    /* Old "Import Database…" header button removed — it actually did
     * Open (replace), not Import (merge), and lived right next to the
     * File ▸ Import menu item which DOES merge. Two same-named ops
     * with different behaviour confused operators. File ▸ Open and
     * File ▸ Import now cover both behaviours unambiguously. */

    /* ---- Run-control toolbar --------------------------------------- */
    auto* controls = new QHBoxLayout();
    run_btn_   = new QPushButton(tr("▶ Run All"),   this);
    pause_btn_ = new QPushButton(tr("⏸ Pause"),     this);
    stop_btn_  = new QPushButton(tr("■ Stop"),      this);
    reset_btn_ = new QPushButton(tr("↻ Reset"),     this);
    continue_on_fail_ = new QCheckBox(tr("Continue on failure"), this);
    pause_btn_->setCheckable(true);
    pause_btn_->setEnabled(false);
    stop_btn_ ->setEnabled(false);
    /* Equal width across the run-control quartet. Pause is checkable
     * — its natural sizeHint already includes the toggle-button
     * extra padding, so plain setMinimumWidth still let it draw
     * wider than the others. Pinning a fixed width on all four
     * forces a uniform bar regardless of the underlying button
     * kind. Size chosen to fit the widest label ("▶ Run All"). */
    constexpr int kCtrlBtnW = 100;
    constexpr int kCtrlBtnH = 25;
    run_btn_  ->setFixedSize(kCtrlBtnW, kCtrlBtnH);
    pause_btn_->setFixedSize(kCtrlBtnW, kCtrlBtnH);
    stop_btn_ ->setFixedSize(kCtrlBtnW, kCtrlBtnH);
    reset_btn_->setFixedSize(kCtrlBtnW, kCtrlBtnH);

    controls->addWidget(run_btn_);
    controls->addWidget(pause_btn_);
    controls->addWidget(stop_btn_);
    controls->addWidget(reset_btn_);
    controls->addWidget(continue_on_fail_);
    controls->addStretch(1);
    /* Status indicator lives on the right of the run-control row so
     * its Y baseline matches the Run/Pause/Stop/Reset buttons — the
     * operator's eye doesn't have to leave the toolbar to read the
     * connection state. setStatusDisconnected runs first (red on
     * pale red); HandWorker's connected/disconnected signals flip
     * it via setStatusConnected/setStatusDisconnected. */
    status_label_ = new QLabel(this);
    setStatusDisconnected();
    controls->addWidget(status_label_);
    /* Add / Delete moved into the Test menu — the toolbar stays
     * focused on the run-control trio so the operator's eye lands
     * on play / pause / stop / reset / continue-on-failure. */

    connect(run_btn_,   &QPushButton::clicked, this, &TestRunnerWindow::runAll);
    connect(stop_btn_,  &QPushButton::clicked, this, &TestRunnerWindow::stop);
    connect(reset_btn_, &QPushButton::clicked, this, &TestRunnerWindow::reset);
    connect(pause_btn_, &QPushButton::toggled, this, &TestRunnerWindow::pause);

    /* ---- Main splitter: suite tree | live + log + results --------- */
    auto* split = new QSplitter(Qt::Horizontal, this);

    /* Suite tree (left). */
    suite_tree_ = new QTreeWidget(this);
    suite_tree_->setHeaderLabels({tr("Test"), tr("Status"), tr("Time (s)")});
    suite_tree_->header()->setStretchLastSection(false);
    suite_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    suite_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    suite_tree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    suite_tree_->setMinimumWidth(360);
    /* Double-click (not single-click) opens the description popup —
     * single-click is reserved for tree row selection / expand-
     * collapse, which the operator uses to navigate without the
     * popup stealing focus on every click. */
    connect(suite_tree_, &QTreeWidget::itemDoubleClicked,
            this,        &TestRunnerWindow::onTreeItemClicked);
    /* Right-click context menu — DB-level vs test-level items get
     * different actions. Hooked here so the menu lives close to the
     * widget it acts on, and so it shares the runner's existing
     * Add/Edit/Delete plumbing instead of duplicating dialogs. */
    suite_tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(suite_tree_, &QTreeWidget::customContextMenuRequested,
            this, &TestRunnerWindow::onSuiteContextMenu);
    split->addWidget(suite_tree_);

    /* Right pane: live + log + results stacked. */
    auto* right = new QWidget(this);
    auto* right_lay = new QVBoxLayout(right);
    /* Zero margins on this layout so the right-side widgets begin at
     * the same Y as the bare suite_tree_ on the left half of the
     * splitter — otherwise QVBoxLayout's default 9-px top/bottom
     * margins push Live Test down and Results up, breaking the
     * "TOP suite tree == TOP Live test" and "BOTTOM suite tree ==
     * BOTTOM Results" alignment the operator expects. */
    right_lay->setContentsMargins(0, 0, 0, 0);
    right_lay->setSpacing(0);

    /* Live | Log | Results sit inside a vertical QSplitter so the
     * operator can drag the dividers to give whichever pane they
     * currently care about more room. Each pane is wrapped in a
     * QWidget so its header label stays glued to the body when the
     * splitter resizes it. */
    auto* right_split = new QSplitter(Qt::Vertical, right);
    right_split->setChildrenCollapsible(false);
    right_split->setHandleWidth(6);
    right_lay->addWidget(right_split, 1);

    auto* live_box = new QWidget(right_split);
    auto* live_lay = new QVBoxLayout(live_box);
    live_lay->setContentsMargins(0, 0, 0, 0);
    live_lay->addWidget(new QLabel(tr("<b>Live test</b>"), live_box));
    live_panel_ = new QTextEdit(live_box);
    live_panel_->setReadOnly(true);
    /* White background with dark text — the panel content is meant
     * to be read like a printed test card, not a terminal. Sticks
     * out against the surrounding dark Fusion theme on purpose.
     * QTextEdit (not QPlainTextEdit) so the test-name HTML <b> takes
     * effect inside an otherwise monospace card. */
    live_panel_->setStyleSheet(
        "QTextEdit { font-family: monospace; "
        "background: #ffffff; color: #1a1a1a; border: 1px solid #888; }");
    live_lay->addWidget(live_panel_, 1);
    right_split->addWidget(live_box);

    auto* log_box = new QWidget(right_split);
    auto* log_lay = new QVBoxLayout(log_box);
    log_lay->setContentsMargins(0, 0, 0, 0);
    log_lay->addWidget(new QLabel(tr("<b>Log</b>"), log_box));
    log_panel_ = new QTextEdit(log_box);
    log_panel_->setReadOnly(true);
    log_panel_->setStyleSheet("QTextEdit { font-family: monospace; }");
    log_lay->addWidget(log_panel_, 1);
    right_split->addWidget(log_box);

    auto* results_box = new QWidget(right_split);
    auto* results_lay = new QVBoxLayout(results_box);
    results_lay->setContentsMargins(0, 0, 0, 0);
    results_lay->addWidget(new QLabel(tr("<b>Results</b>"), results_box));
    results_table_ = new QTableWidget(0, kColCount, results_box);
    results_table_->setHorizontalHeaderLabels(
        {tr("ID"), tr("Name"), tr("Status"), tr("Time (s)"),
         tr("Expected Result"), tr("Actual Result")});
    /* No stretch on the last column — it would override our explicit
     * Expected==Actual width seed and let Actual grow beyond Expected
     * once the window widens. The mirror connect() below keeps both
     * sides in sync when the operator drags. */
    results_table_->horizontalHeader()->setStretchLastSection(false);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColId,       QHeaderView::ResizeToContents);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColName,     QHeaderView::Interactive);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColStatus,   QHeaderView::ResizeToContents);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColTime,     QHeaderView::ResizeToContents);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColExpected, QHeaderView::Interactive);
    results_table_->horizontalHeader()->setSectionResizeMode(
        kColActual,   QHeaderView::Interactive);
    results_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    results_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    results_table_->setWordWrap(true);
    /* Seed default column widths. Expected and Actual MUST stay equal
     * — they describe the two halves of the same comparison and
     * reading them side-by-side at different widths feels lopsided.
     * Use the same value for both and keep them in sync inside
     * onTestFinished when content forces a resize. */
    constexpr int kExpectedActualWidth = 280;
    results_table_->setColumnWidth(kColName,     180);
    results_table_->setColumnWidth(kColExpected, kExpectedActualWidth);
    results_table_->setColumnWidth(kColActual,   kExpectedActualWidth);
    /* When the operator drags one to resize, mirror the change to the
     * other so they stay equal. */
    connect(results_table_->horizontalHeader(),
            &QHeaderView::sectionResized,
            this,
            [this](int section, int /*old*/, int newSize){
                auto* h = results_table_->horizontalHeader();
                if (section == kColExpected
                    && h->sectionSize(kColActual) != newSize){
                    QSignalBlocker b(h);
                    results_table_->setColumnWidth(kColActual, newSize);
                } else if (section == kColActual
                    && h->sectionSize(kColExpected) != newSize){
                    QSignalBlocker b(h);
                    results_table_->setColumnWidth(kColExpected, newSize);
                }
            });
    results_lay->addWidget(results_table_, 1);
    right_split->addWidget(results_box);

    /* Initial pane sizes — Live small (card), Log medium, Results
     * the largest because rows accumulate. Operator can drag any
     * divider to override. */
    right_split->setSizes({180, 240, 320});

    /* PDF/CSV moved out of the right-pane footer so the bottom of
     * the Results table can align with the bottom of the suite tree
     * across the horizontal splitter. The export buttons now sit on
     * the outermost layout, just above the status bar — see below. */

    split->addWidget(right);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);

    /* ---- Outermost layout ----------------------------------------- */
    auto* outer = new QVBoxLayout(this);

    /* Menubar — File for whole-database ops, Test for per-test ops.
     * Lives at the top of the window's own layout because this is a
     * QWidget, not a QMainWindow (no native menubar slot). */
    auto* menubar = new QMenuBar(this);
    {
        QMenu* file_menu = menubar->addMenu(tr("&File"));
        QAction* a_new = file_menu->addAction(tr("&New TestDatabase…"));
        QAction* a_open = file_menu->addAction(tr("&Open TestDatabase…"));
        QAction* a_reload = file_menu->addAction(tr("&Reload TestDatabase"));
        a_reload->setShortcut(QKeySequence::Refresh);  /* F5 */
        QAction* a_del = file_menu->addAction(tr("&Delete TestDatabase…"));
        file_menu->addSeparator();
        QAction* a_imp = file_menu->addAction(tr("&Import TestDatabase…"));
        QAction* a_exp = file_menu->addAction(tr("&Export TestDatabase…"));
        file_menu->addSeparator();
        QAction* a_exit = file_menu->addAction(tr("E&xit"));
        a_exit->setShortcut(QKeySequence::Close);  /* Ctrl+W */
        connect(a_exit, &QAction::triggered, this, &QWidget::close);
        connect(a_new, &QAction::triggered, this, [this]{
            const QString path = QFileDialog::getSaveFileName(this,
                tr("New TestDatabase"),
                default_config_dir_,
                tr("YAML (*.yaml *.yml);;JSON (*.json);;All files (*)"));
            if (path.isEmpty()){ return; }
            bool ok = false;
            const int dof = QInputDialog::getInt(this, tr("New TestDatabase"),
                tr("Expected DOF:"), 6, 1, 64, 1, &ok);
            if (!ok){ return; }
            QString err;
            if (!createNewDatabase(path, dof, &err)){
                QMessageBox::warning(this, tr("New failed"), err);
            }
        });
        connect(a_open, &QAction::triggered, this, [this]{
            const QString picked = QFileDialog::getOpenFileName(this,
                tr("Pick TestDatabase"), default_config_dir_,
                tr("YAML / JSON (*.yaml *.yml *.json);;All files (*)"));
            if (picked.isEmpty()){ return; }
            QString err;
            if (!loadConfig(picked, &err)){
                appendLog(LogLevel::Error, err);
                return;
            }
            default_config_dir_ = QFileInfo(picked).absolutePath();
            AppConfig::instance().setValue(
                QStringLiteral("test_runner/last_db_dir"),
                default_config_dir_);
        });
        /* Reload — re-read the currently-loaded file from disk so
         * the operator picks up edits made outside the runner (or
         * via the menu's add/edit/delete flows that already write
         * to disk and reload, but this also covers manual JSON
         * editing in a text editor). */
        connect(a_reload, &QAction::triggered, this, [this]{
            if (loaded_config_path_.isEmpty()){
                QMessageBox::information(this, tr("Reload TestDatabase"),
                    tr("No file is loaded — pick one via File ▸ Open."));
                return;
            }
            QString err;
            if (!loadConfig(loaded_config_path_, &err)){
                QMessageBox::warning(this, tr("Reload failed"), err);
                return;
            }
            appendLog(LogLevel::Info,
                tr("reloaded %1").arg(loaded_config_path_));
        });
        connect(a_del, &QAction::triggered, this, [this]{
            if (loaded_config_path_.isEmpty()){
                QMessageBox::information(this, tr("Delete TestDatabase"),
                    tr("No file is loaded.")); return;
            }
            const auto ans = QMessageBox::warning(this,
                tr("Delete TestDatabase"),
                tr("This will permanently delete:\n\n%1\n\nProceed?")
                    .arg(loaded_config_path_),
                QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
            if (ans != QMessageBox::Yes){ return; }
            QString err;
            if (!deleteCurrentDatabase(&err)){
                QMessageBox::warning(this, tr("Delete failed"), err);
            }
        });
        connect(a_imp, &QAction::triggered, this, [this]{
            const QString picked = QFileDialog::getOpenFileName(this,
                tr("Import TestDatabase"), default_config_dir_,
                tr("YAML / JSON (*.yaml *.yml *.json);;All files (*)"));
            if (picked.isEmpty()){ return; }
            int added = 0, skipped = 0, dbs_added = 0;
            QString err;
            if (!importDatabase(picked, &added, &skipped, &dbs_added, &err)){
                QMessageBox::warning(this, tr("Import failed"), err);
                return;
            }
            /* Explicit confirmation popup — the previous silent path
             * felt broken because nothing visible changed when an
             * import added zero new tests (everything was a dupe). */
            QMessageBox::information(this, tr("Import complete"),
                tr("Imported from:\n%1\n\n"
                   "Added: %2 tests\n"
                   "Skipped (duplicate id): %3 tests\n"
                   "New databases appended: %4")
                    .arg(picked).arg(added).arg(skipped).arg(dbs_added));
        });
        connect(a_exp, &QAction::triggered, this, [this]{
            const QString path = QFileDialog::getSaveFileName(this,
                tr("Export TestDatabase"), default_config_dir_,
                tr("YAML (*.yaml *.yml);;JSON (*.json);;All files (*)"));
            if (path.isEmpty()){ return; }
            QString err;
            if (!exportDatabase(path, &err)){
                QMessageBox::warning(this, tr("Export failed"), err);
            }
        });

        QMenu* test_menu = menubar->addMenu(tr("&Test"));
        QAction* a_add  = test_menu->addAction(tr("&Add testcase…"));
        QAction* a_edit = test_menu->addAction(tr("&Edit testcase…"));
        QAction* a_tdel = test_menu->addAction(tr("&Delete testcase"));
        connect(a_add,  &QAction::triggered, this, &TestRunnerWindow::onAddTestClicked);
        connect(a_tdel, &QAction::triggered, this, &TestRunnerWindow::onDeleteTestClicked);
        connect(a_edit, &QAction::triggered, this, [this]{
            auto* current = suite_tree_->currentItem();
            if (!current){
                QMessageBox::information(this, tr("Edit testcase"),
                    tr("Pick a test in the suite tree first.")); return;
            }
            const QString test_id = current->data(0, Qt::UserRole).toString();
            if (test_id.isEmpty()){
                QMessageBox::information(this, tr("Edit testcase"),
                    tr("Select a TEST row, not a database header."));
                return;
            }
            const TestSpec spec = findSpec(test_id);
            QStringList db_ids;
            for (const auto& db : databases_){ db_ids << db.id; }
            AddTestDialog dlg(AddTestDialog::Mode::Edit, db_ids, this);
            dlg.setAcquisitionTable(&acquisition_table_);
            dlg.loadFromSpec(spec);
            if (dlg.exec() != QDialog::Accepted){ return; }
            QString err;
            if (!replaceTest(spec.id, dlg.toTestJson(), &err)){
                QMessageBox::warning(this, tr("Edit failed"),
                    tr("Could not update testcase:\n%1").arg(err));
            }
        });

        /* Tools — report exports. Moved here from the bottom bar so
         * the bar stays focused on status + run feedback. */
        QMenu* tools_menu = menubar->addMenu(tr("T&ools"));
        QAction* a_pdf = tools_menu->addAction(tr("&Generate PDF report…"));
        QAction* a_csv = tools_menu->addAction(tr("&Save CSV…"));
        connect(a_pdf, &QAction::triggered, this, &TestRunnerWindow::onGeneratePdf);
        connect(a_csv, &QAction::triggered, this, &TestRunnerWindow::onSaveCsv);

        /* Help — operator-facing reference for the test schema. */
        QMenu* help_menu = menubar->addMenu(tr("&Help"));
        QAction* a_fields = help_menu->addAction(tr("&Field reference…"));
        connect(a_fields, &QAction::triggered, this, &TestRunnerWindow::onShowFieldHelp);
    }
    outer->setMenuBar(menubar);

    outer->addLayout(controls);
    outer->addWidget(split, 1);

    /* Status moved up into the run-control row (see above) so the
     * bottom is free for future additions; no widget here today. */

    desc_popup_   = new TestDescriptionDialog(this);
    /* Wire the persist target — surfaces the Edit Params… button
     * inside the description popup. On save, refresh the popup so
     * the operator sees the just-edited values without reopening. */
    desc_popup_->setPersistTarget(this);
    desc_popup_->setAcquisitionTable(&acquisition_table_);
    connect(desc_popup_, &TestDescriptionDialog::paramsEdited,
            this, [this](const QString& test_id){
                desc_popup_->show(findSpec(test_id));
            });
    result_popup_ = new TestResultDialog(this);
    result_popup_->setAcquisitionTable(&acquisition_table_);
    /* Double-click any row → open the result detail popup for that
     * test's outcome. Single-click on the SUITE TREE still routes to
     * the description popup (spec-only). Two distinct popups so the
     * operator's not surprised by which one shows up. */
    connect(results_table_, &QTableWidget::cellDoubleClicked,
            this, [this](int row, int /*col*/){
                auto* id_item = results_table_->item(row, kColId);
                if (!id_item){ return; }
                const QString id = id_item->text();
                /* Find the matching result row + its spec. */
                for (const auto& r : results_){
                    if (r.test_id != id){ continue; }
                    /* Convert ResultRow.metrics (QJsonObject) into
                     * the QHash<QString,QVariant> the result popup
                     * expects. */
                    QHash<QString, QVariant> m;
                    for (auto it = r.metrics.constBegin();
                              it != r.metrics.constEnd(); ++it){
                        m.insert(it.key(), it.value().toVariant());
                    }
                    result_popup_->show(findSpec(id), r.passed,
                                         r.duration_s, r.actual_result, m);
                    return;
                }
            });

    /* Subscribe to the worker's connected signal so the test bodies
     * have a fresh dof + joint_names cache to work against — without
     * crossing the worker's thread boundary inside a test. */
    if (worker_){
        connect(worker_, &HandWorker::connected,
                this,    &TestRunnerWindow::onWorkerConnected);
        /* Mirror disconnect — flip the status bar back to red and
         * clear the cached dof / joint_names so a subsequent run
         * doesn't think the worker still knows the hand. */
        connect(worker_, &HandWorker::disconnected, this, [this]{
            cached_dof_         = 0;
            cached_joint_names_.clear();
            cached_error_codes_.clear();
            setStatusDisconnected();
        });
        connect(worker_, &HandWorker::errorCodesUpdated,
                this,    &TestRunnerWindow::onWorkerErrorCodes);
        /* Always subscribe — slot is a no-op when window_.active is
         * false. Saves connecting/disconnecting per test (and avoids
         * lost frames in the gap between connect() and the first
         * emit when frames arrive fast). */
        connect(worker_, &HandWorker::stateUpdated,
                this,    &TestRunnerWindow::onWorkerStateUpdated);
        /* URDF-driven joint limits — cached for snapshot resolution
         * so probes can reference $urdf.joint_lo / $urdf.joint_hi
         * without re-implementing the signal subscription. */
        connect(worker_, &HandWorker::urdfJointLimits, this,
                [this](QStringList /*names*/,
                       QVector<double> /*lo*/,
                       QVector<double> /*hi*/){
            /* Pull dof-aligned arrays through the worker accessors;
             * the raw signal carries URDF-order data which doesn't
             * match the actuator dof order on hands whose URDFs
             * include mimic / fixed joints (Inspire). */
            cached_joint_lo_ = worker_->currentJointLimitsLo();
            cached_joint_hi_ = worker_->currentJointLimitsHi();
        });
        /* Seed the cache synchronously — the worker may already be
         * connected by the time the runner window opens (Tools menu
         * is typically clicked AFTER connect), in which case the
         * connected() signal fired before we wired up and our cache
         * would stay at dof=0 forever. Without this, every incoming
         * stateUpdated frame trips the size-mismatch malformed check
         * (q.size() != cached_dof_(=0)) and DB1-01/02 both FAIL. */
        if (worker_->isConnected()){
            onWorkerConnected(worker_->currentDof(),
                              worker_->currentJointNames());
            /* URDF limits — same race as connected/joint_names; the
             * signal already fired before we wired up. */
            cached_joint_lo_ = worker_->currentJointLimitsLo();
            cached_joint_hi_ = worker_->currentJointLimitsHi();
        }

#if VR_MC_TESTRUNNER_PROBES_ENABLED
        /* Round-trip probe — owns the DB1-07 state machine on the
         * worker's thread for tight, queue-free timing. Created with
         * no QObject parent so we can manage its lifetime across the
         * worker-thread boundary; explicit deleteLater in the
         * destructor instead. */
        round_trip_probe_ = new RoundTripProbe(worker_);
        bandwidth_probe_  = new BandwidthSweepProbe(worker_);
        opmode_probe_     = new OperationModeProbe(worker_);
        softlimit_probe_  = new SoftLimitProbe(worker_);
        estop_probe_      = new EstopProbe(worker_);
#endif
    }

    appendLog(LogLevel::Info,
        tr("Test Runner ready. Load a TestDatabase to begin."));
}


TestRunnerWindow::~TestRunnerWindow()
{
    /* Probes live in the worker thread — deleteLater posts each
     * destruction back to that thread so we don't fight Qt's
     * thread-affinity guards on shutdown. Safe to call even when a
     * batch is in-flight; pending QTimer::singleShots on the probe
     * stop firing once their receiver is gone. */
#if VR_MC_TESTRUNNER_PROBES_ENABLED
    auto teardown = [](auto*& p){
        if (p){ p->deleteLater(); p = nullptr; }
    };
    teardown(round_trip_probe_);
    teardown(bandwidth_probe_);
    teardown(opmode_probe_);
    teardown(softlimit_probe_);
    teardown(estop_probe_);
#endif
}


void TestRunnerWindow::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    if (!results_table_){ return; }
    /* Reflow Expected + Actual to consume the leftover width equally.
     * The fixed columns (ID / Status / Time / Name) keep their seeded
     * widths; whatever the table viewport has left over splits 50/50.
     * Guard against minPx so a very narrow window doesn't collapse
     * the prose columns to zero. */
    auto* h = results_table_->horizontalHeader();
    const int viewport_w = results_table_->viewport()->width();
    const int fixed_w    = h->sectionSize(kColId)
                         + h->sectionSize(kColName)
                         + h->sectionSize(kColStatus)
                         + h->sectionSize(kColTime);
    constexpr int kMinPx = 120;
    const int per = std::max(kMinPx, (viewport_w - fixed_w) / 2);
    QSignalBlocker b(h);          /* avoid the mirror-on-drag connect re-firing */
    results_table_->setColumnWidth(kColExpected, per);
    results_table_->setColumnWidth(kColActual,   per);
}


/* ============================================================== *
 *  Config load
 * ============================================================== */
bool TestRunnerWindow::loadConfig(const QString& path, QString* out_error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(path); }
        return false;
    }
    const QByteArray bytes = f.readAll();
    f.close();
    QString yerr;
    const auto doc = yaml_bridge::loadYamlAsJson(bytes, &yerr);
    if (doc.isNull() || !doc.isObject()){
        if (out_error){
            *out_error = yerr.isEmpty()
                ? tr("malformed YAML — root must be a map")
                : yerr;
        }
        return false;
    }
    const auto root = doc.object();
    defaults_           = root.value("_defaults").toObject();
    expected_dof_       = root.value("expected_dof").toInt(0);
    schema_version_     = root.value("schema_version").toInt(1);
    top_meta_           = root.value("_meta").toString();
    loaded_config_path_ = path;

    /* Acquisition catalog. Two on-disk shapes supported:
     *
     *   1. Legacy v2-map (each entry typed in full):
     *        acquisition_table:
     *          telemetry.cycle_mean_us:
     *            kind: window_agg
     *            stat: interval_mean_us
     *
     *   2. New flat-list (kind/resolver derived from prefix at parse time):
     *        acquisition_table:
     *          - state.dof
     *          - telemetry.cycle_mean_us
     *          - probe.round_trip.*
     *
     * The new form lets a `tests.yaml` author whitelist a measurement
     * by name only — we synthesize the dispatch fields here from the
     * `<area>.<rest>` convention so the runner's harvest loop (which
     * still keys off entry.kind/source/stat/probe/output) keeps working
     * unchanged. Keys starting with "_" are documentation (e.g. legacy
     * "_meta") and skipped. */
    acquisition_table_.clear();
    {
        const QJsonValue raw = root.value("acquisition_table");
        if (raw.isArray()){
            for (const auto& nv : raw.toArray()){
                const QString name = nv.toString().trimmed();
                if (name.isEmpty()){ continue; }
                acquisition_table_.insert(name, deriveAcquisitionEntry(name));
            }
        } else {
            const auto cat = raw.toObject();
            for (auto it = cat.constBegin(); it != cat.constEnd(); ++it){
                if (it.key().startsWith('_')){ continue; }
                const QJsonObject e = it.value().toObject();
                AcquisitionEntry a;
                a.kind      = e.value("kind").toString();
                a.source    = e.value("source").toString();
                a.stat      = e.value("stat").toString();
                a.probe     = e.value("probe").toString();
                a.output    = e.value("output").toString();
                a.type_hint = e.value("type").toString();
                a.meaning   = e.value("meaning").toString();
                acquisition_table_.insert(it.key(), a);
            }
        }
    }

    databases_.clear();
    for (const auto& db_v : root.value("databases").toArray()){
        const auto db_o = db_v.toObject();
        TestDatabase db;
        db.id          = db_o.value("id").toString();
        db.name        = db_o.value("name").toString();
        db.description = db_o.value("description").toString();
        for (const auto& t_v : db_o.value("tests").toArray()){
            const auto t_o = t_v.toObject();
            TestSpec s;
            s.id          = t_o.value("id").toString();
            s.name        = t_o.value("name").toString();
            s.description = t_o.value("description").toString();
            /* pre_conditions accepts two shapes:
             *   - list of strings (legacy)
             *   - list of objects {check, on_fail: skip|fail, msg}
             *     (schema-v2; stringified here so render code is
             *     unchanged downstream). */
            for (const auto& p : t_o.value("pre_conditions").toArray()){
                if (p.isString()){
                    s.pre_conditions.append(p.toString());
                } else if (p.isObject()){
                    const auto po = p.toObject();
                    const QString check   = po.value("check").toString();
                    const QString on_fail = po.value("on_fail").toString();
                    const QString msg     = po.value("msg").toString();
                    QStringList parts;
                    if (!check.isEmpty()){ parts << check; }
                    if (!on_fail.isEmpty() || !msg.isEmpty()){
                        QStringList sub;
                        if (!on_fail.isEmpty()){ sub << QStringLiteral("on_fail: %1").arg(on_fail); }
                        if (!msg.isEmpty()){     sub << msg; }
                        parts << QStringLiteral("(%1)").arg(sub.join(QStringLiteral(" — ")));
                    }
                    s.pre_conditions.append(parts.join(QChar(' ')));
                }
            }
            for (const auto& p : t_o.value("procedure").toArray()){
                s.procedure.append(p.toString());
            }
            /* teardown — schema-v2 cleanup actions. Display-only. */
            for (const auto& p : t_o.value("teardown").toArray()){
                s.teardown.append(p.toString());
            }
            /* (schema-v1 `metrics: [{name, meaning}]`, `params: [...]`,
             * `window_s`, `not_implemented`, `pending_reason` readers
             * were dropped — schema v2 uses captures / probe_params for
             * the same intent. Legacy files need migration.) */
            /* pass_criteria accepts two shapes:
             *   - bare string (legacy free-form text) → store verbatim
             *   - list of objects {check, error_code, error_msg}
             *     (schema-v2) → parse each `check:` expression into a
             *     PassRule, accumulate a human-readable multi-line
             *     summary into s.pass_criteria so existing render code
             *     (description popup, result table) keeps working. */
            {
                const QJsonValue pcv = t_o.value("pass_criteria");
                if (pcv.isArray()){
                    QStringList summary;
                    for (const auto& cv : pcv.toArray()){
                        if (!cv.isObject()){ continue; }
                        const auto co = cv.toObject();
                        const QString check = co.value("check").toString();
                        const QString ecode = co.value("error_code").toString();
                        const QString emsg  = co.value("error_msg").toString();
                        QString lhs, op;
                        QJsonValue rhs;
                        TestSpec::PassRule rule;
                        rule.lhs_raw    = check;
                        rule.error_code = ecode;
                        rule.error_msg  = emsg;
                        if (parseCheckExpr(check, &lhs, &op, &rhs)){
                            rule.op  = op;
                            rule.rhs = rhs;
                            using R = TestSpec::PassRule::Reducer;
                            const int colon = lhs.indexOf(':');
                            if (colon < 0){
                                rule.lhs_reducer = R::None;
                                rule.lhs_arg     = lhs;
                            } else {
                                const QString head = lhs.left(colon);
                                const QString tail = lhs.mid(colon + 1);
                                if      (head == "max")           { rule.lhs_reducer = R::Max; }
                                else if (head == "min")           { rule.lhs_reducer = R::Min; }
                                else if (head == "mean")          { rule.lhs_reducer = R::Mean; }
                                else if (head == "p99")           { rule.lhs_reducer = R::P99; }
                                else if (head == "sum")           { rule.lhs_reducer = R::Sum; }
                                else if (head == "stdev")         { rule.lhs_reducer = R::Stdev; }
                                else if (head == "range")         { rule.lhs_reducer = R::Range; }
                                else if (head == "cv")            { rule.lhs_reducer = R::Cv; }
                                else if (head == "len")           { rule.lhs_reducer = R::Len; }
                                else if (head == "count_nonzero") { rule.lhs_reducer = R::CountNonzero; }
                                else if (head == "div")           { rule.lhs_reducer = R::Div; }
                                else                              { rule.lhs_reducer = R::None; }
                                if (rule.lhs_reducer == R::Div){
                                    const int slash = tail.indexOf('/');
                                    rule.lhs_arg  = (slash >= 0) ? tail.left(slash)  : tail;
                                    rule.lhs_arg2 = (slash >= 0) ? tail.mid(slash+1) : QString();
                                } else {
                                    rule.lhs_arg = tail.endsWith("[]")
                                                      ? tail.left(tail.size() - 2)
                                                      : tail;
                                }
                            }
                        }
                        s.pass_rules.append(rule);
                        QStringList line;
                        line << check;
                        if (!ecode.isEmpty()){ line << QStringLiteral("[%1]").arg(ecode); }
                        if (!emsg .isEmpty()){ line << QStringLiteral("— %1").arg(emsg); }
                        summary << line.join(QChar(' '));
                    }
                    s.pass_criteria = summary.join(QChar('\n'));
                } else {
                    s.pass_criteria = pcv.toString();
                }
            }
            s.pass_combiner = t_o.value("pass_combiner").toString("AND").toUpper();

            /* captures: flat list of acquisition-table names that this
             * test wants the runner to materialise into its metrics
             * object before pass_criteria evaluation. */
            for (const auto& cv : t_o.value("captures").toArray()){
                s.captures.append(cv.toString());
            }
            /* probe_params: free-form map of probe knobs (thresholds,
             * trial counts, ...). Also serves as the lookup table for
             * bare-identifier $names referenced from pass_criteria's
             * `check:` expressions. */
            s.probe_params         = t_o.value("probe_params").toObject();
            s.duration_estimate_s  = t_o.value("duration_estimate_s").toDouble();
            s.num_samples          = t_o.value("num_samples").toInt(0);
            s.blocks_if_fails      = t_o.value("blocks_if_fails").toBool();
            for (const auto& d : t_o.value("depends_on").toArray()){
                s.depends_on.append(d.toString());
            }
            s.db_id = db.id;
            db.tests.append(s);
        }
        databases_.append(db);
    }
    rebuildTree();
    int total = 0;
    for (const auto& db : databases_){ total += db.tests.size(); }
    appendLog(LogLevel::Info, 
        tr("loaded %1 databases, %2 tests from %3 (expected_dof=%4)")
            .arg(databases_.size())
            .arg(total)
            .arg(path)
            .arg(expected_dof_));
    return true;
}


bool TestRunnerWindow::writeParams(const QString& test_id,
                                    const QJsonObject& new_params,
                                    QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    /* Read-mutate-write the on-disk JSON so non-params fields (and
     * any keys the parser doesn't know about) survive intact. We
     * could re-serialise from the in-memory tree, but that loses
     * fields we don't model — safer to mutate in place. */
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1 for read")
                                       .arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }

    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool found = false;
    for (int di = 0; di < dbs.size() && !found; ++di){
        QJsonObject db = dbs[di].toObject();
        QJsonArray tests = db.value("tests").toArray();
        for (int ti = 0; ti < tests.size(); ++ti){
            QJsonObject t = tests[ti].toObject();
            if (t.value("id").toString() != test_id){ continue; }
            /* Schema v2: probe_params is a plain map. Read it, splice
             * in the operator's edits (existing keys take new values;
             * new keys append), and write back. The order in YAML
             * follows the iteration order of QJsonObject (alphabetical
             * by key) which is good enough for diff-friendly output. */
            QJsonObject pp = t.value("probe_params").toObject();
            for (auto it = new_params.constBegin();
                      it != new_params.constEnd(); ++it){
                pp.insert(it.key(), it.value());
            }
            t.insert("probe_params", pp);
            tests[ti] = t;
            found = true;
            break;
        }
        if (found){
            db.insert("tests", tests);
            dbs[di] = db;
        }
    }
    if (!found){
        if (out_error){ *out_error = tr("test id %1 not in file")
                                       .arg(test_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);

    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write")
                                       .arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();

    /* Mirror in the in-memory tree so a re-run picks the new values
     * up without forcing a full reload. probe_params on disk is the
     * authoritative knob source; merge the operator's edits onto
     * whatever was already loaded so untouched keys survive. */
    for (auto& db : databases_){
        for (auto& t : db.tests){
            if (t.id == test_id){
                for (auto it = new_params.constBegin();
                          it != new_params.constEnd(); ++it){
                    t.probe_params.insert(it.key(), it.value());
                }
            }
        }
    }
    appendLog(LogLevel::Info,
        tr("%1 params updated and saved to %2")
            .arg(test_id, loaded_config_path_));
    return true;
}


/* ============================================================== *
 *  Add / Delete test — read-mutate-write on the on-disk JSON, then
 *  reload the in-memory tree from the same file so the suite-tree
 *  rebuild path is shared with the regular load. Keeps the two
 *  views in lock-step at the cost of one extra parse on every
 *  mutation (cheap, tests.yaml is small).
 * ============================================================== */
bool TestRunnerWindow::addTest(const QString& db_id,
                                 const QJsonObject& test_json,
                                 QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    const QString new_id = test_json.value("id").toString();
    if (new_id.isEmpty()){
        if (out_error){ *out_error = tr("test JSON is missing 'id'"); }
        return false;
    }
    /* Duplicate-id guard against everything currently in memory. */
    for (const auto& db : databases_){
        for (const auto& t : db.tests){
            if (t.id == new_id){
                if (out_error){ *out_error = tr("test id '%1' already exists")
                                                  .arg(new_id); }
                return false;
            }
        }
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1 for read")
                                       .arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool found_db = false;
    for (int di = 0; di < dbs.size(); ++di){
        QJsonObject db = dbs[di].toObject();
        if (db.value("id").toString() != db_id){ continue; }
        QJsonArray tests = db.value("tests").toArray();
        tests.append(test_json);
        db.insert("tests", tests);
        dbs[di] = db;
        found_db = true;
        break;
    }
    if (!found_db){
        if (out_error){ *out_error = tr("database '%1' not in file")
                                       .arg(db_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write")
                                       .arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    /* Re-read so the in-memory tree + tree widget refresh through
     * the regular path. Suppress its error (we just wrote it
     * ourselves, so reading must succeed). */
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info,
        tr("test '%1' added to %2 and saved").arg(new_id, db_id));
    return true;
}


bool TestRunnerWindow::deleteTest(const QString& test_id,
                                    QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1 for read")
                                       .arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool removed = false;
    for (int di = 0; di < dbs.size() && !removed; ++di){
        QJsonObject db = dbs[di].toObject();
        QJsonArray tests = db.value("tests").toArray();
        for (int ti = 0; ti < tests.size(); ++ti){
            if (tests[ti].toObject().value("id").toString() != test_id){
                continue;
            }
            tests.removeAt(ti);
            db.insert("tests", tests);
            dbs[di] = db;
            removed = true;
            break;
        }
    }
    if (!removed){
        if (out_error){ *out_error = tr("test id '%1' not found")
                                       .arg(test_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write")
                                       .arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info,
        tr("test '%1' deleted and saved").arg(test_id));
    return true;
}


bool TestRunnerWindow::updateTestMeta(const QString& test_id,
                                        const QJsonObject& meta,
                                        QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool found = false;
    for (int di = 0; di < dbs.size() && !found; ++di){
        QJsonObject db = dbs[di].toObject();
        QJsonArray tests = db.value("tests").toArray();
        for (int ti = 0; ti < tests.size(); ++ti){
            QJsonObject t = tests[ti].toObject();
            if (t.value("id").toString() != test_id){ continue; }
            /* Only touch meta keys — params, metrics, pre_conditions,
             * procedure, pass_criteria, etc. stay as-is. */
            if (meta.contains("name"))       { t.insert("name",        meta.value("name")); }
            if (meta.contains("description")){ t.insert("description", meta.value("description")); }
            tests[ti] = t;
            db.insert("tests", tests);
            dbs[di] = db;
            found = true;
            break;
        }
    }
    if (!found){
        if (out_error){ *out_error = tr("test id '%1' not found").arg(test_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write").arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info,
        tr("test '%1' metadata updated and saved").arg(test_id));
    return true;
}


bool TestRunnerWindow::addDatabase(const QString& id,
                                     const QString& name,
                                     const QString& description,
                                     QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    if (id.trimmed().isEmpty()){
        if (out_error){ *out_error = tr("database ID can't be empty"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    for (const auto& dv : dbs){
        if (dv.toObject().value("id").toString() == id){
            if (out_error){ *out_error = tr("database id '%1' already exists")
                                              .arg(id); }
            return false;
        }
    }
    QJsonObject db;
    db.insert("id",          id);
    db.insert("name",        name);
    db.insert("description", description);
    db.insert("tests",       QJsonArray{});
    dbs.append(db);
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write").arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info, tr("database '%1' added and saved").arg(id));
    return true;
}


bool TestRunnerWindow::updateDatabase(const QString& old_id,
                                        const QString& new_id,
                                        const QString& new_name,
                                        const QString& new_description,
                                        QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    /* If renaming, refuse if the new id collides with another DB. */
    if (new_id != old_id){
        for (const auto& dv : dbs){
            if (dv.toObject().value("id").toString() == new_id){
                if (out_error){ *out_error = tr("database id '%1' already exists")
                                                  .arg(new_id); }
                return false;
            }
        }
    }
    bool found = false;
    for (int di = 0; di < dbs.size() && !found; ++di){
        QJsonObject db = dbs[di].toObject();
        if (db.value("id").toString() != old_id){ continue; }
        db.insert("id",          new_id);
        db.insert("name",        new_name);
        db.insert("description", new_description);
        /* If id changed, NB: every test's db_id is rendered from the
         * parent DB's id on the next loadConfig, so no test-side
         * rewrite is needed — the parser re-derives db_id when it
         * walks the file. */
        dbs[di] = db;
        found = true;
    }
    if (!found){
        if (out_error){ *out_error = tr("database id '%1' not found").arg(old_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write").arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info,
        tr("database '%1' updated and saved%2")
            .arg(new_id, (new_id != old_id)
                            ? tr(" (renamed from %1)").arg(old_id)
                            : QString()));
    return true;
}


bool TestRunnerWindow::deleteDatabase(const QString& id, QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool removed = false;
    for (int di = 0; di < dbs.size(); ++di){
        if (dbs[di].toObject().value("id").toString() != id){ continue; }
        dbs.removeAt(di);
        removed = true;
        break;
    }
    if (!removed){
        if (out_error){ *out_error = tr("database id '%1' not found").arg(id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write").arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info, tr("database '%1' deleted and saved").arg(id));
    return true;
}


bool TestRunnerWindow::replaceTest(const QString& test_id,
                                     const QJsonObject& test_json,
                                     QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(loaded_config_path_); }
        return false;
    }
    QString yerr;
    auto doc = yaml_bridge::loadYamlAsJson(fr.readAll(), &yerr);
    fr.close();
    if (doc.isNull() || !doc.isObject()){
        if (out_error){ *out_error = yerr.isEmpty()
                                       ? tr("malformed YAML")
                                       : yerr; }
        return false;
    }
    QJsonObject root = doc.object();
    QJsonArray dbs = root.value("databases").toArray();
    bool found = false;
    for (int di = 0; di < dbs.size() && !found; ++di){
        QJsonObject db = dbs[di].toObject();
        QJsonArray tests = db.value("tests").toArray();
        for (int ti = 0; ti < tests.size(); ++ti){
            if (tests[ti].toObject().value("id").toString() != test_id){
                continue;
            }
            tests[ti] = test_json;
            db.insert("tests", tests);
            dbs[di] = db;
            found = true;
            break;
        }
    }
    if (!found){
        if (out_error){ *out_error = tr("test id '%1' not found").arg(test_id); }
        return false;
    }
    root.insert("databases", dbs);
    doc.setObject(root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write")
                                       .arg(loaded_config_path_); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    appendLog(LogLevel::Info,
        tr("test '%1' replaced and saved").arg(test_id));
    return true;
}


bool TestRunnerWindow::createNewDatabase(const QString& path_in,
                                           int expected_dof,
                                           QString* out_error)
{
    /* Default-extension policy: if the user typed a bare name in the
     * file picker, append ".yaml" so the new file matches the on-disk
     * format we now write. JSON files are still readable on load. */
    QString path = path_in;
    {
        const QString suffix = QFileInfo(path).suffix().toLower();
        if (suffix != "yaml" && suffix != "yml" && suffix != "json"){
            path += QStringLiteral(".yaml");
        }
    }
    /* Minimal skeleton — schema header + two empty DBs. Operator
     * adds tests via the menu / Add Test. Mirrors what the Python
     * template generator emits for an unfilled hand. */
    QJsonObject root;
    root.insert("schema_version", 1);
    root.insert("hand_type",      QStringLiteral("TODO"));
    root.insert("expected_dof",   expected_dof);
    auto make_db = [](const char* id, const char* name, const char* desc){
        QJsonObject o;
        o.insert("id",          QString::fromLatin1(id));
        o.insert("name",        QString::fromLatin1(name));
        o.insert("description", QString::fromLatin1(desc));
        o.insert("tests",       QJsonArray{});
        return o;
    };
    QJsonArray dbs;
    dbs.append(make_db("DB1", "Communication",
                        "Bus + telemetry tests."));
    dbs.append(make_db("DB2", "Functions",
                        "Joint-level functional coverage."));
    root.insert("databases", dbs);

    QJsonDocument doc(root);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't open %1 for write").arg(path); }
        return false;
    }
    f.write(yaml_bridge::dumpJsonAsYaml(doc));
    f.close();
    QString err;
    if (!loadConfig(path, &err)){
        if (out_error){ *out_error = err; }
        return false;
    }
    appendLog(LogLevel::Info, tr("new TestDatabase created at %1").arg(path));
    return true;
}


bool TestRunnerWindow::deleteCurrentDatabase(QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("no test database loaded"); }
        return false;
    }
    const QString path = loaded_config_path_;
    if (!QFile::remove(path)){
        if (out_error){ *out_error = tr("could not delete %1").arg(path); }
        return false;
    }
    databases_.clear();
    loaded_config_path_.clear();
    rebuildTree();
    appendLog(LogLevel::Warn, tr("deleted %1").arg(path));
    return true;
}


bool TestRunnerWindow::importDatabase(const QString& path,
                                        int* out_added,
                                        int* out_skipped,
                                        int* out_dbs_added,
                                        QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("load a destination test database first"); }
        return false;
    }
    QFile fi(path);
    if (!fi.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open %1").arg(path); }
        return false;
    }
    QString src_err;
    const auto src_doc = yaml_bridge::loadYamlAsJson(fi.readAll(), &src_err);
    fi.close();
    if (src_doc.isNull() || !src_doc.isObject()){
        if (out_error){ *out_error = tr("malformed source YAML: %1")
                                       .arg(src_err); }
        return false;
    }
    QFile fd(loaded_config_path_);
    if (!fd.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't open destination"); }
        return false;
    }
    QString dst_err;
    auto dst_doc = yaml_bridge::loadYamlAsJson(fd.readAll(), &dst_err);
    fd.close();
    if (dst_doc.isNull() || !dst_doc.isObject()){
        if (out_error){ *out_error = tr("malformed destination YAML: %1")
                                       .arg(dst_err); }
        return false;
    }
    QJsonObject dst_root = dst_doc.object();
    QJsonArray dst_dbs = dst_root.value("databases").toArray();
    QSet<QString> existing_ids;
    for (const auto& dv : dst_dbs){
        for (const auto& tv : dv.toObject().value("tests").toArray()){
            existing_ids.insert(tv.toObject().value("id").toString());
        }
    }
    int added = 0, skipped = 0, dbs_added = 0;
    for (const auto& src_dv : src_doc.object().value("databases").toArray()){
        const QJsonObject src_db = src_dv.toObject();
        const QString src_db_id  = src_db.value("id").toString();
        int dst_idx = -1;
        for (int i = 0; i < dst_dbs.size(); ++i){
            if (dst_dbs[i].toObject().value("id").toString() == src_db_id){
                dst_idx = i; break;
            }
        }
        if (dst_idx < 0){
            dst_dbs.append(src_db);
            ++dbs_added;
            for (const auto& tv : src_db.value("tests").toArray()){
                existing_ids.insert(tv.toObject().value("id").toString());
                ++added;
            }
            continue;
        }
        QJsonObject dst_db = dst_dbs[dst_idx].toObject();
        QJsonArray dst_tests = dst_db.value("tests").toArray();
        for (const auto& tv : src_db.value("tests").toArray()){
            const QJsonObject t = tv.toObject();
            const QString id = t.value("id").toString();
            if (existing_ids.contains(id)){ ++skipped; continue; }
            dst_tests.append(t);
            existing_ids.insert(id);
            ++added;
        }
        dst_db.insert("tests", dst_tests);
        dst_dbs[dst_idx] = dst_db;
    }
    dst_root.insert("databases", dst_dbs);
    dst_doc.setObject(dst_root);
    QFile fw(loaded_config_path_);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't write destination"); }
        return false;
    }
    fw.write(yaml_bridge::dumpJsonAsYaml(dst_doc));
    fw.close();
    QString err;
    loadConfig(loaded_config_path_, &err);
    if (out_added)     { *out_added = added; }
    if (out_skipped)   { *out_skipped = skipped; }
    if (out_dbs_added) { *out_dbs_added = dbs_added; }
    appendLog(LogLevel::Info,
        tr("imported %1 — added %2 tests, skipped %3 dupes, "
            "appended %4 new DBs")
            .arg(path).arg(added).arg(skipped).arg(dbs_added));
    return true;
}


bool TestRunnerWindow::exportDatabase(const QString& path,
                                        QString* out_error)
{
    if (loaded_config_path_.isEmpty()){
        if (out_error){ *out_error = tr("nothing to export — no file loaded"); }
        return false;
    }
    /* Byte-for-byte copy so the exported file matches the on-disk
     * source exactly — avoids losing any keys the parser doesn't
     * model. */
    QFile fr(loaded_config_path_);
    if (!fr.open(QIODevice::ReadOnly)){
        if (out_error){ *out_error = tr("can't read source"); }
        return false;
    }
    const QByteArray raw = fr.readAll();
    fr.close();
    QFile fw(path);
    if (!fw.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        if (out_error){ *out_error = tr("can't write %1").arg(path); }
        return false;
    }
    fw.write(raw);
    fw.close();
    appendLog(LogLevel::Info, tr("exported to %1").arg(path));
    return true;
}


void TestRunnerWindow::onAddTestClicked()
{
    if (loaded_config_path_.isEmpty()){
        QMessageBox::information(this, tr("Add test"),
            tr("Load a test database first — Add Test needs a file to "
               "persist into."));
        return;
    }
    if (databases_.isEmpty()){
        QMessageBox::information(this, tr("Add test"),
            tr("The loaded file has no databases. Create a DB block "
               "by hand first, then come back."));
        return;
    }
    QStringList db_ids;
    for (const auto& db : databases_){ db_ids << db.id; }

    /* Suggest the next free id within the currently-selected DB (or
     * the first DB if nothing's selected). Pattern: <DBn>-<NN>. */
    QString suggested_db = db_ids.first();
    auto* current = suite_tree_->currentItem();
    if (current){
        /* Walk up to the DB-level item. */
        QTreeWidgetItem* p = current;
        while (p->parent()){ p = p->parent(); }
        const QString picked = p->text(0).section(' ', 0, 0);
        if (db_ids.contains(picked)){ suggested_db = picked; }
    }
    int next_n = 1;
    for (const auto& db : databases_){
        if (db.id != suggested_db){ continue; }
        for (const auto& t : db.tests){
            const QString suffix = t.id.section('-', 1, 1);
            bool ok = false;
            const int n = suffix.toInt(&ok);
            if (ok && n >= next_n){ next_n = n + 1; }
        }
    }
    const QString suggested_id = tr("%1-%2")
        .arg(suggested_db)
        .arg(next_n, 2, 10, QChar('0'));

    AddTestDialog dlg(AddTestDialog::Mode::Add, db_ids, this);
    dlg.setAcquisitionTable(&acquisition_table_);
    dlg.setSuggestedDb(suggested_db);
    dlg.setSuggestedId(suggested_id);
    if (dlg.exec() != QDialog::Accepted){ return; }
    QString err;
    if (!addTest(dlg.parentDbId(), dlg.toTestJson(), &err)){
        QMessageBox::warning(this, tr("Add failed"),
            tr("Could not add test:\n%1").arg(err));
        return;
    }
}


void TestRunnerWindow::onDeleteTestClicked()
{
    if (loaded_config_path_.isEmpty()){
        QMessageBox::information(this, tr("Delete test"),
            tr("Load a test database first."));
        return;
    }
    auto* current = suite_tree_->currentItem();
    if (!current){
        QMessageBox::information(this, tr("Delete test"),
            tr("Pick a test in the suite tree first."));
        return;
    }
    /* Only leaf items represent actual tests — UserRole carries the
     * test id; an empty UserRole means the operator picked a DB
     * header row. */
    const QString test_id = current->data(0, Qt::UserRole).toString();
    if (test_id.isEmpty()){
        QMessageBox::information(this, tr("Delete test"),
            tr("Select a TEST row (the leaf), not a database header."));
        return;
    }
    const QString test_name = current->text(0);
    const auto choice = QMessageBox::question(this, tr("Delete test"),
        tr("Delete '%1' (%2)?\n\nThis edits the test database on disk and "
           "cannot be undone from here.").arg(test_name, test_id),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (choice != QMessageBox::Yes){ return; }
    QString err;
    if (!deleteTest(test_id, &err)){
        QMessageBox::warning(this, tr("Delete failed"),
            tr("Could not delete test:\n%1").arg(err));
        return;
    }
}


void TestRunnerWindow::onShowFieldHelp()
{
    /* Single-pane HTML reference. Kept inline because there's only
     * one entry point and the content is static — extracting a
     * dedicated dialog class would just add files for no benefit. */
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Field reference"));
    dlg.resize(760, 640);
    auto* lay = new QVBoxLayout(&dlg);
    auto* browser = new QTextBrowser(&dlg);
    browser->setOpenExternalLinks(false);
    browser->setHtml(QStringLiteral(R"(
<h2 style="color:#1565c0;">TestSpec field reference (schema v2)</h2>
<p>Schema v2 (Fork P3-a) makes test authoring <b>YAML-only</b> for
every measurement that fits an existing acquisition kind. The file
declares an <code>acquisition_table</code> (catalog of available
measurements). Each test picks measurements by name in its
<code>captures</code> list, supplies any window length or probe
configuration, and writes pass rules. The runner does the rest —
no <code>spec.id == ...</code> branches in C++.</p>

<h3 style="color:#1565c0;">acquisition_table</h3>
<p>Top-level catalog. Each entry has a <code>kind</code>:</p>
<ul>
<li><b>snapshot</b> — pulls a cached value out of HandWorker
(<code>worker.dof</code>, <code>worker.joint_names</code>,
<code>worker.error_codes</code>).</li>
<li><b>config</b> — pulls a value from the file header
(<code>expected_dof</code>).</li>
<li><b>window_agg</b> — computes a statistic over a telemetry
window. <code>stat</code> picks which: <code>frame_count</code>,
<code>malformed_count</code>, <code>interval_mean_us</code>,
<code>interval_stdev_us</code>, <code>interval_p99_us</code>,
<code>interval_max_us</code>, <code>intervals_us_array</code>,
<code>samples_per_joint</code>, <code>actual_duration_s</code>,
<code>agg_rate_hz</code>.</li>
<li><b>probe</b> — harvests an output from an active probe.
<code>probe</code> picks the probe class (<code>round_trip</code>
or <code>bandwidth_sweep</code>); <code>output</code> picks which
of the probe's emitted metric keys to expose.</li>
</ul>

<h3 style="color:#1565c0;">Per-test fields</h3>
<ul>
<li><code>captures</code>: list of acquisition-table names. The
runner groups by kind and runs each group.</li>
<li><code>window_s</code>: telemetry window length, required when
any capture is kind:<code>window_agg</code>.</li>
<li><code>probe_params</code>: <code>{probe: "...", ...}</code>
configuration passed to the probe's <code>startBatch</code>.
Required when any capture is kind:<code>probe</code>.</li>
<li><code>not_implemented</code>: short-circuit to PASS with
<code>pending_reason</code> shown as actual result.</li>
<li><code>params</code> + <code>pass_rules</code> + <code>pass_combiner</code>
unchanged from Fork E1 — see the next section.</li>
</ul>

<h3 style="color:#1565c0;">Adding a brand-new test</h3>
<p><b>Reusing an existing measurement type:</b> edit JSON, add a
new test entry with appropriate <code>captures</code> + rules. No
rebuild.</p>
<p><b>Adding a new measurement type:</b> implement a new resolver
(or probe), then add one entry to <code>acquisition_table</code>
exposing it. Future tests using that measurement = YAML-only.</p>

<h3 style="color:#1565c0;">Original metadata fields</h3>
<p>Every test case in <code>tests.yaml</code> still carries the
following — same shape as v1:</p>

<h3 style="color:#1565c0;">Identity &amp; ownership</h3>
<table border="1" cellpadding="6" cellspacing="0" style="border-collapse: collapse;">
<tr style="background:#e3f2fd;"><th>Field</th><th>Purpose</th></tr>
<tr><td><code>id</code></td><td>Unique key (e.g. <code>DB1-07</code>). Used by run queues and report references. Locked in Edit mode.</td></tr>
<tr><td><code>name</code></td><td>Human label shown in the suite tree and dialogs.</td></tr>
<tr><td><code>description</code></td><td>Multi-paragraph rationale &mdash; what the test measures and why. Shown in the description dialog.</td></tr>
<tr><td>parent <code>DB</code></td><td>Which database the test belongs to (<code>DB1</code>, <code>DB2</code>, …). Drives the suite-tree grouping. Locked in Edit mode.</td></tr>
</table>

<h3 style="color:#1565c0;">Pre-flight &amp; procedure</h3>
<table border="1" cellpadding="6" cellspacing="0" style="border-collapse: collapse;">
<tr style="background:#e3f2fd;"><th>Field</th><th>Purpose</th></tr>
<tr><td><code>pre_conditions[]</code></td><td>State / environmental assumptions the operator must satisfy before running. <i>Not</i> auto-checked &mdash; documentation only.</td></tr>
<tr><td><code>procedure[]</code></td><td>Numbered step list shown in the description and result dialogs. Documentation.</td></tr>
</table>

<h3 style="color:#1565c0;">Metrics &mdash; what the test MEASURES</h3>
<p>Stored as <code>[{name, meaning}, ...]</code>. Names declare the
numeric outputs the test produces at run time; meanings describe
each value to the operator. The actual numeric values appear only
in the Result dialog after a run.</p>
<p><b>Example</b> (DB1-07 round-trip):</p>
<pre style="background:#f5f5f5; padding:8px;">
"metrics": [
  {"name": "round_trip_ms[]",    "meaning": "Per-trial ack latency in ms"},
  {"name": "round_trip_mean_ms", "meaning": "Mean across acked trials"},
  {"name": "trials_acked",       "meaning": "Trials where motion was detected within cap"}
]
</pre>

<h3 style="color:#1565c0;">Params &mdash; what the test READS</h3>
<p>Stored as <code>[{name, value, meaning}, ...]</code>. Each
param's <code>value</code> is the knob the test body actually
reads (threshold, capture window, joint index, …). Tuning a
threshold is a YAML-only edit &mdash; no rebuild.</p>
<p><b>Value types</b>: bool, int, double, string, array, object.
JSON-parsed on save so types round-trip cleanly. Edit values via
the dialog's Params tab or the dedicated Edit Params&hellip; popup.</p>
<p><b>Example</b> (DB1-07):</p>
<pre style="background:#f5f5f5; padding:8px;">
"params": [
  {"name": "joint_index",       "value": 3,    "meaning": "Which DOF to drive"},
  {"name": "ack_tolerance_rad", "value": 0.005, "meaning": "|q - baseline| ≥ this counts as acked"},
  {"name": "max_round_trip_ms", "value": 100,  "meaning": "Per-trial hard timeout"},
  {"name": "trials",            "value": 20,   "meaning": "Round-trip iterations averaged"}
]
</pre>

<h3 style="color:#1565c0;">Pass criteria &mdash; Fork-E1 rule-driven</h3>
<p>The PASS/FAIL decision is now <b>fully JSON-driven</b> via
<code>pass_rules</code> &mdash; a flat list of comparisons evaluated
after the test body produces its metrics. The C++ acquisition body
emits <code>metrics</code> only; the runner derives PASS/FAIL by
walking <code>pass_rules</code>.</p>

<p><b>Rule shape</b>:</p>
<pre style="background:#f5f5f5; padding:8px;">
"pass_rules": [
  {"lhs": "trials_acked",      "op": "==", "rhs": "$trials"},
  {"lhs": "max:round_trip_ms", "op": "&lt;=", "rhs": "$max_round_trip_ms"}
],
"pass_combiner": "AND"      // or "OR"
</pre>

<p><b>LHS reducers</b> &mdash; prefix the metric name:</p>
<table border="1" cellpadding="6" cellspacing="0" style="border-collapse: collapse;">
<tr style="background:#e3f2fd;"><th>Prefix</th><th>Meaning</th></tr>
<tr><td><code>(none)</code></td><td>The metric value as-is (scalar).</td></tr>
<tr><td><code>max:</code></td><td>Max of an array metric.</td></tr>
<tr><td><code>min:</code></td><td>Min of an array metric.</td></tr>
<tr><td><code>mean:</code></td><td>Arithmetic mean of an array metric.</td></tr>
<tr><td><code>p99:</code></td><td>99th percentile of an array metric.</td></tr>
<tr><td><code>sum:</code></td><td>Sum of an array metric.</td></tr>
<tr><td><code>stdev:</code></td><td>Sample standard deviation (N&minus;1 denominator). Returns 0 when the array has &lt; 2 elements.</td></tr>
<tr><td><code>range:</code></td><td>Spread &mdash; <code>max &minus; min</code>.</td></tr>
<tr><td><code>cv:</code></td><td>Coefficient of variation &mdash; <code>stdev / mean</code>. Zero-mean arrays fail the rule (CV undefined).</td></tr>
<tr><td><code>len:</code></td><td>Array length.</td></tr>
<tr><td><code>count_nonzero:</code></td><td>Count of non-zero elements.</td></tr>
<tr><td><code>div:a/b</code></td><td>a / b (zero divisor fails the rule).</td></tr>
</table>

<p><b>RHS</b> &mdash; either a literal (number, bool, string) or a
context reference <code>"$name"</code> which resolves first against
the runtime metrics, then against <code>params</code>.</p>

<p><b>Operators</b>: <code>==</code> <code>!=</code> <code>&lt;=</code>
<code>&gt;=</code> <code>&lt;</code> <code>&gt;</code></p>

<p><b>Empty <code>pass_rules</code></b> &mdash; the test body's hint
boolean stands. Used for short-circuit cases (e.g.
<code>implemented: false</code>).</p>

<p><b><code>pass_criteria</code></b> (the plain-text field) is now
purely operator-facing documentation. The structured rules are the
source of truth.</p>

<p><b>Editing</b> &mdash; tune a threshold by editing
<code>params</code>; change the comparator structure by editing
<code>pass_rules</code>. Both are YAML-only, no rebuild needed.</p>

<h3 style="color:#1565c0;">Timing &amp; gating</h3>
<table border="1" cellpadding="6" cellspacing="0" style="border-collapse: collapse;">
<tr style="background:#e3f2fd;"><th>Field</th><th>Purpose</th></tr>
<tr><td><code>duration_estimate_s</code></td><td>How long the test typically runs (display + ETA reasoning).</td></tr>
<tr><td><code>num_samples</code></td><td>Hint &mdash; sample / trial count. Display-only, NOT read by bodies. Some tests' <code>params.trials</code> drives the actual count; here, num_samples matches when 1 trial = 1 sample.</td></tr>
<tr><td><code>blocks_if_fails</code></td><td>When true and "Continue on failure" is off, a FAIL on this test stops the run.</td></tr>
<tr><td><code>depends_on[]</code></td><td>Other test ids that must PASS first. Display-only for now (no runtime enforcement).</td></tr>
</table>

<h3 style="color:#1565c0;">Pending / unimplemented tests</h3>
<p>A test with <code>params.implemented: false</code> short-circuits
to PASS with <code>pending_reason</code> shown as actual result &mdash;
useful for placeholders in a hand's test plan where the body isn't
written yet. Removes the risk of an accidental silent PASS
masquerading as real coverage.</p>

<h3 style="color:#1565c0;">PASS/FAIL evaluation flow</h3>
<ol>
<li>Runner dispatches <code>spec.id</code> to the matching test
body (or the unimplemented short-circuit).</li>
<li>Body reads every threshold from
<code>spec.probe_params.value("…")</code> &mdash; defaults are baked in
only as a safety net.</li>
<li>Body acquires data (telemetry window, command step, sweep, &hellip;)
and computes <code>metrics</code>.</li>
<li>Body decides <code>passed</code> via a hard-coded comparator on
metrics &times; params.</li>
<li>Runner stamps Status (PASS green / FAIL red), updates the
suite tree, appends a Results row.</li>
<li>Double-click the Results row to see the metrics table side-by-side
with the params table &mdash; "what we asked for" next to "what we
measured".</li>
</ol>
)"));
    lay->addWidget(browser);
    auto* close_btn = new QPushButton(tr("Close"), &dlg);
    close_btn->setDefault(true);
    connect(close_btn, &QPushButton::clicked, &dlg, &QDialog::accept);
    lay->addWidget(close_btn);
    dlg.exec();
}


void TestRunnerWindow::onSuiteContextMenu(const QPoint& pos)
{
    if (loaded_config_path_.isEmpty()){
        QMessageBox::information(this, tr("Suite tree"),
            tr("Load a test database first.")); return;
    }
    QTreeWidgetItem* item = suite_tree_->itemAt(pos);
    /* Decide what kind of row was clicked. UserRole carries the test
     * id for leaves; DB header rows have an empty UserRole. */
    const bool is_test = item && !item->data(0, Qt::UserRole).toString().isEmpty();
    const bool is_db   = item && !is_test;
    const bool is_empty= !item;

    QMenu menu(this);
    QAction* a_add_db    = nullptr;
    QAction* a_edit_db   = nullptr;
    QAction* a_del_db    = nullptr;
    QAction* a_add_test  = nullptr;
    QAction* a_edit_test = nullptr;
    QAction* a_del_test  = nullptr;

    if (is_db){
        a_add_test = menu.addAction(tr("Add testcase to this DB…"));
        menu.addSeparator();
        a_edit_db  = menu.addAction(tr("Edit database…"));
        a_del_db   = menu.addAction(tr("Delete database…"));
    } else if (is_test){
        a_edit_test = menu.addAction(tr("Edit testcase…"));
        a_del_test  = menu.addAction(tr("Delete testcase"));
        menu.addSeparator();
        a_add_test  = menu.addAction(tr("Add testcase…"));
    } else if (is_empty){
        a_add_db = menu.addAction(tr("Add database…"));
    }
    if (menu.actions().isEmpty()){ return; }

    QAction* chosen = menu.exec(suite_tree_->viewport()->mapToGlobal(pos));
    if (!chosen){ return; }

    /* Resolve which DB the click belongs to — needed for Add testcase. */
    QString clicked_db_id;
    if (item){
        QTreeWidgetItem* p = item;
        while (p->parent()){ p = p->parent(); }
        clicked_db_id = p->text(0).section(' ', 0, 0);
    }

    /* -------- Database-level actions ----------------------------- */
    if (chosen == a_add_db){
        DatabaseEditDialog dlg(DatabaseEditDialog::Mode::Add, this);
        if (dlg.exec() != QDialog::Accepted){ return; }
        QString err;
        if (!addDatabase(dlg.id(), dlg.name(), dlg.description(), &err)){
            QMessageBox::warning(this, tr("Add failed"), err);
        }
        return;
    }
    if (chosen == a_edit_db){
        const QString db_id = clicked_db_id;
        const TestDatabase* db = nullptr;
        for (const auto& d : databases_){
            if (d.id == db_id){ db = &d; break; }
        }
        if (!db){ return; }
        DatabaseEditDialog dlg(DatabaseEditDialog::Mode::Edit, this);
        dlg.loadFromValues(db->id, db->name, db->description);
        if (dlg.exec() != QDialog::Accepted){ return; }
        QString err;
        if (!updateDatabase(db_id, dlg.id(), dlg.name(),
                              dlg.description(), &err)){
            QMessageBox::warning(this, tr("Edit failed"), err);
        }
        return;
    }
    if (chosen == a_del_db){
        const QString db_id = clicked_db_id;
        int test_count = 0;
        for (const auto& d : databases_){
            if (d.id == db_id){ test_count = d.tests.size(); break; }
        }
        const auto ans = QMessageBox::warning(this, tr("Delete database"),
            tr("Delete '%1' and its %2 test(s)?\n\nThis cannot be "
               "undone from here.").arg(db_id).arg(test_count),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (ans != QMessageBox::Yes){ return; }
        QString err;
        if (!deleteDatabase(db_id, &err)){
            QMessageBox::warning(this, tr("Delete failed"), err);
        }
        return;
    }

    /* -------- Test-level actions --------------------------------- */
    if (chosen == a_add_test){
        /* Route through the runner's existing Add handler. We
         * preselect the right DB by stashing it on the suite-tree
         * current item so onAddTestClicked picks it up. */
        for (int i = 0; i < suite_tree_->topLevelItemCount(); ++i){
            auto* db_item = suite_tree_->topLevelItem(i);
            if (db_item->text(0).section(' ', 0, 0) == clicked_db_id){
                suite_tree_->setCurrentItem(db_item);
                break;
            }
        }
        onAddTestClicked();
        return;
    }
    if (chosen == a_edit_test){
        suite_tree_->setCurrentItem(item);
        const QString test_id = item->data(0, Qt::UserRole).toString();
        const TestSpec spec = findSpec(test_id);
        QStringList db_ids;
        for (const auto& d : databases_){ db_ids << d.id; }
        AddTestDialog dlg(AddTestDialog::Mode::Edit, db_ids, this);
        dlg.loadFromSpec(spec);
        if (dlg.exec() != QDialog::Accepted){ return; }
        QString err;
        if (!replaceTest(spec.id, dlg.toTestJson(), &err)){
            QMessageBox::warning(this, tr("Edit failed"),
                tr("Could not update testcase:\n%1").arg(err));
        }
        return;
    }
    if (chosen == a_del_test){
        suite_tree_->setCurrentItem(item);
        onDeleteTestClicked();
        return;
    }
}


void TestRunnerWindow::onWorkerConnected(int dof, QStringList joint_names)
{
    cached_dof_         = dof;
    cached_joint_names_ = std::move(joint_names);
    cached_error_codes_.clear();          /* fresh connect — pre-warmup */
    setStatusConnected(cached_dof_);
}


void TestRunnerWindow::setStatusConnected(int dof)
{
    /* Material green (#2e7d32) on a pale tint — same palette used
     * for PASS in the Results table so the operator's eye doesn't
     * have to context-switch between widgets. */
    status_label_->setText(tr("● Status: connected — dof %1").arg(dof));
    status_label_->setStyleSheet(
        "QLabel { color: #2e7d32; font-weight: bold; "
        "background: #e8f5e9; padding: 2px 8px; "
        "border-top: 1px solid #ccc; }");
}


void TestRunnerWindow::setStatusDisconnected()
{
    status_label_->setText(tr("● Status: not connected"));
    status_label_->setStyleSheet(
        "QLabel { color: #c62828; font-weight: bold; "
        "background: #ffebee; padding: 2px 8px; "
        "border-top: 1px solid #ccc; }");
}


void TestRunnerWindow::onWorkerErrorCodes(QVector<int> codes)
{
    cached_error_codes_ = std::move(codes);
}


void TestRunnerWindow::onWorkerStateUpdated(QVector<double> q,
                                              QVector<double> tau)
{
    /* DB1-07 round-trip ack detection moved out — it now runs on the
     * worker thread inside RoundTripProbe (see WorkerThreadProbe).
     * This slot only services passive window aggregations now. */
    if (!window_.active){ return; }
    ++window_.frames;
    window_.ts_ms.append(QDateTime::currentMSecsSinceEpoch());
    /* Malformed = wrong size OR any NaN / inf in q. */
    bool bad = (q.size() != cached_dof_);
    for (int i = 0; !bad && i < q.size(); ++i){
        if (!std::isfinite(q[i])){ bad = true; }
    }
    if (bad){ ++window_.malformed; }
    /* Per-joint sample counter — we treat one frame as one sample
     * per joint, since SDK adapters publish all joints together. */
    if (window_.samples_per_joint.size() != q.size()){
        window_.samples_per_joint.fill(0, q.size());
    }
    for (int i = 0; i < q.size(); ++i){
        ++window_.samples_per_joint[i];
    }
    /* DB2-03 — accumulate per-joint tau sum + sum-of-squares for
     * O(1) mean/stdev at window close. Skipped silently when tau is
     * empty or sized differently (e.g. force-mode-only adapters). */
    if (tau.size() == q.size() && !tau.isEmpty()){
        if (window_.tau_sum_per_joint.size() != tau.size()){
            window_.tau_sum_per_joint.fill(0.0, tau.size());
            window_.tau_sq_sum_per_joint.fill(0.0, tau.size());
        }
        for (int i = 0; i < tau.size(); ++i){
            window_.tau_sum_per_joint[i]    += tau[i];
            window_.tau_sq_sum_per_joint[i] += tau[i] * tau[i];
        }
        ++window_.tau_samples;
    }
}


/* ============================================================== *
 *  Suite tree
 * ============================================================== */
void TestRunnerWindow::rebuildTree()
{
    suite_tree_->clear();
    for (const auto& db : databases_){
        auto* db_item = new QTreeWidgetItem(suite_tree_,
            {db.id + " " + db.name, {}, {}});
        db_item->setExpanded(true);
        db_item->setFlags(db_item->flags() | Qt::ItemIsUserCheckable
                                           | Qt::ItemIsAutoTristate);
        db_item->setCheckState(0, Qt::Checked);
        QFont f = db_item->font(0); f.setBold(true); db_item->setFont(0, f);

        for (const auto& t : db.tests){
            auto* leaf = new QTreeWidgetItem(db_item,
                {t.id + "  " + t.name, tr("—"), {}});
            leaf->setFlags(leaf->flags() | Qt::ItemIsUserCheckable);
            leaf->setCheckState(0, Qt::Checked);
            leaf->setData(0, Qt::UserRole, t.id);     /* lookup key for click */
        }
    }
}


void TestRunnerWindow::onTreeItemClicked(QTreeWidgetItem* item, int /*col*/)
{
    if (!item || !item->parent()){ return; }         /* DB header click — no popup */
    const QString id = item->data(0, Qt::UserRole).toString();
    if (id.isEmpty()){ return; }
    desc_popup_->show(findSpec(id));
}


TestSpec TestRunnerWindow::findSpec(const QString& test_id) const
{
    for (const auto& db : databases_){
        for (const auto& t : db.tests){
            if (t.id == test_id){ return t; }
        }
    }
    return {};
}


/* ============================================================== *
 *  Run loop
 * ============================================================== */
void TestRunnerWindow::runAll()
{
    if (run_active_){ return; }                  /* re-entry guard */

    run_queue_.clear();
    for (int i = 0; i < suite_tree_->topLevelItemCount(); ++i){
        auto* db_item = suite_tree_->topLevelItem(i);
        for (int j = 0; j < db_item->childCount(); ++j){
            auto* leaf = db_item->child(j);
            if (leaf->checkState(0) == Qt::Checked){
                run_queue_.append(leaf->data(0, Qt::UserRole).toString());
            }
        }
    }
    if (run_queue_.isEmpty()){
        appendLog(LogLevel::Warn, tr("nothing checked to run"));
        return;
    }
    run_cursor_ = 0;
    run_active_ = true;
    paused_     = false;
    results_.clear();
    results_table_->setRowCount(0);
    /* Fresh run — wipe the live panel so the accumulated cards from
     * a previous run don't get mistaken for the new run. */
    live_panel_->clear();

    run_btn_   ->setEnabled(false);
    pause_btn_ ->setEnabled(true);
    stop_btn_  ->setEnabled(true);
    reset_btn_ ->setEnabled(false);

    appendLog(LogLevel::Info, 
        tr("run started — %1 tests queued").arg(run_queue_.size()));
    QTimer::singleShot(0, this, &TestRunnerWindow::stepNext);
}


void TestRunnerWindow::stepNext()
{
    /* Queue drained — UI back to idle. Live panel keeps the LAST
     * test's card visible (and only that — no run-complete footer is
     * appended so the operator's eye stays on a single readable
     * card). The completion message goes to the log instead. */
    if (run_cursor_ < 0 || run_cursor_ >= run_queue_.size()){
        run_active_ = false;
        run_btn_   ->setEnabled(true);
        pause_btn_ ->setEnabled(false);
        pause_btn_ ->setChecked(false);
        stop_btn_  ->setEnabled(false);
        reset_btn_ ->setEnabled(true);
        int passed = 0;
        for (const auto& r : results_){ if (r.passed){ ++passed; } }
        appendLog(LogLevel::Info,
                  tr("run complete — %1 / %2 passed")
                      .arg(passed).arg(results_.size()));
        return;
    }
    if (paused_){
        QTimer::singleShot(200, this, &TestRunnerWindow::stepNext);
        return;
    }
    const QString id = run_queue_[run_cursor_];
    const TestSpec spec = findSpec(id);

    /* Live panel: APPEND each test's card as HTML so the test name
     * shows in bold. Separator above each card except the very
     * first. The panel is wiped only by Reset / on a new Run All. */
    QString html;
    if (run_cursor_ > 0){
        html += QStringLiteral(
            "<hr style='border:0; border-top:2px solid #888;'/>");
    }
    /* Header line — bold name, plain ID. */
    html += QStringLiteral(
        "<p>▶ <b>%1</b> &mdash; %2</p>")
            .arg(spec.name.toHtmlEscaped(), spec.id.toHtmlEscaped());
    if (!spec.pre_conditions.isEmpty()){
        html += QStringLiteral("<p style='margin:4px 0;'>Pre-conditions:</p><ul>");
        for (const QString& pc : spec.pre_conditions){
            html += QStringLiteral("<li>%1</li>").arg(pc.toHtmlEscaped());
        }
        html += QStringLiteral("</ul>");
    }
    if (!spec.procedure.isEmpty()){
        html += QStringLiteral("<p style='margin:4px 0;'>Procedure:</p><ol>");
        for (const QString& step : spec.procedure){
            html += QStringLiteral("<li>%1</li>").arg(step.toHtmlEscaped());
        }
        html += QStringLiteral("</ol>");
    }
    html += QStringLiteral(
        "<p style='margin:4px 0;'>Expected result (pass criteria):"
        "<br/><tt>&nbsp;&nbsp;%1</tt></p>")
            .arg(spec.pass_criteria.toHtmlEscaped());
    /* Append at end-of-document. moveCursor + insertHtml is the
     * QTextEdit equivalent of appendPlainText. */
    live_panel_->moveCursor(QTextCursor::End);
    live_panel_->insertHtml(html);
    live_panel_->moveCursor(QTextCursor::End);
    live_panel_->verticalScrollBar()->setValue(
        live_panel_->verticalScrollBar()->maximum());

    appendLog(LogLevel::Info, tr("%1 START").arg(spec.id));

    current_test_start_ms_ = QDateTime::currentMSecsSinceEpoch();
    runTestBody(spec);
}


/* ============================================================== *
 *  Test bodies — driven entirely by spec.probe_params + captures
 *
 *  Every threshold + capture-window + implementation flag lives in
 *  tests.yaml under each test's "params": {...} block. The C++ here
 *  only chooses which TYPE of body to run (bus-enum, telemetry-window,
 *  error-scan, unimplemented) and reads the numeric limits straight
 *  from spec.params. Tuning a threshold = JSON edit, no rebuild.
 *
 *  Implementation flag (A1):
 *      params.implemented == false  →  PASS with "(not implemented)"
 *      otherwise                    →  real body runs
 *  The unimplemented-fallback at the bottom assumes implemented==true
 *  when the key is missing — protects against accidentally silencing
 *  a typo'd id by tagging it "pending".
 * ============================================================== */
/* ============================================================== *
 *  Schema-v2 dispatcher (Fork P3-a)
 *
 *  Reads spec.captures, looks each entry up in acquisition_table_,
 *  groups them by kind (snapshot / config / window_agg / probe),
 *  runs each group, packages the results into one metrics object
 *  under the catalog names, then calls finishCurrent.
 *
 *  No "spec.id == ..." branches — every test of an existing kind is
 *  pure JSON. Adding a new acquisition kind = one new resolver
 *  + one new table entry.
 * ============================================================== */

/** @brief Compute one window-aggregator stat from raw window fields.
 *  Free function, no friendship needed — takes only the fields it
 *  reads. Returns Undefined on unknown stat name. */
namespace {

QJsonValue computeWindowStat(const QString& stat,
                              int frames,
                              int malformed,
                              const QVector<qint64>& ts_ms,
                              const QVector<int>& samples_per_joint,
                              const QVector<double>& tau_sum_per_joint,
                              const QVector<double>& tau_sq_sum_per_joint,
                              int tau_samples,
                              double dur_s)
{
    if (stat == "frame_count")      { return frames; }
    if (stat == "malformed_count")  { return malformed; }
    if (stat == "actual_duration_s"){ return dur_s; }
    if (stat == "agg_rate_hz")      {
        return dur_s > 0.0 ? (frames / dur_s) : 0.0;
    }
    if (stat == "samples_per_joint"){
        QJsonArray a;
        for (int s : samples_per_joint){ a.append(s); }
        return a;
    }
    /* ---- DB2-03 torque stats. Per-joint mean = sum/N; per-joint
     * stdev = sqrt(E[x²] - mean²). Returns empty array when no τ
     * samples were captured (force-mode-only adapters). */
    if (stat == "tau_mean_per_joint"){
        QJsonArray a;
        if (tau_samples > 0){
            for (double s : tau_sum_per_joint){
                a.append(s / tau_samples);
            }
        }
        return a;
    }
    if (stat == "tau_stdev_per_joint"){
        QJsonArray a;
        if (tau_samples > 1){
            for (int i = 0; i < tau_sum_per_joint.size(); ++i){
                const double mean = tau_sum_per_joint[i] / tau_samples;
                const double e_sq = tau_sq_sum_per_joint[i] / tau_samples;
                const double var  = std::max(0.0, e_sq - mean * mean);
                a.append(std::sqrt(var));
            }
        }
        return a;
    }
    /* Max-abs across all joints — handy as a single scalar pass rule. */
    if (stat == "tau_max_abs"){
        if (tau_samples == 0){ return 0.0; }
        double worst = 0.0;
        for (double s : tau_sum_per_joint){
            const double m = std::abs(s / tau_samples);
            worst = std::max(worst, m);
        }
        return worst;
    }
    /* All remaining stats are derived from inter-arrival intervals. */
    QVector<double> intervals_us;
    for (int i = 1; i < ts_ms.size(); ++i){
        intervals_us.append((ts_ms[i] - ts_ms[i-1]) * 1000.0);
    }
    if (stat == "intervals_us_array"){
        QJsonArray a;
        for (double v : intervals_us){ a.append(v); }
        return a;
    }
    if (intervals_us.isEmpty()){ return 0.0; }
    double mean = 0.0;
    for (double v : intervals_us){ mean += v; }
    mean /= intervals_us.size();
    if (stat == "interval_mean_us"){ return mean; }
    if (stat == "interval_max_us"){
        double m = intervals_us.first();
        for (double v : intervals_us){ m = std::max(m, v); }
        return m;
    }
    if (stat == "interval_min_us"){
        double m = intervals_us.first();
        for (double v : intervals_us){ m = std::min(m, v); }
        return m;
    }
    if (stat == "interval_stdev_us"){
        if (intervals_us.size() < 2){ return 0.0; }
        double var = 0.0;
        for (double v : intervals_us){ var += (v - mean) * (v - mean); }
        return std::sqrt(var / (intervals_us.size() - 1));
    }
    if (stat == "interval_p99_us"){
        auto sorted = intervals_us;
        std::sort(sorted.begin(), sorted.end());
        return sorted[std::min<int>(sorted.size() - 1,
                                      int(sorted.size() * 0.99))];
    }
    return QJsonValue::Undefined;
}

}  // namespace


QJsonValue TestRunnerWindow::resolveSnapshot(const QString& source) const
{
    if (source == "worker.dof")         { return cached_dof_; }
    if (source == "worker.joint_names"){
        QJsonArray a;
        for (const auto& n : cached_joint_names_){ a.append(n); }
        return a;
    }
    if (source == "worker.error_codes"){
        QJsonArray a;
        for (int c : cached_error_codes_){ a.append(c); }
        return a;
    }
    /* URDF-derived limits — backs the urdf.* catalog kind. Used by
     * motion-test probe_params to drive commanded targets from the
     * URDF instead of hand-typing arrays in tests.yaml. */
    if (source == "worker.joint_lo"){
        QJsonArray a;
        for (double v : cached_joint_lo_){ a.append(v); }
        return a;
    }
    if (source == "worker.joint_hi"){
        QJsonArray a;
        for (double v : cached_joint_hi_){ a.append(v); }
        return a;
    }
    /* Convenience: midpoint of (lo, hi) per joint — a reasonable
     * "home" pose when no explicit home is encoded. */
    if (source == "worker.joint_home"){
        QJsonArray a;
        const int n = std::min(cached_joint_lo_.size(),
                                  cached_joint_hi_.size());
        for (int i = 0; i < n; ++i){
            a.append(0.5 * (cached_joint_lo_[i] + cached_joint_hi_[i]));
        }
        return a;
    }
    return QJsonValue::Undefined;
}


QJsonValue TestRunnerWindow::resolveConfig(const QString& source) const
{
    if (source == "expected_dof"){ return expected_dof_; }
    return QJsonValue::Undefined;
}


void TestRunnerWindow::runTestBody(const TestSpec& spec)
{
    /* Schema v2 dropped spec.not_implemented / spec.pending_reason —
     * a stub test is now expressed by having an empty captures list
     * and an empty probe_params, which falls through to the
     * "nothing-to-do" path below and finishes immediately. */

    /* Schema-v2 captures dispatcher — the only acquisition path. */
    if (!spec.captures.isEmpty()){
        /* 1. Categorise captures by acquisition kind. */
        QStringList snap_caps, cfg_caps, win_caps, probe_caps;
        for (const QString& cap : spec.captures){
            const auto it = acquisition_table_.constFind(cap);
            if (it == acquisition_table_.constEnd()){
                appendLog(LogLevel::Warn,
                    tr("unknown capture '%1' — not in acquisition_table")
                        .arg(cap));
                continue;
            }
            const QString& kind = it.value().kind;
            if      (kind == "snapshot")  { snap_caps.append(cap); }
            else if (kind == "config")    { cfg_caps.append(cap); }
            else if (kind == "window_agg"){ win_caps.append(cap); }
            else if (kind == "probe")     { probe_caps.append(cap); }
        }

        /* 2. Resolve snapshot + config captures synchronously. */
        QJsonObject metrics;
        for (const QString& cap : snap_caps){
            metrics[cap] = resolveSnapshot(
                acquisition_table_.value(cap).source);
        }
        for (const QString& cap : cfg_caps){
            metrics[cap] = resolveConfig(
                acquisition_table_.value(cap).source);
        }

        /* 3. Pick the async path. Mixing window + probe in one test
         *    isn't supported — flag it loudly so the spec author
         *    catches it. */
        if (!win_caps.isEmpty() && !probe_caps.isEmpty()){
            QTimer::singleShot(50, this, [this, metrics]{
                finishCurrent(false,
                    tr("FAIL: spec mixes window + probe captures "
                       "(not supported in v2.0)"),
                    metrics);
            });
            return;
        }

        if (!win_caps.isEmpty()){
            /* Telemetry-window acquisition — open a fixed-length
             * window and compute every requested aggregate. Schema v2
             * dropped spec.window_s; a future field
             * (probe_params.window_s) could carry per-test overrides
             * but for now every window_agg test uses the interactive
             * cap of 5 s. */
            constexpr double kInteractiveCapS = 5.0;
            const double pp_window =
                spec.probe_params.value("window_s").toDouble(0.0);
            const double window_s = std::min(
                pp_window > 0.0 ? pp_window : 5.0,
                kInteractiveCapS);
            window_ = {};
            window_.active   = true;
            window_.start_ms = QDateTime::currentMSecsSinceEpoch();
            QTimer::singleShot((int)(window_s * 1000.0), this,
                [this, win_caps, metrics_in = metrics]() mutable {
                window_.active = false;
                const double dur_s = (QDateTime::currentMSecsSinceEpoch()
                                       - window_.start_ms) / 1000.0;
                QJsonObject metrics = metrics_in;
                for (const QString& cap : win_caps){
                    const QString stat = acquisition_table_.value(cap).stat;
                    metrics[cap] = computeWindowStat(
                        stat, window_.frames, window_.malformed,
                        window_.ts_ms, window_.samples_per_joint,
                        window_.tau_sum_per_joint,
                        window_.tau_sq_sum_per_joint,
                        window_.tau_samples,
                        dur_s);
                }
                finishCurrent(true,
                    tr("window %1 s, %2 frames").arg(dur_s, 0, 'f', 2)
                                                  .arg(window_.frames),
                    metrics);
            });
            return;
        }

        if (!probe_caps.isEmpty()){
            /* Probe acquisition — invoke the named probe with
             * spec.probe_params; on batchDone, map probe outputs to
             * catalog names and finish. */
            const QString probe_name = spec.probe_params
                                          .value("probe").toString();
            /* One-shot listener — typed signal so the function-pointer
             * connect supports the lambda. Branches on probe class
             * because batchDone is declared on each derived probe,
             * not on the shared WorkerThreadProbe base. */
            auto onBatch = [this, probe_caps, metrics_in = metrics](
                              QJsonObject probe_metrics,
                              QString summary) mutable {
                QJsonObject metrics = metrics_in;
                for (const QString& cap : probe_caps){
                    const QString key = acquisition_table_.value(cap).output;
                    if (key == QLatin1String("*")){
                        /* Wildcard: lift every probe-emitted metric
                         * into the metrics object under its original
                         * (probe-side) name. lookupName's prefix-strip
                         * fallback lets pass_criteria reference these
                         * via either bare names ("trials_acked") or
                         * dotted shortcuts ("probe.round_trip_ms"). */
                        for (auto it = probe_metrics.constBegin();
                                  it != probe_metrics.constEnd(); ++it){
                            metrics[it.key()] = it.value();
                        }
                    } else if (probe_metrics.contains(key)){
                        metrics[cap] = probe_metrics.value(key);
                    }
                }
                finishCurrent(true, summary, metrics);
            };

            /* Fork U2 — substitute "$name" string values in
             * probe_params with the resolved JSON from the
             * acquisition table. Lets motion-test probes pull
             * commanded targets straight from URDF (or any other
             * snapshot) instead of hand-typed arrays in tests.yaml. */
            QJsonObject resolved_pp = spec.probe_params;
            for (auto it = resolved_pp.begin(); it != resolved_pp.end(); ++it){
                if (!it.value().isString()){ continue; }
                const QString s = it.value().toString();
                if (!s.startsWith('$')){ continue; }
                const QString cap = s.mid(1);
                const auto entry_it = acquisition_table_.constFind(cap);
                if (entry_it == acquisition_table_.constEnd()){
                    appendLog(LogLevel::Warn,
                        tr("probe_params $%1 — no such entry in "
                           "acquisition_table; leaving as string")
                            .arg(cap));
                    continue;
                }
                const auto& e = entry_it.value();
                if (e.kind == "snapshot"){
                    it.value() = resolveSnapshot(e.source);
                } else if (e.kind == "config"){
                    it.value() = resolveConfig(e.source);
                }
                /* probe / window_agg sources are never substitutable
                 * here — they need a probe run / a window to produce
                 * a value, which we don't have at probe-start time. */
            }

            /* Generic dispatcher: each probe class has the same
             * batchDone signature, so the one-shot lambda is shared.
             * Add a new probe by listing its name + pointer + class
             * here — the rest of the runner stays untouched. */
            auto run_probe = [this, &onBatch, &resolved_pp](
                                auto* probe, auto signal){
                auto* conn = new QMetaObject::Connection;
                *conn = connect(probe, signal,
                    this, [conn, onBatch](bool /*ok*/,
                                            QJsonObject m,
                                            QString s) mutable {
                        disconnect(*conn);
                        delete conn;
                        onBatch(m, s);
                    });
                QMetaObject::invokeMethod(probe, "startBatch",
                                          Qt::QueuedConnection,
                                          Q_ARG(QJsonObject, resolved_pp));
            };

#if VR_MC_TESTRUNNER_PROBES_ENABLED
            if      (probe_name == "round_trip")
                run_probe(round_trip_probe_, &RoundTripProbe::batchDone);
            else if (probe_name == "bandwidth_sweep")
                run_probe(bandwidth_probe_, &BandwidthSweepProbe::batchDone);
            else if (probe_name == "operation_modes")
                run_probe(opmode_probe_, &OperationModeProbe::batchDone);
            else if (probe_name == "soft_limit")
                run_probe(softlimit_probe_, &SoftLimitProbe::batchDone);
            else if (probe_name == "estop")
                run_probe(estop_probe_, &EstopProbe::batchDone);
            else
#endif
            {
                QTimer::singleShot(50, this, [this, probe_name, metrics]{
                    finishCurrent(false,
                        tr("FAIL: probe '%1' not yet ported to MC "
                           "(see TestRunner/probes/ for the Hand template)")
                            .arg(probe_name),
                        metrics);
                });
            }
            return;
        }

        /* Snapshot-only test — nothing async to wait for. */
        QTimer::singleShot(50, this, [this, metrics]{
            finishCurrent(true,
                tr("%1 captures resolved").arg(metrics.size()),
                metrics);
        });
        return;
    }

    /* ---- Authoring fallback ---------------------------------------
     * Reached only when a spec carries no captures AND no
     * not_implemented flag. Surface the slip as PASS with a clear
     * marker so the operator notices and fixes the JSON rather than
     * see a generic FAIL that masks the real cause. */
    {
        QJsonObject metrics;
        metrics["status"] = QStringLiteral("unhandled");
        const QString msg = tr("spec '%1' has no captures and no "
                                "not_implemented flag — did not run")
                                .arg(spec.id);
        QTimer::singleShot(150, this, [this, msg, metrics]{
            finishCurrent(true, msg, metrics);
        });
        return;
    }

}


/** @brief Pretty-print a metrics object as a single-line "k=v, k=v"
 *  string suitable for the Actual Result column. Arrays render
 *  compactly with `[...]` so a 9-DOF joint list doesn't blow up the
 *  cell. */
static QString formatMetrics(const QJsonObject& m)
{
    if (m.isEmpty()){ return {}; }
    QStringList parts;
    for (auto it = m.constBegin(); it != m.constEnd(); ++it){
        const auto& v = it.value();
        QString rendered;
        if (v.isArray()){
            QStringList items;
            for (const auto& x : v.toArray()){
                items << (x.isString() ? x.toString()
                                       : QString::number(x.toDouble()));
            }
            rendered = QStringLiteral("[%1]").arg(items.join(", "));
        } else if (v.isString()){
            rendered = v.toString();
        } else if (v.isBool()){
            rendered = v.toBool() ? "true" : "false";
        } else if (v.isDouble()){
            const double d = v.toDouble();
            const long long i = (long long)d;
            rendered = ((double)i == d) ? QString::number(i)
                                         : QString::number(d, 'g', 6);
        } else {
            rendered = "null";
        }
        parts << QStringLiteral("%1=%2").arg(it.key(), rendered);
    }
    return parts.join(", ");
}


void TestRunnerWindow::finishCurrent(bool passed,
                                      const QString& summary,
                                      const QJsonObject& metrics)
{
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    const double dur_s  = (now_ms - current_test_start_ms_) / 1000.0;
    const QString id    = run_queue_[run_cursor_];
    const TestSpec spec = findSpec(id);

    /* Fork-E1: if the spec carries structured pass_rules, the
     * evaluator's outcome is AUTHORITATIVE — the @p passed argument
     * from the test body becomes a hint we may override. The runner
     * appends the per-failed-rule reasons to the body's summary so
     * the Actual Result cell quotes the specific predicate that
     * tripped. When pass_rules is empty (short-circuit cases such
     * as implemented:false) the body's hint stays in charge. */
    bool        final_passed = passed;
    QString     final_summary = summary;
    if (!spec.pass_rules.isEmpty()){
        QStringList failed_reasons;
        final_passed = evaluateRules(spec, metrics, &failed_reasons);
        /* Prefix the verdict so the operator can see at a glance
         * whether rules ruled the call, and what they said. */
        const QString verdict = final_passed ? tr("PASS") : tr("FAIL");
        final_summary = tr("%1 — %2").arg(verdict, summary);
        if (!final_passed && !failed_reasons.isEmpty()){
            final_summary += tr("\n  Failed rules: %1.")
                                  .arg(failed_reasons.join("; "));
        }
    }

    /* Stash the full row before invoking onTestFinished (which reads
     * results_.last() for its tree/table updates). */
    ResultRow r;
    r.test_id         = id;
    r.test_name       = spec.name;
    r.db_id           = spec.db_id;
    r.description     = spec.description;
    r.pre_conditions  = spec.pre_conditions;
    r.procedure       = spec.procedure;
    r.expected_result = spec.pass_criteria;
    r.actual_result   = metrics.isEmpty()
                          ? final_summary
                          : QStringLiteral("%1 [%2]")
                              .arg(final_summary, formatMetrics(metrics));
    r.passed          = final_passed;
    r.duration_s      = dur_s;
    r.summary         = final_summary;
    r.metrics         = metrics;
    results_.append(r);

    onTestFinished(id, final_passed,
                   final_summary + tr(" (%1 s)").arg(dur_s, 0, 'f', 2),
                   metrics);
}


void TestRunnerWindow::onTestFinished(QString test_id,
                                       bool    passed,
                                       QString summary,
                                       QJsonObject /*metrics*/)
{
    /* FAIL lines log at Error level so the whole row paints red —
     * easy to spot when skimming a long log. PASS stays at Info. */
    appendLog(passed ? LogLevel::Info : LogLevel::Error,
              tr("%1 %2  %3")
                  .arg(test_id)
                  .arg(passed ? "PASS" : "FAIL")
                  .arg(summary));

    /* Update the leaf's status + time columns in the tree. */
    const double dur_s = results_.isEmpty() ? 0.0 : results_.last().duration_s;
    for (int i = 0; i < suite_tree_->topLevelItemCount(); ++i){
        auto* db_item = suite_tree_->topLevelItem(i);
        for (int j = 0; j < db_item->childCount(); ++j){
            auto* leaf = db_item->child(j);
            if (leaf->data(0, Qt::UserRole).toString() == test_id){
                leaf->setText(1, passed ? tr("PASS") : tr("FAIL"));
                leaf->setText(2, QString::number(dur_s, 'f', 2));
                break;
            }
        }
    }

    /* Append a row to the results table — ID, name, status, time,
     * Expected Result (from spec.pass_criteria), Actual Result (from
     * the latest ResultRow which finishCurrent already stashed).
     * Column alignment: ID + Status + Time are centred so the eye
     * scans down them quickly; Name + Expected + Actual stay
     * left-aligned with word-wrap on for the prose-heavy fields. */
    const int row = results_table_->rowCount();
    const TestSpec spec = findSpec(test_id);
    const ResultRow& rr = results_.last();
    /* Every cell aligns Top — when a multi-line cell (Expected /
     * Actual) wraps to 3+ lines, the short ID / Status / Time cells
     * would otherwise float in the middle of the tall row and the
     * eye sees a misaligned, "stepped" row. Top-align across the
     * board pins every cell to the same baseline. */
    auto makeCell = [](const QString& text, int hAlign){
        auto* it = new QTableWidgetItem(text);
        it->setTextAlignment(hAlign | Qt::AlignTop);
        return it;
    };
    results_table_->insertRow(row);
    results_table_->setItem(row, kColId,
        makeCell(test_id, Qt::AlignHCenter));
    results_table_->setItem(row, kColName,
        makeCell(spec.name, Qt::AlignLeft));
    auto* status_item = makeCell(passed ? tr("PASS") : tr("FAIL"),
                                  Qt::AlignHCenter);
    status_item->setForeground(passed ? QColor("#4caf50") : QColor("#f44336"));
    QFont sf = status_item->font(); sf.setBold(true); status_item->setFont(sf);
    results_table_->setItem(row, kColStatus, status_item);
    results_table_->setItem(row, kColTime,
        makeCell(QString::number(dur_s, 'f', 2), Qt::AlignHCenter));
    results_table_->setItem(row, kColExpected,
        makeCell(rr.expected_result, Qt::AlignLeft));
    results_table_->setItem(row, kColActual,
        makeCell(rr.actual_result, Qt::AlignLeft));
    results_table_->resizeRowToContents(row);
    (void)summary;                      /* now folded into rr.actual_result */

    /* Continue-on-failure logic: blocking failures stop the run when
     * the checkbox is off. */
    if (!passed && !continue_on_fail_->isChecked() && spec.blocks_if_fails){
        appendLog(LogLevel::Warn, 
            tr("%1 marked blocking — aborting run").arg(test_id));
        run_cursor_ = run_queue_.size();
    } else {
        ++run_cursor_;
    }
    QTimer::singleShot(0, this, &TestRunnerWindow::stepNext);
}


void TestRunnerWindow::stop()
{
    run_cursor_ = run_queue_.size();           /* short-circuit the loop */
    paused_     = false;
    appendLog(LogLevel::Info, tr("stop requested — finishing current test"));
}


void TestRunnerWindow::pause(bool on)
{
    paused_ = on;
    appendLog(LogLevel::Info, on ? tr("paused — will resume after this test")
                                    : tr("[info] resumed"));
}


void TestRunnerWindow::reset()
{
    run_active_ = false;
    run_cursor_ = -1;
    run_queue_.clear();
    paused_     = false;
    results_.clear();
    results_table_->setRowCount(0);
    live_panel_->clear();
    log_panel_ ->clear();
    rebuildTree();
    run_btn_  ->setEnabled(true);
    pause_btn_->setEnabled(false);
    pause_btn_->setChecked(false);
    stop_btn_ ->setEnabled(false);
}


/* ============================================================== *
 *  Export — CSV + PDF
 * ============================================================== */
void TestRunnerWindow::onSaveCsv()
{
    if (results_.isEmpty()){
        appendLog(LogLevel::Warn, tr("no results to save"));
        return;
    }
    const QString suggested = default_export_dir_ + "/test_results_"
        + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save results CSV"), suggested,
        tr("CSV (*.csv);;All files (*)"));
    if (path.isEmpty()){ return; }
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)){
        appendLog(LogLevel::Error, tr("cannot write %1").arg(path));
        return;
    }
    QTextStream s(&f);
    s.setEncoding(QStringConverter::Utf8);
    /* UTF-8 BOM + explicit `sep=,` magic line — without these, Excel
     * in non-US locales (Vietnamese, French, German, …) defaults to
     * `;` as the field separator, sees our `,`-separated row as a
     * single string, and shoves the whole thing into column A. With
     * both hints, Excel + LibreOffice + Numbers all parse correctly
     * AND honour UTF-8 (so accented characters in test descriptions
     * survive). Bytes: EF BB BF for the BOM, then `sep=,\n`. */
    s << QChar(0xFEFF) << "sep=,\n";

    /* Fixed-width CSV — eleven columns regardless of how many
     * pre-conditions or procedure steps any test has. pre_conditions
     * and procedure each occupy ONE cell whose value is a multi-line
     * string listing every item in order; spreadsheets honour
     * embedded \n inside quoted cells and render the cell as a
     * wrapped block. Keeps the column count predictable for
     * downstream tooling (pandas, BigQuery, etc.) that can't deal
     * with a per-test variable width. */
    auto csvCell = [](const QString& in){
        QString out = in;
        out.replace('"', "\"\"");
        return QStringLiteral("\"%1\"").arg(out);
    };
    auto numbered = [](const QStringList& xs){
        QStringList out;
        int n = 1;
        for (const auto& x : xs){
            out << QStringLiteral("%1. %2").arg(n++).arg(x);
        }
        return out.join("\n");
    };
    /* Header. */
    s << "id,name,database,status,duration_s,description,"
         "pre_conditions,procedure,expected_result,actual_result,metrics_json\n";
    /* Rows. */
    for (const auto& r : results_){
        const QString metrics_json = QString::fromUtf8(
            QJsonDocument(r.metrics).toJson(QJsonDocument::Compact));
        s << csvCell(r.test_id)                       << ","
          << csvCell(r.test_name)                     << ","
          << csvCell(r.db_id)                         << ","
          << (r.passed ? "PASS" : "FAIL")             << ","
          << QString::number(r.duration_s, 'f', 3)    << ","
          << csvCell(r.description)                   << ","
          << csvCell(numbered(r.pre_conditions))      << ","
          << csvCell(numbered(r.procedure))           << ","
          << csvCell(r.expected_result)               << ","
          << csvCell(r.actual_result)                 << ","
          << csvCell(metrics_json)                    << "\n";
    }
    f.close();
    default_export_dir_ = QFileInfo(path).absolutePath();
    appendLog(LogLevel::Info, tr("CSV saved to %1").arg(path));
}


void TestRunnerWindow::appendLog(LogLevel level, const QString& message)
{
    QString colour;
    QString tag;
    switch (level){
        case LogLevel::Warn:  colour = "#f9a825"; tag = "warn";  break;
        case LogLevel::Error: colour = "#e53935"; tag = "error"; break;
        case LogLevel::Info:
        default:              colour = "#bdbdbd"; tag = "info";  break;
    }
    /* Move cursor to end so each line lands at the bottom regardless
     * of the operator scrolling around. Escape so a stray `<` in a
     * test name doesn't get interpreted as HTML. */
    const QString line = QStringLiteral(
        "<span style='color:%1;'>[%2]</span> %3").arg(
            colour, tag, message.toHtmlEscaped());
    log_panel_->moveCursor(QTextCursor::End);
    log_panel_->insertHtml(line + "<br/>");
    log_panel_->moveCursor(QTextCursor::End);
}


QString TestRunnerWindow::resultsAsHtml() const
{
    /* Build the per-DB tables in a single PASS-through. Mirror the
     * exploded-CSV layout: each test is ONE row; every field
     * (description, each pre-condition, each procedure step, expected,
     * actual, duration, status) is its OWN column. Multi-value spec
     * fields (pre_conditions, procedure) get one column per slot,
     * widened to whatever the worst case in this DB needs so the row
     * shape is uniform across the table.
     *
     * Layout choice: landscape A4 + small font, since 14 tests + 8
     * cols/test averages 50–80 narrow columns in practice. The PDF is
     * meant to be archived, not skimmed in print; reviewers Cmd-F to
     * find a specific value. */
    QString html;
    html += R"(<html><head><meta charset="utf-8"/><style>
        body { font-family: sans-serif; color: #222; font-size: 9pt; }
        h1 { color: #4a90e2; }
        h2 { color: #4a90e2; border-bottom: 1px solid #aaa; }
        table { border-collapse: collapse; margin: 10px 0; font-size: 8pt;
                table-layout: fixed; word-wrap: break-word; }
        th, td { border: 1px solid #888; padding: 4px 6px; vertical-align: top;
                 text-align: left; }
        th { background: #f0f0f0; font-weight: bold; }
        .pass { color: #2e7d32; font-weight: bold; background: #e8f5e9; }
        .fail { color: #c62828; font-weight: bold; background: #ffebee; }
        .meta { color: #555; font-size: 0.95em; }
        .mono { font-family: monospace; font-size: 8pt; }
    </style></head><body>)";

    int passed = 0;
    for (const auto& r : results_){ if (r.passed){ ++passed; } }
    html += QStringLiteral("<h1>Test Runner Report</h1>");
    html += QStringLiteral("<p class='meta'>Generated %1<br/>"
                           "Total tests: %2 &mdash; "
                           "<span class='pass'>passed %3</span> &mdash; "
                           "<span class='fail'>failed %4</span></p>")
              .arg(QDateTime::currentDateTime().toString(Qt::ISODate))
              .arg(results_.size()).arg(passed).arg(results_.size() - passed);

    /* Group by database. */
    QStringList dbs;
    for (const auto& r : results_){
        if (!dbs.contains(r.db_id)){ dbs.append(r.db_id); }
    }

    /* Helper — render a QStringList as a numbered HTML list contained
     * inside a single table cell. Each item gets its own line and a
     * 1.-style prefix so the cell reads as a list, not a paragraph. */
    auto numberedHtml = [](const QStringList& xs){
        if (xs.isEmpty()){ return QString{}; }
        QStringList rows;
        int n = 1;
        for (const auto& x : xs){
            rows << QStringLiteral("%1. %2").arg(n++).arg(x.toHtmlEscaped());
        }
        return rows.join("<br/>");
    };

    for (const QString& db : dbs){
        html += QStringLiteral("<h2>%1</h2>").arg(db);
        /* Fixed nine-column table per DB: ID, Name, Status, Time,
         * Description, Pre-conditions, Procedure, Expected, Actual.
         * Multi-item fields are listed line-by-line inside their cell
         * so the column count stays predictable and operators always
         * see the same shape regardless of which DB they're reading. */
        html += QStringLiteral("<table><colgroup>"
            "<col style='width:55px;'/>"    /* ID           */
            "<col style='width:110px;'/>"   /* Name         */
            "<col style='width:48px;'/>"    /* Status       */
            "<col style='width:48px;'/>"    /* Time         */
            "<col style='width:200px;'/>"   /* Description  */
            "<col style='width:200px;'/>"   /* Pre-conditions */
            "<col style='width:240px;'/>"   /* Procedure    */
            "<col style='width:200px;'/>"   /* Expected     */
            "<col style='width:200px;'/>"   /* Actual       */
            "</colgroup>");
        html += QStringLiteral("<tr>"
            "<th>ID</th><th>Name</th><th>Status</th><th>Time (s)</th>"
            "<th>Description</th>"
            "<th>Pre-conditions</th>"
            "<th>Procedure</th>"
            "<th>Expected Result</th><th>Actual Result</th></tr>");

        for (const auto& r : results_){
            if (r.db_id != db){ continue; }
            html += QStringLiteral("<tr>"
                "<td>%1</td><td>%2</td>"
                "<td class='%3'>%4</td><td>%5</td>"
                "<td>%6</td>"
                "<td>%7</td>"
                "<td>%8</td>"
                "<td class='mono'>%9</td>"
                "<td class='mono'>%10</td></tr>")
                .arg(r.test_id.toHtmlEscaped(),
                     r.test_name.toHtmlEscaped(),
                     r.passed ? "pass" : "fail",
                     r.passed ? "PASS" : "FAIL",
                     QString::number(r.duration_s, 'f', 2),
                     r.description.toHtmlEscaped(),
                     numberedHtml(r.pre_conditions),
                     numberedHtml(r.procedure),
                     r.expected_result.toHtmlEscaped())
                .arg(r.actual_result.toHtmlEscaped());
        }
        html += QStringLiteral("</table>");
    }
    html += QStringLiteral("</body></html>");
    return html;
}


void TestRunnerWindow::onGeneratePdf()
{
    if (results_.isEmpty()){
        appendLog(LogLevel::Warn, tr("no results to render"));
        return;
    }
    const QString suggested = default_export_dir_ + "/test_results_"
        + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".pdf";
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Save results PDF"), suggested,
        tr("PDF (*.pdf);;All files (*)"));
    if (path.isEmpty()){ return; }

    QTextDocument doc;
    doc.setHtml(resultsAsHtml());

    QPrinter printer(QPrinter::HighResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(path);
    printer.setPageSize(QPageSize(QPageSize::A4));
    /* Landscape — the comprehensive per-test table has many columns
     * (pre_cond_1..N, procedure_step_1..M, plus the fixed fields)
     * so portrait would force everything into unreadably-narrow
     * cells. */
    printer.setPageOrientation(QPageLayout::Landscape);
    printer.setPageMargins(QMarginsF(10, 10, 10, 10), QPageLayout::Millimeter);

    doc.print(&printer);
    default_export_dir_ = QFileInfo(path).absolutePath();
    appendLog(LogLevel::Info, tr("PDF saved to %1").arg(path));
}
