#include "TuningPane.hpp"
#include "BwFromKp.hpp"

#include <QChart>
#include <QSignalBlocker>
#include <QChartView>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineSeries>
#include <QLogValueAxis>
#include <QPushButton>
#include <QScatterSeries>
#include <QSpinBox>
#include <QTabWidget>
#include <QValueAxis>
#include <QVBoxLayout>

#include <QPainter>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vrmc {

/* ====================================================================
 *  Shared step-response metrics
 *
 *  One routine feeds both the Step tab (signal-generator capture) and the
 *  Auto-Tune tab (board STEP_RP_EN capture). It measures the steady levels
 *  from the data itself rather than assuming the commanded offset, so the
 *  numbers stay meaningful for position/velocity steps that don't start at
 *  the commanded baseline.
 * ==================================================================== */

namespace {

/* CiA-402 statusword: Operation-Enabled bit (matches MasterWorker's check). */
constexpr uint16_t kSwOperationEnabled = 0x0004;

struct StepMetrics {
    bool   valid        = false;
    double initial      = 0.0;   /**< measured pre-step steady level   */
    double ssValue      = 0.0;   /**< measured post-step steady level  */
    double commanded    = 0.0;   /**< requested final                  */
    double ssError      = 0.0;   /**< commanded - ssValue              */
    double peak         = 0.0;
    double overshootPct = 0.0;
    double riseTime     = -1.0;  /**< 10->90 % of measured range, s    */
    double settlingTime = -1.0;  /**< last exit from 2 % band, s       */
    double bandwidthHz  = -1.0;  /**< 0.35 / riseTime                  */
};

/* t and y are parallel; t is seconds, monotonically increasing, with the
 * step assumed at t≈t[0]. commanded is the requested final value. */
StepMetrics computeStepMetrics(const QVector<double>& t,
                               const QVector<double>& y,
                               double commanded)
{
    StepMetrics m;
    const int N = y.size();
    if (N < 4 || t.size() != N){ return m; }

    /* Steady levels: average a small head/tail window so a single noisy
     * sample doesn't define the baseline or the final value. */
    const int head = std::max(1, N / 20);
    const int tail = std::max(1, N / 10);
    double s0 = 0.0, s1 = 0.0;
    for (int i = 0; i < head; ++i){ s0 += y[i]; }
    for (int i = N - tail; i < N; ++i){ s1 += y[i]; }
    m.initial   = s0 / head;
    m.ssValue   = s1 / tail;
    m.commanded = commanded;
    m.ssError   = commanded - m.ssValue;

    const double range = m.ssValue - m.initial;
    const bool rising  = range > 0.0;

    /* Peak in the step direction. */
    m.peak = m.initial;
    for (int i = 0; i < N; ++i){
        m.peak = rising ? std::max(m.peak, y[i]) : std::min(m.peak, y[i]);
    }

    m.valid = true;
    if (std::abs(range) < 1e-9){
        /* Flat response: steady-state error is still meaningful, the rest
         * is not. Leave rise/overshoot/settle/bw at their "N/A" defaults. */
        return m;
    }

    /* Overshoot past the measured steady value, as % of the step size. */
    const double ov = (m.peak - m.ssValue) / range * 100.0;
    m.overshootPct  = std::max(0.0, ov);

    /* Rise time: first 10 % crossing -> first 90 % crossing. */
    const double lo10 = m.initial + 0.10 * range;
    const double hi90 = m.initial + 0.90 * range;
    double t10 = -1.0, t90 = -1.0;
    for (int i = 0; i < N; ++i){
        if (t10 < 0 && ((rising && y[i] >= lo10) || (!rising && y[i] <= lo10))){
            t10 = t[i];
        }
        if (t10 >= 0 && t90 < 0 &&
            ((rising && y[i] >= hi90) || (!rising && y[i] <= hi90))){
            t90 = t[i]; break;
        }
    }
    if (t10 >= 0 && t90 >= t10){
        m.riseTime    = t90 - t10;
        if (m.riseTime > 1e-9){
            /* First-order/2nd-order rule of thumb: f_-3dB ≈ 0.35 / t_rise. */
            m.bandwidthHz = 0.35 / m.riseTime;
        }
    }

    /* Settling: last time outside the 2 % band around the steady value. */
    const double band = 0.02 * std::abs(range);
    const double tEdge = t.front();
    m.settlingTime = -1.0;
    for (int i = 0; i < N; ++i){
        if (std::abs(y[i] - m.ssValue) > band){ m.settlingTime = t[i] - tEdge; }
    }
    return m;
}

QString formatStepMetrics(const StepMetrics& m)
{
    if (!m.valid){ return QStringLiteral("no samples"); }
    auto f = [](double v, int p = 3){ return QString::number(v, 'f', p); };

    const QString rise = (m.riseTime >= 0)
        ? QStringLiteral("%1 s").arg(f(m.riseTime))
        : QStringLiteral("—");
    const QString bw = (m.bandwidthHz >= 0)
        ? QStringLiteral("%1 Hz").arg(f(m.bandwidthHz, 2))
        : QStringLiteral("—");
    const QString settle = (m.settlingTime >= 0)
        ? QStringLiteral("%1 s (2%%)").arg(f(m.settlingTime))
        : QStringLiteral("&lt; tick");

    return QStringLiteral(
        "<b>Overshoot:</b> %1%% &nbsp; "
        "<b>Settling:</b> %2 &nbsp; "
        "<b>Rise:</b> %3 &nbsp; "
        "<b>BW:</b> %4<br>"
        "<b>Steady:</b> %5 &nbsp; "
        "<b>Ref:</b> %6 &nbsp; "
        "<b>SS err:</b> %7 &nbsp; "
        "<b>Peak:</b> %8")
        .arg(f(m.overshootPct, 1), settle, rise, bw,
             f(m.ssValue), f(m.commanded), f(m.ssError), f(m.peak));
}

}  // namespace

/* ====================================================================
 *  StepResponseView
 * ==================================================================== */

StepResponseView::StepResponseView(QWidget* parent) : QWidget(parent)
{
    m_label = new QLabel(tr("(select a slave)"), this);

    m_target = new QComboBox(this);
    m_target->addItem(tr("Position (rad)"),    int(TargetKind::Position));
    m_target->addItem(tr("Velocity (rad/s)"),  int(TargetKind::Velocity));
    m_target->addItem(tr("Torque (Nm)"),       int(TargetKind::Torque));

    auto makeSpin = [this](double init, double step, int decimals){
        auto* sb = new QDoubleSpinBox(this);
        sb->setRange(-1e6, 1e6); sb->setDecimals(decimals);
        sb->setSingleStep(step); sb->setValue(init);
        return sb;
    };
    m_amp      = makeSpin(1.0, 0.1, 3);
    m_offset   = makeSpin(0.0, 0.1, 3);
    m_duration = makeSpin(2.0, 0.5, 2); m_duration->setSuffix(tr(" s"));
    m_runBtn   = new QPushButton(tr("Run step"), this);

    m_metrics = new QLabel(tr("—"), this);
    m_metrics->setTextFormat(Qt::RichText);
    m_metrics->setWordWrap(true);

    /* --- chart --- */
    m_chart = new QChart;
    m_chart->setTitle(tr("Step response"));
    m_chart->legend()->setAlignment(Qt::AlignBottom);
    m_cmdLine = new QLineSeries(m_chart); m_cmdLine->setName(tr("commanded"));
    m_actual  = new QLineSeries(m_chart); m_actual ->setName(tr("actual"));
    m_chart->addSeries(m_cmdLine);
    m_chart->addSeries(m_actual);
    m_axisX = new QValueAxis(m_chart); m_axisX->setTitleText(tr("t (s)"));
    m_axisY = new QValueAxis(m_chart);
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
    m_cmdLine->attachAxis(m_axisX); m_cmdLine->attachAxis(m_axisY);
    m_actual ->attachAxis(m_axisX); m_actual ->attachAxis(m_axisY);

    auto* view = new QChartView(m_chart, this);
    view->setRenderHint(QPainter::Antialiasing, true);
    view->setMinimumHeight(280);

    /* --- layout --- */
    auto* form = new QFormLayout;
    form->addRow(tr("Target"),       m_target);
    form->addRow(tr("Step size"),    m_amp);
    form->addRow(tr("Offset"),       m_offset);
    form->addRow(tr("Duration"),     m_duration);

    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(m_runBtn);
    btnRow->addStretch();

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(form);
    root->addLayout(btnRow);
    root->addWidget(view, 1);
    root->addWidget(m_metrics);

    connect(m_runBtn, &QPushButton::clicked, this, &StepResponseView::onRunClicked);
    setActiveSlave(-1);
}

void StepResponseView::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    if (idx < 0){
        m_label->setText(tr("(select a slave)"));
    } else {
        m_label->setText(tr("Slave %1 — %2").arg(idx).arg(name));
    }
    const bool en = (idx >= 0) && !m_capturing;
    m_runBtn->setEnabled(en);
}

void StepResponseView::resetChart()
{
    m_cmdLine->clear();
    m_actual->clear();
    m_samples.clear();
    m_metrics->setText(tr("capturing…"));
}

void StepResponseView::onRunClicked()
{
    if (m_idx < 0){ return; }

    /* Guard: a step into a disabled drive just draws a flat line. Refuse
     * with a clear message when we have fresh status showing the drive is
     * not in Operation-Enabled. (When no PDO status is available yet we
     * can't prove it's disabled, so we let the run proceed.) */
    if (m_haveStatus && !m_opEnabled){
        m_metrics->setText(tr(
            "<b style='color:#c0392b;'>Drive not enabled.</b> "
            "Enable it in the matching mode (Control panel) before running "
            "a step — the command can't move a disabled drive."));
        return;
    }

    m_stepBase    = m_offset->value();
    m_stepMag     = m_amp->value();
    m_duration_s  = m_duration->value();

    GenCfg cfg;
    cfg.shape       = GenCfg::Step;
    cfg.target      = static_cast<TargetKind>(m_target->currentData().toInt());
    cfg.amplitude   = static_cast<float>(m_stepMag);
    cfg.offset      = static_cast<float>(m_stepBase);
    cfg.durationSec = static_cast<float>(m_duration_s);
    cfg.rateHz      = 50;

    resetChart();
    m_t0.restart();
    m_capturing = true;
    m_runBtn->setEnabled(false);
    emit startGenRequested(m_idx, cfg);
}

void StepResponseView::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }

    /* Track Operation-Enabled for the active slave regardless of capture
     * state, so the Run guard reflects the live drive. */
    for (const auto& s : snaps){
        if (s.idx != m_idx){ continue; }
        if (s.pdoFresh){
            m_haveStatus = true;
            m_opEnabled  = (s.statusword & kSwOperationEnabled) != 0;
        }
        break;
    }

    if (!m_capturing){ return; }
    for (const auto& s : snaps){
        if (s.idx != m_idx){ continue; }
        const double t = m_t0.elapsed() / 1000.0;
        const double final_ = m_stepBase + m_stepMag;

        float actual = s.position;
        switch (static_cast<TargetKind>(m_target->currentData().toInt())){
        case TargetKind::Position: actual = s.position; break;
        case TargetKind::Velocity: actual = s.velocity; break;
        case TargetKind::Torque:   actual = s.torque;   break;
        }
        m_cmdLine->append(t, final_);
        m_actual->append(t, actual);
        m_samples.push_back({t, final_, static_cast<double>(actual)});
    }
    /* Auto-fit X. */
    if (!m_samples.isEmpty()){
        m_axisX->setRange(0, std::max(m_duration_s, m_samples.back().t));
    }
    /* Y: symmetric around commanded final with 25% headroom. */
    double lo =  1e9, hi = -1e9;
    for (const auto& p : m_samples){
        lo = std::min(lo, std::min(p.cmd, p.actual));
        hi = std::max(hi, std::max(p.cmd, p.actual));
    }
    if (lo > hi){ lo = -1; hi = 1; }
    const double pad = std::max(0.1, 0.25 * (hi - lo));
    m_axisY->setRange(lo - pad, hi + pad);
}

void StepResponseView::onGeneratorStarted(int /*idx*/) {}

void StepResponseView::onGeneratorStopped(int idx)
{
    if (!m_capturing || idx != m_idx){ return; }
    m_capturing = false;
    m_runBtn->setEnabled(m_idx >= 0);
    computeAndShowMetrics();
}

void StepResponseView::computeAndShowMetrics()
{
    if (m_samples.isEmpty()){
        m_metrics->setText(tr("no samples"));
        return;
    }
    /* Hand the measured (t, actual) trace plus the commanded final to the
     * shared analyser. It derives the pre-/post-step steady levels from the
     * data, so the metrics hold even when the actual didn't start at the
     * commanded offset (the usual case for position / velocity). */
    QVector<double> t, y;
    t.reserve(m_samples.size());
    y.reserve(m_samples.size());
    for (const auto& s : m_samples){ t.push_back(s.t); y.push_back(s.actual); }

    const StepMetrics m = computeStepMetrics(t, y, m_stepBase + m_stepMag);
    m_metrics->setText(formatStepMetrics(m));
}


/* ====================================================================
 *  BodeView
 * ==================================================================== */

BodeView::BodeView(QWidget* parent) : QWidget(parent)
{
    m_label = new QLabel(tr("(select a slave)"), this);

    m_target = new QComboBox(this);
    m_target->addItem(tr("Position (rad)"),    int(TargetKind::Position));
    m_target->addItem(tr("Velocity (rad/s)"),  int(TargetKind::Velocity));
    m_target->addItem(tr("Torque (Nm)"),       int(TargetKind::Torque));

    auto makeSpin = [this](double init, double step, int decimals, double lo, double hi){
        auto* sb = new QDoubleSpinBox(this);
        sb->setRange(lo, hi); sb->setDecimals(decimals);
        sb->setSingleStep(step); sb->setValue(init);
        return sb;
    };
    m_fStart  = makeSpin(0.1, 0.1, 3, 0.001, 1000); m_fStart->setSuffix(tr(" Hz"));
    m_fEnd    = makeSpin(5.0, 0.5, 3, 0.001, 1000); m_fEnd->setSuffix(tr(" Hz"));
    m_points  = new QSpinBox(this); m_points->setRange(3, 40); m_points->setValue(8);
    m_ampSpin = makeSpin(0.5, 0.05, 3, 0.001, 1000);
    m_cycles  = new QSpinBox(this); m_cycles->setRange(1, 20); m_cycles->setValue(3);
    m_runBtn  = new QPushButton(tr("Run sweep"), this);
    m_status  = new QLabel(tr("idle"), this);

    /* --- magnitude chart --- */
    m_magChart = new QChart;
    m_magChart->setTitle(tr("Magnitude (dB)"));
    m_magChart->legend()->hide();
    m_magLine = new QLineSeries(m_magChart);
    m_magDots = new QScatterSeries(m_magChart);
    m_magDots->setMarkerSize(6.0);
    m_magChart->addSeries(m_magLine);
    m_magChart->addSeries(m_magDots);
    m_magAxisX = new QLogValueAxis(m_magChart);
    m_magAxisX->setTitleText(tr("f (Hz)"));
    m_magAxisX->setBase(10.0);
    m_magAxisX->setMinorTickCount(8);
    m_magAxisY = new QValueAxis(m_magChart);
    m_magAxisY->setTitleText(tr("|H| (dB)"));
    m_magChart->addAxis(m_magAxisX, Qt::AlignBottom);
    m_magChart->addAxis(m_magAxisY, Qt::AlignLeft);
    m_magLine->attachAxis(m_magAxisX); m_magLine->attachAxis(m_magAxisY);
    m_magDots->attachAxis(m_magAxisX); m_magDots->attachAxis(m_magAxisY);

    /* --- phase chart --- */
    m_phChart = new QChart;
    m_phChart->setTitle(tr("Phase (deg)"));
    m_phChart->legend()->hide();
    m_phLine = new QLineSeries(m_phChart);
    m_phDots = new QScatterSeries(m_phChart);
    m_phDots->setMarkerSize(6.0);
    m_phChart->addSeries(m_phLine);
    m_phChart->addSeries(m_phDots);
    m_phAxisX = new QLogValueAxis(m_phChart);
    m_phAxisX->setTitleText(tr("f (Hz)"));
    m_phAxisX->setBase(10.0);
    m_phAxisX->setMinorTickCount(8);
    m_phAxisY = new QValueAxis(m_phChart);
    m_phAxisY->setTitleText(tr("∠H (deg)"));
    m_phAxisY->setRange(-180, 180);
    m_phChart->addAxis(m_phAxisX, Qt::AlignBottom);
    m_phChart->addAxis(m_phAxisY, Qt::AlignLeft);
    m_phLine->attachAxis(m_phAxisX); m_phLine->attachAxis(m_phAxisY);
    m_phDots->attachAxis(m_phAxisX); m_phDots->attachAxis(m_phAxisY);

    auto* magView = new QChartView(m_magChart, this);
    magView->setRenderHint(QPainter::Antialiasing, true);
    magView->setMinimumHeight(180);
    auto* phView  = new QChartView(m_phChart, this);
    phView->setRenderHint(QPainter::Antialiasing, true);
    phView->setMinimumHeight(180);

    /* --- layout --- */
    auto* form = new QFormLayout;
    form->addRow(tr("Target"),       m_target);
    form->addRow(tr("f start"),      m_fStart);
    form->addRow(tr("f end"),        m_fEnd);
    form->addRow(tr("Points"),       m_points);
    form->addRow(tr("Amplitude"),    m_ampSpin);
    form->addRow(tr("Cycles / pt."), m_cycles);

    auto* btnRow = new QHBoxLayout;
    btnRow->addWidget(m_runBtn);
    btnRow->addWidget(m_status, 1);

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(form);
    root->addLayout(btnRow);
    root->addWidget(magView, 1);
    root->addWidget(phView,  1);

    connect(m_runBtn, &QPushButton::clicked, this, &BodeView::onRunClicked);
    setActiveSlave(-1);
}

void BodeView::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    if (idx < 0){
        m_label->setText(tr("(select a slave)"));
    } else {
        m_label->setText(tr("Slave %1 — %2").arg(idx).arg(name));
    }
    /* Form parameter widgets gate on (slave selected && not sweeping)
     * so the operator can't change them mid-sweep. m_runBtn is gated
     * only on slave-selected — it doubles as the Stop button while a
     * sweep is in flight, so it must stay clickable. */
    const bool formEn = (idx >= 0) && !m_sweeping;
    for (QWidget* w : { static_cast<QWidget*>(m_target),
                        static_cast<QWidget*>(m_fStart),
                        static_cast<QWidget*>(m_fEnd),
                        static_cast<QWidget*>(m_points),
                        static_cast<QWidget*>(m_ampSpin),
                        static_cast<QWidget*>(m_cycles) }){
        w->setEnabled(formEn);
    }
    m_runBtn->setEnabled(idx >= 0);
}

void BodeView::resetCharts()
{
    m_magLine->clear(); m_magDots->clear();
    m_phLine->clear();  m_phDots->clear();
    m_magAxisY->setRange(-40, 10);
    m_phAxisY->setRange(-180, 180);
}

void BodeView::onRunClicked()
{
    /* Toggle behaviour: while a sweep is in flight the same button
     * acts as Stop. Earlier revisions disabled the button during the
     * sweep, which left the operator no way to abort a long log-sweep
     * mid-way without disconnecting. */
    if (m_sweeping){
        m_sweeping = false;                /* gate onGeneratorStopped so
                                            * it doesn't advance to the
                                            * next point after our abort */
        emit stopGenRequested(m_idx);      /* tell the worker to halt the
                                            * in-flight point generator */
        m_status->setText(tr("stopped"));
        setActiveSlave(m_idx);             /* re-enables form widgets +
                                            * restores the Run button   */
        m_runBtn->setText(tr("Run sweep"));
        return;
    }

    if (m_idx < 0){ return; }
    const double f0 = m_fStart->value();
    const double f1 = m_fEnd->value();
    const int nPts  = m_points->value();
    if (f1 <= f0 || nPts < 2){ return; }
    m_amp      = m_ampSpin->value();
    m_pointDur = std::max(0.3, m_cycles->value() / f0);

    /* Log-spaced frequencies. */
    m_freqs.clear();
    m_freqs.reserve(nPts);
    const double log0 = std::log10(f0), log1 = std::log10(f1);
    for (int i = 0; i < nPts; ++i){
        const double frac = static_cast<double>(i) / (nPts - 1);
        m_freqs.push_back(std::pow(10.0, log0 + (log1 - log0) * frac));
    }
    m_magAxisX->setRange(f0, f1);
    m_phAxisX->setRange(f0, f1);

    resetCharts();
    m_sweeping = true;
    m_pointIdx = 0;
    /* Form widgets disabled so the operator can't change parameters
     * mid-sweep, but keep m_runBtn live and re-label it as Stop. */
    for (QWidget* w : { static_cast<QWidget*>(m_target),
                        static_cast<QWidget*>(m_fStart),
                        static_cast<QWidget*>(m_fEnd),
                        static_cast<QWidget*>(m_points),
                        static_cast<QWidget*>(m_ampSpin),
                        static_cast<QWidget*>(m_cycles) }){ w->setEnabled(false); }
    m_runBtn->setText(tr("Stop"));

    startNextPoint();
}

void BodeView::startNextPoint()
{
    if (m_pointIdx >= m_freqs.size()){
        m_sweeping = false;
        m_status->setText(tr("done"));
        setActiveSlave(m_idx);
        m_runBtn->setText(tr("Run sweep"));
        return;
    }
    const double f = m_freqs[m_pointIdx];
    /* Adapt duration so each point captures a fixed number of cycles. */
    m_pointDur = std::max(0.3, m_cycles->value() / f);

    GenCfg cfg;
    cfg.shape        = GenCfg::Sine;
    cfg.target       = static_cast<TargetKind>(m_target->currentData().toInt());
    cfg.amplitude    = static_cast<float>(m_amp);
    cfg.offset       = 0.0f;
    cfg.frequency    = static_cast<float>(f);
    cfg.durationSec  = static_cast<float>(m_pointDur);
    cfg.rateHz       = 50;

    m_samples.clear();
    m_t0.restart();
    m_captureStart = 0.0;
    m_status->setText(tr("sweep %1/%2 — f = %3 Hz")
                          .arg(m_pointIdx + 1).arg(m_freqs.size())
                          .arg(f, 0, 'f', 3));
    emit startGenRequested(m_idx, cfg);
}

void BodeView::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (!m_sweeping || m_idx < 0){ return; }
    for (const auto& s : snaps){
        if (s.idx != m_idx){ continue; }
        const double t = m_t0.elapsed() / 1000.0;
        float actual = s.position;
        switch (static_cast<TargetKind>(m_target->currentData().toInt())){
        case TargetKind::Position: actual = s.position; break;
        case TargetKind::Velocity: actual = s.velocity; break;
        case TargetKind::Torque:   actual = s.torque;   break;
        }
        m_samples.push_back({t, static_cast<double>(actual)});
    }
}

void BodeView::onGeneratorStarted(int /*idx*/) {}

void BodeView::onGeneratorStopped(int idx)
{
    if (!m_sweeping || idx != m_idx){ return; }
    analyseCurrentPoint();
    ++m_pointIdx;
    startNextPoint();
}

void BodeView::analyseCurrentPoint()
{
    if (m_pointIdx >= m_freqs.size()){ return; }
    const double f = m_freqs[m_pointIdx];
    const double omega = 2.0 * M_PI * f;

    /* Skip first 20% of samples so the plant settles. */
    const int N = m_samples.size();
    const int skip = std::min(N / 5, N - 1);
    if (N - skip < 4){ return; }

    double sumS = 0.0, sumC = 0.0;
    int   used = 0;
    for (int i = skip; i < N; ++i){
        const double t = m_samples[i].t;
        sumS += m_samples[i].actual * std::sin(omega * t);
        sumC += m_samples[i].actual * std::cos(omega * t);
        ++used;
    }
    if (used < 2){ return; }
    const double re = 2.0 * sumS / used;
    const double im = 2.0 * sumC / used;
    const double mag = std::sqrt(re * re + im * im) / std::max(1e-12, m_amp);
    const double magDb = 20.0 * std::log10(std::max(1e-12, mag));
    const double phDeg = std::atan2(im, re) * 180.0 / M_PI;

    m_magLine->append(f, magDb); m_magDots->append(f, magDb);
    m_phLine ->append(f, phDeg); m_phDots ->append(f, phDeg);

    /* Auto-fit magnitude Y. */
    double lo =  1e9, hi = -1e9;
    for (const auto& p : m_magLine->points()){
        lo = std::min(lo, p.y()); hi = std::max(hi, p.y());
    }
    if (lo < hi){
        const double pad = std::max(3.0, 0.2 * (hi - lo));
        m_magAxisY->setRange(lo - pad, hi + pad);
    }
}


/* ====================================================================
 *  TuningPane
 * ==================================================================== */

/* ======================================================================
 *  AutoTuneView
 * ====================================================================== */

static double defaultTuneBw(Loop l)
{
    switch (l){
    case Loop::Current:  return 1500.0;
    case Loop::Velocity: return  100.0;
    case Loop::Position: return   20.0;
    }
    return 100.0;
}

static double defaultStepAmp(Loop l)
{
    /* Conservative per-loop defaults:
     *   - Current in amperes
     *   - Velocity in rad/s (master frame; gear-converted on board)
     *   - Position in rad (master frame; ~1 rad = ~57° -- big enough to
     *     see settle behaviour but well inside any sane position limit). */
    switch (l){
    case Loop::Current:  return  0.2;   /* A     */
    case Loop::Velocity: return 20.0;   /* rad/s */
    case Loop::Position: return  1.0;   /* rad   */
    }
    return 1.0;
}

AutoTuneView::AutoTuneView(QWidget* parent) : QWidget(parent)
{
    m_label = new QLabel(tr("(no slave selected)"), this);
    m_label->setStyleSheet(QStringLiteral("color: #555; font-style: italic;"));

    m_loop = new QComboBox(this);
    m_loop->addItem(tr("Current"),  int(Loop::Current));
    m_loop->addItem(tr("Velocity"), int(Loop::Velocity));
    m_loop->addItem(tr("Position"), int(Loop::Position));

    m_bw = new QDoubleSpinBox(this);
    m_bw->setRange(0.1, 5000.0);
    m_bw->setDecimals(1);
    m_bw->setSingleStep(10.0);
    m_bw->setSuffix(QStringLiteral(" Hz"));
    m_bw->setValue(defaultTuneBw(Loop::Current));

    m_tuneBtn = new QPushButton(tr("▶ Tune"), this);
    m_tuneStatus = new QLabel(this);
    m_tuneStatus->setMinimumWidth(120);

    m_kpView = new QDoubleSpinBox(this);
    m_kiView = new QDoubleSpinBox(this);
    for (QDoubleSpinBox* sb : { m_kpView, m_kiView }){
        sb->setRange(-1e9, 1e9);
        sb->setDecimals(6);
        sb->setReadOnly(true);
        sb->setButtonSymbols(QAbstractSpinBox::NoButtons);
        sb->setStyleSheet(QStringLiteral("background: #f4f4f4;"));
    }

    m_amp = new QDoubleSpinBox(this);
    m_amp->setRange(-1000.0, 1000.0);
    m_amp->setDecimals(3);
    m_amp->setSingleStep(0.1);
    m_amp->setValue(defaultStepAmp(Loop::Current));

    m_refDefault = new QDoubleSpinBox(this);
    m_refDefault->setRange(-1000.0, 1000.0);
    m_refDefault->setDecimals(3);
    m_refDefault->setSingleStep(0.1);
    m_refDefault->setValue(0.0);

    m_captureBtn = new QPushButton(tr("▶ Capture Step"), this);
    m_captureStatus = new QLabel(this);
    m_captureStatus->setMinimumWidth(120);

    /* Layout: top control row (tune), middle result row, then a thin
     * follow-up row (step capture), then the chart fills the rest. */
    auto* tuneRow = new QHBoxLayout;
    tuneRow->addWidget(new QLabel(tr("Loop:"), this));
    tuneRow->addWidget(m_loop);
    tuneRow->addSpacing(8);
    tuneRow->addWidget(new QLabel(tr("BW:"), this));
    tuneRow->addWidget(m_bw);
    tuneRow->addSpacing(8);
    tuneRow->addWidget(m_tuneBtn);
    tuneRow->addWidget(m_tuneStatus);
    tuneRow->addStretch(1);

    auto* resultRow = new QHBoxLayout;
    resultRow->addWidget(new QLabel(tr("Result Kp:"), this));
    resultRow->addWidget(m_kpView);
    resultRow->addSpacing(8);
    resultRow->addWidget(new QLabel(tr("Ki:"), this));
    resultRow->addWidget(m_kiView);
    resultRow->addStretch(1);

    auto* captureRow = new QHBoxLayout;
    captureRow->addWidget(new QLabel(tr("Verify — ref:"), this));
    captureRow->addWidget(m_refDefault);
    captureRow->addSpacing(8);
    captureRow->addWidget(new QLabel(tr("step amp:"), this));
    captureRow->addWidget(m_amp);
    captureRow->addSpacing(8);
    captureRow->addWidget(m_captureBtn);
    captureRow->addWidget(m_captureStatus);
    captureRow->addStretch(1);

    /* Chart: measured signal (buffer0) + reference / control effort
     * (buffer1). Time on X (ms), two Y series sharing one axis -- the
     * units differ between loops but Qt Charts handles a unified axis
     * fine for a rough plot. */
    m_meas = new QLineSeries;
    m_ref  = new QLineSeries;
    m_meas->setName(tr("Measured (buf0)"));
    m_ref ->setName(tr("Reference / Iq (buf1)"));

    m_chart = new QChart;
    m_chart->addSeries(m_meas);
    m_chart->addSeries(m_ref);
    m_chart->legend()->setAlignment(Qt::AlignBottom);
    m_chart->setMargins(QMargins(2, 2, 2, 2));

    m_axisX = new QValueAxis;
    m_axisX->setTitleText(tr("Time (ms)"));
    m_axisX->setRange(0.0, 128.0);
    m_axisY = new QValueAxis;
    m_axisY->setTitleText(tr("Signal"));
    m_axisY->setRange(-1.0, 1.0);
    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);
    m_meas->attachAxis(m_axisX); m_meas->attachAxis(m_axisY);
    m_ref ->attachAxis(m_axisX); m_ref ->attachAxis(m_axisY);

    auto* chartView = new QChartView(m_chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    m_metrics = new QLabel(tr("—"), this);
    m_metrics->setTextFormat(Qt::RichText);
    m_metrics->setWordWrap(true);

    auto* root = new QVBoxLayout(this);
    root->addWidget(m_label);
    root->addLayout(tuneRow);
    root->addLayout(resultRow);
    root->addLayout(captureRow);
    root->addWidget(chartView, /*stretch=*/1);
    root->addWidget(m_metrics);

    connect(m_tuneBtn,    &QPushButton::clicked, this, &AutoTuneView::onTuneClicked);
    connect(m_captureBtn, &QPushButton::clicked, this, &AutoTuneView::onCaptureClicked);
    connect(m_loop, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,   &AutoTuneView::onLoopChanged);

    setActiveSlave(-1);
}

void AutoTuneView::setActiveSlave(int idx, const QString& name)
{
    m_idx = idx;
    const bool en = (idx >= 0);
    m_label->setText(en ? tr("Slave %1 — %2").arg(idx).arg(name)
                         : tr("(no slave selected)"));
    m_loop->setEnabled(en);
    m_bw->setEnabled(en);
    m_tuneBtn->setEnabled(en);
    m_amp->setEnabled(en);
    m_refDefault->setEnabled(en);
    m_captureBtn->setEnabled(en);
    if (en){
        /* Pull the current PI gains for the selected loop so Result Kp/Ki
         * reflects what the live controller is using -- second open of the
         * dock (or slave re-select) repopulates from the OD instead of
         * showing the stale 0.0 / 0.0 from the previous tear-down. Routes
         * through MasterWorker -> SDO 0x60F6/F9/FB -> board on_read hook
         * -> latest motor_get_*_gain values. onGainRead also back-computes
         * BW from the readback Kp, so the spinbox lands on the BW that
         * actually corresponds to the live controller. */
        const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
        emit readGainRequested(idx, l);
    } else {
        m_tuneStatus->clear();
        m_captureStatus->clear();
        m_kpView->setValue(0.0);
        m_kiView->setValue(0.0);
        resetChart();
    }
    /* Drive state is unknown until the next snapshot for this slave. */
    m_haveStatus = false;
    m_opEnabled  = false;
    if (m_metrics){ m_metrics->setText(tr("—")); }
}

void AutoTuneView::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }
    for (const auto& s : snaps){
        if (s.idx != m_idx){ continue; }
        if (s.pdoFresh){
            m_haveStatus = true;
            m_opEnabled  = (s.statusword & kSwOperationEnabled) != 0;
        }
        break;
    }
}

void AutoTuneView::onLoopChanged(int)
{
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    /* Seed BW to the conservative per-loop default while we wait for the
     * SDO readback; onGainRead will overwrite it with the back-computed
     * value once Kp lands. */
    m_bw->setValue(defaultTuneBw(l));
    m_amp->setValue(defaultStepAmp(l));
    /* Refresh the displayed Kp/Ki for the new loop -- the previous selection's
     * values are stale once the operator switches. Clear them first so the
     * SDO round-trip latency doesn't leave the wrong loop's numbers visible. */
    m_kpView->setValue(0.0);
    m_kiView->setValue(0.0);
    if (m_idx >= 0){
        emit readGainRequested(m_idx, l);
    }
}

void AutoTuneView::onGainRead(int idx, Loop loop, float kp, float ki, bool ok)
{
    if (idx != m_idx){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    if (loop != l){ return; }
    if (ok){
        m_kpView->setValue(kp);
        m_kiView->setValue(ki);
        /* Back-compute BW from live Kp + cached motor profile so the
         * BW spinbox reflects what the controller actually realises.
         * Block the spinbox signal -- not strictly required (no
         * valueChanged hook anymore) but keeps the behaviour clean if
         * one is added later. */
        const float bw = bwFromKp(loop, kp, m_motorParams);
        if (std::isfinite(bw) && bw > 0.0f){
            QSignalBlocker blk(m_bw);
            m_bw->setValue(bw);
        }
    } else {
        m_kpView->setValue(0.0);
        m_kiView->setValue(0.0);
    }
}

void AutoTuneView::setMotorParams(const MotorParams& p)
{
    m_motorParams = p;
    /* Refresh BW from the existing Kp readback (if any) against the new
     * profile -- a fresh Ls_q / J / Kt shifts what BW the live Kp maps
     * to, so the spinbox should follow. */
    const float kp = static_cast<float>(m_kpView->value());
    if (kp == 0.0f){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    const float bw = bwFromKp(l, kp, m_motorParams);
    if (std::isfinite(bw) && bw > 0.0f){
        QSignalBlocker blk(m_bw);
        m_bw->setValue(bw);
    }
}

void AutoTuneView::onTuneClicked()
{
    if (m_idx < 0){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    m_tuneStatus->setText(tr("tuning…"));
    m_tuneStatus->setStyleSheet(QStringLiteral("color: #555;"));
    m_tuneBtn->setEnabled(false);
    emit tuneRequested(m_idx, l, static_cast<float>(m_bw->value()));
}

void AutoTuneView::onGainTuned(int idx, Loop loop, float kp, float ki, bool ok)
{
    if (idx != m_idx){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    if (loop != l){ return; }
    if (ok){
        m_kpView->setValue(kp);
        m_kiView->setValue(ki);
        /* Re-derive BW from the tuned Kp -- normally matches the
         * requested BW, but reflects any clamping/rounding the solver
         * applied. */
        const float bw = bwFromKp(loop, kp, m_motorParams);
        if (std::isfinite(bw) && bw > 0.0f){
            QSignalBlocker blk(m_bw);
            m_bw->setValue(bw);
        }
        m_tuneStatus->setText(tr("tuned"));
        m_tuneStatus->setStyleSheet(QStringLiteral("color: #228822;"));
    } else {
        m_tuneStatus->setText(tr("failed"));
        m_tuneStatus->setStyleSheet(QStringLiteral("color: #c0392b;"));
    }
    m_tuneBtn->setEnabled(m_idx >= 0);
}

void AutoTuneView::onCaptureClicked()
{
    if (m_idx < 0){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());

    /* Guard: the board step harness can't excite a disabled drive, so a
     * capture would come back flat. Refuse with a clear message when the
     * live status shows the drive isn't enabled. */
    if (m_haveStatus && !m_opEnabled){
        m_captureStatus->setText(tr("drive not enabled"));
        m_captureStatus->setStyleSheet(QStringLiteral("color: #c0392b;"));
        m_metrics->setText(tr(
            "<b style='color:#c0392b;'>Drive not enabled.</b> Enable it in "
            "the matching mode (Control panel) before capturing a step."));
        return;
    }

    /* All three loops now supported on the board side; Position uses an
     * app-local 100 Hz sampler (the SDK stepRPVars only covers current
     * / speed / torque). Same MasterWorker capture flow regardless. */
    m_captureStatus->setText(tr("capturing…"));
    m_captureStatus->setStyleSheet(QStringLiteral("color: #555;"));
    m_captureBtn->setEnabled(false);
    resetChart();
    emit captureStepRequested(m_idx, l,
                              static_cast<float>(m_amp->value()),
                              static_cast<float>(m_refDefault->value()));
}

void AutoTuneView::onStepCaptured(int idx, Loop loop,
                                    QVector<float> buf0, QVector<float> buf1,
                                    float sample_rate_hz, bool ok)
{
    if (idx != m_idx){ return; }
    const Loop l = static_cast<Loop>(m_loop->currentData().toInt());
    if (loop != l){ return; }
    if (ok && buf0.size() == 256 && buf1.size() == 256){
        replotStep(buf0, buf1, sample_rate_hz);

        /* Analyse the measured response (buf0) against the commanded step
         * (ref_default -> ref_default + amp). dt comes from the board's
         * reported sample rate. */
        const double dt = sample_rate_hz > 0.0f ? 1.0 / double(sample_rate_hz) : 0.0005;
        QVector<double> t, y;
        t.reserve(buf0.size());
        y.reserve(buf0.size());
        for (int i = 0; i < buf0.size(); ++i){ t.push_back(i * dt); y.push_back(buf0[i]); }
        const double commanded = m_refDefault->value() + m_amp->value();
        m_metrics->setText(formatStepMetrics(computeStepMetrics(t, y, commanded)));

        m_captureStatus->setText(tr("done @ %1 kHz").arg(sample_rate_hz / 1000.0, 0, 'f', 1));
        m_captureStatus->setStyleSheet(QStringLiteral("color: #228822;"));
    } else {
        m_metrics->setText(tr("—"));
        m_captureStatus->setText(tr("failed"));
        m_captureStatus->setStyleSheet(QStringLiteral("color: #c0392b;"));
    }
    m_captureBtn->setEnabled(m_idx >= 0);
}

void AutoTuneView::replotStep(const QVector<float>& buf0,
                                const QVector<float>& buf1,
                                float sample_rate_hz)
{
    m_meas->clear();
    m_ref ->clear();
    const double dt_ms = sample_rate_hz > 0.0f
        ? 1000.0 / double(sample_rate_hz) : 0.5;
    float y_min = std::numeric_limits<float>::infinity();
    float y_max = -y_min;
    for (int i = 0; i < buf0.size(); ++i){
        const double t = i * dt_ms;
        m_meas->append(t, buf0[i]);
        if (buf0[i] < y_min){ y_min = buf0[i]; }
        if (buf0[i] > y_max){ y_max = buf0[i]; }
    }
    for (int i = 0; i < buf1.size(); ++i){
        const double t = i * dt_ms;
        m_ref->append(t, buf1[i]);
        if (buf1[i] < y_min){ y_min = buf1[i]; }
        if (buf1[i] > y_max){ y_max = buf1[i]; }
    }
    const double pad = std::max(0.05, (y_max - y_min) * 0.1);
    m_axisX->setRange(0.0, (buf0.size() - 1) * dt_ms);
    m_axisY->setRange(y_min - pad, y_max + pad);
}

void AutoTuneView::resetChart()
{
    m_meas->clear();
    m_ref ->clear();
    m_axisX->setRange(0.0, 128.0);
    m_axisY->setRange(-1.0, 1.0);
}


/* ====================================================================
 *  TuningPane
 * ==================================================================== */

TuningPane::TuningPane(QWidget* parent) : QWidget(parent)
{
    m_step     = new StepResponseView;
    m_bode     = new BodeView;
    m_autoTune = new AutoTuneView;
    m_tabs = new QTabWidget(this);
    m_tabs->addTab(m_step,     tr("Step"));
    m_tabs->addTab(m_bode,     tr("Bode"));
    m_tabs->addTab(m_autoTune, tr("Auto-Tune"));

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(m_tabs);
}

void TuningPane::setActiveSlave(int idx, const QString& name)
{
    m_step    ->setActiveSlave(idx, name);
    m_bode    ->setActiveSlave(idx, name);
    m_autoTune->setActiveSlave(idx, name);
}

}  // namespace vrmc
