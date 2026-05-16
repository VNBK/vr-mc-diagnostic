#include "TuningPane.hpp"

#include <QChart>
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

#include <algorithm>
#include <cmath>

namespace vrmc {

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
    if (!m_capturing || m_idx < 0){ return; }
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
    const double initial = m_stepBase;
    const double final_  = m_stepBase + m_stepMag;
    const double range   = final_ - initial;
    if (std::abs(range) < 1e-9){
        m_metrics->setText(tr("zero step magnitude — skip metrics"));
        return;
    }

    /* Rise time: first time actual crosses 10% → first time crosses 90%. */
    const double lo10 = initial + 0.10 * range;
    const double hi90 = initial + 0.90 * range;
    double t10 = -1.0, t90 = -1.0;
    for (const auto& s : m_samples){
        const bool rising = (range > 0);
        if (t10 < 0 && ((rising && s.actual >= lo10) ||
                        (!rising && s.actual <= lo10))){ t10 = s.t; }
        if (t90 < 0 && ((rising && s.actual >= hi90) ||
                        (!rising && s.actual <= hi90))){ t90 = s.t; break; }
    }

    /* Overshoot: max deviation past final (in direction of step). */
    double peak = initial;
    for (const auto& s : m_samples){
        if (range > 0){ peak = std::max(peak, s.actual); }
        else          { peak = std::min(peak, s.actual); }
    }
    const double overshootPct = ((peak - final_) / range) * 100.0;

    /* Settling: last time |actual - final| > 2% of range. */
    const double band = 0.02 * std::abs(range);
    double tSettle = -1.0;
    for (const auto& s : m_samples){
        if (std::abs(s.actual - final_) > band){ tSettle = s.t; }
    }

    auto fmt = [](double v, int prec = 3){
        return QString::number(v, 'f', prec);
    };
    QString riseTxt = (t10 >= 0 && t90 >= 0)
        ? tr("%1 s (10→90%%)").arg(fmt(t90 - t10))
        : tr("—");
    QString settleTxt = (tSettle >= 0)
        ? tr("%1 s (2%% band)").arg(fmt(tSettle))
        : tr("< tick (good)");

    m_metrics->setText(tr(
        "<b>Rise time:</b> %1 &nbsp;&nbsp;"
        "<b>Overshoot:</b> %2%% &nbsp;&nbsp;"
        "<b>Settling:</b> %3 &nbsp;&nbsp;"
        "<b>Peak:</b> %4  <b>Final:</b> %5")
            .arg(riseTxt)
            .arg(fmt(overshootPct, 1))
            .arg(settleTxt)
            .arg(fmt(peak)).arg(fmt(final_)));
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

TuningPane::TuningPane(QWidget* parent) : QWidget(parent)
{
    m_step = new StepResponseView;
    m_bode = new BodeView;
    m_tabs = new QTabWidget(this);
    m_tabs->addTab(m_step, tr("Step"));
    m_tabs->addTab(m_bode, tr("Bode"));

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(m_tabs);
}

void TuningPane::setActiveSlave(int idx, const QString& name)
{
    m_step->setActiveSlave(idx, name);
    m_bode->setActiveSlave(idx, name);
}

}  // namespace vrmc
