/*
 * BandwidthSweepProbe.cpp — see header for purpose + math.
 */
#include "BandwidthSweepProbe.hpp"
#include "../../HandWorker.hpp"

#include <QJsonArray>
#include <QTimer>
#include <algorithm>
#include <cmath>


namespace { constexpr double kTwoPi = 6.28318530717958647692; }


BandwidthSweepProbe::BandwidthSweepProbe(HandWorker* worker, QObject* parent)
    : WorkerThreadProbe(worker, parent)
{
}


void BandwidthSweepProbe::startBatch(QJsonObject params)
{
    if (active_){ return; }
    if (!worker() || !worker()->isConnected()){
        fail_(tr("hand not connected"));
        return;
    }
    if (latestQ().isEmpty()){
        fail_(tr("no telemetry observed yet — open later or warm the bus first"));
        return;
    }

    joint_index_       = params.value("joint_index").toInt(0);
    amplitude_rad_     = params.value("amplitude_rad").toDouble(0.05);
    cycles_per_freq_   = std::max(1, params.value("cycles_per_freq").toInt(8));
    settle_between_ms_ = std::max(0,
                          params.value("settle_between_ms").toInt(300));
    min_bandwidth_hz_  = params.value("min_bandwidth_hz").toDouble(3.0);

    frequencies_hz_.clear();
    const QJsonArray flist = params.value("frequencies_hz").toArray();
    for (const auto& v : flist){
        const double f = v.toDouble(0.0);
        if (f > 0.0){ frequencies_hz_.append(f); }
    }
    if (frequencies_hz_.isEmpty()){
        /* Sensible fallback covering finger-DOF expected bandwidth. */
        frequencies_hz_ = {0.5, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0};
    }
    std::sort(frequencies_hz_.begin(), frequencies_hz_.end());

    if (joint_index_ < 0 || joint_index_ >= latestQ().size()){
        fail_(tr("joint_index %1 out of range").arg(joint_index_));
        return;
    }

    /* Bump poll rate for the whole sweep — same reasoning as
     * RoundTripProbe: at 25 Hz GUI default the sample resolution is
     * 40 ms, which is meaningless against a 10 Hz sinusoid. */
    const int test_hz = std::max(1,
                          params.value("test_poll_rate_hz").toInt(1000));
    saved_poll_hz_ = worker()->currentPollRate();
    if (test_hz != saved_poll_hz_){
        worker()->setPollRate(test_hz);
        poll_hz_pushed_ = true;
    }

    gain_db_.clear();
    gain_lin_.clear();
    phase_deg_.clear();
    freq_index_     = -1;
    freq_finishing_ = false;
    baseline_q_     = latestQ().value(joint_index_, 0.0);
    active_         = true;
    startNextFrequency_();
}


void BandwidthSweepProbe::startNextFrequency_()
{
    if (!active_){ return; }
    ++freq_index_;
    if (freq_index_ >= frequencies_hz_.size()){
        finalize_();
        return;
    }
    current_freq_ = frequencies_hz_[freq_index_];
    /* sweep_secs = cycles / f  → an integer number of cycles, which
     * makes the demod integral leakage-free. */
    sweep_secs_   = double(cycles_per_freq_) / current_freq_;
    m_sin_acc_    = 0.0;
    m_cos_acc_    = 0.0;
    samples_      = 0;
    /* Re-baseline per frequency so plant drift between frequencies
     * doesn't bias the demod (which subtracts baseline). */
    baseline_q_   = latestQ().value(joint_index_, 0.0);
    freq_start_ns_= timer().nsecsElapsed();
    /* Re-arm the finish guard — frames are again welcome until the
     * sweep window for THIS frequency expires. */
    freq_finishing_ = false;
}


void BandwidthSweepProbe::onFrame(const QVector<double>& q,
                                    const QVector<double>& /*tau*/,
                                    qint64                 ts_ns)
{
    if (!active_ || freq_index_ < 0
        || freq_index_ >= frequencies_hz_.size()){ return; }
    if (joint_index_ < 0 || joint_index_ >= q.size()){ return; }
    /* Already harvested this frequency — discard frames that arrive
     * during the inter-frequency settle until startNextFrequency_
     * clears the flag. Without this guard, every settle-window
     * frame would re-fire finishCurrentFrequency_, duplicating the
     * gain/phase entry and tripping interp_minus3db's size check. */
    if (freq_finishing_){ return; }

    const double t = double(ts_ns - freq_start_ns_) / 1e9;
    if (t > sweep_secs_){
        /* Window done — record results, then either move to next
         * frequency or schedule the inter-frequency settle. */
        freq_finishing_ = true;
        finishCurrentFrequency_();
        return;
    }

    /* Issue this frame's commanded sample. Same vector shape as
     * incoming q, all other joints held at their current value. */
    QVector<double> cmd = q;
    if (cmd.size() <= joint_index_){ cmd.resize(joint_index_ + 1); }
    const double phase = kTwoPi * current_freq_ * t;
    const double sinp  = std::sin(phase);
    const double cosp  = std::cos(phase);
    cmd[joint_index_] = baseline_q_ + amplitude_rad_ * sinp;
    worker()->setPosition(cmd);

    /* Demod: accumulate measured deviation against the sin/cos of
     * the commanded phase. (2/N) scaling applied at finish. */
    const double dq = q[joint_index_] - baseline_q_;
    m_sin_acc_ += dq * sinp;
    m_cos_acc_ += dq * cosp;
    ++samples_;
}


void BandwidthSweepProbe::finishCurrentFrequency_()
{
    /* Extract magnitude + phase. Guard against zero-sample case
     * (e.g. user aborted mid-sweep). */
    double mag_lin = 0.0;
    double phi_deg = 0.0;
    if (samples_ > 0){
        const double m_sin = (2.0 / samples_) * m_sin_acc_;
        const double m_cos = (2.0 / samples_) * m_cos_acc_;
        const double mag   = std::sqrt(m_sin * m_sin + m_cos * m_cos);
        mag_lin = (amplitude_rad_ > 0.0) ? (mag / amplitude_rad_) : 0.0;
        phi_deg = std::atan2(m_cos, m_sin) * 180.0 / 3.14159265358979323846;
    }
    const double gain_db = (mag_lin > 0.0) ? (20.0 * std::log10(mag_lin))
                                            : -120.0;   /* floor when silent */
    gain_lin_.append(mag_lin);
    gain_db_.append(gain_db);
    phase_deg_.append(phi_deg);

    /* Restore the joint to baseline before the inter-frequency
     * settle so the next frequency starts from rest. */
    QVector<double> restore = latestQ();
    if (restore.size() <= joint_index_){
        restore.resize(joint_index_ + 1);
    }
    restore[joint_index_] = baseline_q_;
    worker()->setPosition(restore);

    QTimer::singleShot(settle_between_ms_, this,
                        [this]{ startNextFrequency_(); });
}


/** Find the -3 dB crossover by linear interpolation in (f, gain_lin)
 *  pairs. -3 dB is gain_lin ≈ 0.7079 (10^(-3/20)). Returns 0 when
 *  every measured gain is above -3 dB (bandwidth > sweep range) and
 *  -1 when nothing exceeds the threshold (controller dead). */
static double interp_minus3db(const QVector<double>& freqs,
                               const QVector<double>& gains_lin)
{
    constexpr double kMinus3dB = 0.7079457843841379;
    if (freqs.size() != gains_lin.size() || freqs.isEmpty()){ return -1.0; }
    /* Walk in ascending frequency; bandwidth is the first crossing
     * from above to below the threshold. */
    for (int i = 1; i < freqs.size(); ++i){
        const double g0 = gains_lin[i - 1];
        const double g1 = gains_lin[i];
        if (g0 >= kMinus3dB && g1 < kMinus3dB){
            const double f0 = freqs[i - 1];
            const double f1 = freqs[i];
            const double t  = (g0 - kMinus3dB) / (g0 - g1);
            return f0 + t * (f1 - f0);
        }
    }
    if (gains_lin.last() >= kMinus3dB){
        return 0.0;   /* still above -3 dB at the top of the sweep */
    }
    return -1.0;       /* never above -3 dB → controller dead */
}


void BandwidthSweepProbe::finalize_()
{
    active_ = false;
    restorePollRate_();

    QJsonObject metrics;
    QJsonArray  f_arr, g_db_arr, ph_arr;
    for (double v : frequencies_hz_){ f_arr.append(v); }
    for (double v : gain_db_)       { g_db_arr.append(v); }
    for (double v : phase_deg_)     { ph_arr.append(v); }
    metrics["frequencies_hz[]"] = f_arr;
    metrics["gain_db[]"]        = g_db_arr;
    metrics["phase_deg[]"]      = ph_arr;
    metrics["joint_index"]      = joint_index_;
    metrics["amplitude_rad"]    = amplitude_rad_;

    const double bw_hz = interp_minus3db(frequencies_hz_, gain_lin_);
    metrics["bandwidth_hz"] = bw_hz;

    /* Worst measured gain — handy for spotting controller dropouts
     * at low frequencies (resonance dips, friction-dominated DOFs). */
    double min_gain_db = gain_db_.isEmpty() ? -120.0 : gain_db_.first();
    for (double v : gain_db_){ min_gain_db = std::min(min_gain_db, v); }
    metrics["min_gain_db"] = min_gain_db;

    /* Neutral descriptive summary — Fork-E1 rules decide PASS/FAIL. */
    QString bw_str = (bw_hz > 0.0)
        ? tr("%1 Hz").arg(bw_hz, 0, 'f', 2)
        : (bw_hz == 0.0
            ? tr("> %1 Hz (still flat at top of sweep)")
                .arg(frequencies_hz_.isEmpty() ? 0.0
                     : frequencies_hz_.last(), 0, 'f', 1)
            : tr("not reached (no frequency above -3 dB)"));

    QString summary = tr("j%1 bandwidth %2 (need ≥ %3 Hz). "
                          "Min gain %4 dB across %5 freq.")
        .arg(joint_index_)
        .arg(bw_str)
        .arg(min_bandwidth_hz_, 0, 'f', 2)
        .arg(min_gain_db, 0, 'f', 1)
        .arg(frequencies_hz_.size());

    emit batchDone(true, metrics, summary);
}


void BandwidthSweepProbe::fail_(const QString& reason)
{
    active_ = false;
    restorePollRate_();
    QJsonObject metrics;
    metrics["error"] = reason;
    emit batchDone(false, metrics,
        tr("FAIL — batch did not run.\n  Reason: %1.").arg(reason));
}


void BandwidthSweepProbe::restorePollRate_()
{
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
}
