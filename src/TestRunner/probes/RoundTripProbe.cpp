/*
 * RoundTripProbe.cpp — see header for purpose.
 */
#include "RoundTripProbe.hpp"
#include "../../HandWorker.hpp"

#include <QJsonArray>
#include <QTimer>
#include <algorithm>
#include <cmath>


RoundTripProbe::RoundTripProbe(HandWorker* worker, QObject* parent)
    : WorkerThreadProbe(worker, parent)
{
}


void RoundTripProbe::startBatch(QJsonObject params)
{
    if (active_){ return; }                 /* re-entry guard */
    if (!worker() || !worker()->isConnected()){
        fail_(tr("hand not connected"));
        return;
    }
    if (latestQ().isEmpty()){
        fail_(tr("no telemetry observed yet — open later or warm the bus first"));
        return;
    }

    joint_index_       = params.value("joint_index").toInt(0);
    step_amplitude_    = params.value("step_amplitude_rad").toDouble(0.05);
    ack_tol_           = params.value("ack_tolerance_rad").toDouble(0.005);
    max_round_trip_ms_ = std::max(5,
                          params.value("max_round_trip_ms").toInt(100));
    trials_target_     = std::max(1, params.value("trials").toInt(5));
    settle_between_ms_ = std::max(0,
                          params.value("settle_between_ms").toInt(200));

    if (joint_index_ < 0 || joint_index_ >= latestQ().size()){
        fail_(tr("joint_index %1 out of range").arg(joint_index_));
        return;
    }

    /* Push the SDK poll rate up for the duration of the batch — the
     * 25 Hz GUI default quantises every measurement to ±40 ms, which
     * dominates the bus + plant round-trip. EtherCAT can sustain
     * 1 kHz at the SDK; the param lets non-EtherCAT hands cap it.
     * Saved + restored in finalize_/fail_ so the rest of the GUI
     * gets its rate back when the batch ends, however it ends. */
    const int test_hz = std::max(1,
                          params.value("test_poll_rate_hz").toInt(1000));
    saved_poll_hz_   = worker()->currentPollRate();
    if (test_hz != saved_poll_hz_){
        worker()->setPollRate(test_hz);
        poll_hz_pushed_ = true;
    }

    /* Reset accumulators + state machine. */
    trials_done_   = 0;
    trials_acked_  = 0;
    round_trip_ms_.clear();
    timeouts_.clear();
    active_ = true;
    runNextTrial_();
}


void RoundTripProbe::runNextTrial_()
{
    if (!active_){ return; }
    if (trials_done_ >= trials_target_){
        finalize_();
        return;
    }
    /* Re-baseline per trial — plant may have drifted after the last
     * restore. Build the full-dof command vector: target on tested
     * joint, hold every other joint at its current value so we don't
     * yank an unrelated finger as a side effect. */
    QVector<double> cmd = latestQ();
    if (cmd.size() <= joint_index_){ cmd.resize(joint_index_ + 1); }
    baseline_q_ = latestQ().value(joint_index_, 0.0);
    target_q_   = baseline_q_ + step_amplitude_;
    cmd[joint_index_] = target_q_;

    awaiting_ack_ = true;
    /* Stamp T0 as close as possible to the SDK call. Worker is on
     * this thread, so setPosition is just a plain method call — no
     * QueuedConnection in the measurement window. */
    trial_t0_ns_  = timer().nsecsElapsed();
    worker()->setPosition(cmd);

    /* Hard per-trial timeout — fires in worker thread (single-shot
     * with `this` as receiver runs in receiver's affinity thread). */
    QTimer::singleShot(max_round_trip_ms_, this, [this]{
        if (!active_){ return; }
        if (awaiting_ack_){
            timeouts_.append(trials_done_);
            awaiting_ack_ = false;
        }
        ++trials_done_;
        /* Restore baseline before next trial so fingers don't ratchet. */
        QVector<double> restore = latestQ();
        if (restore.size() <= joint_index_){ restore.resize(joint_index_ + 1); }
        restore[joint_index_] = baseline_q_;
        worker()->setPosition(restore);
        QTimer::singleShot(settle_between_ms_, this,
                            [this]{ runNextTrial_(); });
    });
}


void RoundTripProbe::onFrame(const QVector<double>& q,
                              const QVector<double>& /*tau*/,
                              qint64                 ts_ns)
{
    /* Runs synchronously inside HandWorker::stateUpdated emit() —
     * sub-µs from the SDK callback. */
    if (!active_ || !awaiting_ack_){ return; }
    if (joint_index_ < 0 || joint_index_ >= q.size()){ return; }
    const double dq = std::abs(q[joint_index_] - baseline_q_);
    if (dq < ack_tol_){ return; }

    const double rt_ms = double(ts_ns - trial_t0_ns_) / 1e6;
    round_trip_ms_.append(rt_ms);
    ++trials_acked_;
    awaiting_ack_ = false;
}


void RoundTripProbe::finalize_()
{
    active_ = false;
    /* Restore poll rate before anything else — even if the result
     * emit somehow throws, the GUI's rate is back to normal. */
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
    QJsonObject metrics;
    QJsonArray per_trial;
    for (double v : round_trip_ms_){ per_trial.append(v); }
    metrics["round_trip_ms[]"] = per_trial;
    metrics["trials_acked"]    = trials_acked_;
    metrics["trials_total"]    = trials_target_;
    metrics["timeouts"]        = timeouts_.size();
    metrics["joint_index"]     = joint_index_;

    double mean = 0.0;
    for (double v : round_trip_ms_){ mean += v; }
    if (!round_trip_ms_.isEmpty()){ mean /= round_trip_ms_.size(); }
    auto sorted = round_trip_ms_;
    std::sort(sorted.begin(), sorted.end());
    const double p99 = sorted.isEmpty() ? 0.0
        : sorted[(int)(sorted.size() * 0.99)];
    const double maxv = sorted.isEmpty() ? 0.0 : sorted.last();
    metrics["round_trip_mean_ms"] = mean;
    metrics["round_trip_p99_ms"]  = p99;

    /* Neutral descriptive summary — Fork-E1 rules own PASS/FAIL +
     * the per-rule failure reasons. We always report ok=true here;
     * the runner overrides downstream when rules disagree. */
    QString summary = tr("j%1 %2/%3 acked. Mean %4 ms, p99 %5 ms, "
                          "max %6 ms (cap %7 ms).")
        .arg(joint_index_)
        .arg(trials_acked_).arg(trials_target_)
        .arg(mean, 0, 'f', 2)
        .arg(p99,  0, 'f', 2)
        .arg(maxv, 0, 'f', 2)
        .arg(max_round_trip_ms_);

    emit batchDone(true, metrics, summary);
}


void RoundTripProbe::fail_(const QString& reason)
{
    active_ = false;
    /* Same restore as finalize_ — covers the "fail before any
     * trial ran" path too, where saved_poll_hz_ may still be set
     * but poll_hz_pushed_ stays false. */
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
    QJsonObject metrics;
    metrics["error"] = reason;
    emit batchDone(false, metrics,
        tr("FAIL — batch did not run.\n  Reason: %1.").arg(reason));
}
