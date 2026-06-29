/*
 * EstopProbe.cpp — see header.
 */
#include "EstopProbe.hpp"
#include "../../HandWorker.hpp"

#include <QTimer>
#include <algorithm>
#include <cmath>


EstopProbe::EstopProbe(HandWorker* worker, QObject* parent)
    : WorkerThreadProbe(worker, parent) {}


void EstopProbe::startBatch(QJsonObject params)
{
    if (active_){ return; }
    if (!worker() || !worker()->isConnected()){
        fail_(tr("hand not connected")); return;
    }
    if (latestQ().isEmpty()){
        fail_(tr("no telemetry observed yet")); return;
    }

    joint_idx_      = params.value("joint_index").toInt(0);
    const double drive_target = params.value("drive_target_rad").toDouble(0.5);
    const int    pre_drive_ms = std::max(50,
                                  params.value("pre_drive_ms").toInt(300));
    v_thresh_       = params.value("velocity_threshold_rad_s").toDouble(0.05);
    confirm_target_ = std::max(1, params.value("confirm_frames").toInt(3));
    max_lat_ms_     = std::max(50, params.value("max_estop_latency_ms").toInt(500));
    const int hz    = std::max(1, params.value("test_poll_rate_hz").toInt(1000));
    saved_poll_hz_  = worker()->currentPollRate();
    if (hz != saved_poll_hz_){
        worker()->setPollRate(hz);
        poll_hz_pushed_ = true;
    }

    if (joint_idx_ < 0 || joint_idx_ >= latestQ().size()){
        fail_(tr("joint_index %1 out of range").arg(joint_idx_)); return;
    }
    active_         = true;
    phase_          = Phase::Driving;
    confirm_count_  = 0;
    last_q_observed_= latestQ().value(joint_idx_, 0.0);
    last_ts_ns_     = timer().nsecsElapsed();

    /* Drive the joint to start it moving. */
    QVector<double> cmd = latestQ();
    if (cmd.size() <= joint_idx_){ cmd.resize(joint_idx_ + 1); }
    cmd[joint_idx_] = drive_target;
    worker()->setPosition(cmd);

    /* After pre_drive_ms the actuator should be moving; fire stop. */
    QTimer::singleShot(pre_drive_ms, this, [this]{
        if (!active_){ return; }
        issueEstop_();
    });
    /* Hard timeout — fail the trial if velocity never drops. */
    QTimer::singleShot(pre_drive_ms + max_lat_ms_, this, [this]{
        if (!active_ || phase_ == Phase::Settled){ return; }
        finalize_(max_lat_ms_, std::nan(""));
    });
}


void EstopProbe::issueEstop_()
{
    if (!active_){ return; }
    phase_     = Phase::AwaitingStop;
    t_stop_ns_ = timer().nsecsElapsed();
    worker()->stop();
}


void EstopProbe::onFrame(const QVector<double>& q,
                          const QVector<double>& /*tau*/,
                          qint64                 ts_ns)
{
    if (!active_ || joint_idx_ >= q.size()){ return; }
    if (phase_ != Phase::AwaitingStop){
        last_q_observed_ = q[joint_idx_];
        last_ts_ns_      = ts_ns;
        return;
    }
    /* Compute instantaneous velocity from the previous frame. */
    const double dt = double(ts_ns - last_ts_ns_) / 1e9;
    if (dt <= 0.0){
        last_q_observed_ = q[joint_idx_];
        last_ts_ns_      = ts_ns;
        return;
    }
    const double v = std::abs(q[joint_idx_] - last_q_observed_) / dt;
    last_q_observed_ = q[joint_idx_];
    last_ts_ns_      = ts_ns;
    if (v < v_thresh_){
        if (++confirm_count_ >= confirm_target_){
            phase_ = Phase::Settled;
            const double lat = double(ts_ns - t_stop_ns_) / 1e6;
            finalize_(lat, v);
        }
    } else {
        confirm_count_ = 0;
    }
}


void EstopProbe::finalize_(double latency_ms, double stop_v)
{
    active_ = false;
    restorePollRate_();
    QJsonObject m;
    m["estop_latency_ms"]    = latency_ms;
    m["stop_velocity_rad_s"] = std::isfinite(stop_v) ? stop_v : -1.0;
    emit batchDone(true, m,
        tr("j%1: estop latency %2 ms (cap %3 ms)")
            .arg(joint_idx_)
            .arg(latency_ms, 0, 'f', 1)
            .arg(max_lat_ms_));
}


void EstopProbe::fail_(const QString& reason)
{
    active_ = false;
    restorePollRate_();
    QJsonObject m; m["error"] = reason;
    emit batchDone(false, m,
        tr("FAIL — estop probe did not run.\n  Reason: %1.").arg(reason));
}


void EstopProbe::restorePollRate_()
{
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
}
