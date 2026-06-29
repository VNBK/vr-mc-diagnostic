/*
 * SoftLimitProbe.cpp — see header.
 */
#include "SoftLimitProbe.hpp"
#include "../../HandWorker.hpp"

#include <QJsonArray>
#include <QTimer>
#include <algorithm>
#include <cmath>


SoftLimitProbe::SoftLimitProbe(HandWorker* worker, QObject* parent)
    : WorkerThreadProbe(worker, parent) {}


void SoftLimitProbe::startBatch(QJsonObject params)
{
    if (active_){ return; }
    if (!worker() || !worker()->isConnected()){
        fail_(tr("hand not connected")); return;
    }
    if (latestQ().isEmpty()){
        fail_(tr("no telemetry observed yet")); return;
    }

    joint_indices_.clear();
    for (const auto& v : params.value("joint_indices").toArray()){
        joint_indices_.append(v.toInt());
    }
    hi_.clear();
    for (const auto& v : params.value("hi").toArray()){
        hi_.append(v.toDouble());
    }
    if (joint_indices_.isEmpty()){
        fail_(tr("joint_indices is empty")); return;
    }
    if (hi_.isEmpty()){
        fail_(tr("hi empty — URDF didn't load (or has no <limit> "
                 "blocks). Check the hand's URDF file.")); return;
    }
    /* hi may arrive aligned with joint_indices_ (literal JSON) or
     * URDF-length (via $urdf.joint_hi). Normalise to the aligned
     * shape so stepNext_ stays uniform. */
    if (hi_.size() != joint_indices_.size()){
        int max_idx = 0;
        for (int j : joint_indices_){ max_idx = std::max(max_idx, j); }
        if (hi_.size() <= max_idx){
            fail_(tr("hi too short (got %1, need >=%2 for the largest "
                     "joint_index)")
                     .arg(hi_.size()).arg(max_idx + 1));
            return;
        }
        QVector<double> tmp;
        for (int j : joint_indices_){ tmp.append(hi_[j]); }
        hi_ = tmp;
    }

    epsilon_    = params.value("epsilon_rad").toDouble(0.05);
    slop_       = params.value("small_slop_rad").toDouble(0.01);
    settle_ms_  = std::max(100, params.value("settle_ms").toInt(600));
    const int hz = std::max(1, params.value("test_poll_rate_hz").toInt(1000));
    saved_poll_hz_ = worker()->currentPollRate();
    if (hz != saved_poll_hz_){
        worker()->setPollRate(hz);
        poll_hz_pushed_ = true;
    }

    /* Snapshot baseline for restore-after. */
    baseline_q_.clear();
    for (int j : joint_indices_){
        baseline_q_.append(latestQ().value(j, 0.0));
    }
    attempted_.fill(0.0, joint_indices_.size());
    reached_  .fill(0.0, joint_indices_.size());
    overshoots_.fill(0.0, joint_indices_.size());
    clamped_ok_.fill(0,  joint_indices_.size());

    active_ = true;
    cursor_ = 0;
    stepNext_();
}


void SoftLimitProbe::stepNext_()
{
    if (!active_){ return; }
    if (cursor_ >= joint_indices_.size()){
        finalize_();
        return;
    }
    const int j = joint_indices_[cursor_];
    const double target = hi_[cursor_] + epsilon_;
    attempted_[cursor_] = target;
    QVector<double> cmd = latestQ();
    if (cmd.size() <= j){ cmd.resize(j + 1); }
    cmd[j] = target;
    worker()->setPosition(cmd);

    QTimer::singleShot(settle_ms_, this, [this, j]{
        if (!active_){ return; }
        const double reached = latestQ().value(j, 0.0);
        reached_  [cursor_] = reached;
        overshoots_[cursor_] = reached - hi_[cursor_];
        clamped_ok_[cursor_] =
            (overshoots_[cursor_] <= slop_) ? 1 : 0;

        /* Restore baseline before moving to the next joint. */
        QVector<double> restore = latestQ();
        if (restore.size() <= j){ restore.resize(j + 1); }
        restore[j] = baseline_q_[cursor_];
        worker()->setPosition(restore);

        QTimer::singleShot(settle_ms_, this, [this]{
            if (!active_){ return; }
            ++cursor_;
            stepNext_();
        });
    });
}


void SoftLimitProbe::finalize_()
{
    active_ = false;
    restorePollRate_();
    QJsonArray att, reach, over, ok;
    double max_over = 0.0;
    int    ok_count = 0;
    for (int i = 0; i < joint_indices_.size(); ++i){
        att  .append(attempted_[i]);
        reach.append(reached_[i]);
        over .append(overshoots_[i]);
        ok   .append(clamped_ok_[i]);
        max_over = std::max(max_over, overshoots_[i]);
        if (clamped_ok_[i]){ ++ok_count; }
    }
    QJsonObject m;
    m["attempted[]"]    = att;
    m["reached[]"]      = reach;
    m["overshoots[]"]   = over;
    m["clamped_ok[]"]   = ok;
    m["max_overshoot"]  = max_over;
    m["joints_clamped"] = ok_count;
    m["joints_total"]   = joint_indices_.size();
    emit batchDone(true, m,
        tr("%1/%2 joints clamped; max overshoot %3 rad")
            .arg(ok_count).arg(joint_indices_.size())
            .arg(max_over, 0, 'f', 4));
}


void SoftLimitProbe::fail_(const QString& reason)
{
    active_ = false;
    restorePollRate_();
    QJsonObject m; m["error"] = reason;
    emit batchDone(false, m,
        tr("FAIL — soft-limit probe did not run.\n  Reason: %1.").arg(reason));
}


void SoftLimitProbe::restorePollRate_()
{
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
}
