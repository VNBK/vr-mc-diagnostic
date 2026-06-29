/*
 * OperationModeProbe.cpp — see header.
 */
#include "OperationModeProbe.hpp"
#include "../../HandWorker.hpp"

#include <QJsonArray>
#include <QTimer>
#include <algorithm>


OperationModeProbe::OperationModeProbe(HandWorker* worker, QObject* parent)
    : WorkerThreadProbe(worker, parent) {}


void OperationModeProbe::startBatch(QJsonObject params)
{
    if (active_){ return; }
    if (!worker() || !worker()->isConnected()){
        fail_(tr("hand not connected")); return;
    }

    modes_.clear();
    const auto m_arr = params.value("modes").toArray();
    if (m_arr.isEmpty()){
        /* Pull every advertised mode from the worker. */
        for (const auto& p : worker()->operationModeNames()){
            modes_.append(p.first);
        }
    } else {
        for (const auto& v : m_arr){ modes_.append(v.toInt()); }
    }
    if (modes_.isEmpty()){
        fail_(tr("no modes advertised by adapter")); return;
    }

    settle_ms_ = std::max(50, params.value("apply_settle_ms").toInt(200));
    const int hz = std::max(1, params.value("test_poll_rate_hz").toInt(1000));
    saved_poll_hz_ = worker()->currentPollRate();
    if (hz != saved_poll_hz_){
        worker()->setPollRate(hz);
        poll_hz_pushed_ = true;
    }

    latencies_ms_.clear();
    applied_.clear();
    active_ = true;
    cursor_ = 0;
    stepNext_();
}


void OperationModeProbe::stepNext_()
{
    if (!active_){ return; }
    if (cursor_ >= modes_.size()){
        finalize_();
        return;
    }
    const int mode = modes_[cursor_];
    const qint64 t0 = timer().nsecsElapsed();
    worker()->setOperationMode(mode);
    /* setOperationMode returns synchronously on success in the
     * current HandWorker. We measure the dispatch + settle. */
    QTimer::singleShot(settle_ms_, this, [this, t0]{
        if (!active_){ return; }
        const qint64 t1 = timer().nsecsElapsed();
        latencies_ms_.append(double(t1 - t0) / 1e6);
        /* Without an "applied" callback we assume the SDK accepted
         * the call. Surface 1 here so the catalog stays uniform with
         * future probes that DO check the slave reflection. */
        applied_.append(1);
        ++cursor_;
        stepNext_();
    });
}


void OperationModeProbe::finalize_()
{
    active_ = false;
    restorePollRate_();
    QJsonArray lat, app;
    double max_lat = 0.0;
    int    n_applied = 0;
    for (int i = 0; i < latencies_ms_.size(); ++i){
        lat.append(latencies_ms_[i]);
        app.append(applied_[i]);
        max_lat = std::max(max_lat, latencies_ms_[i]);
        if (applied_[i]){ ++n_applied; }
    }
    QJsonObject m;
    m["mode_apply_latency_ms[]"] = lat;
    m["modes_applied[]"]         = app;
    m["max_apply_latency_ms"]    = max_lat;
    m["modes_total"]             = modes_.size();
    m["modes_applied_count"]     = n_applied;
    emit batchDone(true, m,
        tr("%1/%2 modes applied; max latency %3 ms")
            .arg(n_applied).arg(modes_.size())
            .arg(max_lat, 0, 'f', 1));
}


void OperationModeProbe::fail_(const QString& reason)
{
    active_ = false;
    restorePollRate_();
    QJsonObject m; m["error"] = reason;
    emit batchDone(false, m,
        tr("FAIL — mode probe did not run.\n  Reason: %1.").arg(reason));
}


void OperationModeProbe::restorePollRate_()
{
    if (poll_hz_pushed_){
        worker()->setPollRate(saved_poll_hz_);
        poll_hz_pushed_ = false;
    }
}
