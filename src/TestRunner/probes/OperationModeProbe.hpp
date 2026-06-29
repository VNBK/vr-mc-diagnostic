/**
 * @file    OperationModeProbe.hpp
 * @brief   DB2-04 — Cycle through advertised operation modes.
 *
 *  params:
 *      modes               : int[]   (modes to apply, default = all advertised)
 *      apply_settle_ms     : int   (default 200 — wait between mode + verify)
 *      test_poll_rate_hz   : int   (default 1000)
 *
 *  emits:
 *      mode_apply_latency_ms[]   : per-mode wall-clock from setOperationMode
 *      modes_applied[]           : 1 if SDK acknowledged, 0 otherwise
 *      max_apply_latency_ms
 *
 *  Caveat: HandWorker::setOperationMode is fire-and-forget today —
 *  there's no "mode applied" callback we can latch onto. So the
 *  probe measures the time it takes the SDK call to return + one
 *  settle period; it can't catch a slave that silently rejects.
 *  Improvement path: surface an operationModeChanged() signal in
 *  HandWorker and switch the probe to wait on it.
 */
#pragma once

#include "WorkerThreadProbe.hpp"
#include <QJsonObject>


class OperationModeProbe : public WorkerThreadProbe
{
    Q_OBJECT
public:
    explicit OperationModeProbe(HandWorker* worker, QObject* parent = nullptr);

public slots:
    void startBatch(QJsonObject params);

signals:
    void batchDone(bool ok, QJsonObject metrics, QString summary);

private:
    void stepNext_();
    void finalize_();
    void fail_(const QString& reason);
    void restorePollRate_();

    bool         active_ = false;
    int          cursor_ = 0;
    int          settle_ms_ = 200;
    QVector<int> modes_;
    QVector<double> latencies_ms_;
    QVector<int>    applied_;
    int          saved_poll_hz_ = 0;
    bool         poll_hz_pushed_ = false;
};
