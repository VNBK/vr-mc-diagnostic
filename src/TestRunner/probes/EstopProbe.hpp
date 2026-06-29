/**
 * @file    EstopProbe.hpp
 * @brief   DB2-06 — Drive a joint, fire HandWorker::stop(), time how
 *          long until every joint velocity falls below threshold.
 *
 *  params:
 *      joint_index             : int (which joint to drive)
 *      drive_target_rad        : double (where to drive to start motion)
 *      pre_drive_ms            : int   (how long to drive before stop, default 300)
 *      velocity_threshold_rad_s: double (motion-considered-stopped, default 0.05)
 *      confirm_frames          : int   (consecutive sub-threshold frames, default 3)
 *      max_estop_latency_ms    : int   (hard timeout, default 500)
 *      test_poll_rate_hz       : int   (default 1000)
 *
 *  emits:
 *      estop_latency_ms
 *      stop_velocity_rad_s
 *
 *  Safety: hand must be securely mounted; probe drives a joint to a
 *  target then commands stop(). If your hand can't be safely stopped
 *  in motion, leave this test not_implemented.
 */
#pragma once

#include "WorkerThreadProbe.hpp"
#include <QJsonObject>


class EstopProbe : public WorkerThreadProbe
{
    Q_OBJECT
public:
    explicit EstopProbe(HandWorker* worker, QObject* parent = nullptr);

public slots:
    void startBatch(QJsonObject params);

signals:
    void batchDone(bool ok, QJsonObject metrics, QString summary);

protected:
    void onFrame(const QVector<double>& q,
                  const QVector<double>& tau,
                  qint64                 ts_ns) override;

private:
    void issueEstop_();
    void finalize_(double latency_ms, double stop_v);
    void fail_(const QString& reason);
    void restorePollRate_();

    enum class Phase { Idle, Driving, AwaitingStop, Settled };

    bool   active_   = false;
    Phase  phase_    = Phase::Idle;
    int    joint_idx_= 0;
    int    confirm_target_ = 3;
    int    confirm_count_  = 0;
    int    max_lat_ms_ = 500;
    double v_thresh_  = 0.05;
    double last_q_observed_ = 0.0;
    qint64 last_ts_ns_      = 0;
    qint64 t_stop_ns_       = 0;
    int    saved_poll_hz_ = 0;
    bool   poll_hz_pushed_ = false;
};
