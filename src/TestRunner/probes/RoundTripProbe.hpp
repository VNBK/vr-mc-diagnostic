/**
 * @file    RoundTripProbe.hpp
 * @brief   Worker-thread probe implementing DB1-07 command-to-effect
 *          round-trip measurement.
 *
 *  Lives in HandWorker's thread (inherits @ref WorkerThreadProbe).
 *  Stamps T0 right before the SDK setPosition call, captures T1 on
 *  the very next stateUpdated emission, both via the same
 *  QElapsedTimer — sub-µs resolution, zero Qt-queue dispatch.
 *
 *  Lifecycle / threading
 *  ---------------------
 *  - GUI kicks off a batch via QMetaObject::invokeMethod with
 *    Qt::QueuedConnection on @ref startBatch (one-way, no return).
 *  - Probe owns its trial state machine inside the worker thread —
 *    onFrame() handles ack detection synchronously inside emit().
 *  - When the batch completes (or aborts), @ref batchDone fires
 *    with the metrics object; Qt auto-routes to GUI as a
 *    QueuedConnection. The return hop is OUTSIDE the measurement
 *    window, so its dispatch jitter doesn't matter.
 */
#pragma once

#include "WorkerThreadProbe.hpp"

#include <QJsonObject>
#include <QVector>


class RoundTripProbe : public WorkerThreadProbe
{
    Q_OBJECT
public:
    explicit RoundTripProbe(HandWorker* worker, QObject* parent = nullptr);

public slots:
    /** @brief Kick off a fresh batch of round-trip trials. Reads
     *  every knob from @p params (joint_index, step_amplitude_rad,
     *  ack_tolerance_rad, max_round_trip_ms, trials,
     *  settle_between_ms, min_motion_per_trial_rad). Idempotent —
     *  calling while a batch is already running is a no-op. */
    void startBatch(QJsonObject params);

signals:
    /** @brief Emitted once per batch when the state machine drains.
     *  @p ok is false only when prerequisites (worker not connected,
     *  no telemetry observed, joint index out of range) blocked the
     *  batch; in that case @p metrics["error"] carries the reason. */
    void batchDone(bool ok, QJsonObject metrics, QString summary);

protected:
    void onFrame(const QVector<double>& q,
                  const QVector<double>& tau,
                  qint64                 ts_ns) override;

private:
    /** @brief Issue the trial-N+1 step. Runs in worker thread. */
    void runNextTrial_();
    /** @brief Wrap up, compute aggregates, emit batchDone. */
    void finalize_();
    /** @brief Common path for abort with a human-readable reason. */
    void fail_(const QString& reason);

    bool   active_           = false;
    /* Knobs (loaded from params on startBatch). */
    int    joint_index_      = 0;
    double step_amplitude_   = 0.0;
    double ack_tol_          = 0.0;
    int    max_round_trip_ms_= 0;
    int    trials_target_    = 0;
    int    settle_between_ms_= 0;
    /* Per-trial state. */
    double baseline_q_       = 0.0;
    double target_q_         = 0.0;
    qint64 trial_t0_ns_      = 0;
    bool   awaiting_ack_     = false;
    int    trials_done_      = 0;
    int    trials_acked_     = 0;
    QVector<double> round_trip_ms_;   /**< per-acked trial */
    QVector<int>    timeouts_;        /**< trial indices that hit cap */
    /** @brief Poll-rate stash. EtherCAT can sustain a 1 kHz SDK
     *  poll, but the GUI runs at 25 Hz to keep heavy URDFs smooth.
     *  Probe bumps the rate on startBatch and restores in
     *  finalize_/fail_ so the rest of the GUI gets its rate back. */
    int             saved_poll_hz_ = 0;
    bool            poll_hz_pushed_ = false;
};
