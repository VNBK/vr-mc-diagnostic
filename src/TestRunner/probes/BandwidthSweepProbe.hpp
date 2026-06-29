/**
 * @file    BandwidthSweepProbe.hpp
 * @brief   DB1-08 — Closed-loop bandwidth sweep.
 *
 *  Drives one joint with a low-amplitude sinusoid at a list of test
 *  frequencies. For each frequency, demodulates the measured q
 *  against sin/cos of the commanded waveform to extract gain (dB)
 *  and phase (deg). Interpolates the -3 dB point as the closed-loop
 *  bandwidth.
 *
 *  This is the "real" control-bandwidth measurement that pairs with
 *  DB1-07's step-response onset latency — together they bracket the
 *  hand's responsiveness from two complementary directions.
 *
 *  Threading
 *  ---------
 *  Inherits @ref WorkerThreadProbe: lives in HandWorker's thread,
 *  per-frame hook runs DirectConnection-tight. Sinusoidal drive is
 *  issued one sample per onFrame() so command and measurement share
 *  the same clock (no separate drive timer means no aliasing
 *  between cmd-rate and measurement-rate).
 *
 *  Math sketch (per frequency f, sweep duration T)
 *  -----------------------------------------------
 *    cmd(t)  = baseline + A·sin(2π f t)
 *    meas(t) = baseline + A·G(f)·sin(2π f t + φ(f))
 *
 *    M_sin = (2/N) Σ (meas[i]-baseline)·sin(2π f t[i])
 *    M_cos = (2/N) Σ (meas[i]-baseline)·cos(2π f t[i])
 *    G(f)  = sqrt(M_sin² + M_cos²) / A
 *    φ(f)  = atan2(M_cos, M_sin)       (radians, lagging convention)
 *
 *  Integer-cycle integration (N samples covering exactly an integer
 *  number of cycles) avoids spectral leakage.
 */
#pragma once

#include "WorkerThreadProbe.hpp"

#include <QJsonObject>
#include <QVector>


class BandwidthSweepProbe : public WorkerThreadProbe
{
    Q_OBJECT
public:
    explicit BandwidthSweepProbe(HandWorker* worker, QObject* parent = nullptr);

public slots:
    /** @brief Kick off a sweep. Reads from @p params:
     *      joint_index, amplitude_rad, frequencies_hz[],
     *      cycles_per_freq, settle_between_ms, min_bandwidth_hz,
     *      test_poll_rate_hz. */
    void startBatch(QJsonObject params);

signals:
    void batchDone(bool ok, QJsonObject metrics, QString summary);

protected:
    void onFrame(const QVector<double>& q,
                  const QVector<double>& tau,
                  qint64                 ts_ns) override;

private:
    void startNextFrequency_();
    void finishCurrentFrequency_();
    void finalize_();
    void fail_(const QString& reason);
    /** @brief Restore the saved poll rate (idempotent). Called at
     *  every exit path so the GUI always gets its rate back. */
    void restorePollRate_();

    /* Knobs from params. */
    int             joint_index_       = 0;
    double          amplitude_rad_     = 0.0;
    QVector<double> frequencies_hz_;
    int             cycles_per_freq_   = 0;
    int             settle_between_ms_ = 0;
    double          min_bandwidth_hz_  = 0.0;

    /* Run state. */
    bool            active_       = false;
    int             freq_index_   = -1;
    double          current_freq_ = 0.0;
    double          sweep_secs_   = 0.0;        /**< cycles_per_freq / f */
    qint64          freq_start_ns_= 0;
    double          baseline_q_   = 0.0;
    /** @brief Set true the moment the current frequency's sweep
     *  window elapses; cleared in startNextFrequency_. Without it,
     *  every frame arriving during the inter-frequency settle would
     *  re-fire finishCurrentFrequency_, appending the same gain/phase
     *  dozens of times and breaking the interp_minus3db size check. */
    bool            freq_finishing_ = false;

    /* Per-frequency demod accumulators. */
    double          m_sin_acc_    = 0.0;
    double          m_cos_acc_    = 0.0;
    int             samples_      = 0;

    /* Results — one entry per frequency, in test order. */
    QVector<double> gain_db_;
    QVector<double> gain_lin_;          /**< |G(f)| linear, kept for -3 dB interp */
    QVector<double> phase_deg_;

    /* Poll-rate stash — same pattern as RoundTripProbe. */
    int             saved_poll_hz_  = 0;
    bool            poll_hz_pushed_ = false;
};
