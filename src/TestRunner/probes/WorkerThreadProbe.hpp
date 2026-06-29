/**
 * @file    WorkerThreadProbe.hpp
 * @brief   Base class for test probes that need TIGHT, worker-thread
 *          timing on HandWorker's stateUpdated stream.
 *
 *  Motivation
 *  ----------
 *  The default TestRunner pattern (slots on the GUI thread connected
 *  via Qt::AutoConnection / QueuedConnection) is fine for window-
 *  aggregate tests but adds 1-5 ms of Qt-queue jitter to anything
 *  that times a single event. WorkerThreadProbe eliminates that by:
 *      1. Living in HandWorker's thread (moveToThread).
 *      2. Connecting to stateUpdated with Qt::DirectConnection — slot
 *         runs synchronously inside the emit(), no queue.
 *      3. Calling HandWorker slots (setPosition, …) as plain method
 *         calls — same thread, no QueuedConnection.
 *
 *  Resolution: sub-µs timestamps via QElapsedTimer::nsecsElapsed().
 *
 *  Reuse pattern
 *  -------------
 *  Subclass and override @ref onFrame. Use @ref timer to stamp T0/T1,
 *  @ref worker to issue commands, @ref isOnWorkerThread to guard
 *  assertions. Emit your own result signal back to the GUI — Qt's
 *  auto-connection becomes a QueuedConnection across threads, which
 *  is correct (and OUTSIDE the measurement window, so jitter on the
 *  return hop doesn't matter).
 *
 *  Existing subclasses
 *  -------------------
 *    - RoundTripProbe — DB1-07 command-to-effect latency.
 *  Future tests that need tight per-event timing (bandwidth sweep,
 *  step-response analysis, settle-time measurement) follow the same
 *  pattern.
 */
#pragma once

#include <QElapsedTimer>
#include <QObject>
#include <QVector>

class HandWorker;


class WorkerThreadProbe : public QObject
{
    Q_OBJECT
public:
    /** @param worker  must outlive the probe. Probe moves to
     *                 worker->thread() and connects to its
     *                 stateUpdated signal with Qt::DirectConnection. */
    explicit WorkerThreadProbe(HandWorker* worker, QObject* parent = nullptr);
    ~WorkerThreadProbe() override;

protected:
    /** @brief Per-frame hook — fires inside HandWorker's emit, in
     *  the worker thread. @p ts_ns is QElapsedTimer::nsecsElapsed()
     *  captured at the top of the slot (before any work the subclass
     *  does, for tighter timestamps). Default: no-op. */
    virtual void onFrame(const QVector<double>& q,
                          const QVector<double>& tau,
                          qint64                 ts_ns);

    /** @brief Subclass-accessible timer. Started in the constructor;
     *  same monotonic origin for every onFrame() and any T0/T1 the
     *  subclass stamps. Read via @c nsecsElapsed(). */
    const QElapsedTimer& timer() const { return timer_; }

    /** @brief Underlying worker — used to call SDK-facing slots
     *  (e.g. setPosition) directly. Same thread as @c this, so plain
     *  method calls are safe and queue-free. */
    HandWorker* worker() const { return worker_; }

    /** @brief Latest q observed on the bus. Snapshot — copy if you
     *  need to hold it across onFrame() calls. */
    const QVector<double>& latestQ() const { return last_q_; }

    /** @brief Latest τ observed on the bus. */
    const QVector<double>& latestTau() const { return last_tau_; }

    /** @brief nsecsElapsed() at the moment the last frame's slot was
     *  entered. Useful when you need the previous frame's timestamp
     *  outside onFrame(). */
    qint64 latestFrameNs() const { return last_frame_ns_; }

    /** @brief Diagnostic — true if the calling code is on the
     *  worker's thread. Helps catch ownership bugs. */
    bool isOnWorkerThread() const;

private slots:
    /* Q_OBJECT private slot — DirectConnection target from
     * HandWorker::stateUpdated. Stamps @c last_frame_ns_ at entry,
     * stashes q+tau, then calls the virtual onFrame() hook. */
    void onStateUpdatedInternal_(QVector<double> q, QVector<double> tau);

private:
    HandWorker*       worker_       = nullptr;
    QElapsedTimer     timer_;
    QVector<double>   last_q_;
    QVector<double>   last_tau_;
    qint64            last_frame_ns_ = 0;
};
