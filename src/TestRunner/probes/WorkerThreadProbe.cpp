/*
 * WorkerThreadProbe.cpp — see header for purpose.
 */
#include "WorkerThreadProbe.hpp"
#include "../../HandWorker.hpp"

#include <QThread>


WorkerThreadProbe::WorkerThreadProbe(HandWorker* worker, QObject* parent)
    : QObject(parent), worker_(worker)
{
    timer_.start();
    if (!worker_){ return; }

    /* Move BEFORE connecting so the slot binding sees the right
     * thread affinity. After this call, every slot on this probe
     * runs in HandWorker's thread. */
    moveToThread(worker_->thread());

    /* Qt::DirectConnection: slot fires synchronously inside emit(),
     * in whichever thread emit happens to run on. Since both
     * HandWorker and this probe are now on the same thread,
     * "current emit thread" == "probe's affinity thread" — clean. */
    connect(worker_, &HandWorker::stateUpdated,
            this,    &WorkerThreadProbe::onStateUpdatedInternal_,
            Qt::DirectConnection);
}


WorkerThreadProbe::~WorkerThreadProbe() = default;


void WorkerThreadProbe::onFrame(const QVector<double>& /*q*/,
                                 const QVector<double>& /*tau*/,
                                 qint64                 /*ts_ns*/)
{
    /* Default: no-op. Subclasses override. */
}


bool WorkerThreadProbe::isOnWorkerThread() const
{
    return worker_ && (QThread::currentThread() == worker_->thread());
}


void WorkerThreadProbe::onStateUpdatedInternal_(QVector<double> q,
                                                  QVector<double> tau)
{
    /* Stamp FIRST — closest possible to the SDK-callback moment.
     * Anything after this point (move, assignment, virtual dispatch)
     * adds to the apparent latency, so we anchor T1 right here. */
    last_frame_ns_ = timer_.nsecsElapsed();
    last_q_   = q;
    last_tau_ = tau;
    onFrame(last_q_, last_tau_, last_frame_ns_);
}
