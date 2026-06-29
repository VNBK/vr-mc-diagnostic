/*
 * HandWorkerCompat.cpp — see header for the rationale.
 */

#include "HandWorkerCompat.hpp"
#include "../MasterWorker.hpp"


HandWorker::HandWorker(vrmc::MasterWorker* mc, QObject* parent)
    : QObject(parent)
{
    if (!mc){ return; }
    /* Re-shape MasterWorker::connected(int slaveCount) into the
     * (dof, joint_names) tuple the Hand TestRunner expects. We don't
     * have real slave names — name them slave_<i> so the suite tree
     * still shows something the operator can correlate. */
    connect(mc, &vrmc::MasterWorker::connected, this, [this](int n){
        dof_       = n;
        joint_names_.clear();
        joint_names_.reserve(n);
        for (int i = 0; i < n; ++i){
            joint_names_.push_back(QString("slave_%1").arg(i));
        }
        connected_ = true;
        emit connected(dof_, joint_names_);
    });
    connect(mc, &vrmc::MasterWorker::disconnected, this, [this]{
        connected_ = false;
        dof_       = 0;
        joint_names_.clear();
        emit disconnected();
    });
}
