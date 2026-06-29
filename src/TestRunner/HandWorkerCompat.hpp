/**
 * @file    HandWorkerCompat.hpp
 * @brief   Thin shim that lets the cloned vr_hand_diagnostic TestRunner
 *          compile inside vr_mc_diagnostic without rewriting it.
 *
 * The Hand TestRunner is wired against `class HandWorker` — a Qt object
 * whose signals + accessors describe a robotic hand (DOF, joint names,
 * URDF joint limits, …). The Motor Control diagnostic doesn't have one
 * of those; its live SDK glue is @ref vrmc::MasterWorker, which speaks
 * slaves / snapshots / CAN. Rather than fork ~6 000 lines of TestRunner
 * code, this header declares a HandWorker-shaped class **in this
 * diagnostic's own translation unit** so every `#include
 * "../HandWorker.hpp"` in the cloned TestRunner resolves against it.
 *
 * The shim:
 *   - Forwards @ref vrmc::MasterWorker::connected → re-emits as
 *     `connected(int dof, QStringList joint_names)` where dof = slave
 *     count and joint_names = ["slave_0", "slave_1", …]. That's enough
 *     for the TestRunner's snapshot-capture path to land typed values.
 *   - Forwards @ref vrmc::MasterWorker::disconnected verbatim.
 *   - Returns empty data for URDF joint limits / kinematics — none
 *     exist on the MC side. Tests that depend on them will simply
 *     report "(not applicable)" in their actual-result cell.
 *
 * Probes (which call HandWorker::setPosition / setForce / …) are NOT
 * built in v1 — they live on disk under TestRunner/probes/ as
 * reference templates only. When MC-flavoured probes are written they
 * should target MasterWorker directly instead of growing this shim.
 */

#pragma once

#include <QObject>
#include <QStringList>
#include <QVector>

namespace vrmc { class MasterWorker; }


class HandWorker : public QObject
{
    Q_OBJECT
public:
    explicit HandWorker(vrmc::MasterWorker* mc, QObject* parent = nullptr);

    /* HandWorker API surface the TestRunner reads synchronously. */
    int             currentDof()            const { return dof_; }
    QStringList     currentJointNames()     const { return joint_names_; }
    QVector<double> currentJointLimitsLo()  const { return {}; }
    QVector<double> currentJointLimitsHi()  const { return {}; }
    bool            isConnected()           const { return connected_; }

signals:
    /* HandWorker signals the TestRunner connects to. The cached
     * MasterWorker rebroadcasts via these so the cloned code wires
     * up unchanged. */
    void connected      (int dof, QStringList joint_names);
    void disconnected   ();
    void stateUpdated   (QVector<double> q, QVector<double> tau);
    void currentUpdated (QVector<double> amps);
    void temperatureUpdated(QVector<double> celsius);
    void errorCodesUpdated (QVector<int> codes);
    void urdfJointLimits   (QStringList names,
                              QVector<double> lo,
                              QVector<double> hi);

private:
    int         dof_       = 0;
    QStringList joint_names_;
    bool        connected_ = false;
};
