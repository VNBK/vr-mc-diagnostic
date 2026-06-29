/**
 * @file    SoftLimitProbe.hpp
 * @brief   DB2-05 — Probe that confirms the adapter clamps / rejects
 *          out-of-envelope commands instead of passing them through.
 *
 *  params:
 *      joint_indices : int[]
 *      hi            : double[] (radians — soft-upper for each joint)
 *      epsilon_rad   : double (default 0.05; how far past hi[] to attempt)
 *      small_slop_rad: double (default 0.01; tolerated overshoot)
 *      settle_ms     : int (default 600)
 *      test_poll_rate_hz : int (default 1000)
 *
 *  emits:
 *      attempted[]    : (hi + epsilon) actually commanded
 *      reached[]      : q observed after settle
 *      overshoots[]   : reached - hi (negative = clamped below limit)
 *      clamped_ok[]   : 1 if overshoot <= small_slop_rad, else 0
 *      max_overshoot  : worst over the joints
 */
#pragma once

#include "WorkerThreadProbe.hpp"
#include <QJsonObject>


class SoftLimitProbe : public WorkerThreadProbe
{
    Q_OBJECT
public:
    explicit SoftLimitProbe(HandWorker* worker, QObject* parent = nullptr);

public slots:
    void startBatch(QJsonObject params);

signals:
    void batchDone(bool ok, QJsonObject metrics, QString summary);

private:
    void stepNext_();
    void finalize_();
    void fail_(const QString& reason);
    void restorePollRate_();

    bool   active_     = false;
    int    cursor_     = 0;
    int    settle_ms_  = 600;
    double epsilon_    = 0.05;
    double slop_       = 0.01;
    QVector<int>    joint_indices_;
    QVector<double> hi_, attempted_, reached_, overshoots_;
    QVector<int>    clamped_ok_;
    QVector<double> baseline_q_;
    int    saved_poll_hz_ = 0;
    bool   poll_hz_pushed_ = false;
};
