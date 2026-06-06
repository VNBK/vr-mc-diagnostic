/**
 * @file   GainEditor.hpp
 * @brief  Per-loop Kp/Ki spinbox grid + Read / Apply for one slave.
 */

#pragma once

#include "MasterWorker.hpp"
#include "MotorProfile.hpp"
#include <QWidget>

class QDoubleSpinBox;
class QLabel;
class QPushButton;

namespace vrmc {

class GainEditor : public QWidget
{
    Q_OBJECT
public:
    explicit GainEditor(QWidget* parent = nullptr);

    void setActiveSlave(int idx, const QString& name = {});

    /** Hand the latest motor name-plate down so the row's BW spinbox can be
     *  back-computed from Kp via @ref bwFromKp. Called by MainWindow after
     *  every successful @c readMotorProfile / editor commit. Triggers an
     *  immediate refresh of the displayed BW for whichever rows already
     *  have a Kp populated. */
    void setMotorParams(const MotorParams& p);

signals:
    /** Request a refresh of the chosen loop; reply arrives via @c onGainRead. */
    void readGainRequested (int idx, vrmc::Loop loop);
    /** Commit Kp/Ki for the chosen loop. */
    void writeGainRequested(int idx, vrmc::Loop loop, float kp, float ki);
    /** Ask the worker to run a model-based PI auto-tune at @p bw_hz on the
     *  slave's loop @p loop. The worker writes the result back via
     *  @ref onGainTuned (and the row's Kp/Ki spinboxes refresh on success). */
    void tuneGainRequested (int idx, vrmc::Loop loop, float bw_hz);

public slots:
    /** Slot wired to @ref MasterWorker::gainRead. */
    void onGainRead(int idx, vrmc::Loop loop, float kp, float ki, bool ok);
    /** Slot wired to @ref MasterWorker::gainTuned. Refreshes the row's
     *  Kp/Ki spinboxes + status label. */
    void onGainTuned(int idx, vrmc::Loop loop, float kp, float ki, bool ok);

private:
    struct Row {
        Loop             loop;
        QDoubleSpinBox*  kp     = nullptr;
        QDoubleSpinBox*  ki     = nullptr;
        QDoubleSpinBox*  bw     = nullptr;   /* target bandwidth (Hz) */
        QPushButton*     read   = nullptr;
        QPushButton*     apply  = nullptr;
        QPushButton*     tune   = nullptr;
        QLabel*          status = nullptr;   /* "" / "tuning..." / "ok" / "fail" */
    };

    void emitRead (Loop loop);
    void emitApply(Loop loop);
    void emitTune (Loop loop);

    int           m_idx = -1;
    QVector<Row>  m_rows;
    MotorParams   m_motorParams;       /**< pushed from MainWindow */
};

}  // namespace vrmc
