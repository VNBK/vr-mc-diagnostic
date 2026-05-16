/**
 * @file   MotorView.hpp
 * @brief  Per-slave rotational dial: shows angle + direction live.
 *
 * Sits to the right of the slave table. When a row is selected, the
 * dial shows that slave's current shaft angle (rad → rendered as a
 * clock-needle), with an arrow tip coloured by velocity sign — green
 * = CW (positive ω), blue = CCW (negative), grey = idle. Beneath the
 * dial: live rad / deg / rpm read-outs.
 */

#pragma once

#include "MasterWorker.hpp"

#include <QWidget>

class QLabel;

namespace vrmc {

class MotorView : public QWidget
{
    Q_OBJECT
public:
    explicit MotorView(QWidget* parent = nullptr);

    /** Track which slave to display. -1 clears. */
    void setActiveSlave(int idx, const QString& name = {});

public slots:
    /** Feed the per-tick snapshot stream so the dial stays live. */
    void onSnapshots(const QVector<vrmc::SlaveSnapshot>& snaps);

protected:
    void paintEvent(QPaintEvent* e) override;

private:
    int     m_idx        = -1;
    QString m_name;
    bool    m_hasData    = false;
    float   m_position   = 0.0f;  /**< rad, wrapped to ±π for display */
    float   m_velocity   = 0.0f;  /**< rad/s, raw sign for direction  */
    float   m_torque     = 0.0f;  /**< Nm                             */

    QLabel* m_caption    = nullptr;
    /* q / ω numeric readouts were removed — the picker bar at the top
     * of the main window already shows the same values, and operators
     * found the duplication noisy. The dial alone is enough for
     * at-a-glance direction + magnitude here. */
};

}  // namespace vrmc
