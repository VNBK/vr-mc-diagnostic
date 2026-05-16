/**
 * @file   TelemetryWidget.hpp
 * @brief  Single-chart live telemetry with grouped show/hide checkboxes.
 *
 * Every channel shares one QChart so the operator can correlate across
 * units at a glance. Y-axis autoscales across the currently-visible
 * series, so hiding a high-magnitude channel (e.g. temperature in °C)
 * lets the small-magnitude ones (e.g. tracking error Δq) breathe.
 *
 * The checkbox row above the chart is organised into labelled column
 * groups (Position / Velocity / Torque / Tracking / Electrical) so the
 * operator can mentally cluster related signals without the chart
 * having to fan out into multiple tabs or sub-charts.
 *
 * Channel catalogue lives in the .cpp; see kChannels.
 */

#pragma once

#include "MasterWorker.hpp"
#include <QWidget>
#include <functional>

QT_FORWARD_DECLARE_CLASS(QCheckBox)
QT_FORWARD_DECLARE_CLASS(QChart)
QT_FORWARD_DECLARE_CLASS(QChartView)
QT_FORWARD_DECLARE_CLASS(QLineSeries)
QT_FORWARD_DECLARE_CLASS(QValueAxis)
QT_FORWARD_DECLARE_CLASS(QElapsedTimer)

namespace vrmc {

class TelemetryWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TelemetryWidget(QWidget* parent = nullptr);

    void setActiveSlave(int idx);
    void clear();

public slots:
    void push(const QVector<vrmc::SlaveSnapshot>& snaps);

private:
    static constexpr int kWindowSec = 10;

    enum class Group {
        Position = 0,
        Velocity,
        Torque,
        Tracking,
        Electrical,
    };

    struct Channel {
        Group                                                  group;
        QString                                                name;
        QColor                                                 colour;
        bool                                                   defaultOn;
        std::function<float(const SlaveSnapshot&)>             extract;
        QLineSeries*                                           series  = nullptr;
        QCheckBox*                                             check   = nullptr;
    };

    void rescale();

    int                 m_idx   = -1;
    QChart*             m_chart = nullptr;
    QValueAxis*         m_axisX = nullptr;
    QValueAxis*         m_axisY = nullptr;
    QElapsedTimer*      m_t0    = nullptr;
    QVector<Channel>    m_channels;
};

}  // namespace vrmc
