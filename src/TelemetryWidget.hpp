/**
 * @file   TelemetryWidget.hpp
 * @brief  Live multi-channel chart with per-line show/hide checkboxes.
 *
 * Channels are declared at widget construction (see kChannels in the
 * .cpp). Each has a name, colour, default-visible flag, and a lambda
 * that extracts its value from a SlaveSnapshot. A QCheckBox row above
 * the chart toggles the matching QLineSeries' visibility; the Y-axis
 * autoscales across currently-visible channels.
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

    struct Channel {
        QString                                                name;
        QColor                                                 colour;
        bool                                                   defaultOn;
        std::function<float(const SlaveSnapshot&)>             extract;
        /* Filled at runtime. */
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
