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
QT_FORWARD_DECLARE_CLASS(QPushButton)
QT_FORWARD_DECLARE_CLASS(QScrollBar)
QT_FORWARD_DECLARE_CLASS(QEvent)

namespace vrmc {

class TelemetryWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TelemetryWidget(QWidget* parent = nullptr);

    void setActiveSlave(int idx);
    void clear();

protected:
    /** Intercept wheel + mouse-drag events on the chart view so the
     *  operator can scroll through history. Auto-pauses on first
     *  scroll/drag if the chart was running live. */
    bool eventFilter(QObject* obj, QEvent* ev) override;

public slots:
    void push(const QVector<vrmc::SlaveSnapshot>& snaps);

private:
    /** Default visible X-axis window width on first show / resume. */
    static constexpr double kWindowSecDefault = 10.0;
    /** Zoom-in floor — too small and the chart is unreadable. */
    static constexpr double kWindowSecMin     = 0.5;
    /** Total ring-buffer depth — how far back the user can scroll once
     *  the chart is paused. Also serves as the zoom-out ceiling. */
    static constexpr double kBufferSec        = 60.0;

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
    /** Update the X-axis range from @c m_viewStart (paused) or follow
     *  the latest sample (live). Also resizes scrollbar geometry. */
    void refitView();

    int                 m_idx   = -1;
    QChart*             m_chart = nullptr;
    QValueAxis*         m_axisX = nullptr;
    QValueAxis*         m_axisY = nullptr;
    QElapsedTimer*      m_t0    = nullptr;
    QVector<Channel>    m_channels;

    /* Capture controls. While @c m_paused is true the chart freezes
     * but @c push() keeps appending to the buffer ring. The scrollbar
     * is only enabled in this mode and lets the operator pan the
     * visible window over the last @c kBufferSec of samples. */
    QPushButton*        m_pauseBtn   = nullptr;
    QScrollBar*         m_scrollBar  = nullptr;
    QChartView*         m_view       = nullptr;
    bool                m_paused     = false;
    double              m_viewStart  = 0.0;   /**< left edge of visible window, sec */
    double              m_latestT    = 0.0;   /**< time of most recent sample, sec  */
    /** Width of the visible X window in seconds. Mutated by Ctrl+wheel
     *  zoom; reset to @ref kWindowSecDefault on Resume + on clear(). */
    double              m_windowSec  = kWindowSecDefault;

    /* Mouse-drag pan state. m_dragStartPx is in QChartView viewport
     * coordinates; m_dragStartView snapshots m_viewStart at the moment
     * the drag begins so subsequent moves compute a stable delta. */
    bool                m_dragging      = false;
    double              m_dragStartPx   = 0.0;
    double              m_dragStartView = 0.0;

    /** Flip the chart into paused mode without re-emitting the button
     *  toggled signal (used by mouse-pan handlers). No-op if already
     *  paused. */
    void enterPausedMode();
};

}  // namespace vrmc
