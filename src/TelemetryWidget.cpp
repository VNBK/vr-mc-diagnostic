#include "TelemetryWidget.hpp"

#include <QCheckBox>
#include <QChart>
#include <QChartView>
#include <QElapsedTimer>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineSeries>
#include <QMouseEvent>
#include <QPen>
#include <QPushButton>
#include <QScrollBar>
#include <QValueAxis>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>

namespace vrmc {

TelemetryWidget::TelemetryWidget(QWidget* parent) : QWidget(parent)
{
    /* --- channel catalogue, tagged with a group so the checkbox row
     * below can build labelled columns from a single source of truth. */
    using G = Group;
    m_channels = {
        { G::Position,   tr("pos (rad)"),       QColor("#4caf50"), true,
          [](const SlaveSnapshot& s){ return s.position; },       nullptr, nullptr },
        { G::Position,   tr("cmd pos (rad)"),   QColor("#8bc34a"), false,
          [](const SlaveSnapshot& s){ return s.cmdPosition; },    nullptr, nullptr },
        { G::Velocity,   tr("vel (rad/s)"),     QColor("#2196f3"), true,
          [](const SlaveSnapshot& s){ return s.velocity; },       nullptr, nullptr },
        { G::Velocity,   tr("cmd vel (rad/s)"), QColor("#03a9f4"), false,
          [](const SlaveSnapshot& s){ return s.cmdVelocity; },    nullptr, nullptr },
        { G::Torque,     tr("trq (Nm)"),        QColor("#f44336"), true,
          [](const SlaveSnapshot& s){ return s.torque; },         nullptr, nullptr },
        { G::Torque,     tr("cmd trq (Nm)"),    QColor("#ff9800"), false,
          [](const SlaveSnapshot& s){ return s.cmdTorque; },      nullptr, nullptr },
        { G::Tracking,   tr("Δq"),              QColor("#e91e63"), false,
          [](const SlaveSnapshot& s){ return s.trackingError; },  nullptr, nullptr },
        { G::Electrical, tr("current (A)"),     QColor("#9c27b0"), false,
          [](const SlaveSnapshot& s){ return s.current; },        nullptr, nullptr },
        { G::Electrical, tr("temp (°C)"),       QColor("#795548"), false,
          [](const SlaveSnapshot& s){ return s.temperature; },    nullptr, nullptr },
    };

    /* --- single chart + axes (all channels share the same Y). --- */
    m_chart = new QChart;
    m_chart->setTitle(tr("Live telemetry"));
    m_chart->legend()->setAlignment(Qt::AlignBottom);

    m_axisX = new QValueAxis(m_chart);
    m_axisX->setTitleText(tr("t (s)"));
    m_axisX->setRange(0, m_windowSec);
    m_axisY = new QValueAxis(m_chart);
    m_axisY->setRange(-1.0, 1.0);

    m_chart->addAxis(m_axisX, Qt::AlignBottom);
    m_chart->addAxis(m_axisY, Qt::AlignLeft);

    for (auto& c : m_channels){
        c.series = new QLineSeries(m_chart);
        c.series->setName(c.name);
        QPen pen(c.colour);
        pen.setWidthF(1.6);
        c.series->setPen(pen);
        /* Offload line rasterisation to OpenGL. With 9 series × 60 s
         * × 30 Hz = ~16 K cached points (~3 K visible per window), Qt's
         * software backend chews ~30-50 ms per paint on a typical
         * laptop and the snapshot signal queue on the UI thread grows
         * unboundedly — that's the "lag after a while" symptom. OpenGL
         * cuts paint cost to ~1 ms and the queue stays empty.
         * Falls back silently if no GL context (Qt prints a warning). */
        c.series->setUseOpenGL(true);
        m_chart->addSeries(c.series);
        c.series->attachAxis(m_axisX);
        c.series->attachAxis(m_axisY);
        c.series->setVisible(c.defaultOn);
    }

    m_view = new QChartView(m_chart);
    m_view->setRenderHint(QPainter::Antialiasing, true);
    /* Chart view itself has no frame — the boundary lives on the
     * outer QFrame that wraps the checkbox header + chart together
     * (see below). */
    m_view->setFrameShape(QFrame::NoFrame);
    /* Install ourselves as an event filter so wheel + mouse-drag on
     * the chart pan the visible window through history. Auto-pause on
     * first scroll so the user gets feedback without first clicking
     * the Pause button. */
    m_view->viewport()->installEventFilter(this);

    /* --- checkbox row: one labelled column per group, separated by
     * thin vertical dividers. Each group's checkboxes stack
     * vertically so the row stays compact and the visual grouping is
     * obvious without needing tabs or sub-charts. --- */
    auto groupTitle = [](Group g) -> QString {
        switch (g){
        case Group::Position:   return QStringLiteral("Position");
        case Group::Velocity:   return QStringLiteral("Velocity");
        case Group::Torque:     return QStringLiteral("Torque");
        case Group::Tracking:   return QStringLiteral("Tracking");
        case Group::Electrical: return QStringLiteral("Electrical");
        }
        return {};
    };

    auto* boxesRow = new QHBoxLayout;
    boxesRow->setContentsMargins(8, 4, 8, 4);
    boxesRow->setSpacing(0);

    const Group kOrder[] = { Group::Position, Group::Velocity, Group::Torque,
                             Group::Tracking, Group::Electrical };
    bool firstGroup = true;
    for (Group g : kOrder){
        if (!firstGroup){
            auto* sep = new QFrame(this);
            sep->setFrameShape(QFrame::VLine);
            sep->setFrameShadow(QFrame::Sunken);
            boxesRow->addWidget(sep);
        }
        firstGroup = false;

        auto* col = new QVBoxLayout;
        col->setContentsMargins(10, 0, 10, 0);
        col->setSpacing(2);

        auto* header = new QLabel(groupTitle(g), this);
        QFont hf = header->font();
        hf.setBold(true);
        header->setFont(hf);
        col->addWidget(header);

        for (auto& c : m_channels){
            if (c.group != g){ continue; }
            c.check = new QCheckBox(c.name, this);
            c.check->setChecked(c.defaultOn);
            /* Splash of the channel colour on the label so it's easy to
             * cross-reference with the chart. */
            c.check->setStyleSheet(QStringLiteral("QCheckBox { color: %1; }")
                                       .arg(c.colour.name()));
            connect(c.check, &QCheckBox::toggled, this,
                    [this, s = c.series](bool on){
                        s->setVisible(on);
                        rescale();
                    });
            col->addWidget(c.check);
        }
        col->addStretch(1);
        boxesRow->addLayout(col);
    }
    boxesRow->addStretch(1);

    /* Capture row: Pause/Resume button on the left + scrollbar that
     * lets the operator pan through buffered history once paused.
     * Live mode: scrollbar disabled and pinned to the right edge. */
    m_pauseBtn = new QPushButton(tr("Pause"), this);
    m_pauseBtn->setCheckable(true);
    m_pauseBtn->setToolTip(tr("Freeze the chart so it can be inspected "
                              "or screenshotted. While paused you can "
                              "scroll back up to %1 s of history, and "
                              "Ctrl+wheel to zoom in/out.")
                              .arg(int(kBufferSec)));
    m_pauseBtn->setStyleSheet(QStringLiteral(
        "QPushButton:checked { background: #b03030; color: white; "
        "                       font-weight: 600; }"));

    m_scrollBar = new QScrollBar(Qt::Horizontal, this);
    m_scrollBar->setEnabled(false);
    m_scrollBar->setRange(0, 0);
    m_scrollBar->setPageStep(int(m_windowSec * 1000.0));   /* 1 ms granularity */
    m_scrollBar->setSingleStep(100);
    connect(m_scrollBar, &QScrollBar::valueChanged, this, [this](int v){
        if (!m_paused){ return; }
        m_viewStart = double(v) / 1000.0;
        m_axisX->setRange(m_viewStart, m_viewStart + m_windowSec);
        rescale();
    });
    connect(m_pauseBtn, &QPushButton::toggled, this, [this](bool on){
        if (on){
            enterPausedMode();
        } else {
            m_paused = false;
            m_pauseBtn->setText(tr("Pause"));
            m_scrollBar->setEnabled(false);
            /* Resume snaps back to live + default zoom so the chart
             * looks like it does at session start. Zoom changes
             * persist only while paused. */
            m_windowSec = kWindowSecDefault;
            refitView();
        }
    });

    auto* captureRow = new QHBoxLayout;
    captureRow->setContentsMargins(8, 0, 8, 4);
    captureRow->setSpacing(8);
    captureRow->addWidget(m_pauseBtn);
    captureRow->addWidget(m_scrollBar, 1);

    /* Wrap checkbox header + chart + capture row in a single QFrame so
     * the boundary surrounds the whole pane (not just the chart view).
     * This visually unifies the controls with the plot they affect. */
    auto* frame = new QFrame(this);
    frame->setFrameShape(QFrame::StyledPanel);
    frame->setFrameShadow(QFrame::Sunken);
    frame->setLineWidth(1);
    auto* inner = new QVBoxLayout(frame);
    inner->setContentsMargins(0, 0, 0, 0);
    inner->setSpacing(0);
    inner->addLayout(boxesRow);
    inner->addWidget(m_view, 1);
    inner->addLayout(captureRow);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    root->addWidget(frame);

    m_t0 = new QElapsedTimer;
    m_t0->start();
}

void TelemetryWidget::setActiveSlave(int idx)
{
    m_idx = idx;
    clear();
}

void TelemetryWidget::clear()
{
    for (auto& c : m_channels){
        if (c.series){ c.series->clear(); }
    }
    if (m_t0){ m_t0->restart(); }
    m_latestT   = 0.0;
    m_viewStart = 0.0;
    m_windowSec = kWindowSecDefault;
    m_axisX->setRange(0, m_windowSec);
    m_axisY->setRange(-1.0, 1.0);
    if (m_scrollBar){
        QSignalBlocker block(m_scrollBar);
        m_scrollBar->setRange(0, 0);
        m_scrollBar->setValue(0);
    }
}

void TelemetryWidget::push(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }
    const SlaveSnapshot* target = nullptr;
    for (const auto& s : snaps){
        if (s.idx == m_idx){ target = &s; break; }
    }
    if (!target){ return; }

    /* Always buffer the sample so paused-mode scrollback shows the
     * fresh data once resumed. Ring buffer trims to kBufferSec to cap
     * memory; only the visible window (m_windowSec) is on screen. */
    const double t = m_t0->elapsed() / 1000.0;
    m_latestT = t;
    for (const auto& c : m_channels){
        c.series->append(t, c.extract(*target));
        const double cutoff = t - kBufferSec;
        while (c.series->count() > 0 && c.series->at(0).x() < cutoff){
            c.series->remove(0);
        }
    }
    refitView();
}

void TelemetryWidget::refitView()
{
    /* Compute the visible window: in live mode it tracks the latest
     * sample; in paused mode it sits at m_viewStart (controlled by
     * the scrollbar). Either way, scrollbar range mirrors the
     * available history so it stays accurate as samples age out. */
    const double bufferStart = std::max(0.0, m_latestT - kBufferSec);
    const double maxStart    = std::max(bufferStart, m_latestT - m_windowSec);
    if (!m_paused){
        m_viewStart = maxStart;
    } else {
        m_viewStart = std::clamp(m_viewStart, bufferStart, maxStart);
    }
    m_axisX->setRange(m_viewStart, m_viewStart + m_windowSec);

    if (m_scrollBar){
        QSignalBlocker block(m_scrollBar);
        const int lo = int(bufferStart * 1000.0);
        const int hi = int(maxStart    * 1000.0);
        m_scrollBar->setRange(lo, hi);
        m_scrollBar->setPageStep(int(m_windowSec * 1000.0));   /* tracks zoom */
        m_scrollBar->setValue(int(m_viewStart * 1000.0));
    }
    rescale();
}

void TelemetryWidget::rescale()
{
    /* Only autoscale across samples inside the visible X window, so
     * scrolling back through history doesn't get rescaled by spikes
     * outside the user's current view. */
    const double xLo = m_viewStart;
    const double xHi = m_viewStart + m_windowSec;
    double lo =  1e9, hi = -1e9;
    for (const auto& c : m_channels){
        if (!c.series->isVisible()){ continue; }
        for (const auto& p : c.series->points()){
            if (p.x() < xLo || p.x() > xHi){ continue; }
            lo = std::min<double>(lo, p.y());
            hi = std::max<double>(hi, p.y());
        }
    }
    if (lo > hi){ lo = -1.0; hi = 1.0; }
    if (hi - lo < 0.1){
        const double c = 0.5 * (lo + hi);
        lo = c - 0.5; hi = c + 0.5;
    }
    const double pad = std::max(0.05, 0.1 * (hi - lo));
    m_axisY->setRange(lo - pad, hi + pad);
}

void TelemetryWidget::enterPausedMode()
{
    if (m_paused){ return; }
    m_paused = true;
    /* Sync the Pause button + scrollbar without retriggering the
     * button's toggled slot (which would loop back here). */
    if (m_pauseBtn){
        QSignalBlocker block(m_pauseBtn);
        m_pauseBtn->setChecked(true);
        m_pauseBtn->setText(tr("Resume"));
    }
    if (m_scrollBar){ m_scrollBar->setEnabled(true); }
    m_viewStart = std::max(0.0, m_latestT - m_windowSec);
    refitView();
}

bool TelemetryWidget::eventFilter(QObject* obj, QEvent* ev)
{
    if (m_view && obj == m_view->viewport()){
        switch (ev->type()){
        case QEvent::Wheel: {
            auto* we = static_cast<QWheelEvent*>(ev);
            enterPausedMode();
            const double notches = double(we->angleDelta().y()) / 120.0;
            if (we->modifiers() & Qt::ControlModifier){
                /* Ctrl+wheel: zoom around the cursor's data coordinate
                 * so the point under the pointer stays put while the
                 * window contracts (wheel up) or expands (wheel down).
                 * Factor 1.25 per notch — comfortable few-step zoom
                 * range. Clamp to [kWindowSecMin, kBufferSec]. */
                const QRectF plot = m_chart->plotArea();
                if (plot.width() <= 0){ return true; }
                const double cursorPx = we->position().x() - plot.left();
                const double secPerPx = m_windowSec / plot.width();
                const double cursorT  = m_viewStart + cursorPx * secPerPx;

                /* Wheel up (notches>0) = zoom in = shrink window. */
                const double factor = std::pow(1.25, -notches);
                double newWindow = std::clamp(m_windowSec * factor,
                                              kWindowSecMin, kBufferSec);
                /* Recompute viewStart so cursorT stays at the same
                 * fractional position within the new window. */
                const double frac = (m_windowSec > 0) ? cursorPx / plot.width() : 0.5;
                m_viewStart = cursorT - frac * newWindow;
                m_windowSec = newWindow;
            } else {
                /* Plain wheel: pan one second per notch. Up → past,
                 * down → toward latest. */
                m_viewStart -= notches;
            }
            refitView();
            return true;
        }
        case QEvent::MouseButtonPress: {
            auto* me = static_cast<QMouseEvent*>(ev);
            if (me->button() == Qt::LeftButton){
                enterPausedMode();
                m_dragging      = true;
                m_dragStartPx   = me->position().x();
                m_dragStartView = m_viewStart;
                m_view->viewport()->setCursor(Qt::ClosedHandCursor);
                return true;
            }
            break;
        }
        case QEvent::MouseMove: {
            if (m_dragging){
                auto* me = static_cast<QMouseEvent*>(ev);
                const QRectF plot = m_chart->plotArea();
                if (plot.width() <= 0){ return true; }
                /* Drag right (positive dx) pans the view to the past
                 * (decreasing m_viewStart) — content follows the
                 * cursor, like grabbing the chart and dragging it. */
                const double dxPx    = me->position().x() - m_dragStartPx;
                const double secPerPx = m_windowSec / plot.width();
                m_viewStart = m_dragStartView - dxPx * secPerPx;
                refitView();
                return true;
            }
            break;
        }
        case QEvent::MouseButtonRelease: {
            if (m_dragging){
                m_dragging = false;
                m_view->viewport()->unsetCursor();
                return true;
            }
            break;
        }
        default: break;
        }
    }
    return QWidget::eventFilter(obj, ev);
}

}  // namespace vrmc
