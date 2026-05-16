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
#include <QPen>
#include <QValueAxis>
#include <QVBoxLayout>

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
    m_axisX->setRange(0, kWindowSec);
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
        m_chart->addSeries(c.series);
        c.series->attachAxis(m_axisX);
        c.series->attachAxis(m_axisY);
        c.series->setVisible(c.defaultOn);
    }

    auto* view = new QChartView(m_chart);
    view->setRenderHint(QPainter::Antialiasing, true);
    /* Chart view itself has no frame — the boundary lives on the
     * outer QFrame that wraps the checkbox header + chart together
     * (see below). */
    view->setFrameShape(QFrame::NoFrame);

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

    /* Wrap checkbox header + chart in a single QFrame so the
     * boundary surrounds the whole pane (not just the chart view).
     * This visually unifies the controls with the plot they affect. */
    auto* frame = new QFrame(this);
    frame->setFrameShape(QFrame::StyledPanel);
    frame->setFrameShadow(QFrame::Sunken);
    frame->setLineWidth(1);
    auto* inner = new QVBoxLayout(frame);
    inner->setContentsMargins(0, 0, 0, 0);
    inner->setSpacing(0);
    inner->addLayout(boxesRow);
    inner->addWidget(view, 1);

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
    m_axisX->setRange(0, kWindowSec);
    m_axisY->setRange(-1.0, 1.0);
}

void TelemetryWidget::push(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }
    const SlaveSnapshot* target = nullptr;
    for (const auto& s : snaps){
        if (s.idx == m_idx){ target = &s; break; }
    }
    if (!target){ return; }

    const double t = m_t0->elapsed() / 1000.0;
    for (const auto& c : m_channels){
        c.series->append(t, c.extract(*target));
        /* Ring-buffer: drop samples older than kWindowSec. */
        const double cutoff = t - kWindowSec;
        while (c.series->count() > 0 && c.series->at(0).x() < cutoff){
            c.series->remove(0);
        }
    }
    const double xMax = std::max<double>(kWindowSec, t);
    m_axisX->setRange(xMax - kWindowSec, xMax);
    rescale();
}

void TelemetryWidget::rescale()
{
    double lo =  1e9, hi = -1e9;
    for (const auto& c : m_channels){
        if (!c.series->isVisible()){ continue; }
        for (const auto& p : c.series->points()){
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

}  // namespace vrmc
