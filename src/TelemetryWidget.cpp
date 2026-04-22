#include "TelemetryWidget.hpp"

#include <QCheckBox>
#include <QChart>
#include <QChartView>
#include <QElapsedTimer>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLineSeries>
#include <QPen>
#include <QValueAxis>
#include <QVBoxLayout>

#include <algorithm>

namespace vrmc {

TelemetryWidget::TelemetryWidget(QWidget* parent) : QWidget(parent)
{
    /* --- channel catalogue. Ordered by expected relevance. --- */
    m_channels = {
        { tr("pos (rad)"),   QColor("#4caf50"), true,
          [](const SlaveSnapshot& s){ return s.position; },       nullptr, nullptr },
        { tr("vel (rad/s)"), QColor("#2196f3"), true,
          [](const SlaveSnapshot& s){ return s.velocity; },       nullptr, nullptr },
        { tr("trq (Nm)"),    QColor("#f44336"), true,
          [](const SlaveSnapshot& s){ return s.torque; },         nullptr, nullptr },
        { tr("cmd pos"),     QColor("#8bc34a"), false,
          [](const SlaveSnapshot& s){ return s.cmdPosition; },    nullptr, nullptr },
        { tr("cmd vel"),     QColor("#03a9f4"), false,
          [](const SlaveSnapshot& s){ return s.cmdVelocity; },    nullptr, nullptr },
        { tr("cmd trq"),     QColor("#ff9800"), false,
          [](const SlaveSnapshot& s){ return s.cmdTorque; },      nullptr, nullptr },
        { tr("err"),         QColor("#e91e63"), false,
          [](const SlaveSnapshot& s){ return s.trackingError; },  nullptr, nullptr },
        { tr("current (A)"), QColor("#9c27b0"), false,
          [](const SlaveSnapshot& s){ return s.current; },        nullptr, nullptr },
        { tr("temp (°C)"),   QColor("#795548"), false,
          [](const SlaveSnapshot& s){ return s.temperature; },    nullptr, nullptr },
    };

    /* --- chart + axes --- */
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

    auto* view = new QChartView(m_chart, this);
    view->setRenderHint(QPainter::Antialiasing, true);

    /* --- checkbox grid --- */
    auto* boxes = new QGridLayout;
    boxes->setContentsMargins(8, 4, 8, 4);
    boxes->setHorizontalSpacing(12);
    int col = 0;
    int row = 0;
    const int cols = 5;
    for (auto& c : m_channels){
        c.check = new QCheckBox(c.name, this);
        c.check->setChecked(c.defaultOn);
        /* Splash of the channel colour on the label so it's easy to
         * cross-reference with the chart. */
        c.check->setStyleSheet(QStringLiteral("QCheckBox { color: %1; }")
                                   .arg(c.colour.name()));
        connect(c.check, &QCheckBox::toggled, this, [this, s = c.series](bool on){
            s->setVisible(on);
            rescale();
        });
        boxes->addWidget(c.check, row, col);
        ++col;
        if (col >= cols){ col = 0; ++row; }
    }

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);
    root->addLayout(boxes);
    root->addWidget(view, 1);

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
