#include "MotorView.hpp"

#include <QLabel>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPen>
#include <QVBoxLayout>

#include <cmath>

namespace vrmc {

MotorView::MotorView(QWidget* parent) : QWidget(parent)
{
    /* Just enough for caption + dial. No readouts means we can shrink
     * the vertical floor; the dial scales to whatever the splitter
     * gives us. */
    setMinimumSize(220, 260);

    m_caption = new QLabel(tr("(no slave)"), this);
    /* Bold caption — the slave title doubles as the widget header. */
    m_caption->setAlignment(Qt::AlignHCenter);
    {
        QFont f = m_caption->font();
        f.setBold(true);
        m_caption->setFont(f);
    }

    auto* root = new QVBoxLayout(this);
    root->setSpacing(6);
    root->addWidget(m_caption);
    root->addStretch(1);             /* dial drawn into the gap        */
}

void MotorView::setActiveSlave(int idx, const QString& name)
{
    m_idx     = idx;
    m_name    = name;
    m_hasData = false;
    m_position = m_velocity = m_torque = 0.0f;
    if (idx < 0){
        m_caption->setText(tr("(no slave)"));
    } else {
        m_caption->setText(tr("Slave %1  —  %2").arg(idx).arg(name));
    }
    update();
}

void MotorView::onSnapshots(const QVector<SlaveSnapshot>& snaps)
{
    if (m_idx < 0){ return; }
    for (const auto& s : snaps){
        if (s.idx != m_idx){ continue; }
        m_hasData  = s.pdoFresh;
        m_position = s.position;
        m_velocity = s.velocity;
        m_torque   = s.torque;
        update();
        return;
    }
}

void MotorView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    /* Carve out a square dial region between the caption (above) and
     * the bottom of the widget. Now that the q / ω readouts are
     * gone, the dial can use almost all of the vertical real-estate
     * below the caption — a modest bottom pad keeps the needle tip
     * from kissing the widget edge. */
    const int availTop = m_caption->geometry().bottom() + 8;
    const int availBot = height() - 8;
    const int availH   = std::max(60, availBot - availTop);
    const int side     = std::min(availH, width() - 20);
    const int x0       = (width() - side) / 2;
    const int y0       = availTop + (availH - side) / 2;
    const QRectF dialRect(x0, y0, side, side);

    /* Dial face. */
    p.setPen(QPen(QColor("#666"), 2));
    p.setBrush(QColor("#1a1a1a"));
    p.drawEllipse(dialRect);

    /* Hour-style minor tick marks every 30° (12 tick equivalent). */
    const QPointF centre = dialRect.center();
    const double  radius = dialRect.width() / 2.0;
    p.setPen(QPen(QColor("#555"), 1));
    for (int i = 0; i < 12; ++i){
        const double a = i * (2.0 * M_PI / 12.0) - M_PI / 2.0;
        const QPointF p1(centre.x() + std::cos(a) * (radius - 6),
                         centre.y() + std::sin(a) * (radius - 6));
        const QPointF p2(centre.x() + std::cos(a) * (radius - 2),
                         centre.y() + std::sin(a) * (radius - 2));
        p.drawLine(p1, p2);
    }
    /* Cardinal label at 0 rad ("0") on the right edge. Qt's
     * convention here: angle 0 → 3 o'clock (mathematical, not
     * clock face). */
    p.setPen(QColor("#888"));
    QFont small = p.font();
    small.setPointSize(7);
    p.setFont(small);
    p.drawText(QRectF(centre.x() + radius - 14, centre.y() - 7, 14, 14),
               Qt::AlignCenter, QStringLiteral("0"));
    p.drawText(QRectF(centre.x() - 14, centre.y() - radius + 1, 28, 12),
               Qt::AlignCenter, QStringLiteral("+π/2"));
    p.drawText(QRectF(centre.x() - radius + 1, centre.y() - 7, 18, 14),
               Qt::AlignLeft   | Qt::AlignVCenter, QStringLiteral("±π"));
    p.drawText(QRectF(centre.x() - 14, centre.y() + radius - 13, 28, 12),
               Qt::AlignCenter, QStringLiteral("−π/2"));

    if (!m_hasData){ return; }

    /* Needle: angle 0 points right (east) ⇒ rad value is the screen
     * angle directly, but Qt's Y grows downward so we render with
     * +q rotating counter-clockwise in screen space. */
    const double q     = double(m_position);
    const double angle = q - M_PI / 2.0;        /* 0 rad → up? no, see comment */
    /* Convention: 0 rad → east (3 o'clock). +π/2 → north (12).
     *   x = r*cos(q),  y = -r*sin(q)   (flip y for screen) */
    const QPointF tip(centre.x() + std::cos(q) * (radius - 14),
                      centre.y() - std::sin(q) * (radius - 14));

    /* Colour by velocity sign — sticky to last non-zero for a few px
     * of dead-band so noise around zero doesn't flicker. */
    QColor needleCol("#888");
    const double vel = double(m_velocity);
    if      (vel > 0.05) needleCol = QColor("#3aa83a");  /* CW  – green */
    else if (vel < -0.05) needleCol = QColor("#3a8ed8"); /* CCW – blue  */
    /* (vel) here is positive = increasing q. Mechanical convention
     * varies; the dial just shows the math sign. The arrow head
     * leaves no ambiguity about which way the shaft is moving. */

    /* Needle line + arrowhead. */
    QPen needlePen(needleCol, 3);
    needlePen.setCapStyle(Qt::RoundCap);
    p.setPen(needlePen);
    p.drawLine(centre, tip);

    /* Arrowhead — small triangle perpendicular to the needle. */
    const double a   = q;
    const double aL  = a + M_PI - 0.35;
    const double aR  = a + M_PI + 0.35;
    const double hLen = 10.0;
    QPointF tL(tip.x() + std::cos(aL) * hLen,
               tip.y() - std::sin(aL) * hLen);
    QPointF tR(tip.x() + std::cos(aR) * hLen,
               tip.y() - std::sin(aR) * hLen);
    QPainterPath head;
    head.moveTo(tip);
    head.lineTo(tL);
    head.lineTo(tR);
    head.closeSubpath();
    p.setBrush(needleCol);
    p.setPen(Qt::NoPen);
    p.drawPath(head);

    /* Rotation arc — small arc near the dial edge whose direction
     * depicts CW vs CCW. Only drawn when |ω| > deadband so static
     * motors don't get a misleading swirl. */
    if (std::fabs(vel) > 0.05){
        const QRectF arcRect = dialRect.adjusted(8, 8, -8, -8);
        const int    span    = (vel > 0) ? -45*16 : 45*16;
        const int    startA  = int((q * 180.0 / M_PI) * 16);
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(needleCol, 2, Qt::SolidLine, Qt::FlatCap));
        p.drawArc(arcRect, startA, span);
    }

    /* Hub. */
    p.setBrush(QColor("#333"));
    p.setPen(QPen(QColor("#999"), 1));
    p.drawEllipse(centre, 5.0, 5.0);
    (void)angle;   /* unused after rewrite — keep var to document logic */
}

}  // namespace vrmc
