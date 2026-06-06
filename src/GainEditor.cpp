#include "GainEditor.hpp"
#include "BwFromKp.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <cmath>

namespace vrmc {

static const char* loopLabel(Loop l)
{
    switch (l){
    case Loop::Current:  return "Current";
    case Loop::Velocity: return "Velocity";
    case Loop::Position: return "Position";
    }
    return "?";
}

/** Per-loop default target bandwidth (Hz) for the auto-tune button.
 *  Pulled from `VELOCITY_LOOP_TUNING.md` + the current-loop tuning doc:
 *  current ≈ 10× velocity bandwidth so the cascade has headroom;
 *  velocity at 100 Hz is the documented BW=100 row from the gain table;
 *  position runs ≈ 5× slower than velocity. The spinbox lets the
 *  operator override per session. */
static double defaultBw(Loop l)
{
    switch (l){
    case Loop::Current:  return 1500.0;
    case Loop::Velocity: return  100.0;
    case Loop::Position: return   20.0;
    }
    return 100.0;
}

GainEditor::GainEditor(QWidget* parent) : QWidget(parent)
{
    auto* grid = new QGridLayout;
    grid->addWidget(new QLabel(tr("Loop")),     0, 0);
    grid->addWidget(new QLabel(tr("Kp")),       0, 1);
    grid->addWidget(new QLabel(tr("Ki")),       0, 2);
    grid->addWidget(new QLabel(tr("BW (Hz)")),  0, 3);

    int row = 1;
    for (Loop l : { Loop::Current, Loop::Velocity, Loop::Position }){
        Row r;
        r.loop = l;
        r.kp = new QDoubleSpinBox(this);
        r.ki = new QDoubleSpinBox(this);
        for (QDoubleSpinBox* sb : { r.kp, r.ki }){
            sb->setRange(-1e9, 1e9);
            sb->setDecimals(6);
            sb->setSingleStep(0.01);
        }
        r.bw = new QDoubleSpinBox(this);
        r.bw->setRange(0.1, 5000.0);
        r.bw->setDecimals(1);
        r.bw->setSingleStep(10.0);
        r.bw->setValue(defaultBw(l));
        r.bw->setSuffix(QStringLiteral(" Hz"));

        r.read   = new QPushButton(tr("Read"),  this);
        r.apply  = new QPushButton(tr("Apply"), this);
        r.tune   = new QPushButton(tr("Tune"),  this);
        r.tune->setToolTip(tr("Model-based PI auto-tune at the chosen "
                               "bandwidth (writes OD 0x2080:01..04). The "
                               "slave must be DISABLED -- the bandwidth "
                               "update isn't safe to apply while the "
                               "cascade is live."));
        r.status = new QLabel(this);
        r.status->setMinimumWidth(80);
        r.status->setStyleSheet(QStringLiteral("color: #555;"));

        grid->addWidget(new QLabel(tr(loopLabel(l))), row, 0);
        grid->addWidget(r.kp,     row, 1);
        grid->addWidget(r.ki,     row, 2);
        grid->addWidget(r.bw,     row, 3);
        grid->addWidget(r.read,   row, 4);
        grid->addWidget(r.apply,  row, 5);
        grid->addWidget(r.tune,   row, 6);
        grid->addWidget(r.status, row, 7);
        connect(r.read,  &QPushButton::clicked, this, [this, l]{ emitRead(l);  });
        connect(r.apply, &QPushButton::clicked, this, [this, l]{ emitApply(l); });
        connect(r.tune,  &QPushButton::clicked, this, [this, l]{ emitTune(l);  });
        m_rows.push_back(r);
        ++row;
    }

    auto* readAll = new QPushButton(tr("Read all"), this);
    connect(readAll, &QPushButton::clicked, this, [this]{
        for (const auto& r : m_rows){ emitRead(r.loop); }
    });
    grid->addWidget(readAll, row, 0, 1, 8);

    auto* root = new QVBoxLayout(this);
    root->addLayout(grid);
    root->addStretch();

    setActiveSlave(-1);
}

void GainEditor::setActiveSlave(int idx, const QString& /*name*/)
{
    m_idx = idx;
    const bool en = (idx >= 0);
    for (const auto& r : m_rows){
        r.kp->setEnabled(en);
        r.ki->setEnabled(en);
        r.bw->setEnabled(en);
        r.read->setEnabled(en);
        r.apply->setEnabled(en);
        r.tune->setEnabled(en);
        if (!en){ r.status->clear(); }
    }
}

void GainEditor::setMotorParams(const MotorParams& p)
{
    m_motorParams = p;
    /* Recompute BW for every row whose Kp spinbox already holds a non-zero
     * value from the most recent read -- a fresh profile read can shift
     * Ls_q / J / Kt and therefore the BW that the live Kp corresponds to. */
    for (const auto& r : m_rows){
        const float kp = static_cast<float>(r.kp->value());
        if (kp == 0.0f){ continue; }
        const float bw = bwFromKp(r.loop, kp, m_motorParams);
        if (!std::isfinite(bw) || bw <= 0.0f){ continue; }
        QSignalBlocker blk(r.bw);
        r.bw->setValue(bw);
    }
}

void GainEditor::emitRead(Loop loop)
{
    if (m_idx < 0){ return; }
    emit readGainRequested(m_idx, loop);
}

void GainEditor::emitApply(Loop loop)
{
    if (m_idx < 0){ return; }
    for (const auto& r : m_rows){
        if (r.loop == loop){
            emit writeGainRequested(m_idx, loop,
                                    static_cast<float>(r.kp->value()),
                                    static_cast<float>(r.ki->value()));
            return;
        }
    }
}

void GainEditor::emitTune(Loop loop)
{
    if (m_idx < 0){ return; }
    for (auto& r : m_rows){
        if (r.loop != loop){ continue; }
        r.status->setText(tr("tuning…"));
        r.status->setStyleSheet(QStringLiteral("color: #555;"));
        r.tune->setEnabled(false);
        emit tuneGainRequested(m_idx, loop,
                                static_cast<float>(r.bw->value()));
        return;
    }
}

void GainEditor::onGainRead(int idx, Loop loop, float kp, float ki, bool ok)
{
    if (idx != m_idx){ return; }
    for (const auto& r : m_rows){
        if (r.loop != loop){ continue; }
        r.kp->setSpecialValueText(ok ? QString() : QStringLiteral("—"));
        r.kp->setValue(ok ? kp : 0.0);
        r.ki->setValue(ok ? ki : 0.0);
        r.kp->setEnabled(ok && m_idx >= 0);
        r.ki->setEnabled(ok && m_idx >= 0);
        r.apply->setEnabled(ok && m_idx >= 0);
        /* Back-compute BW from the live Kp + cached motor profile -- the
         * board does not persist BW, so this is the only consistent way
         * to show "what BW does the controller currently realise". Block
         * the spinbox signal so the recompute doesn't masquerade as an
         * operator edit. */
        if (ok){
            const float bw = bwFromKp(loop, kp, m_motorParams);
            if (std::isfinite(bw) && bw > 0.0f){
                QSignalBlocker blk(r.bw);
                r.bw->setValue(bw);
            }
        }
        break;
    }
}

void GainEditor::onGainTuned(int idx, Loop loop, float kp, float ki, bool ok)
{
    if (idx != m_idx){ return; }
    for (auto& r : m_rows){
        if (r.loop != loop){ continue; }
        if (ok){
            r.kp->setValue(kp);
            r.ki->setValue(ki);
            /* After tune, refresh BW from the freshly-returned Kp so the
             * spinbox shows the BW that actually landed (may differ from
             * the requested BW if the solver clamped or rounded). */
            const float bw = bwFromKp(loop, kp, m_motorParams);
            if (std::isfinite(bw) && bw > 0.0f){
                QSignalBlocker blk(r.bw);
                r.bw->setValue(bw);
            }
            r.status->setText(tr("tuned"));
            r.status->setStyleSheet(QStringLiteral("color: #228822;"));
        } else {
            r.status->setText(tr("failed"));
            r.status->setStyleSheet(QStringLiteral("color: #c0392b;"));
        }
        r.tune->setEnabled(m_idx >= 0);
        break;
    }
}

}  // namespace vrmc
