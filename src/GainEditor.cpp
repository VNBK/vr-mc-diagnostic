#include "GainEditor.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

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

GainEditor::GainEditor(QWidget* parent) : QWidget(parent)
{
    auto* grid = new QGridLayout;
    grid->addWidget(new QLabel(tr("Loop")),  0, 0);
    grid->addWidget(new QLabel(tr("Kp")),    0, 1);
    grid->addWidget(new QLabel(tr("Ki")),    0, 2);

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
        r.read  = new QPushButton(tr("Read"),  this);
        r.apply = new QPushButton(tr("Apply"), this);
        grid->addWidget(new QLabel(tr(loopLabel(l))), row, 0);
        grid->addWidget(r.kp,    row, 1);
        grid->addWidget(r.ki,    row, 2);
        grid->addWidget(r.read,  row, 3);
        grid->addWidget(r.apply, row, 4);
        connect(r.read,  &QPushButton::clicked, this, [this, l]{ emitRead(l);  });
        connect(r.apply, &QPushButton::clicked, this, [this, l]{ emitApply(l); });
        m_rows.push_back(r);
        ++row;
    }

    auto* readAll = new QPushButton(tr("Read all"), this);
    connect(readAll, &QPushButton::clicked, this, [this]{
        for (const auto& r : m_rows){ emitRead(r.loop); }
    });
    grid->addWidget(readAll, row, 0, 1, 5);

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
        r.read->setEnabled(en);
        r.apply->setEnabled(en);
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
        break;
    }
}

}  // namespace vrmc
