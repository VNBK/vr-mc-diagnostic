#include "MotorProfileView.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace vrmc {

namespace {
constexpr int kRowCount = 11;
}

MotorProfileView::MotorProfileView(QWidget* parent) : QWidget(parent)
{
    m_table = new QTableWidget(kRowCount, 3, this);
    m_table->setHorizontalHeaderLabels(
        { tr("Parameter"), tr("Value"), tr("Unit") });
    m_table->verticalHeader()->setVisible(false);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_table->setSelectionMode(QAbstractItemView::NoSelection);
    m_table->setFocusPolicy(Qt::NoFocus);
    m_table->setAlternatingRowColors(true);
    m_table->horizontalHeader()->setStretchLastSection(false);
    /* All three columns auto-fit content. With Stretch on the Value
     * column the pane could shrink it below the actual number width
     * (especially after real motor data with long floats arrives),
     * silently clipping. ResizeToContents keeps each cell readable;
     * the parent splitter's horizontal scrollbar covers the case where
     * the user shrinks the pane below the natural total. */
    m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    m_table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    m_table->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    m_edit = new QPushButton(tr("Edit…"), this);
    m_edit->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_E));
    connect(m_edit, &QPushButton::clicked, this, &MotorProfileView::editRequested);

    auto* btnRow = new QHBoxLayout;
    btnRow->addStretch();
    btnRow->addWidget(m_edit);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(0, 0, 0, 0);
    root->addWidget(m_table, 1);
    root->addLayout(btnRow);

    /* Default values so the table is never empty. */
    setParams(MotorParams{});
}

void MotorProfileView::setRow(int row, const QString& name,
                              const QString& value, const QString& unit)
{
    auto put = [this, row](int col, const QString& text, bool right = false){
        auto* it = m_table->item(row, col);
        if (!it){
            it = new QTableWidgetItem(text);
            m_table->setItem(row, col, it);
        } else {
            it->setText(text);
        }
        if (right){
            it->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        }
    };
    put(0, name);
    put(1, value, /*right=*/true);
    put(2, unit);
}

void MotorProfileView::setParams(const MotorParams& mp)
{
    auto fmt = [](double v, int prec = 4){ return QString::number(v, 'g', prec); };

    int r = 0;
    setRow(r++, tr("Type"),           profile::motorTypeToString(mp.type));
    setRow(r++, tr("Pole pairs"),     QString::number(mp.pole_pair));
    setRow(r++, tr("Rs"),             fmt(mp.rs),         tr("Ω"));
    setRow(r++, tr("Ls_d"),           fmt(mp.ls_d, 6),    tr("H"));
    setRow(r++, tr("Ls_q"),           fmt(mp.ls_q, 6),    tr("H"));
    setRow(r++, tr("Rated PM flux λ"), fmt(mp.rated_flux), tr("Wb"));
    setRow(r++, tr("Inertia J"),      fmt(mp.inertia, 6), tr("kg·m²"));
    setRow(r++, tr("Rated torque (derived)"), fmt(mp.rated_torque), tr("Nm"));
    setRow(r++, tr("Rated speed"),    QString::number(mp.rated_speed), tr("rad/s"));
    setRow(r++, tr("Rated voltage"),  QString::number(mp.rated_vol),   tr("V"));
    setRow(r++, tr("Rated current"),  QString::number(mp.rated_cur),   tr("A"));
}

}  // namespace vrmc
